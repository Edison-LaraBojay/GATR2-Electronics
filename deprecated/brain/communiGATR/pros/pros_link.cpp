// pros_link.cpp
// PROS only.

#include "communigatr/pros_link.h"

#include <algorithm>
#include <cerrno>
#include <limits>

#include "pros/misc.hpp"

namespace communigatr
{

namespace
{

constexpr Seconds  kReopenPeriod = 1.0; // transport open attempts while closed
constexpr uint32_t kStopWaitMs   = 250;
constexpr uint32_t kStopStepMs   = 2;

// murmur3 finalizer.
uint32_t finish(uint32_t h) {
    h ^= h >> 16;
    h *= 0x85EBCA6Bu;
    h ^= h >> 13;
    h *= 0xC2B2AE35u;
    h ^= h >> 16;
    return h;
}

uint32_t mix(uint32_t h, uint32_t value) {
    return finish(h ^ (value + 0x9E3779B9u + (h << 6) + (h >> 2)));
}

uint32_t addressBits(const void* p) {
    return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(p));
}

ClientConfig clientConfig(const LinkConfig& config) {
    ClientConfig c = config.client;
    if (config.profile) {
        c.profile = makeProfileDocument(*config.profile);
    }
    return c;
}

// A queued USB frame older than most of a response timeout would be answered
// after the client gave up on it.
uint32_t usbLifetimeMs(const ClientConfig& c) {
    return static_cast<uint32_t>(std::clamp(c.response_timeout * 800.0, 1.0, 10000.0));
}

} // namespace

const char* toString(Transport transport) {
    switch (transport) {
    case Transport::kUsb: return "USB";
    case Transport::kSmartPort: return "RS-485";
    }
    return "?";
}

// Bounded take of the link mutex, released on scope exit.
class ProsLink::Lock {
public:
    Lock(pros::Mutex& mutex, uint32_t timeout_ms)
        : mutex_(mutex), held_(mutex.take(timeout_ms)) {}
    ~Lock() {
        if (held_) {
            mutex_.give();
        }
    }
    Lock(const Lock&)            = delete;
    Lock& operator=(const Lock&) = delete;

    explicit operator bool() const { return held_; }

private:
    pros::Mutex& mutex_;
    bool         held_;
};

ProsLink::ProsLink(const LinkConfig& config)
    : config_(config),
      serial_(config.transport == Transport::kSmartPort
                  ? new ProsSerialPort(config.smart_port, config.baud)
                  : nullptr),
      usb_(config.transport == Transport::kUsb ? new ProsUsbPort(usbLifetimeMs(config.client))
                                               : nullptr),
      port_(serial_ ? static_cast<BytePort&>(*serial_) : static_cast<BytePort&>(*usb_)),
      client_(port_, [this] { return nonce(); }, clientConfig(config)),
      driver_(client_, config.driver) {
    if (config.profile) {
        driver_.setProfile(*config.profile); // records the SI profile; same document
    }
    entropy_ = mix(static_cast<uint32_t>(pros::micros()), addressBits(this));
}

// The poll task never blocks longer than a smart port open; deleting it is
// the last resort against it outliving this object.
ProsLink::~ProsLink() {
    stopping_.store(true);
    for (uint32_t waited = 0; waited < kStopWaitMs && running_.load(); waited += kStopStepMs) {
        pros::delay(kStopStepMs);
    }
    if (running_.load() && task_ != nullptr) {
        pros::c::task_delete(task_);
    }
}

bool ProsLink::start() {
    if (started_.load()) {
        return true;
    }
    bool idle = false;
    if (!starting_.compare_exchange_strong(idle, true)) {
        return started_.load(); // another task is starting it
    }
    entropy_ = mix(entropy_, static_cast<uint32_t>(pros::micros()));
    running_.store(true);
    task_ = pros::c::task_create(&ProsLink::entry, this, config_.task_priority,
                                 TASK_STACK_DEPTH_DEFAULT, "communigatr");
    if (task_ == nullptr) {
        running_.store(false);
        starting_.store(false);
        errno = ENOMEM;
        return false;
    }
    started_.store(true);
    starting_.store(false);
    return true;
}

Seconds ProsLink::now() {
    return static_cast<double>(pros::micros()) * 1e-6;
}

void ProsLink::entry(void* self) {
    static_cast<ProsLink*>(self)->run();
}

// Opening happens outside the mutex: only this task touches the port, and a
// smart port open may wait about 100 ms for the port to settle.
void ProsLink::run() {
    const uint32_t period    = std::max<uint32_t>(1, config_.poll_period_ms);
    uint32_t       wake      = pros::millis();
    Seconds        next_open = 0;
    while (!stopping_.load()) {
        if (!portOpen() && now() >= next_open) {
            next_open = now() + kReopenPeriod;
            openPort();
        }
        port_open_.store(portOpen());
        {
            Lock lock(mutex_, period);
            if (lock) {
                client_.poll(now());
            } else {
                poll_lock_misses_.fetch_add(1);
            }
        }
        pros::Task::delay_until(&wake, period);
    }
    running_.store(false);
}

void ProsLink::openPort() {
    opens_.fetch_add(1);
    entropy_ = mix(entropy_, static_cast<uint32_t>(pros::micros()));
    if (serial_) {
        serial_->open();
    } else {
        usb_->open();
    }
}

bool ProsLink::portOpen() const {
    return serial_ ? serial_->isOpen() : usb_->isOpen();
}

void ProsLink::callMissed() const {
    call_lock_misses_.fetch_add(1);
}

ProsLinkStatus ProsLink::status() const {
    ProsLinkStatus s;
    s.started                = started_.load();
    s.transport              = config_.transport;
    s.port_open              = port_open_.load();
    s.link.poll_lock_misses  = poll_lock_misses_.load();
    s.link.opens             = opens_.load();
    if (usb_) {
        s.usb = usb_->stats();
    }
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        s.busy                  = true;
        s.link.call_lock_misses = call_lock_misses_.load();
        return s;
    }
    const Seconds t = now();
    s.ready         = client_.ready();
    s.connected     = client_.connected(t);
    s.link_age      = client_.linkAge(t);
    s.session       = client_.session();
    s.pi_instance   = client_.piInstance();
    s.error         = client_.error();
    s.peer_version  = client_.peerVersion();
    s.summary       = driver_.readiness(t);
    s.readiness     = s.summary.state;
    s.profile       = client_.profile();
    s.state         = client_.state();
    if (s.state.valid && (s.state.state.robot_flags & translagatr::kRobotPoseValid) != 0) {
        s.heading_valid = true;
        s.heading = investigatr::wrapAngle(s.state.state.heading_cdeg * investigatr::kPi / 18000.0);
    }
    const FieldPublication& field = client_.field();
    s.field_sync                  = client_.fieldSync();
    s.field_generation            = field.generation;
    s.map_id                      = field.map_id;
    s.estimate_id                 = field.estimate_id;
    s.field_age                   = field.generation != 0 ? t - field.completed_at
                                                          : std::numeric_limits<Seconds>::infinity();
    s.stats                       = client_.stats();
    s.link.serial_closes          = serial_ ? serial_->closes() : 0;
    s.link.call_lock_misses       = call_lock_misses_.load();
    s.telemetry_unsupported       = client_.telemetryUnsupported();
    return s;
}

investigatr::RobotState ProsLink::robot(Seconds now) {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return investigatr::RobotState{}; // kNoLink
    }
    return driver_.robot(now);
}

bool ProsLink::field(investigatr::Field& out) {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return out.generation != 0; // the field out already holds, unchanged
    }
    return driver_.field(out);
}

void ProsLink::reportPath(investigatr::CommandId command, const investigatr::Path& path) {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return;
    }
    driver_.reportPath(command, path);
}

PlacementTicket ProsLink::place(const investigatr::Pose& pose) {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return 0;
    }
    return driver_.place(pose);
}

// Busy: an unsettled answer for a nonzero ticket, asked again next time.
PlacementStatus ProsLink::placement(PlacementTicket ticket) const {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        PlacementStatus s;
        s.ticket = ticket;
        s.state  = ticket != 0 ? PlacementResult::kPending : PlacementResult::kNone;
        return s;
    }
    return client_.placementStatus(ticket);
}

ControlTicket ProsLink::recalibrate() {
    Lock lock(mutex_, config_.call_timeout_ms);
    return lock ? client_.recalibrate() : (callMissed(), ControlTicket{0});
}

ControlTicket ProsLink::reinitialize() {
    Lock lock(mutex_, config_.call_timeout_ms);
    return lock ? client_.reinitialize() : (callMissed(), ControlTicket{0});
}

ControlTicket ProsLink::reinitImu() {
    Lock lock(mutex_, config_.call_timeout_ms);
    return lock ? client_.reinitImu() : (callMissed(), ControlTicket{0});
}

ControlTicket ProsLink::restartAcquisition() {
    Lock lock(mutex_, config_.call_timeout_ms);
    return lock ? client_.restartAcquisition() : (callMissed(), ControlTicket{0});
}

ControlStatus ProsLink::control(ControlTicket ticket) const {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        ControlStatus s;
        s.ticket = ticket;
        s.state  = ticket != 0 ? ControlResult::kPending : ControlResult::kNone;
        return s;
    }
    return client_.controlStatus(ticket);
}

WheelTicket ProsLink::requestWheels() {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return 0;
    }
    return client_.requestWheels();
}

// Busy: an unsettled answer for a nonzero ticket, asked again next time.
WheelStatus ProsLink::wheels(WheelTicket ticket) const {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        WheelStatus s;
        s.ticket = ticket;
        s.state  = ticket != 0 ? WheelResult::kPending : WheelResult::kNone;
        return s;
    }
    return client_.wheelStatus(ticket);
}

// Busy: busy set, no readings (sequence 0, result NotReady).
WheelReadings ProsLink::wheelReadings() const {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        WheelReadings none;
        none.busy   = true;
        none.result = translagatr::kResultNotReady;
        return none;
    }
    return client_.wheelReadings();
}

bool ProsLink::setProfile(const RobotProfile& profile) {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return false;
    }
    return driver_.setProfile(profile);
}

// Busy: an empty profile, which the Brain check refuses if set again.
RobotProfile ProsLink::profile() const {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return RobotProfile{};
    }
    return driver_.profile();
}

bool ProsLink::resubmitProfile() {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return false;
    }
    client_.resubmitProfile();
    return true;
}

bool ProsLink::reportTelemetry(const translagatr::BrainTelemetry& telemetry) {
    Lock lock(mutex_, config_.call_timeout_ms);
    if (!lock) {
        callMissed();
        return false;
    }
    return client_.reportTelemetry(telemetry);
}

// Called by the client inside poll(), once per new HELLO. Mixes micros() at
// construction, start, each open and this call, battery voltage and current,
// and two addresses. Only uniqueness against the Pi's last four opening
// nonces matters; a collision costs one kResultStale and a new nonce.
uint32_t ProsLink::nonce() {
    const uint64_t t = pros::micros();
    uint32_t       h = entropy_;

    h = mix(h, static_cast<uint32_t>(t));
    h = mix(h, static_cast<uint32_t>(t >> 32));
    h = mix(h, static_cast<uint32_t>(pros::battery::get_voltage()));
    h = mix(h, static_cast<uint32_t>(pros::battery::get_current()));
    h = mix(h, addressBits(&h));
    h = mix(h, ++nonces_);

    entropy_ = h;
    return h;
}

} // namespace communigatr
