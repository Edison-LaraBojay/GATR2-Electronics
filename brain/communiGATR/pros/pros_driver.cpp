// pros_driver.cpp
// PROS only.

#include "communigatr/pros_driver.h"

#include <mutex>

#include "pros/misc.hpp"

namespace communigatr
{

namespace
{

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

} // namespace

ProsDriver::ProsDriver(const ProsDriverConfig& config)
    : config_(config), port_(config.port, config.baud),
      client_(port_, [this] { return nonce(); }, config.client), driver_(client_, config.driver) {
    entropy_ = mix(static_cast<uint32_t>(pros::micros()), addressBits(this));
}

// Keep the object for the life of the program; this only stops the task.
ProsDriver::~ProsDriver() {
    if (task_) {
        std::lock_guard<pros::Mutex> lock(mutex_);
        task_->remove();
    }
}

bool ProsDriver::start() {
    std::lock_guard<pros::Mutex> lock(mutex_);
    if (task_) {
        return true;
    }
    entropy_ = mix(entropy_, static_cast<uint32_t>(pros::micros()));
    if (!port_.open()) {
        return false;
    }
    task_.reset(new pros::Task([this] { run(); }, config_.task_priority, TASK_STACK_DEPTH_DEFAULT,
                               "communigatr"));
    return true;
}

Seconds ProsDriver::now() { return static_cast<double>(pros::micros()) * 1e-6; }

void ProsDriver::request(const investigatr::InputRequest& request) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    driver_.request(request);
}

investigatr::InputSnapshot ProsDriver::latest(Seconds now) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    return driver_.latest(now);
}

PlacementTicket ProsDriver::submitPlacement(const investigatr::Pose& pose) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    return driver_.submitPlacement(pose);
}

PlacementResult ProsDriver::placementResult(PlacementTicket ticket) const {
    std::lock_guard<pros::Mutex> lock(mutex_);
    return driver_.placementResult(ticket);
}

PlacementStatus ProsDriver::placementStatus(PlacementTicket ticket) const {
    std::lock_guard<pros::Mutex> lock(mutex_);
    return driver_.placementStatus(ticket);
}

ProsDriverStatus ProsDriver::status() const {
    std::lock_guard<pros::Mutex> lock(mutex_);

    const Seconds    t = now();
    ProsDriverStatus s;
    s.started      = task_ != nullptr;
    s.ready        = client_.ready();
    s.connected    = client_.connected(t);
    s.link_age     = client_.linkAge(t);
    s.session      = client_.session();
    s.pi_instance  = client_.piInstance();
    s.error        = client_.error();
    s.peer_version = client_.peerVersion();
    s.selection    = client_.selection();
    s.stats        = client_.stats();
    return s;
}

void ProsDriver::run() {
    const uint32_t period = config_.poll_period_ms > 0 ? config_.poll_period_ms : 1;
    uint32_t       wake   = pros::millis();
    while (true) {
        {
            std::lock_guard<pros::Mutex> lock(mutex_);
            client_.poll(now());
        }
        pros::Task::delay_until(&wake, period);
    }
}

// Called by the client, inside poll(), once per new HELLO. Mixes micros() at
// construction, start() and this call, battery voltage and current, and two
// addresses. Only uniqueness against the Pi's last four opening nonces
// matters; a collision costs one kResultStale and a new nonce.
uint32_t ProsDriver::nonce() {
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
