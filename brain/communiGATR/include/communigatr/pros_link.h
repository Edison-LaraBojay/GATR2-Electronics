// pros_link.h
// The brain link for a PROS program: one class for both transports, the V5
// USB user console or an RS-485 adapter on a smart port. Owns the port, the
// Client and LinkDriver, and a poll task that opens the transport, reopens
// it at most once a second while it is closed, and polls the client.
//
// Every public call takes the link mutex with a bounded wait and gives a safe
// answer when it cannot (busy status, kNoLink robot, refused ticket), so no
// caller blocks for long and a competition task deleted while holding it
// costs at most skipped poll cycles. Calls copy values out; nothing shared
// outlives the lock. Source in pros/, PROS builds only.

#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

#include "communigatr/client.h"
#include "communigatr/link_driver.h"
#include "communigatr/pros_serial_port.h"
#include "communigatr/pros_usb_port.h"
#include "communigatr/readiness.h"
#include "communigatr/robot_profile.h"
#include "investigatr/path.h"
#include "investigatr/state_source.h"
#include "pros/rtos.hpp"

namespace communigatr
{

enum class Transport : uint8_t {
    kUsb,       // V5 USB user console, NG1 lines; the Pi config uses pros_usb_link
    kSmartPort, // RS-485 adapter on smart_port; the Pi config uses linux_serial_link
};

const char* toString(Transport transport);

struct LinkConfig {
    Transport transport  = Transport::kUsb;
    uint8_t   smart_port = 0;      // kSmartPort: V5 port 1..21 wired to the RS-485 adapter
    int32_t   baud       = 115200; // kSmartPort: must match the Pi serial resource

    uint32_t poll_period_ms  = 2;  // at least 1
    uint32_t task_priority   = TASK_PRIORITY_DEFAULT + 1;
    uint32_t call_timeout_ms = 20; // bounded mutex wait of every public call

    // Robot profile uploaded to the Pi. Empty: client.profile as given (none
    // means the Pi runs its XML localization).
    std::optional<RobotProfile> profile;

    ClientConfig     client; // bench_imu runs in the poll task under the link mutex
    LinkDriverConfig driver;
};

struct ProsLinkStats {
    uint32_t poll_lock_misses = 0; // poll cycles skipped, mutex not taken in time
    uint32_t call_lock_misses = 0; // public calls answered busy
    uint32_t opens            = 0; // transport open attempts
    uint32_t serial_closes    = 0; // smart port closed after repeated errors
};

struct ProsLinkStatus {
    bool      busy      = false; // mutex not taken in time: nothing below is current
    bool      started   = false; // poll task running
    Transport transport = Transport::kUsb;
    bool      port_open = false;

    bool      ready        = false; // session open and a state reply in it
    bool      connected    = false; // ready and a reply within link_timeout
    Seconds   link_age     = 0;     // infinity without a session
    uint32_t  session      = 0;
    uint32_t  pi_instance  = 0;
    LinkError error        = LinkError::kNone;
    uint8_t   peer_version = 0;

    Readiness     readiness = Readiness::kConnecting; // summary.state
    LinkReadiness summary;                            // decoded health, calibration, IMU use
    ProfileStatus profile;
    StateSample   state; // latest GET_STATE Ok of this session, raw wire units

    // Raw Pi heading, placed or not, for calibration checks: valid while the
    // estimator has a pose. Wrapped to (-pi, pi].
    bool                 heading_valid = false;
    investigatr::Radians heading       = 0;

    FieldSyncStatus field_sync;
    uint32_t        field_generation = 0; // published map and estimate pairs, 0 = none
    uint32_t        map_id           = 0; // published map
    uint32_t        estimate_id      = 0; // published estimate
    Seconds         field_age        = 0; // since the published estimate completed; infinity without

    ClientStats   stats;
    ProsLinkStats link;
    ProsUsbStats  usb; // kUsb only
};

class ProsLink : public investigatr::StateSource, public investigatr::PathSink {
public:
    explicit ProsLink(const LinkConfig& config);

    // Stops the poll task. Keep a started link for the program's life.
    ~ProsLink() override;
    ProsLink(const ProsLink&)            = delete;
    ProsLink& operator=(const ProsLink&) = delete;

    // Starts the poll task. False when it could not be created (errno set);
    // call again to retry. True once started. The transport opens in the task.
    bool start();

    // Poll task clock: seconds since PROS started. Pass it to robot() and to
    // every controller that consumes this source.
    static Seconds now();

    ProsLinkStatus status() const;

    // StateSource and PathSink. On a busy mutex: kNoLink robot; field() leaves
    // out as it was; a path report is dropped.
    investigatr::RobotState robot(Seconds now) override;
    bool                    field(investigatr::Field& out) override;
    void reportPath(investigatr::CommandId command, const investigatr::Path& path) override;

    // Placement: 0 when refused (no link, profile not applied, one pending,
    // pose not finite) or busy.
    PlacementTicket place(const investigatr::Pose& pose);
    PlacementStatus placement(PlacementTicket ticket) const;

    // Pi control, see gatr2::ControlAction. 0 when refused or busy. The Pi
    // checks the stationary condition.
    ControlTicket recalibrate();
    ControlTicket reinitialize();
    ControlTicket reinitImu();
    ControlTicket restartAcquisition();
    ControlStatus control(ControlTicket ticket) const;

    // One READ_WHEELS read. 0 when refused (no session, another read
    // pending) or busy. wheels(ticket) is kPending for a nonzero ticket when
    // busy. wheelReadings() is the latest Ok reply of any read; busy sets
    // its busy flag.
    WheelTicket   requestWheels();
    WheelStatus   wheels(WheelTicket ticket) const;
    WheelReadings wheelReadings() const;

    // Robot profile. setProfile validates with the shared rules and starts
    // the upload of a new id; false when the Brain check refuses it (the
    // running profile stays) or busy. profile() is the configured profile.
    bool         setProfile(const RobotProfile& profile);
    RobotProfile profile() const;
    bool         resubmitProfile();

private:
    class Lock;

    static void entry(void* self);
    void        run();
    void        openPort();
    bool        portOpen() const;
    uint32_t    nonce();
    void        callMissed() const;

    LinkConfig                      config_;
    std::unique_ptr<ProsSerialPort> serial_;
    std::unique_ptr<ProsUsbPort>    usb_;
    BytePort&                       port_;
    Client                          client_;
    LinkDriver                      driver_;
    mutable pros::Mutex             mutex_;

    std::atomic<bool>             starting_{false};
    std::atomic<bool>             started_{false};
    std::atomic<bool>             stopping_{false};
    std::atomic<bool>             running_{false};
    std::atomic<bool>             port_open_{false};
    mutable std::atomic<uint32_t> poll_lock_misses_{0};
    mutable std::atomic<uint32_t> call_lock_misses_{0};
    std::atomic<uint32_t>         opens_{0};

    pros::task_t    task_    = nullptr;
    uint32_t        entropy_ = 0; // poll task only after start
    uint32_t        nonces_  = 0;
};

} // namespace communigatr
