// vex_imu_recalibration.h
// Recalibration of the Brain VEX IMU that never starts while the robot moves.
// VEX firmware calibrates the IMU (pros::Imu::reset); the Pi judges
// stillness. A run first sends CONTROL recalibrate, which for a Brain VEX IMU
// profile checks that every profile wheel stayed still over the last 300 ms
// and calibrates nothing on the Pi, and starts the VEX calibration only after
// its Ok. With a Pico IMU profile CONTROL recalibrate is the whole
// recalibration and this helper is not used. Portable and I/O free: the
// program passes the link's answer and the IMU sample in and calls the IMU.

#pragma once
#include <cstdint>

#include "communigatr/client.h"

namespace communigatr
{

enum class VexRecalibrationState : uint8_t {
    kIdle,        // no run yet
    kChecking,    // waiting for the Pi's stationary check
    kStart,       // still: call ProsVexImu::recalibrate() now, then started()
    kCalibrating, // VEX firmware calibrating; keep still
    kDone,        // finished with a valid sample
    kMoving,      // the Pi saw movement; nothing started
    kRefused,     // the Pi refused or never answered, see check()
    kImuFailed,   // the IMU did not start, did not finish in time, or is invalid after
};

const char* toString(VexRecalibrationState state);

class VexImuRecalibration {
public:
    // limit: calibration start through its end.
    explicit VexImuRecalibration(Seconds limit = 10.0);

    // A run for the ticket of a CONTROL recalibrate (ProsLink::recalibrate).
    // False when the ticket is 0 (the link refused it) or a run is active.
    bool begin(ControlTicket ticket);

    // Every loop while active(): the ticket's status from the link and the
    // IMU's latest sample (ProsVexImu::sample).
    VexRecalibrationState update(const ControlStatus& check, const BenchImuSample& imu,
                                 Seconds now);

    // After kStart: the result of ProsVexImu::recalibrate().
    void started(bool ok, Seconds now);

    VexRecalibrationState state() const { return state_; }
    bool                  active() const;
    ControlTicket         ticket() const { return check_.ticket; }
    const ControlStatus&  check() const { return check_; } // the Pi's last answer

private:
    Seconds               limit_;
    VexRecalibrationState state_      = VexRecalibrationState::kIdle;
    ControlStatus         check_;
    Seconds               started_at_ = 0;
    bool                  seen_       = false; // IMU reported calibrating since the start
};

} // namespace communigatr
