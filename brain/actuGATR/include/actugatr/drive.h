// drive.h
// Chassis command to motor velocity targets. Every command is checked; the
// motors stop at once for anything Drive cannot execute: a non-body frame,
// non-finite values, motion the kinematics refuses (sideways on a tank), or
// a command older than command_timeout. Wheel speeds are desaturated
// together so the chassis direction is kept.
//
// Control method: V5 smart motor internal velocity loop (PROS
// move_velocity), with each wheel speed converted through the gearing.

#pragma once
#include <cstdint>

#include "actugatr/drivetrain.h"
#include "actugatr/kinematics.h"
#include "actugatr/motor_output.h"

namespace actugatr
{

enum class DriveFault : uint8_t {
    kNone,
    kWrongFrame,        // field frame command; convert with toBody first
    kNonFinite,
    kUnsupportedMotion, // e.g. vy on a tank
    kStale,             // older than command_timeout
};

const char* toString(DriveFault fault);

struct DriveConfig {
    Seconds         command_timeout = 0.1;
    StopMode        stop_mode       = StopMode::kBrake;
    MetersPerSecond stop_below      = 1e-3; // wheel speeds all below this stop instead
};

struct DriveStatus {
    DriveFault     fault   = DriveFault::kNone;
    bool           stopped = true;
    ChassisCommand command;        // last applied, after desaturation
    WheelSpeeds    wheels;         // last targets, m/s
    double         saturation = 1; // desaturation factor of the last command

    // Motor velocity targets sent per wheel group (wheels.count of them),
    // positive driving forward; 0 while stopped.
    double motor_rpm[kMaxWheelGroups] = {};
};

class Drive {
public:
    Drive(const Kinematics& kinematics, const WheelDrive& wheels, MotorOutput& output,
          const DriveConfig& config = {});

    // issued_at: when the command was computed, on the same clock as now.
    DriveFault apply(const ChassisCommand& command, Seconds issued_at, Seconds now);

    // Stops every group with the configured stop mode and records why.
    void stop(DriveFault fault = DriveFault::kNone);

    const DriveStatus& status() const { return status_; }
    const Kinematics&  kinematics() const { return kinematics_; }
    MetersPerSecond    maxWheelSpeed() const { return max_speed_; }

private:
    DriveFault fail(DriveFault fault);

    const Kinematics& kinematics_;
    WheelDrive        wheels_;
    MotorOutput&      output_;
    DriveConfig       config_;
    MetersPerSecond   max_speed_ = 0;
    DriveStatus       status_;
};

} // namespace actugatr
