// robot_config.h
// Testing program settings: drivetrain, movement limits and gains, test
// destinations and controls. The robot's localization description (tracking
// wheels, IMU, footprint, Pi link, start pose) is shared by every program:
// brain/robot/gatr2_robot.h.
//
// PLACEHOLDER: depends on how the robot is built; set before driving.
// Units: meters, radians, seconds. Field frame +x right, +y up on the field
// diagram, heading CCW from +x. Robot frame +x forward, +y left.

#pragma once
#include <cstdint>

#include "actugatr/drive.h"
#include "actugatr/drive_owner.h"
#include "actugatr/drivetrain.h"
#include "actugatr/follower.h"
#include "actugatr/motion.h"
#include "api.h"
#include "communigatr/startup_placement.h"
#include "field_references.h"
#include "gatr2_robot.h"

namespace robot_config
{

constexpr double kDeg = investigatr::kPi / 180.0;

// ---------------------------------------------------------------------------
// Drivetrain. Tank is the current robot; the mecanum settings are a complete
// example for a mecanum chassis.
// ---------------------------------------------------------------------------
enum class Drivetrain : uint8_t { kTank, kMecanum };
constexpr Drivetrain kDrivetrain = Drivetrain::kTank;

// PLACEHOLDER ports. Smart Port 1 is the VEX IMU; the port check at startup
// refuses to drive if a motor shares a port with any active device.
inline actugatr::TankConfig tank() {
    actugatr::TankConfig c;
    c.left.count             = 3;
    c.left.motors[0]         = {11, true};
    c.left.motors[1]         = {12, true};
    c.left.motors[2]         = {13, true};
    c.right.count            = 3;
    c.right.motors[0]        = {18, false};
    c.right.motors[1]        = {19, false};
    c.right.motors[2]        = {20, false};
    c.track_width            = 0.30;   // PLACEHOLDER, driven wheel contact spacing
    c.wheels.wheel_diameter  = 0.1016; // PLACEHOLDER, 4 in driven wheels
    c.wheels.gear_ratio      = 0.6;    // PLACEHOLDER, 36:60, wheel turns per motor turn
    c.wheels.cartridge       = actugatr::Cartridge::kBlue;
    c.wheels.usable_fraction = 0.9;
    c.stop_mode              = actugatr::StopMode::kBrake;
    return c;
}

inline actugatr::MecanumConfig mecanum() {
    actugatr::MecanumConfig c;
    c.front_left.count        = 1;
    c.front_left.motors[0]    = {11, true};
    c.front_right.count       = 1;
    c.front_right.motors[0]   = {18, false};
    c.rear_left.count         = 1;
    c.rear_left.motors[0]     = {12, true};
    c.rear_right.count        = 1;
    c.rear_right.motors[0]    = {19, false};
    c.track_width             = 0.30; // PLACEHOLDER
    c.wheelbase               = 0.28; // PLACEHOLDER
    c.wheels.wheel_diameter   = 0.1016;
    c.wheels.gear_ratio       = 1.0;
    c.wheels.cartridge        = actugatr::Cartridge::kBlue;
    c.wheels.usable_fraction  = 0.9;
    c.stop_mode               = actugatr::StopMode::kBrake;
    return c;
}

// ---------------------------------------------------------------------------
// Movement. Clearance is added around the footprint for obstacle avoidance;
// keep the follower tracking_tolerance below it.
// ---------------------------------------------------------------------------
constexpr double kClearance = 0.06;

inline investigatr::MotionLimits limits() {
    investigatr::MotionLimits l;
    l.max_speed = 0.8; // m/s
    l.max_accel = 1.5; // m/s^2
    l.max_omega = 3.0; // rad/s
    l.max_alpha = 6.0; // rad/s^2
    return l;
}

// Tuning: see docs/actugatr.md. PLACEHOLDER, not tuned on the robot.
inline actugatr::FollowerConfig follower() {
    actugatr::FollowerConfig f;
    f.position_tolerance = 0.02;
    f.heading_tolerance  = 0.03;
    f.settle_time        = 0.2;
    f.tracking_tolerance = 0.04;
    f.along              = {3.0, 0.0, 0.0, 0.0, 100.0};  // (m/s) per m remaining
    f.cross              = {3.0, 0.0, 0.0, 0.0, 100.0};  // tank (rad/s) per m, mecanum (m/s) per m
    f.heading            = {4.0, 0.0, 0.1, 0.0, 100.0};  // (rad/s) per rad
    f.turn               = {4.0, 0.0, 0.2, 0.0, 100.0};  // (rad/s) per rad
    f.min_speed          = 0.0;
    f.min_omega          = 0.0;
    return f;
}

inline actugatr::MotionConfig motion(const investigatr::MotionModel& model) {
    actugatr::MotionConfig m;
    m.model           = model;
    m.default_timeout = 15.0;
    return m;
}

// Manual driving: full stick in physical units.
inline actugatr::DriveOwnerConfig manual() {
    actugatr::DriveOwnerConfig c;
    c.manual_speed   = 1.0; // m/s
    c.manual_omega   = 3.0; // rad/s
    c.manual_timeout = 0.25;
    return c;
}

// ---------------------------------------------------------------------------
// Tests. PLACEHOLDER destinations: pick clear floor space on your field.
// ---------------------------------------------------------------------------
// Direct: field origin reference, no obstacles checked.
constexpr investigatr::Pose kDirectGoal{1.4, 0.6, 0.0};
// Avoiding: field origin reference, routes around the field's obstacles.
constexpr investigatr::Pose kAvoidGoal{1.4, 1.2, 90.0 * kDeg};
// Landmark relative: 0.45 m on the landmark's +x side, facing it.
constexpr investigatr::Pose kLandmarkOffset{0.45, 0.0, 180.0 * kDeg};
inline investigatr::Reference landmark() { return Field::RedGoal2West; }
// Without a camera the landmark pose is the nominal map pose.
constexpr bool kRequireObservedLandmark = false;

constexpr double kSpeedScale = 0.5; // test runs start slow; LEFT/RIGHT change it

// Place at gatr2_robot::kStartPose once at program start, when the link,
// profile and sensors are ready; UP places explicitly after that.
constexpr communigatr::StartupPolicy kStartupPolicy = communigatr::StartupPolicy::kAlways;
constexpr double kStartupWaitSeconds = 90.0; // covers a cold Pi boot

// Controller: left stick Y forward, right stick X turn, left stick X strafe
// (mecanum). A direct test, X avoiding test, Y landmark test, B cancel,
// UP place at the start pose, DOWN recalibrate (robot still, drive idle),
// LEFT/RIGHT speed scale.
constexpr int32_t kStickDeadband = 5; // of 127

} // namespace robot_config
