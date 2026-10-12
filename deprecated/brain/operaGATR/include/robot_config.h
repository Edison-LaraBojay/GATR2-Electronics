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
constexpr double kInch = 0.0254;

// ---------------------------------------------------------------------------
// Drivetrain. Pick one, then fill in its block below.
//   kTank     left and right sides, 1 to 4 motors per side
//   kMecanum  four wheels, each driven on its own, 1 to 4 motors per wheel
//             (usually one)
//
// Motors are {Smart Port, reversed}. reversed = true for a motor that spins
// backward when told to drive forward (usually every motor on one side).
// Check with the wheels off the ground: sticks forward slowly, every driven
// wheel must roll forward. A port
// may not repeat or be the VEX IMU port; the program refuses to drive if one
// does.
//
// Every value marked PLACEHOLDER below is a guess. Replace it with your
// robot's value before driving.
// ---------------------------------------------------------------------------
enum class Drivetrain : uint8_t { kTank, kMecanum };
constexpr Drivetrain kDrivetrain = Drivetrain::kMecanum;

// Shared wheel settings, the same for tank and mecanum.
inline actugatr::WheelDrive drivenWheels(double diameter, double gear_ratio) {
    actugatr::WheelDrive w;
    // Confirmed blue 600 RPM cartridges for the tank and mecanum builds.
    w.cartridge = actugatr::Cartridge::kBlue;
    // Share of top speed used, headroom for the motor's speed loop.
    w.usable_fraction = 0.9;
    w.wheel_diameter  = diameter;
    w.gear_ratio      = gear_ratio;
    return w;
}

inline actugatr::TankConfig tank() {
    actugatr::TankConfig c;
    // Every motor on the left side, then every motor on the right. reversed
    // true is a negative port in PROS terms.
    c.left  = actugatr::motorGroup({
        {18, true},  // front
        {15, false}, // middle
        {11, true},  // rear
        {13, true},  // aux
    });

    c.right = actugatr::motorGroup({
        {19, false}, // front
        {16, true},  // middle
        {12, false}, // rear
        {14, false}, // aux
    });
    // Full left-to-right driven-wheel center spacing, 10.65 inches.
    c.track_width = 10.65 * kInch;
    // 2.75-inch driven wheels, 1:1 external gearing (wheel turns/motor turn).
    c.wheels    = drivenWheels(2.75 * kInch, 1.0);
    c.stop_mode = actugatr::StopMode::kBrake;
    return c;
}

inline actugatr::MecanumConfig mecanum() {
    actugatr::MecanumConfig c;
    // Four motors: corners named facing forward from behind the robot.
    // Positive wheel commands should roll forward: 2/12 normal, 1/11 reversed.
    // Verify all directions with the wheels lifted.
    // Standard X roller layout only: matching wheel handedness on opposite
    // diagonals. A/B wheel labels are mechanical, not motor reversal flags.
    c.front_left  = actugatr::motorGroup({{12, false}});
    c.front_right = actugatr::motorGroup({{11, true}});
    c.rear_left   = actugatr::motorGroup({{2, false}});
    c.rear_right  = actugatr::motorGroup({{1, true}});
    // Measured full spans: 22.3 cm between left/right tread centerlines,
    // 24.2 cm between front/rear shafts. Origin is this rectangle's center.
    c.track_width = 0.223;
    c.wheelbase   = 0.242;
    // Approximately 70 mm mecanum wheel diameter.
    // Confirmed 1:1 direct drive: motor -> shaft -> wheel.
    c.wheels    = drivenWheels(0.070, 1.0);
    c.stop_mode = actugatr::StopMode::kBrake;
    return c;
}

// ---------------------------------------------------------------------------
// Movement. Clearance is added around the footprint for obstacle avoidance;
// keep the follower tracking_tolerance below it.
// ---------------------------------------------------------------------------
constexpr double kClearance = 0.06;

// Safe starting limits; raise them after the first runs. Speed and turn rate
// are also capped at what the drivetrain above can reach.
inline investigatr::MotionLimits limits() {
    investigatr::MotionLimits l;
    l.max_speed = 0.8; // m/s
    l.max_accel = 1.5; // m/s^2
    l.max_omega = 3.0; // rad/s
    l.max_alpha = 6.0; // rad/s^2
    return l;
}

// PLACEHOLDER gains: starting values, not tuned on the robot. Tune them on
// the robot: docs/actugatr.md.
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
// A/X plan from the latest localized pose to the same fixed destination;
// neither resets the start pose. A is direct, X checks field obstacles.
// Starting pose is in gatr2_robot.h; Y remains a separate landmark example.
// Poses are {x, y, heading}: meters from the field origin (the inside
// bottom left corner of the field diagram), heading CCW from +x.
// ---------------------------------------------------------------------------
// Edit this destination for both A and X: (72 inches, 24 inches, 90 degrees).
constexpr investigatr::Pose kDirectGoal{35.0 * kInch, 24.0 * kInch, 90.0 * kDeg};
// Same field-relative destination with obstacle avoidance.
constexpr investigatr::Pose kAvoidGoal = kDirectGoal;
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
