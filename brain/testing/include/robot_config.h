// robot_config.h
// Robot specific values for the testing application. Every value marked
// PLACEHOLDER is not measured on the robot; set it before driving.
// Units: meters, radians, seconds, heading CCW from +x, robot +x forward, +y left.

#pragma once
#include <cstdint>

#include "api.h"
#include "investigatr/geometry.h"
#include "investigatr/input.h"
#include "investigatr/navigator.h"

namespace robot_config
{

// Drivetrain. Smart ports, negative port = reversed motor (PROS convention).
// PLACEHOLDER: set so +forward drives the robot toward +x on both sides.
constexpr int8_t kLeftMotorPorts[]  = {-1, -2, -3};
constexpr int8_t kRightMotorPorts[] = {4, 5, 6};

// PLACEHOLDER: motor cartridge and behavior at zero demand.
constexpr pros::MotorGears kGearset   = pros::MotorGears::blue;
constexpr pros::MotorBrake kBrakeMode = pros::MotorBrake::brake;

// PLACEHOLDER: full drive demand maps to this voltage, mV.
constexpr int32_t kMaxVoltageMv = 8000;
static_assert(kMaxVoltageMv > 0 && kMaxVoltageMv <= 12000, "V5 motor range is 12000 mV");

// Navigatr link. PLACEHOLDER: smart port wired to the RS-485 adapter. Baud
// must match the Pi serial resource <Baud> (Pi default 115200).
constexpr uint8_t kNavigatrPort = 10;
constexpr int32_t kNavigatrBaud = 115200;

// PLACEHOLDER: robot pose sent to the Pi as the starting placement.
constexpr investigatr::Pose kStartPose{0.0, 0.0, 0.0};

// Navigator tuning. PLACEHOLDER: library defaults, not tuned on the robot.
// Fields not set here keep the NavigatorConfig defaults.
inline investigatr::NavigatorConfig navigatorConfig() {
    investigatr::NavigatorConfig config;
    config.position_tolerance = 0.03;
    config.heading_tolerance  = 0.035;
    config.max_forward        = 0.8;
    config.max_turn           = 0.7;
    config.min_forward        = 0.0;
    config.min_turn           = 0.0;
    config.drive_pid          = {2.0, 0.0, 0.1, 0.0, 1.0};
    config.heading_pid        = {1.5, 0.0, 0.05, 0.0, 1.0};
    config.turn_pid           = {1.2, 0.0, 0.06, 0.0, 1.0};
    return config;
}

// Demo destinations, field frame. PLACEHOLDER: pick clear floor space. The
// path runs from the goal back to the start; only its last point stops.
constexpr investigatr::Pose kDemoGoal{0.6, 0.0, 0.0};
constexpr investigatr::Pose kDemoPath[] = {
    {0.6, 0.6, 0.0},
    {0.0, 0.6, 0.0},
    {0.0, 0.0, 0.0},
};

// Landmark for the relative demo. PLACEHOLDER: wire id of a <FieldObject>
// mapping on the Pi (1..255), and the wanted robot pose in the landmark frame.
constexpr investigatr::LandmarkId kDemoLandmarkId = 1;
constexpr investigatr::Pose       kDemoLandmarkOffset{-0.5, 0.0, 0.0};

// false also accepts the Pi's nominal (map) landmark pose.
constexpr bool kDemoRequireObserved = true;

// Motion timeout per demo step.
constexpr investigatr::Seconds kDemoTimeout = 15.0;

// Controller: left stick Y forward, right stick X turn.
constexpr pros::controller_digital_e_t kDemoButton    = pros::E_CONTROLLER_DIGITAL_A;
constexpr pros::controller_digital_e_t kCancelButton  = pros::E_CONTROLLER_DIGITAL_B;
constexpr int32_t                      kStickDeadband = 5; // of 127

} // namespace robot_config
