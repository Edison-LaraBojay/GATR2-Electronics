// robot_config.h
// Localization test settings. The robot itself (tracking wheels, encoders,
// IMU source, footprint, Pi link, start pose) is described once, for every
// program, in brain/robot/gatr2_robot.h.

#pragma once
#include <cstdint>

#include "communigatr/startup_placement.h"
#include "communigatr/wheel_calibration.h"
#include "gatr2_robot.h"

namespace robot_config
{

// Place at gatr2_robot::kStartPose once, when the program starts and the link,
// profile and sensors are ready. kIfUnplaced keeps a placement the Pi still
// holds after a Brain restart; kNever leaves placing to controller A.
constexpr communigatr::StartupPolicy kStartupPolicy = communigatr::StartupPolicy::kAlways;
constexpr double                     kStartupWaitSeconds = 90.0; // covers a cold Pi boot

// Wheel calibration: the pushed distance you measure yourself, starting
// value; L1/R1 change it by 1 cm and L2/R2 by 10 cm on the calibration page.
constexpr double kCalibrationDistance = 1.0; // meters

inline communigatr::WheelCalibrationConfig wheelCalibration() {
    communigatr::WheelCalibrationConfig c;
    c.min_reference  = 0.5;
    c.max_reference  = 5.0;
    c.max_correction = 0.1;
    c.max_rotation   = 0.5 * gatr2_robot::kDeg; // tape point lever x this adds error
    c.max_cross      = 0.05;
    c.max_age        = 0.25;
    c.min_trials     = 3;
    c.max_spread     = 0.01;
    return c;
}

// A pose is LIVE when connected, valid and at most this old.
constexpr double   kMaxPoseAgeSeconds = 0.25;
constexpr uint32_t kLoopPeriodMs      = 20;
constexpr uint32_t kDisplayPeriodMs   = 100;

} // namespace robot_config
