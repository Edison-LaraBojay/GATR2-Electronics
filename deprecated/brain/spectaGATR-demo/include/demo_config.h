// demo_config.h
// spectaGATR demo settings. Lengths in inches and angles in degrees; the
// code converts them with spectagatr::inchesToMeters and degreesToRadians.
// Field frame (spectaGATR/docs/PROTOCOL.md): origin at the bottom left corner
// of the field diagram, +x right, +y up, heading CCW from +x.

#pragma once
#include <cstdint>

#include "spectagatr/pose.h"

namespace demo_config
{

using spectagatr::Degrees;
using spectagatr::Hertz;
using spectagatr::Inches;
using spectagatr::Milliseconds;

enum class Source : uint8_t { kSynthetic, kVexImu, kCallerOdometry };

// Source at power on; LEFT/RIGHT on the controller cycles it.
constexpr Source kDefaultSource = Source::kSynthetic;

// Lines per second to the laptop, 1 to 50.
constexpr Hertz kRateHz = 10;

// The demo supplies a pose this often; the publisher sends only the newest.
constexpr Milliseconds kLoopMs = 10;

// SYNTHETIC: figure eight on the Override field (about 140.5 in square).
constexpr Inches       kEightCenterXInches    = 70.0;
constexpr Inches       kEightCenterYInches    = 70.0;
constexpr Inches       kEightHalfWidthInches  = 40.0; // along x
constexpr Inches       kEightHalfHeightInches = 24.0; // along y
constexpr Milliseconds kEightPeriodMs         = 16000;
// Heading only for one phase, full attitude for the next, and so on.
constexpr Milliseconds kAttitudePhaseMs     = 10000;
constexpr Degrees      kWobbleRollDegrees   = 5.0;
constexpr Degrees      kWobblePitchDegrees  = 3.0;
constexpr Milliseconds kWobbleRollPeriodMs  = 3000;
constexpr Milliseconds kWobblePitchPeriodMs = 4700;

// VEX IMU ATTITUDE: the IMU gives no position, x/y stay at this point.
// Port 20: the former robot's IMU (brain/robot/gatr2_robot.h at commit 10c568e).
constexpr uint8_t kImuPort         = 20;
constexpr Inches  kImuFixedXInches = 70.0;
constexpr Inches  kImuFixedYInches = 70.0;
// Heading published while the IMU rotation reads 0.
constexpr Degrees kImuStartHeadingDegrees = 90.0;
// IMU +x axis in the robot frame, CCW from forward: 0, 90, 180 or 270. Flat
// mount only.
constexpr Degrees kImuMountYawDegrees = 0.0;
// +1 or -1, from the bench check in README.md.
constexpr double kImuRollSign  = 1.0;
constexpr double kImuPitchSign = 1.0;

} // namespace demo_config
