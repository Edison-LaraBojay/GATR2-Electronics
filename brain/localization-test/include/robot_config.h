#pragma once

#include <cstdint>

#include "investigatr/geometry.h"

namespace robot_config {

// V5 Smart Port connected to the Pi's RS-485 link (confirmed for this test).
constexpr uint8_t kNavigatrPort = 10;
constexpr int32_t kNavigatrBaud = 115200;

// Temporary VEX IMU bench fallback. Pair with bench_vex_imu.xml on the Pi.
// Set false to use the original Pico/BNO08X localization profiles instead.
constexpr bool kUseVexImuBench = true;
constexpr uint8_t kVexImuPort = 1;

// Starting FIELD pose. Edit to match where you place the robot before running.
// Meters; heading in degrees here for convenience, converted to radians below.
// +x right, +y up on the field diagram; heading 0 faces +x, CCW is positive.
constexpr double kStartX = 0.0;
constexpr double kStartY = 0.0;
constexpr double kStartHeadingDegrees = 0.0;
constexpr investigatr::Pose kStartPose{
    kStartX, kStartY, kStartHeadingDegrees * investigatr::kPi / 180.0};

// Try the starting placement once if the link connects within this window.
// After that, controller A or the screen button submits it explicitly.
constexpr double kStartupWaitSeconds = 10.0;
constexpr double kPlacementDeadlineSeconds = 2.0;
constexpr double kMaxPoseAgeSeconds = 0.25;

constexpr uint32_t kLoopPeriodMs = 20;
constexpr uint32_t kDisplayPeriodMs = 100;
constexpr uint32_t kLogPeriodMs = 1000; // USB terminal, not the Pi serial link

static_assert(kNavigatrPort >= 1 && kNavigatrPort <= 21);
static_assert(kNavigatrBaud > 0);
static_assert(kVexImuPort >= 1 && kVexImuPort <= 21 && kVexImuPort != kNavigatrPort);
static_assert(kStartupWaitSeconds > 0 && kPlacementDeadlineSeconds > 0);
static_assert(kMaxPoseAgeSeconds > 0);
static_assert(kLoopPeriodMs > 0 && kDisplayPeriodMs > 0 && kLogPeriodMs > 0);

} // namespace robot_config
