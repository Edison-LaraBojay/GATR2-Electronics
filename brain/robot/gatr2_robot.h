// gatr2_robot.h
// The robot's localization description, shared by every Brain program for
// this robot. Edit here, rebuild and upload; the Brain sends it to the Pi as
// the robot profile when it connects, so no Pi XML changes are needed for
// geometry, encoders, IMU source or footprint.
//
// UNMEASURED: a guess that still needs measuring.
// PLACEHOLDER: a value that depends on how the robot is built.
// Robot frame: +x forward, +y left, origin the point the Pi reports (pick it,
// usually the turning center). Field frame: +x right, +y up on the field
// diagram, heading CCW from +x.

#pragma once
#include <cstdint>

#include "communigatr/robot_profile.h"
#include "investigatr/geometry.h"
#include "investigatr/motion_model.h"

namespace gatr2_robot
{

constexpr double kDeg = investigatr::kPi / 180.0;

// ---------------------------------------------------------------------------
// Pi link. The Pi must run the matching config: brain_profile_usb.xml for
// USB, brain_profile_rs485.xml for the Smart Port RS-485 link.
// ---------------------------------------------------------------------------
constexpr bool    kUseUsb       = true;
constexpr uint8_t kLinkPort     = 10;     // Smart Port, RS-485 only. PLACEHOLDER
constexpr int32_t kLinkBaud     = 115200; // RS-485 only, must match the Pi

// ---------------------------------------------------------------------------
// Localization setup. Pick one.
//   kTwoWheelVexImu    forward wheel on port 0, sideways wheel on port 1, VEX IMU
//   kTwoWheelPicoImu   same wheels, the Pico IMU (external BNO08X or ASM330)
//   kThreeWheelPicoImu three wheels fused with the Pico IMU
// ---------------------------------------------------------------------------
enum class Setup : uint8_t { kTwoWheelVexImu, kTwoWheelPicoImu, kThreeWheelPicoImu };
constexpr Setup kSetup = Setup::kTwoWheelVexImu;

// VEX IMU, read by the Brain (kTwoWheelVexImu only).
constexpr uint8_t kVexImuPort = 1;

// ---------------------------------------------------------------------------
// Tracking wheels (not the driven wheels). Port = naviGATR encoder port:
// 0 J2, 1 J3, 2 J4 on HAT v2.
//
// Three kinds of value, applied once each on the Pi:
//   encoder    counts_per_rev of the encoder shaft, reversed (encoder
//              polarity), gear_ratio (encoder turns per wheel turn)
//   geometry   radius, x, y (contact point), angle (measuring direction)
//   measured   travel_scale from the wheel calibration in localization-test;
//              keep 1.0 until calibrated, never use it to hide a wrong radius
// ---------------------------------------------------------------------------
constexpr double   kTrackingRadius = 0.024; // UNMEASURED, provisional 48 mm wheel
constexpr uint32_t kCountsPerRev   = 4000;  // AS5047P ABI default. CONFIRM for your encoder

inline communigatr::TrackingWheel forwardWheel() {
    communigatr::TrackingWheel w;
    w.encoder_port   = 0;
    w.radius         = kTrackingRadius;
    w.counts_per_rev = kCountsPerRev;
    w.x              = 0.0;  // UNMEASURED
    w.y              = 0.15; // UNMEASURED, left of the origin
    w.angle          = 0.0;  // measures forward travel
    w.reversed       = false;
    w.gear_ratio     = 1.0;
    w.travel_scale   = 1.0;
    return w;
}

inline communigatr::TrackingWheel sidewaysWheel() {
    communigatr::TrackingWheel w;
    w.encoder_port   = 1;
    w.radius         = kTrackingRadius;
    w.counts_per_rev = kCountsPerRev;
    w.x              = 0.15; // UNMEASURED, ahead of the origin
    w.y              = 0.0;  // UNMEASURED
    w.angle          = 90.0 * kDeg; // measures leftward travel
    w.reversed       = false;
    w.gear_ratio     = 1.0;
    w.travel_scale   = 1.0;
    return w;
}

// Three-wheel setup only: two forward wheels either side and the sideways
// wheel. PLACEHOLDER geometry until the three-wheel pod exists.
inline communigatr::TrackingWheel leftWheel() {
    communigatr::TrackingWheel w = forwardWheel();
    w.encoder_port               = 0;
    w.y                          = 0.15; // PLACEHOLDER
    return w;
}

inline communigatr::TrackingWheel rightWheel() {
    communigatr::TrackingWheel w = forwardWheel();
    w.encoder_port               = 2;
    w.y                          = -0.15; // PLACEHOLDER
    return w;
}

// Robot outline about the origin, for planning and the viewer. PLACEHOLDER.
constexpr investigatr::Footprint kFootprint{0.23, 0.23, 0.23, 0.23};

// Pi IMU bias calibration settings; zeros keep the Pi defaults (2 s still
// window). Unused with the VEX IMU, which the Brain calibrates itself.
inline communigatr::ImuCalibrationSettings imuCalibration() {
    return communigatr::ImuCalibrationSettings{};
}

inline communigatr::RobotProfile profile() {
    communigatr::RobotProfile p;
    p.footprint   = kFootprint;
    p.calibration = imuCalibration();
    switch (kSetup) {
    case Setup::kTwoWheelVexImu:
        p.topology       = communigatr::LocalizationTopology::kTwoWheelImu;
        p.wheels         = {forwardWheel(), sidewaysWheel()};
        p.imu_source     = communigatr::ImuSource::kBrainVex;
        p.vex_smart_port = kVexImuPort;
        break;
    case Setup::kTwoWheelPicoImu:
        p.topology   = communigatr::LocalizationTopology::kTwoWheelImu;
        p.wheels     = {forwardWheel(), sidewaysWheel()};
        p.imu_source = communigatr::ImuSource::kPico;
        p.imu_port   = 0;
        break;
    case Setup::kThreeWheelPicoImu:
        p.topology   = communigatr::LocalizationTopology::kThreeWheel;
        p.wheels     = {leftWheel(), rightWheel(), sidewaysWheel()};
        p.imu_source = communigatr::ImuSource::kPico;
        p.imu_port   = 0;
        break;
    }
    return p;
}

constexpr bool usesVexImu() {
    return kSetup == Setup::kTwoWheelVexImu;
}

// ---------------------------------------------------------------------------
// Starting field pose, applied once when a program starts. Meters, degrees.
// ---------------------------------------------------------------------------
constexpr double kStartX              = 0.6; // PLACEHOLDER, inside the field for planning tests
constexpr double kStartY              = 0.6; // PLACEHOLDER
constexpr double kStartHeadingDegrees = 0.0;
constexpr investigatr::Pose kStartPose{kStartX, kStartY, kStartHeadingDegrees * kDeg};

static_assert(kLinkPort >= 1 && kLinkPort <= 21, "Smart Port 1..21");
static_assert(kVexImuPort >= 1 && kVexImuPort <= 21, "Smart Port 1..21");
static_assert(kUseUsb || !usesVexImu() || kVexImuPort != kLinkPort,
              "VEX IMU and RS-485 link on one Smart Port");

} // namespace gatr2_robot
