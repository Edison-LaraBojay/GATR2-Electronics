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
// kSendProfile false: the programs send no robot profile, for an older
// XML-configured Pi config (bench_vex_imu*.xml, parallel_wheels*.xml), whose
// XML then owns the geometry; wheel calibration apply is not available.
// ---------------------------------------------------------------------------
constexpr bool    kSendProfile  = true;
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

// VEX IMU mounting for the viewer's roll and pitch: the direction its +x
// axis points in the robot frame, CCW from forward, degrees (0, 90, 180 or
// 270; the IMU lies flat). PLACEHOLDER. Heading and localization do not use
// it. Check with the bench test in docs/brain_setup.md.
constexpr double kVexImuMountYawDeg = 0.0;

// TELEMETRY to the Pi for the viewer and recordings: attitude from the VEX
// IMU, and from operaGATR the movement and wheel targets. Display only; the
// Pi never uses it for localization. false sends nothing. Each TELEMETRY
// exchange can push the next state poll back by about 10 to 15 ms, one
// exchange plus the 5 ms gap (docs/communigatr.md).
constexpr bool     kSendTelemetry     = true;
constexpr uint32_t kTelemetryPeriodMs = 100;

// ---------------------------------------------------------------------------
// Tracking wheels (not the driven wheels). Port = naviGATR encoder port:
// 0 J2, 1 J3, 2 J4 on HAT v2.
//
// Three kinds of value, applied once each on the Pi:
//   encoder    counts_per_rev of the encoder shaft, reversed (encoder
//              polarity), gear_ratio (encoder turns per wheel turn)
//   geometry   radius, x, y (contact point), angle (measuring direction)
//   measured   travel_scale from the wheel calibration in locaGATR;
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
// PLACEHOLDER: open floor west of the center goal. With this footprint and
// the testing clearance, the corner pockets between the wall and the two
// nearest goals (around (0.6, 0.6) and the other corners) are closed to
// avoiding moves: a robot started there only gets direct moves out.
// ---------------------------------------------------------------------------
constexpr double kStartX              = 1.2; // PLACEHOLDER
constexpr double kStartY              = 1.8; // PLACEHOLDER
constexpr double kStartHeadingDegrees = 0.0;
constexpr investigatr::Pose kStartPose{kStartX, kStartY, kStartHeadingDegrees * kDeg};

static_assert(kLinkPort >= 1 && kLinkPort <= 21, "Smart Port 1..21");
static_assert(kVexImuPort >= 1 && kVexImuPort <= 21, "Smart Port 1..21");
static_assert(kVexImuMountYawDeg >= -360.0 && kVexImuMountYawDeg <= 360.0, "degrees");
static_assert(kUseUsb || !usesVexImu() || kVexImuPort != kLinkPort,
              "VEX IMU and RS-485 link on one Smart Port");

} // namespace gatr2_robot
