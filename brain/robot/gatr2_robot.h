// gatr2_robot.h
// The robot's localization description, shared by every Brain program for
// this robot. Edit here, rebuild and upload; the Brain sends it to the Pi as
// the robot profile when it connects, so no Pi XML changes are needed for
// geometry, encoders, IMU source or footprint.
//
// UNMEASURED: a guess that still needs measuring.
// PLACEHOLDER: a value that depends on how the robot is built.
// Robot frame: +x forward, +y left, origin at the center of the driven-wheel
// rectangle. Field frame: +x right, +y up on the field diagram, heading CCW
// from +x.

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
//   kTwoWheelVexImu    forward wheel on port 1, sideways wheel on port 0, VEX IMU
//   kTwoWheelPicoImu   same wheels, the Pico IMU (external BNO08X or ASM330)
//   kThreeWheelPicoImu three wheels fused with the Pico IMU
// ---------------------------------------------------------------------------
enum class Setup : uint8_t { kTwoWheelVexImu, kTwoWheelPicoImu, kThreeWheelPicoImu };
constexpr Setup kSetup = Setup::kTwoWheelVexImu;

// VEX IMU, read by the Brain (kTwoWheelVexImu only), on Smart Port 20.
// Mount it flat and right side up; there is no heading invert setting.
constexpr uint8_t kVexImuPort = 20;

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
// Approximate 48 mm tracking-wheel diameter; refine travel_scale after testing.
constexpr double kTrackingRadius = 0.024;
// Encoder counts per encoder shaft turn, every edge counted:
// 4 x the encoder's pulses per rev (AS5047P default 1000 PPR = 4000).
constexpr uint32_t kCountsPerRev = 4000;

// Per wheel, measured from the robot origin to where the wheel touches the
// floor: x meters forward (negative behind), y meters left (negative right).
// Toggle reversed if the locaGATR Wheels page reads negative when you
// push the robot forward (forward wheel) or left (sideways wheel).
// gear_ratio: encoder turns per wheel turn, 1.0 with the encoder on the
// wheel's axle.
inline communigatr::TrackingWheel forwardWheel() {
    communigatr::TrackingWheel w;
    w.encoder_port   = 1;
    w.radius         = kTrackingRadius;
    w.counts_per_rev = kCountsPerRev;
    w.x              = 0.0;   // Along-wheel position does not affect planar odometry.
    w.y              = -0.1115; // At the right tread centerline: -22.3 / 2 cm.
    w.angle          = 0.0;   // measures forward travel
    w.reversed       = true;  // Inverted so a forward push gives positive travel.
    w.gear_ratio     = 1.0;   // PLACEHOLDER
    w.travel_scale   = 1.0;   // from the locaGATR calibration, 1.0 until then
    return w;
}

inline communigatr::TrackingWheel sidewaysWheel() {
    communigatr::TrackingWheel w;
    w.encoder_port   = 0;
    w.radius         = kTrackingRadius;
    w.counts_per_rev = kCountsPerRev;
    w.x              = 0.011;       // Approx. 11 cm behind front axle: (24.2 / 2 - 11) cm.
    w.y              = 0.0;         // Along-wheel position does not affect planar odometry.
    w.angle          = 90.0 * kDeg; // measures leftward travel
    w.reversed       = true;        // Inverted so a leftward push gives positive travel.
    w.gear_ratio     = 1.0;         // PLACEHOLDER
    w.travel_scale   = 1.0;         // from the locaGATR calibration, 1.0 until then
    return w;
}

// Three-wheel setup only: two forward wheels either side and the sideways
// wheel. PLACEHOLDER geometry until the three-wheel pod exists.
inline communigatr::TrackingWheel leftWheel() {
    communigatr::TrackingWheel w = forwardWheel();
    w.encoder_port               = 1;
    w.y                          = 0.15; // PLACEHOLDER
    return w;
}

inline communigatr::TrackingWheel rightWheel() {
    communigatr::TrackingWheel w = forwardWheel();
    w.encoder_port               = 2;
    w.y                          = -0.15; // PLACEHOLDER
    return w;
}

// Approximate 33.7 cm long by 29.3 cm wide mecanum body. Front/back overhangs
// are roughly equal; equal left/right overhangs are assumed for now.
// Meters from the drivetrain origin to {front, back, left, right}, including
// protrusions. Refine each extent if needed before close obstacle passes.
// This controls planning clearance and the viewer.
constexpr investigatr::Footprint kFootprint{0.1685, 0.1685, 0.1465, 0.1465};

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
// Starting field pose, applied once when a program starts: put the robot
// here before starting it. x, y meters from the field origin (the inside
// bottom left corner of the field diagram); heading degrees CCW from +x
// (0 faces right on the diagram).
// Direct floor test: call the initial robot center (0, 0), facing +x.
// This does not move the configured field: (0, 0) is its corner, so avoiding
// moves from this pose fail the footprint/boundary check. Set a real start
// inside the field before testing avoidance. UP explicitly places here again.
// ---------------------------------------------------------------------------
constexpr double kStartX              = 0.0;
constexpr double kStartY              = 0.0;
constexpr double kStartHeadingDegrees = 0.0;
constexpr investigatr::Pose kStartPose{kStartX, kStartY, kStartHeadingDegrees * kDeg};

static_assert(kLinkPort >= 1 && kLinkPort <= 21, "Smart Port 1..21");
static_assert(kVexImuPort >= 1 && kVexImuPort <= 21, "Smart Port 1..21");
static_assert(kVexImuMountYawDeg >= -360.0 && kVexImuMountYawDeg <= 360.0, "degrees");
static_assert(kUseUsb || !usesVexImu() || kVexImuPort != kLinkPort,
              "VEX IMU and RS-485 link on one Smart Port");

} // namespace gatr2_robot
