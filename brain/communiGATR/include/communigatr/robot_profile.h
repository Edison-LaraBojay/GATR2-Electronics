// robot_profile.h
// Brain robot profile in SI units and its wire document. The Brain owns the
// localization geometry: tracking wheels on naviGATR encoder ports, the IMU
// source, the footprint and camera mounts. The Pi applies the document; it
// never receives XML or paths. Robot frame: +x forward, +y left, origin the
// reported robot point.

#pragma once
#include <cstdint>
#include <vector>

#include "common/link_documents.h"
#include "investigatr/geometry.h"
#include "investigatr/motion_model.h"

namespace communigatr
{

using investigatr::Meters;
using investigatr::Radians;

enum class LocalizationTopology : uint8_t {
    kTwoWheelImu,        // two wheels in independent directions, IMU heading
    kTwoForwardWheelImu, // two forward wheels, IMU heading, no sideways travel
    kThreeWheel,         // three wheels, optional independent Pico IMU fused
};

enum class ImuSource : uint8_t {
    kNone,     // three wheels only
    kPico,     // Pico IMU port imu_port; the chip is fixed by the Pico build
    kBrainVex, // VEX IMU on vex_smart_port, read by the Brain; bench timing
};

// Three kinds of value, each applied once on the Pi: encoder (port,
// counts_per_rev, reversed, gear_ratio), geometry (radius, x, y, angle) and
// the empirical travel_scale. A measured travel test observes only the
// product of radius, gearing and scale; keep radius and gearing physical and
// put the measured correction in travel_scale.
struct TrackingWheel {
    uint8_t  encoder_port   = 0; // naviGATR encoder port 0..2
    Meters   radius         = 0;
    uint32_t counts_per_rev = 0;     // encoder shaft
    Meters   x              = 0;     // contact point, robot frame
    Meters   y              = 0;
    Radians  angle          = 0;     // measurement direction, CCW from +x
    bool     reversed       = false; // encoder polarity: positive counts travel against angle
    double   gear_ratio     = 1.0;   // encoder revolutions per wheel revolution, 0.1..10
    double   travel_scale   = 1.0;   // measured distance correction, 0.9..1.1; 1.0 uncalibrated
};

// Pi IMU bias calibration settings; 0 keeps the Pi default.
struct ImuCalibrationSettings {
    investigatr::Seconds          window       = 0; // stationary window, 0.5..20 s
    investigatr::RadiansPerSecond still_rate   = 0; // gyro rate still counts as still
    Meters                        still_travel = 0; // per wheel over the window
};

struct CameraMount {
    uint8_t slot  = 0; // Pi camera slot
    Meters  x     = 0; // camera frame origin, robot frame
    Meters  y     = 0;
    Meters  z     = 0;
    Radians roll  = 0;
    Radians pitch = 0;
    Radians yaw   = 0;
};

struct RobotProfile {
    LocalizationTopology       topology = LocalizationTopology::kTwoWheelImu;
    std::vector<TrackingWheel> wheels;
    ImuSource                  imu_source     = ImuSource::kNone;
    uint8_t                    imu_port       = 0;     // Pico IMU port with kPico
    uint8_t                    vex_smart_port = 0;     // V5 port 1..21 with kBrainVex
    bool                       imu_invert     = false; // kPico: yaw sign flipped
    ImuCalibrationSettings     calibration;
    investigatr::Footprint     footprint;
    std::vector<CameraMount>   cameras;
};

// Converts to wire units (micrometers, millidegrees) and runs the shared
// validateRobotProfile. False on the first problem, with reason a
// gatr2::ProfileReason and detail the wheel or camera index. Values that do
// not fit the wire report the reason of their group.
bool toProfileDoc(const RobotProfile& in, gatr2::RobotProfileDoc& out, uint8_t& reason,
                  uint8_t& detail);

// Encoded profile for ClientConfig::profile. A document that failed the
// Brain side check keeps its reason; the client reports it and never sends.
struct ProfileDocument {
    uint16_t len    = 0; // 0 = no document
    uint8_t  reason = gatr2::kProfileReasonNone;
    uint8_t  detail = 0;
    uint8_t  bytes[gatr2::kProfileMaxLen] = {};

    bool configured() const { return len != 0 || reason != gatr2::kProfileReasonNone; }
};

ProfileDocument makeProfileDocument(const RobotProfile& in);

// profile_id: crc32 of the document, 0 when there is none.
uint32_t profileId(const ProfileDocument& doc);

// Short name of a gatr2::ProfileReason for status displays.
const char* profileReasonName(uint8_t reason);

} // namespace communigatr
