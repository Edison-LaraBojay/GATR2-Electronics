// robot_profile.cpp

#include "communigatr/robot_profile.h"

#include <cmath>
#include <cstdint>

namespace communigatr
{

namespace
{

constexpr double  kUmPerMeter = 1e6;
constexpr double  kMdegPerRad = 180000.0 / investigatr::kPi;
constexpr double  kCdegPerRad = 18000.0 / investigatr::kPi;
constexpr int32_t kHalfTurnMdeg = 180000;

// Nearest integer, false when not finite or outside the type.
bool toI32(double value, int32_t& out) {
    const double r = std::round(value);
    if (!std::isfinite(r) || r < INT32_MIN || r > INT32_MAX) {
        return false;
    }
    out = static_cast<int32_t>(r);
    return true;
}

bool toU32(double value, uint32_t& out) {
    const double r = std::round(value);
    if (!std::isfinite(r) || r < 0 || r > UINT32_MAX) {
        return false;
    }
    out = static_cast<uint32_t>(r);
    return true;
}

bool toU16(double value, uint16_t& out) {
    uint32_t wide = 0;
    if (!toU32(value, wide) || wide > UINT16_MAX) {
        return false;
    }
    out = static_cast<uint16_t>(wide);
    return true;
}

bool microns(Meters m, int32_t& out) { return toI32(m * kUmPerMeter, out); }

// Wrapped to (-180000, 180000].
bool millidegrees(Radians a, int32_t& out) {
    if (!std::isfinite(a) || !toI32(investigatr::wrapAngle(a) * kMdegPerRad, out)) {
        return false;
    }
    if (out == -kHalfTurnMdeg) {
        out = kHalfTurnMdeg;
    }
    return true;
}

bool fail(uint8_t& reason, uint8_t& detail, uint8_t why, uint8_t index = 0) {
    reason = why;
    detail = index;
    return false;
}

uint8_t topologyCode(LocalizationTopology t) {
    switch (t) {
    case LocalizationTopology::kTwoWheelImu: return gatr2::kTopologyTwoWheelImu;
    case LocalizationTopology::kTwoForwardWheelImu: return gatr2::kTopologyTwoForwardWheelImu;
    case LocalizationTopology::kThreeWheel: return gatr2::kTopologyThreeWheel;
    }
    return 0;
}

uint8_t imuCode(ImuSource s) {
    switch (s) {
    case ImuSource::kNone: return gatr2::kImuSourceNone;
    case ImuSource::kPico: return gatr2::kImuSourcePico;
    case ImuSource::kBrainVex: return gatr2::kImuSourceBrainVex;
    }
    return 0xFF;
}

// Wire units only, no semantic check.
bool convert(const RobotProfile& in, gatr2::RobotProfileDoc& out, uint8_t& reason,
             uint8_t& detail) {
    out    = gatr2::RobotProfileDoc{};
    reason = gatr2::kProfileReasonNone;
    detail = 0;

    out.topology = topologyCode(in.topology);
    if (in.wheels.size() > gatr2::kProfileMaxWheels) {
        return fail(reason, detail, gatr2::kProfileReasonWheelCount);
    }
    if (in.cameras.size() > gatr2::kProfileMaxCameras) {
        return fail(reason, detail, gatr2::kProfileReasonCamera, gatr2::kProfileMaxCameras);
    }
    out.wheel_count  = static_cast<uint8_t>(in.wheels.size());
    out.camera_count = static_cast<uint8_t>(in.cameras.size());

    for (uint8_t i = 0; i < out.wheel_count; ++i) {
        const TrackingWheel& w = in.wheels[i];
        gatr2::ProfileWheel& o = out.wheels[i];
        o.encoder_port         = w.encoder_port;
        o.flags                = w.reversed ? uint8_t{gatr2::kWheelReversed} : uint8_t{0};
        o.counts_per_rev       = w.counts_per_rev;
        if (!toU32(w.radius * kUmPerMeter, o.radius_um) || !microns(w.x, o.x_um) ||
            !microns(w.y, o.y_um) || !millidegrees(w.angle, o.angle_mdeg) ||
            !toU32(w.gear_ratio * gatr2::kUnitMicro, o.gear_micro) ||
            !toU32(w.travel_scale * gatr2::kUnitMicro, o.travel_scale_ppm)) {
            return fail(reason, detail, gatr2::kProfileReasonWheelGeometry, i);
        }
    }

    const ImuCalibrationSettings& c = in.calibration;
    if (!toU16(c.window * 1000.0, out.calibration_window_ms) ||
        !toU16(c.still_rate * kCdegPerRad, out.still_rate_cdps) ||
        !toU16(c.still_travel * kUmPerMeter, out.still_travel_um)) {
        return fail(reason, detail, gatr2::kProfileReasonCalibration);
    }

    out.imu_source     = imuCode(in.imu_source);
    out.imu_port       = in.imu_port;
    out.vex_smart_port = in.vex_smart_port;
    out.imu_flags      = in.imu_invert ? uint8_t{gatr2::kImuInvert} : uint8_t{0};

    const investigatr::Footprint& f = in.footprint;
    if (!microns(f.front, out.footprint_front_um) || !microns(f.back, out.footprint_back_um) ||
        !microns(f.left, out.footprint_left_um) || !microns(f.right, out.footprint_right_um)) {
        return fail(reason, detail, gatr2::kProfileReasonFootprint);
    }

    for (uint8_t i = 0; i < out.camera_count; ++i) {
        const CameraMount&    c = in.cameras[i];
        gatr2::ProfileCamera& o = out.cameras[i];
        o.slot                  = c.slot;
        if (!microns(c.x, o.x_um) || !microns(c.y, o.y_um) || !microns(c.z, o.z_um) ||
            !millidegrees(c.roll, o.roll_mdeg) || !millidegrees(c.pitch, o.pitch_mdeg) ||
            !millidegrees(c.yaw, o.yaw_mdeg)) {
            return fail(reason, detail, gatr2::kProfileReasonCamera, i);
        }
    }
    return true;
}

} // namespace

bool toProfileDoc(const RobotProfile& in, gatr2::RobotProfileDoc& out, uint8_t& reason,
                  uint8_t& detail) {
    return convert(in, out, reason, detail) && gatr2::validateRobotProfile(out, reason, detail);
}

ProfileDocument makeProfileDocument(const RobotProfile& in) {
    ProfileDocument        doc;
    gatr2::RobotProfileDoc wire;
    if (!convert(in, wire, doc.reason, doc.detail)) {
        return doc;
    }
    // Kept when the shared check fails, for inspection; the client never
    // sends a document with a reason.
    doc.len = gatr2::encodeRobotProfile(wire, doc.bytes, sizeof(doc.bytes));
    gatr2::validateRobotProfile(wire, doc.reason, doc.detail);
    return doc;
}

uint32_t profileId(const ProfileDocument& doc) {
    return doc.len == 0 ? 0 : gatr2::crc32(doc.bytes, doc.len);
}

const char* profileReasonName(uint8_t reason) {
    switch (reason) {
    case gatr2::kProfileReasonNone: return "none";
    case gatr2::kProfileReasonFormat: return "format";
    case gatr2::kProfileReasonTopology: return "topology";
    case gatr2::kProfileReasonWheelCount: return "wheel count";
    case gatr2::kProfileReasonEncoderPort: return "encoder port";
    case gatr2::kProfileReasonWheelGeometry: return "wheel geometry";
    case gatr2::kProfileReasonObservability: return "observability";
    case gatr2::kProfileReasonImuSource: return "IMU source";
    case gatr2::kProfileReasonImuPort: return "IMU port";
    case gatr2::kProfileReasonImuCombination: return "IMU combination";
    case gatr2::kProfileReasonCamera: return "camera";
    case gatr2::kProfileReasonFootprint: return "footprint";
    case gatr2::kProfileReasonBuild: return "Pi build";
    case gatr2::kProfileReasonNotAccepted: return "not accepted by this Pi";
    case gatr2::kProfileReasonCalibration: return "calibration settings";
    }
    return "?";
}

} // namespace communigatr
