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
    case LocalizationTopology::kTwoWheelImu: return translagatr::kTopologyTwoWheelImu;
    case LocalizationTopology::kTwoForwardWheelImu: return translagatr::kTopologyTwoForwardWheelImu;
    case LocalizationTopology::kThreeWheel: return translagatr::kTopologyThreeWheel;
    }
    return 0;
}

uint8_t imuCode(ImuSource s) {
    switch (s) {
    case ImuSource::kNone: return translagatr::kImuSourceNone;
    case ImuSource::kPico: return translagatr::kImuSourcePico;
    case ImuSource::kBrainVex: return translagatr::kImuSourceBrainVex;
    }
    return 0xFF;
}

// Wire units only, no semantic check.
bool convert(const RobotProfile& in, translagatr::RobotProfileDoc& out, uint8_t& reason,
             uint8_t& detail) {
    out    = translagatr::RobotProfileDoc{};
    reason = translagatr::kProfileReasonNone;
    detail = 0;

    out.topology = topologyCode(in.topology);
    if (in.wheels.size() > translagatr::kProfileMaxWheels) {
        return fail(reason, detail, translagatr::kProfileReasonWheelCount);
    }
    if (in.cameras.size() > translagatr::kProfileMaxCameras) {
        return fail(reason, detail, translagatr::kProfileReasonCamera, translagatr::kProfileMaxCameras);
    }
    out.wheel_count  = static_cast<uint8_t>(in.wheels.size());
    out.camera_count = static_cast<uint8_t>(in.cameras.size());

    for (uint8_t i = 0; i < out.wheel_count; ++i) {
        const TrackingWheel& w = in.wheels[i];
        translagatr::ProfileWheel& o = out.wheels[i];
        o.encoder_port         = w.encoder_port;
        o.flags                = w.reversed ? uint8_t{translagatr::kWheelReversed} : uint8_t{0};
        o.counts_per_rev       = w.counts_per_rev;
        if (!toU32(w.radius * kUmPerMeter, o.radius_um) || !microns(w.x, o.x_um) ||
            !microns(w.y, o.y_um) || !millidegrees(w.angle, o.angle_mdeg) ||
            !toU32(w.gear_ratio * translagatr::kUnitMicro, o.gear_micro) ||
            !toU32(w.travel_scale * translagatr::kUnitMicro, o.travel_scale_ppm)) {
            return fail(reason, detail, translagatr::kProfileReasonWheelGeometry, i);
        }
    }

    const ImuCalibrationSettings& c = in.calibration;
    if (!toU16(c.window * 1000.0, out.calibration_window_ms) ||
        !toU16(c.still_rate * kCdegPerRad, out.still_rate_cdps) ||
        !toU16(c.still_travel * kUmPerMeter, out.still_travel_um)) {
        return fail(reason, detail, translagatr::kProfileReasonCalibration);
    }

    out.imu_source     = imuCode(in.imu_source);
    out.imu_port       = in.imu_port;
    out.vex_smart_port = in.vex_smart_port;
    out.imu_flags      = in.imu_invert ? uint8_t{translagatr::kImuInvert} : uint8_t{0};

    const investigatr::Footprint& f = in.footprint;
    if (!microns(f.front, out.footprint_front_um) || !microns(f.back, out.footprint_back_um) ||
        !microns(f.left, out.footprint_left_um) || !microns(f.right, out.footprint_right_um)) {
        return fail(reason, detail, translagatr::kProfileReasonFootprint);
    }

    for (uint8_t i = 0; i < out.camera_count; ++i) {
        const CameraMount&    c = in.cameras[i];
        translagatr::ProfileCamera& o = out.cameras[i];
        o.slot                  = c.slot;
        if (!microns(c.x, o.x_um) || !microns(c.y, o.y_um) || !microns(c.z, o.z_um) ||
            !millidegrees(c.roll, o.roll_mdeg) || !millidegrees(c.pitch, o.pitch_mdeg) ||
            !millidegrees(c.yaw, o.yaw_mdeg)) {
            return fail(reason, detail, translagatr::kProfileReasonCamera, i);
        }
    }
    return true;
}

} // namespace

bool toProfileDoc(const RobotProfile& in, translagatr::RobotProfileDoc& out, uint8_t& reason,
                  uint8_t& detail) {
    return convert(in, out, reason, detail) && translagatr::validateRobotProfile(out, reason, detail);
}

ProfileDocument makeProfileDocument(const RobotProfile& in) {
    ProfileDocument        doc;
    translagatr::RobotProfileDoc wire;
    if (!convert(in, wire, doc.reason, doc.detail)) {
        return doc;
    }
    // Kept when the shared check fails, for inspection; the client never
    // sends a document with a reason.
    doc.len = translagatr::encodeRobotProfile(wire, doc.bytes, sizeof(doc.bytes));
    translagatr::validateRobotProfile(wire, doc.reason, doc.detail);
    return doc;
}

uint32_t profileId(const ProfileDocument& doc) {
    return doc.len == 0 ? 0 : translagatr::crc32(doc.bytes, doc.len);
}

const char* profileReasonName(uint8_t reason) {
    switch (reason) {
    case translagatr::kProfileReasonNone: return "none";
    case translagatr::kProfileReasonFormat: return "format";
    case translagatr::kProfileReasonTopology: return "topology";
    case translagatr::kProfileReasonWheelCount: return "wheel count";
    case translagatr::kProfileReasonEncoderPort: return "encoder port";
    case translagatr::kProfileReasonWheelGeometry: return "wheel geometry";
    case translagatr::kProfileReasonObservability: return "observability";
    case translagatr::kProfileReasonImuSource: return "IMU source";
    case translagatr::kProfileReasonImuPort: return "IMU port";
    case translagatr::kProfileReasonImuCombination: return "IMU combination";
    case translagatr::kProfileReasonCamera: return "camera";
    case translagatr::kProfileReasonFootprint: return "footprint";
    case translagatr::kProfileReasonBuild: return "Pi build";
    case translagatr::kProfileReasonNotAccepted: return "not accepted by this Pi";
    case translagatr::kProfileReasonCalibration: return "calibration settings";
    }
    return "?";
}

} // namespace communigatr
