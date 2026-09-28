// link_documents.cpp

#include "link_documents.h"

#include <math.h>
#include <string.h>

namespace gatr2
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// Profile ranges shared by the Brain and the Pi.
constexpr uint32_t kMaxCountsPerRev = 1000000;
constexpr uint32_t kMinRadiusUm     = 1000;
constexpr uint32_t kMaxRadiusUm     = 200000;
constexpr int32_t  kMaxOffsetUm     = 1000000;
constexpr int32_t  kMaxFootprintUm  = 2000000;
constexpr int32_t  kMaxCameraUm     = 2000000;
constexpr int32_t  kMaxAngleMdeg    = 360000;
constexpr uint32_t kMinGearMicro    = 100000;
constexpr uint32_t kMaxGearMicro    = 10000000;
constexpr uint32_t kMinScalePpm     = 900000;
constexpr uint32_t kMaxScalePpm     = 1100000;

bool zeroOr(uint16_t v, uint16_t lo, uint16_t hi) {
    return v == 0 || (v >= lo && v <= hi);
}

// Same limits as the Pi wheel models: the normal equations of the solve need
// a determinant of at least 1e-6 (two wheels) or 1e-9 (three wheels).
constexpr double kTwoWheelMinDet   = 1e-3;
constexpr double kThreeWheelMinDet = 3.2e-5;

void wr16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void wr32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

uint16_t rd16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t rd32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

int32_t rdi32(const uint8_t* p) {
    return static_cast<int32_t>(rd32(p));
}

int16_t rdi16(const uint8_t* p) {
    return static_cast<int16_t>(rd16(p));
}

bool headingValid(int32_t cdeg) {
    return cdeg > -18000 && cdeg <= 18000;
}

bool within(int32_t v, int32_t limit) {
    return v >= -limit && v <= limit;
}

double angleRad(int32_t mdeg) {
    return static_cast<double>(mdeg) * kPi / 180000.0;
}

bool fail(uint8_t& reason, uint8_t& detail, uint8_t r, uint8_t d = 0) {
    reason = r;
    detail = d;
    return false;
}

uint8_t expectedWheels(uint8_t topology) {
    switch (topology) {
    case kTopologyTwoWheelImu:
    case kTopologyTwoForwardWheelImu: return 2;
    case kTopologyThreeWheel: return 3;
    default: return 0;
    }
}

bool observable(const RobotProfileDoc& p) {
    double ux[kProfileMaxWheels] = {};
    double uy[kProfileMaxWheels] = {};
    double k[kProfileMaxWheels]  = {};
    for (uint8_t i = 0; i < p.wheel_count; ++i) {
        const ProfileWheel& w = p.wheels[i];
        const double        a = angleRad(w.angle_mdeg);
        ux[i]                 = cos(a);
        uy[i]                 = sin(a);
        k[i] = (static_cast<double>(w.x_um) * uy[i] - static_cast<double>(w.y_um) * ux[i]) * 1e-6;
    }
    switch (p.topology) {
    case kTopologyTwoWheelImu: return fabs(ux[0] * uy[1] - uy[0] * ux[1]) >= kTwoWheelMinDet;
    case kTopologyTwoForwardWheelImu:
        for (uint8_t i = 0; i < p.wheel_count; ++i) {
            const int32_t a = p.wheels[i].angle_mdeg;
            if (a != 0 && a != 180000 && a != -180000) {
                return false;
            }
        }
        return true;
    case kTopologyThreeWheel: {
        const double det = ux[0] * (uy[1] * k[2] - k[1] * uy[2]) -
                           uy[0] * (ux[1] * k[2] - k[1] * ux[2]) +
                           k[0] * (ux[1] * uy[2] - uy[1] * ux[2]);
        return fabs(det) >= kThreeWheelMinDet;
    }
    default: return false;
    }
}

} // namespace

uint32_t crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------------------
// Robot profile
// ---------------------------------------------------------------------------

uint16_t robotProfileLen(const RobotProfileDoc& in) {
    if (in.wheel_count > kProfileMaxWheels || in.camera_count > kProfileMaxCameras) {
        return 0;
    }
    return static_cast<uint16_t>(kProfileHeaderLen + in.wheel_count * kProfileWheelLen +
                                 in.camera_count * kProfileCameraLen);
}

uint16_t encodeRobotProfile(const RobotProfileDoc& in, uint8_t* buf, uint16_t cap) {
    const uint16_t len = robotProfileLen(in);
    if (len == 0 || len > cap) {
        return 0;
    }
    memset(buf, 0, len);
    buf[0] = in.format;
    buf[1] = in.topology;
    buf[2] = in.wheel_count;
    buf[3] = in.camera_count;
    buf[4] = in.imu_source;
    buf[5] = in.imu_port;
    buf[6] = in.vex_smart_port;
    buf[7] = in.imu_flags;
    wr32(buf + 8, static_cast<uint32_t>(in.footprint_front_um));
    wr32(buf + 12, static_cast<uint32_t>(in.footprint_back_um));
    wr32(buf + 16, static_cast<uint32_t>(in.footprint_left_um));
    wr32(buf + 20, static_cast<uint32_t>(in.footprint_right_um));
    wr16(buf + 24, in.calibration_window_ms);
    wr16(buf + 26, in.still_rate_cdps);
    wr16(buf + 28, in.still_travel_um);

    uint8_t* at = buf + kProfileHeaderLen;
    for (uint8_t i = 0; i < in.wheel_count; ++i) {
        const ProfileWheel& w = in.wheels[i];
        at[0]                 = w.encoder_port;
        at[1]                 = w.flags;
        wr32(at + 4, w.counts_per_rev);
        wr32(at + 8, w.radius_um);
        wr32(at + 12, static_cast<uint32_t>(w.x_um));
        wr32(at + 16, static_cast<uint32_t>(w.y_um));
        wr32(at + 20, static_cast<uint32_t>(w.angle_mdeg));
        wr32(at + 24, w.gear_micro);
        wr32(at + 28, w.travel_scale_ppm);
        at += kProfileWheelLen;
    }
    for (uint8_t i = 0; i < in.camera_count; ++i) {
        const ProfileCamera& c = in.cameras[i];
        at[0]                  = c.slot;
        wr32(at + 4, static_cast<uint32_t>(c.x_um));
        wr32(at + 8, static_cast<uint32_t>(c.y_um));
        wr32(at + 12, static_cast<uint32_t>(c.z_um));
        wr32(at + 16, static_cast<uint32_t>(c.roll_mdeg));
        wr32(at + 20, static_cast<uint32_t>(c.pitch_mdeg));
        wr32(at + 24, static_cast<uint32_t>(c.yaw_mdeg));
        at += kProfileCameraLen;
    }
    return len;
}

bool decodeRobotProfile(const uint8_t* buf, uint16_t len, RobotProfileDoc& out) {
    if (len < kProfileHeaderLen || buf[0] != kProfileFormat) {
        return false;
    }
    RobotProfileDoc p;
    p.format         = buf[0];
    p.topology       = buf[1];
    p.wheel_count    = buf[2];
    p.camera_count   = buf[3];
    p.imu_source     = buf[4];
    p.imu_port       = buf[5];
    p.vex_smart_port = buf[6];
    p.imu_flags      = buf[7];
    const uint16_t want = robotProfileLen(p);
    if (want == 0 || want != len) {
        return false;
    }
    p.footprint_front_um = rdi32(buf + 8);
    p.footprint_back_um  = rdi32(buf + 12);
    p.footprint_left_um  = rdi32(buf + 16);
    p.footprint_right_um = rdi32(buf + 20);
    if (rd16(buf + 30) != 0) {
        return false;
    }
    p.calibration_window_ms = rd16(buf + 24);
    p.still_rate_cdps       = rd16(buf + 26);
    p.still_travel_um       = rd16(buf + 28);

    const uint8_t* at = buf + kProfileHeaderLen;
    for (uint8_t i = 0; i < p.wheel_count; ++i) {
        if (rd16(at + 2) != 0) {
            return false;
        }
        ProfileWheel& w  = p.wheels[i];
        w.encoder_port   = at[0];
        w.flags          = at[1];
        w.counts_per_rev = rd32(at + 4);
        w.radius_um      = rd32(at + 8);
        w.x_um           = rdi32(at + 12);
        w.y_um           = rdi32(at + 16);
        w.angle_mdeg     = rdi32(at + 20);
        w.gear_micro       = rd32(at + 24);
        w.travel_scale_ppm = rd32(at + 28);
        at += kProfileWheelLen;
    }
    for (uint8_t i = 0; i < p.camera_count; ++i) {
        if (at[1] != 0 || at[2] != 0 || at[3] != 0) {
            return false;
        }
        ProfileCamera& c = p.cameras[i];
        c.slot           = at[0];
        c.x_um           = rdi32(at + 4);
        c.y_um           = rdi32(at + 8);
        c.z_um           = rdi32(at + 12);
        c.roll_mdeg      = rdi32(at + 16);
        c.pitch_mdeg     = rdi32(at + 20);
        c.yaw_mdeg       = rdi32(at + 24);
        at += kProfileCameraLen;
    }
    out = p;
    return true;
}

bool validateRobotProfile(const RobotProfileDoc& p, uint8_t& reason, uint8_t& detail) {
    reason = kProfileReasonNone;
    detail = 0;
    if (p.format != kProfileFormat) {
        return fail(reason, detail, kProfileReasonFormat);
    }
    const uint8_t wheels = expectedWheels(p.topology);
    if (wheels == 0) {
        return fail(reason, detail, kProfileReasonTopology);
    }
    if (p.wheel_count != wheels) {
        return fail(reason, detail, kProfileReasonWheelCount);
    }
    if (p.camera_count > kProfileMaxCameras) {
        return fail(reason, detail, kProfileReasonCamera);
    }

    for (uint8_t i = 0; i < p.wheel_count; ++i) {
        const ProfileWheel& w = p.wheels[i];
        for (uint8_t j = 0; j < i; ++j) {
            if (p.wheels[j].encoder_port == w.encoder_port) {
                return fail(reason, detail, kProfileReasonEncoderPort, i);
            }
        }
        const bool geometry = (w.flags & ~kWheelReversed) == 0 && w.counts_per_rev > 0 &&
                              w.counts_per_rev <= kMaxCountsPerRev && w.radius_um >= kMinRadiusUm &&
                              w.radius_um <= kMaxRadiusUm && within(w.x_um, kMaxOffsetUm) &&
                              within(w.y_um, kMaxOffsetUm) && within(w.angle_mdeg, kMaxAngleMdeg) &&
                              w.gear_micro >= kMinGearMicro && w.gear_micro <= kMaxGearMicro &&
                              w.travel_scale_ppm >= kMinScalePpm &&
                              w.travel_scale_ppm <= kMaxScalePpm;
        if (!geometry) {
            return fail(reason, detail, kProfileReasonWheelGeometry, i);
        }
    }

    switch (p.imu_source) {
    case kImuSourceNone:
        if (p.imu_flags != 0) {
            return fail(reason, detail, kProfileReasonImuSource);
        }
        if (p.imu_port != 0 || p.vex_smart_port != 0) {
            return fail(reason, detail, kProfileReasonImuPort);
        }
        if (p.topology != kTopologyThreeWheel) {
            return fail(reason, detail, kProfileReasonImuCombination);
        }
        break;
    case kImuSourcePico:
        if ((p.imu_flags & ~kImuInvert) != 0) {
            return fail(reason, detail, kProfileReasonImuSource);
        }
        if (p.vex_smart_port != 0) {
            return fail(reason, detail, kProfileReasonImuPort);
        }
        break;
    case kImuSourceBrainVex:
        if (p.imu_flags != 0) {
            return fail(reason, detail, kProfileReasonImuSource);
        }
        if (p.imu_port != 0 || p.vex_smart_port < 1 || p.vex_smart_port > 21) {
            return fail(reason, detail, kProfileReasonImuPort);
        }
        if (p.topology == kTopologyThreeWheel) {
            return fail(reason, detail, kProfileReasonImuCombination);
        }
        break;
    default: return fail(reason, detail, kProfileReasonImuSource);
    }

    if (!observable(p)) {
        return fail(reason, detail, kProfileReasonObservability);
    }

    const int32_t f[4] = {p.footprint_front_um, p.footprint_back_um, p.footprint_left_um,
                          p.footprint_right_um};
    for (int32_t v : f) {
        if (v < 0 || v > kMaxFootprintUm) {
            return fail(reason, detail, kProfileReasonFootprint);
        }
    }
    if (f[0] + f[1] <= 0 || f[2] + f[3] <= 0) {
        return fail(reason, detail, kProfileReasonFootprint);
    }

    if (!zeroOr(p.calibration_window_ms, 500, 20000) || !zeroOr(p.still_rate_cdps, 10, 2000) ||
        !zeroOr(p.still_travel_um, 20, 5000)) {
        return fail(reason, detail, kProfileReasonCalibration);
    }

    for (uint8_t i = 0; i < p.camera_count; ++i) {
        const ProfileCamera& c = p.cameras[i];
        for (uint8_t j = 0; j < i; ++j) {
            if (p.cameras[j].slot == c.slot) {
                return fail(reason, detail, kProfileReasonCamera, i);
            }
        }
        const bool mount = within(c.x_um, kMaxCameraUm) && within(c.y_um, kMaxCameraUm) &&
                           within(c.z_um, kMaxCameraUm) && within(c.roll_mdeg, kMaxAngleMdeg) &&
                           within(c.pitch_mdeg, kMaxAngleMdeg) && within(c.yaw_mdeg, kMaxAngleMdeg);
        if (!mount) {
            return fail(reason, detail, kProfileReasonCamera, i);
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Field map
// ---------------------------------------------------------------------------

uint16_t fieldMapLen(uint16_t object_count) {
    if (object_count > kFieldMaxObjects) {
        return 0;
    }
    return static_cast<uint16_t>(kFieldMapHeaderLen + object_count * kFieldMapRecordLen);
}

bool encodeFieldMapHeader(const FieldMapHeader& in, uint8_t* doc, uint16_t cap) {
    const uint16_t len = fieldMapLen(in.object_count);
    if (len == 0 || cap < len) {
        return false;
    }
    memset(doc, 0, kFieldMapHeaderLen);
    doc[0] = in.format;
    doc[1] = static_cast<uint8_t>(kFieldMapRecordLen);
    wr16(doc + 2, in.revision);
    wr16(doc + 4, in.object_count);
    wr32(doc + 8, static_cast<uint32_t>(in.min_x_mm));
    wr32(doc + 12, static_cast<uint32_t>(in.min_y_mm));
    wr32(doc + 16, static_cast<uint32_t>(in.max_x_mm));
    wr32(doc + 20, static_cast<uint32_t>(in.max_y_mm));
    return true;
}

bool encodeFieldObjectRecord(const FieldObjectRecord& in, uint16_t index, uint8_t* doc,
                             uint16_t cap) {
    const uint32_t at = kFieldMapHeaderLen + static_cast<uint32_t>(index) * kFieldMapRecordLen;
    if (index >= kFieldMaxObjects || at + kFieldMapRecordLen > cap) {
        return false;
    }
    uint8_t* p = doc + at;
    memset(p, 0, kFieldMapRecordLen);
    wr16(p, in.object_id);
    p[2] = in.kind;
    p[3] = in.flags;
    wr32(p + 4, static_cast<uint32_t>(in.x_mm));
    wr32(p + 8, static_cast<uint32_t>(in.y_mm));
    wr32(p + 12, static_cast<uint32_t>(in.heading_cdeg));
    wr16(p + 16, static_cast<uint16_t>(in.box_x_mm));
    wr16(p + 18, static_cast<uint16_t>(in.box_y_mm));
    wr16(p + 20, static_cast<uint16_t>(in.box_heading_cdeg));
    wr16(p + 22, in.box_length_mm);
    wr16(p + 24, in.box_width_mm);
    return true;
}

bool decodeFieldMapHeader(const uint8_t* doc, uint16_t len, FieldMapHeader& out) {
    if (len < kFieldMapHeaderLen || doc[0] != kFieldMapFormat || doc[1] != kFieldMapRecordLen) {
        return false;
    }
    const uint16_t count = rd16(doc + 4);
    const uint16_t want  = fieldMapLen(count);
    if (want == 0 || want != len) {
        return false;
    }
    out              = FieldMapHeader{};
    out.format       = doc[0];
    out.revision     = rd16(doc + 2);
    out.object_count = count;
    out.min_x_mm     = rdi32(doc + 8);
    out.min_y_mm     = rdi32(doc + 12);
    out.max_x_mm     = rdi32(doc + 16);
    out.max_y_mm     = rdi32(doc + 20);
    return true;
}

bool decodeFieldObjectRecord(const uint8_t* doc, uint16_t len, uint16_t index,
                             FieldObjectRecord& out) {
    const uint32_t at = kFieldMapHeaderLen + static_cast<uint32_t>(index) * kFieldMapRecordLen;
    if (index >= kFieldMaxObjects || at + kFieldMapRecordLen > len) {
        return false;
    }
    const uint8_t* p     = doc + at;
    out.object_id        = rd16(p);
    out.kind             = p[2];
    out.flags            = p[3];
    out.x_mm             = rdi32(p + 4);
    out.y_mm             = rdi32(p + 8);
    out.heading_cdeg     = rdi32(p + 12);
    out.box_x_mm         = rdi16(p + 16);
    out.box_y_mm         = rdi16(p + 18);
    out.box_heading_cdeg = rdi16(p + 20);
    out.box_length_mm    = rd16(p + 22);
    out.box_width_mm     = rd16(p + 24);
    return true;
}

DocError validateFieldMap(const uint8_t* doc, uint16_t len) {
    if (len < kFieldMapHeaderLen) {
        return DocError::kLength;
    }
    if (doc[0] != kFieldMapFormat || doc[1] != kFieldMapRecordLen) {
        return DocError::kFormat;
    }
    const uint16_t count = rd16(doc + 4);
    if (count > kFieldMaxObjects) {
        return DocError::kCount;
    }
    FieldMapHeader header;
    if (!decodeFieldMapHeader(doc, len, header)) {
        return DocError::kLength;
    }
    if (rd16(doc + 6) != 0) {
        return DocError::kRange;
    }
    if (header.min_x_mm >= header.max_x_mm || header.min_y_mm >= header.max_y_mm) {
        return DocError::kBounds;
    }

    uint16_t previous = 0;
    for (uint16_t i = 0; i < count; ++i) {
        FieldObjectRecord r;
        decodeFieldObjectRecord(doc, len, i, r);
        const uint8_t* raw = doc + kFieldMapHeaderLen + i * kFieldMapRecordLen;
        if (r.object_id == 0 || r.object_id <= previous) {
            return DocError::kOrder;
        }
        previous = r.object_id;
        if (r.kind != kObjectLandmark && r.kind != kObjectFixed) {
            return DocError::kKind;
        }
        const uint8_t known = kObjectObstacle | kObjectEstimated | kObjectReference;
        if ((r.flags & ~known) != 0 || (r.kind == kObjectFixed && (r.flags & kObjectEstimated))) {
            return DocError::kFlags;
        }
        if (r.flags & kObjectObstacle) {
            if (r.box_length_mm == 0 || r.box_width_mm == 0) {
                return DocError::kBox;
            }
        } else if (r.box_x_mm != 0 || r.box_y_mm != 0 || r.box_heading_cdeg != 0 ||
                   r.box_length_mm != 0 || r.box_width_mm != 0) {
            return DocError::kBox;
        }
        if (!headingValid(r.heading_cdeg) || !headingValid(r.box_heading_cdeg) ||
            rd16(raw + 26) != 0) {
            return DocError::kRange;
        }
    }
    return DocError::kNone;
}

// ---------------------------------------------------------------------------
// Field estimate
// ---------------------------------------------------------------------------

uint16_t fieldEstimateLen(uint16_t object_count) {
    if (object_count > kFieldMaxObjects) {
        return 0;
    }
    return static_cast<uint16_t>(kFieldEstimateHeaderLen + object_count * kFieldEstimateRecordLen);
}

bool encodeFieldEstimateHeader(const FieldEstimateHeader& in, uint8_t* doc, uint16_t cap) {
    const uint16_t len = fieldEstimateLen(in.object_count);
    if (len == 0 || cap < len) {
        return false;
    }
    memset(doc, 0, kFieldEstimateHeaderLen);
    doc[0] = in.format;
    doc[1] = static_cast<uint8_t>(kFieldEstimateRecordLen);
    wr16(doc + 2, in.object_count);
    wr32(doc + 4, in.map_id);
    wr32(doc + 8, in.estimate_id);
    wr32(doc + 12, in.odometry_epoch);
    wr32(doc + 16, in.anchor_revision);
    return true;
}

bool encodeFieldEstimateRecord(const FieldEstimateRecord& in, uint16_t index, uint8_t* doc,
                               uint16_t cap) {
    const uint32_t at =
        kFieldEstimateHeaderLen + static_cast<uint32_t>(index) * kFieldEstimateRecordLen;
    if (index >= kFieldMaxObjects || at + kFieldEstimateRecordLen > cap) {
        return false;
    }
    uint8_t* p = doc + at;
    memset(p, 0, kFieldEstimateRecordLen);
    wr16(p, in.object_id);
    p[2] = in.source;
    p[3] = in.flags;
    wr32(p + 4, static_cast<uint32_t>(in.x_mm));
    wr32(p + 8, static_cast<uint32_t>(in.y_mm));
    wr32(p + 12, static_cast<uint32_t>(in.heading_cdeg));
    wr16(p + 16, in.age_ms);
    return true;
}

bool decodeFieldEstimateHeader(const uint8_t* doc, uint16_t len, FieldEstimateHeader& out) {
    if (len < kFieldEstimateHeaderLen || doc[0] != kFieldEstimateFormat ||
        doc[1] != kFieldEstimateRecordLen) {
        return false;
    }
    const uint16_t count = rd16(doc + 2);
    const uint16_t want  = fieldEstimateLen(count);
    if (want == 0 || want != len) {
        return false;
    }
    out                 = FieldEstimateHeader{};
    out.format          = doc[0];
    out.object_count    = count;
    out.map_id          = rd32(doc + 4);
    out.estimate_id     = rd32(doc + 8);
    out.odometry_epoch  = rd32(doc + 12);
    out.anchor_revision = rd32(doc + 16);
    return true;
}

bool decodeFieldEstimateRecord(const uint8_t* doc, uint16_t len, uint16_t index,
                               FieldEstimateRecord& out) {
    const uint32_t at =
        kFieldEstimateHeaderLen + static_cast<uint32_t>(index) * kFieldEstimateRecordLen;
    if (index >= kFieldMaxObjects || at + kFieldEstimateRecordLen > len) {
        return false;
    }
    const uint8_t* p = doc + at;
    out.object_id    = rd16(p);
    out.source       = p[2];
    out.flags        = p[3];
    out.x_mm         = rdi32(p + 4);
    out.y_mm         = rdi32(p + 8);
    out.heading_cdeg = rdi32(p + 12);
    out.age_ms       = rd16(p + 16);
    return true;
}

DocError validateFieldEstimate(const uint8_t* doc, uint16_t len, const uint8_t* map_doc,
                               uint16_t map_len, uint32_t map_id) {
    if (len < kFieldEstimateHeaderLen) {
        return DocError::kLength;
    }
    if (doc[0] != kFieldEstimateFormat || doc[1] != kFieldEstimateRecordLen) {
        return DocError::kFormat;
    }
    if (rd16(doc + 2) > kFieldMaxObjects) {
        return DocError::kCount;
    }
    FieldEstimateHeader header;
    if (!decodeFieldEstimateHeader(doc, len, header)) {
        return DocError::kLength;
    }
    if (rd32(doc + 20) != 0) {
        return DocError::kRange;
    }
    FieldMapHeader map;
    if (!decodeFieldMapHeader(map_doc, map_len, map)) {
        return DocError::kMapMismatch;
    }
    if (header.map_id != map_id) {
        return DocError::kMapMismatch;
    }
    if (header.object_count != map.object_count) {
        return DocError::kCount;
    }

    for (uint16_t i = 0; i < header.object_count; ++i) {
        FieldEstimateRecord e;
        FieldObjectRecord   o;
        decodeFieldEstimateRecord(doc, len, i, e);
        decodeFieldObjectRecord(map_doc, map_len, i, o);
        const uint8_t* raw = doc + kFieldEstimateHeaderLen + i * kFieldEstimateRecordLen;
        if (e.object_id != o.object_id) {
            return DocError::kOrder;
        }
        if (e.source > kEstimateSourceObserved) {
            return DocError::kKind;
        }
        const bool valid = (e.flags & kEstimateValid) != 0;
        if ((e.flags & ~kEstimateValid) != 0 || valid != (e.source != kEstimateSourceNone)) {
            return DocError::kFlags;
        }
        if (!(o.flags & kObjectEstimated) && e.source != kEstimateSourceNominal) {
            return DocError::kFlags;
        }
        if (e.source != kEstimateSourceObserved && e.age_ms != 0) {
            return DocError::kRange;
        }
        if (!headingValid(e.heading_cdeg) || rd16(raw + 18) != 0) {
            return DocError::kRange;
        }
    }
    return DocError::kNone;
}

} // namespace gatr2
