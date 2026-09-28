// frame_codec.cpp

#include "frame_codec.h"

#include <string.h>

namespace gatr2
{
namespace
{

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

// Source value for a sensor bit, in ascending bit order.
int32_t sensorValue(const SensorSample& s, uint8_t bit, uint8_t word) {
    switch (bit) {
    case 0: return s.enc[0];
    case 1: return s.enc[1];
    case 2: return s.enc[2];
    case 3: return s.gyro_z;
    case 4: return s.accel[word];
    default: return 0;
    }
}

void storeSensorValue(SensorSample& s, uint8_t bit, uint8_t word, int32_t v) {
    switch (bit) {
    case 0: s.enc[0] = v; break;
    case 1: s.enc[1] = v; break;
    case 2: s.enc[2] = v; break;
    case 3: s.gyro_z = v; break;
    case 4: s.accel[word] = v; break;
    default: break;
    }
}

bool linkLenValid(uint8_t type, uint8_t len) {
    switch (type) {
    case kFrameBrainRequest: return len >= kBrainRequestHeaderLen && len <= kBrainRequestMaxLen;
    case kFrameBrainReply: return len >= kBrainReplyHeaderLen && len <= kBrainReplyMaxLen;
    case kFramePicoCommand: return len >= kPicoCommandHeaderLen && len <= kPicoCommandMaxLen;
    case kFramePicoStatus: return len == kPicoStatusLen;
    default: return false;
    }
}

bool linkType(uint8_t type) {
    return type == kFrameBrainRequest || type == kFrameBrainReply || type == kFramePicoCommand ||
           type == kFramePicoStatus;
}

// Sync, type, len and crc around a payload already written at buf + 4.
uint16_t finishLinkFrame(uint8_t type, uint8_t len, uint8_t* buf) {
    buf[0] = kSync0;
    buf[1] = kSync1;
    buf[2] = type;
    buf[3] = len;
    wr16(buf + 4 + len, crc16(buf + 2, static_cast<uint16_t>(len + 2)));
    return static_cast<uint16_t>(len + kLinkEnvelopeLen);
}

// Payload of a whole link frame, nullptr on a bad envelope, len or crc.
const uint8_t* linkPayload(uint8_t type, const uint8_t* buf, uint16_t len) {
    if (len < kLinkEnvelopeLen || len > kMaxFrameLen) {
        return nullptr;
    }
    if (buf[0] != kSync0 || buf[1] != kSync1 || buf[2] != type) {
        return nullptr;
    }
    const uint8_t n = buf[3];
    if (!linkLenValid(type, n) || len != n + kLinkEnvelopeLen) {
        return nullptr;
    }
    if (crc16(buf + 2, static_cast<uint16_t>(n + 2)) != rd16(buf + 4 + n)) {
        return nullptr;
    }
    return buf + 4;
}

void wrState(uint8_t* p, const BrainState& s) {
    p[0] = s.robot_flags;
    wr32(p + 1, static_cast<uint32_t>(s.x_mm));
    wr32(p + 5, static_cast<uint32_t>(s.y_mm));
    wr32(p + 9, static_cast<uint32_t>(s.heading_cdeg));
    wr16(p + 13, s.robot_age_ms);
    wr32(p + 15, s.odometry_epoch);
    wr32(p + 19, s.anchor_revision);
    p[23] = s.health;
    p[24] = s.profile_state;
    p[25] = s.profile_reason;
    wr32(p + 26, s.profile_id);
    wr32(p + 30, s.map_id);
    wr32(p + 34, s.estimate_id);
    p[38] = s.calibration;
    p[39] = s.profile_detail;
}

void rdState(const uint8_t* p, BrainState& s) {
    s.robot_flags     = p[0];
    s.x_mm            = static_cast<int32_t>(rd32(p + 1));
    s.y_mm            = static_cast<int32_t>(rd32(p + 5));
    s.heading_cdeg    = static_cast<int32_t>(rd32(p + 9));
    s.robot_age_ms    = rd16(p + 13);
    s.odometry_epoch  = rd32(p + 15);
    s.anchor_revision = rd32(p + 19);
    s.health          = p[23];
    s.profile_state   = p[24];
    s.profile_reason  = p[25];
    s.profile_id      = rd32(p + 26);
    s.map_id          = rd32(p + 30);
    s.estimate_id     = rd32(p + 34);
    s.calibration     = p[38];
    s.profile_detail  = p[39];
}

bool knownResult(uint8_t result) {
    switch (result) {
    case kResultOk:
    case kResultPending:
    case kResultUnknownSession:
    case kResultUnsupportedVersion:
    case kResultUnsupportedOp:
    case kResultInvalidArgument:
    case kResultStale:
    case kResultNotReady:
    case kResultProfileRejected:
    case kResultUnavailable:
    case kResultNotStationary:
    case kResultFailed: return true;
    default: return false;
    }
}

// Request payload length for the encoder, 0 when a variable body is out of range.
uint8_t requestLen(const BrainRequest& in) {
    switch (in.op) {
    case kOpProfileWrite:
        if (in.data_len == 0 || in.data_len > kProfileChunkMax) {
            return 0;
        }
        return static_cast<uint8_t>(kBrainRequestHeaderLen + kProfileWriteHeaderLen + in.data_len);
    case kOpPathReport:
        if (in.point_count > kPathReportMaxPoints) {
            return 0;
        }
        return static_cast<uint8_t>(kBrainRequestHeaderLen + kPathReportHeaderLen +
                                    8 * in.point_count);
    default: return brainRequestMinLen(in.op);
    }
}

uint8_t replyLen(const BrainReply& in) {
    if (in.op == kOpReadWheels && in.result == kResultOk) {
        if (in.wheel_count > kWheelReadingsMax) {
            return 0;
        }
        return static_cast<uint8_t>(kBrainReplyHeaderLen + 1 + kWheelReadingLen * in.wheel_count);
    }
    if (in.op == kOpReadDoc && in.result == kResultOk) {
        if (in.data_len == 0 || in.data_len > kDocChunkMax) {
            return 0;
        }
        return static_cast<uint8_t>(kBrainReplyHeaderLen + kReadDocReplyHeaderLen + in.data_len);
    }
    return brainReplyMinLen(in.op, in.result);
}

} // namespace

uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>(crc ^ (data[i] << 8));
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

bool sensorMaskValid(uint16_t mask) {
    const uint16_t known = static_cast<uint16_t>((1u << kSensorBitCount) - 1u);
    return (mask & ~known) == 0;
}

uint16_t sensorPayloadLen(uint16_t mask) {
    uint16_t n = 0;
    for (uint8_t bit = 0; bit < kSensorBitCount; ++bit) {
        if (mask & (1u << bit)) {
            n = static_cast<uint16_t>(n + kSensorWidth[bit]);
        }
    }
    return n;
}

uint16_t sensorFrameLen(uint16_t mask) {
    return static_cast<uint16_t>(kSensorHeaderLen + sensorPayloadLen(mask) + 1);
}

uint16_t sensorV2FrameLen(uint16_t mask) {
    return static_cast<uint16_t>(kSensorV2HeaderLen + sensorPayloadLen(mask) + 1);
}

uint8_t picoCommandLen(uint8_t op) {
    switch (op) {
    case kPicoOpConfigure:
    case kPicoOpReinitImu: return kPicoCommandHeaderLen + 1;
    case kPicoOpRestartAcquisition: return kPicoCommandHeaderLen;
    default: return 0;
    }
}

uint8_t brainRequestMinLen(uint8_t op) {
    switch (op) {
    case kOpHello: return kBrainRequestHeaderLen + 4;
    case kOpSetPose: return kBrainRequestHeaderLen + 12;
    case kOpGetState: return kBrainRequestHeaderLen + 9;
    case kOpProfileWrite: return kBrainRequestHeaderLen + kProfileWriteHeaderLen + 1;
    case kOpProfileApply: return kBrainRequestHeaderLen + 6;
    case kOpReadDoc: return kBrainRequestHeaderLen + 8;
    case kOpControl: return kBrainRequestHeaderLen + 2;
    case kOpPathReport: return kBrainRequestHeaderLen + kPathReportHeaderLen;
    case kOpReadWheels: return kBrainRequestHeaderLen;
    default: return 0;
    }
}

uint8_t brainRequestMaxLen(uint8_t op) {
    switch (op) {
    case kOpProfileWrite: return kBrainRequestHeaderLen + kProfileWriteHeaderLen + kProfileChunkMax;
    case kOpPathReport:
        return kBrainRequestHeaderLen + kPathReportHeaderLen + 8 * kPathReportMaxPoints;
    default: return brainRequestMinLen(op);
    }
}

uint8_t brainReplyMinLen(uint8_t op, uint8_t result) {
    if (op == kOpHello) {
        return kBrainReplyHeaderLen + 4;
    }
    if (!knownResult(result)) {
        return 0;
    }
    const bool ok = result == kResultOk;
    switch (op) {
    case kOpSetPose:
        return (ok || result == kResultPending) ? kBrainReplyHeaderLen + 8 : kBrainReplyHeaderLen;
    case kOpGetState: return ok ? kBrainReplyHeaderLen + kBrainStateLen : kBrainReplyHeaderLen;
    case kOpProfileWrite: return ok ? kBrainReplyHeaderLen + 6 : kBrainReplyHeaderLen;
    case kOpProfileApply:
        return (ok || result == kResultPending || result == kResultProfileRejected)
                   ? kBrainReplyHeaderLen + 7
                   : kBrainReplyHeaderLen;
    case kOpReadDoc:
        return ok ? kBrainReplyHeaderLen + kReadDocReplyHeaderLen + 1 : kBrainReplyHeaderLen;
    case kOpControl:
        return (ok || result == kResultPending || result == kResultFailed)
                   ? kBrainReplyHeaderLen + 3
                   : kBrainReplyHeaderLen;
    case kOpPathReport: return kBrainReplyHeaderLen;
    case kOpReadWheels: return ok ? kBrainReplyHeaderLen + 1 : kBrainReplyHeaderLen;
    default: return 0;
    }
}

uint8_t brainReplyMaxLen(uint8_t op, uint8_t result) {
    if (op == kOpReadDoc && result == kResultOk) {
        return kBrainReplyHeaderLen + kReadDocReplyHeaderLen + kDocChunkMax;
    }
    if (op == kOpReadWheels && result == kResultOk) {
        return kBrainReplyHeaderLen + 1 + kWheelReadingLen * kWheelReadingsMax;
    }
    return brainReplyMinLen(op, result);
}

uint16_t encodeSensorFrame(const SensorSample& in, uint8_t* buf, uint16_t cap) {
    if (!sensorMaskValid(in.mask)) {
        return 0;
    }
    const uint16_t total = in.identity ? sensorV2FrameLen(in.mask) : sensorFrameLen(in.mask);
    if (total > cap || total > kMaxFrameLen) {
        return 0;
    }

    buf[0] = kSync0;
    buf[1] = kSync1;
    buf[2] = in.identity ? kFrameSensorV2 : kFrameSensor;
    buf[3] = in.seq;
    wr32(buf + 4, in.stamp_ms);
    uint16_t at = kSensorHeaderLen;
    if (in.identity) {
        wr16(buf + 8, in.boot_id);
        buf[10] = in.acq_epoch;
        buf[11] = in.imu_epoch;
        wr16(buf + 12, in.mask);
        at = kSensorV2HeaderLen;
    } else {
        wr16(buf + 8, in.mask);
    }

    for (uint8_t bit = 0; bit < kSensorBitCount; ++bit) {
        if ((in.mask & (1u << bit)) == 0) {
            continue;
        }
        const uint8_t words = static_cast<uint8_t>(kSensorWidth[bit] / 4);
        for (uint8_t w = 0; w < words; ++w) {
            wr32(buf + at, static_cast<uint32_t>(sensorValue(in, bit, w)));
            at = static_cast<uint16_t>(at + 4);
        }
    }

    buf[at] = checksum(buf, at);
    return total;
}

bool decodeSensorFrame(const uint8_t* buf, uint16_t len, SensorSample& out) {
    if (len < kSensorHeaderLen + 1 || len > kMaxFrameLen) {
        return false;
    }
    if (buf[0] != kSync0 || buf[1] != kSync1) {
        return false;
    }
    const bool v2 = buf[2] == kFrameSensorV2;
    if (!v2 && buf[2] != kFrameSensor) {
        return false;
    }
    if (v2 && len < kSensorV2HeaderLen + 1) {
        return false;
    }
    const uint16_t mask = rd16(buf + (v2 ? 12 : 8));
    if (!sensorMaskValid(mask) || (v2 ? sensorV2FrameLen(mask) : sensorFrameLen(mask)) != len) {
        return false;
    }
    if (checksum(buf, len) != 0) {
        return false;
    }

    out          = SensorSample{};
    out.seq      = buf[3];
    out.stamp_ms = rd32(buf + 4);
    out.mask     = mask;
    uint16_t at  = kSensorHeaderLen;
    if (v2) {
        out.identity  = true;
        out.boot_id   = rd16(buf + 8);
        out.acq_epoch = buf[10];
        out.imu_epoch = buf[11];
        at            = kSensorV2HeaderLen;
    }

    for (uint8_t bit = 0; bit < kSensorBitCount; ++bit) {
        if ((mask & (1u << bit)) == 0) {
            continue;
        }
        const uint8_t words = static_cast<uint8_t>(kSensorWidth[bit] / 4);
        for (uint8_t w = 0; w < words; ++w) {
            storeSensorValue(out, bit, w, static_cast<int32_t>(rd32(buf + at)));
            at = static_cast<uint16_t>(at + 4);
        }
    }
    return true;
}

uint16_t encodeBrainRequest(const BrainRequest& in, uint8_t* buf, uint16_t cap) {
    uint8_t n = kBrainRequestHeaderLen;
    if (brainRequestMinLen(in.op) != 0) {
        n = requestLen(in);
        if (n == 0) {
            return 0;
        }
    }
    if (n + kLinkEnvelopeLen > cap) {
        return 0;
    }

    uint8_t* p = buf + 4;
    p[0]       = in.version;
    p[1]       = in.op;
    wr32(p + 2, in.session);
    wr16(p + 6, in.request_id);

    uint8_t* body = p + kBrainRequestHeaderLen;
    switch (in.op) {
    case kOpHello: wr32(body, in.nonce); break;
    case kOpSetPose:
        wr32(body, static_cast<uint32_t>(in.x_mm));
        wr32(body + 4, static_cast<uint32_t>(in.y_mm));
        wr32(body + 8, static_cast<uint32_t>(in.heading_cdeg));
        break;
    case kOpGetState:
        body[0] = in.imu_flags;
        wr32(body + 1, in.imu_stamp_ms);
        wr32(body + 5, static_cast<uint32_t>(in.imu_rotation_mdeg));
        break;
    case kOpProfileWrite:
        wr32(body, in.profile_id);
        wr16(body + 4, in.total_len);
        wr16(body + 6, in.offset);
        memcpy(body + kProfileWriteHeaderLen, in.data, in.data_len);
        break;
    case kOpProfileApply:
        wr32(body, in.profile_id);
        wr16(body + 4, in.total_len);
        break;
    case kOpReadDoc:
        body[0] = in.doc_kind;
        wr32(body + 1, in.doc_id);
        wr16(body + 5, in.doc_offset);
        body[7] = in.max_len;
        break;
    case kOpControl:
        body[0] = in.action;
        body[1] = in.action_arg;
        break;
    case kOpPathReport:
        wr32(body, in.command_id);
        body[4] = in.path_mode;
        body[5] = in.point_count;
        for (uint8_t i = 0; i < in.point_count; ++i) {
            uint8_t* at = body + kPathReportHeaderLen + 8 * i;
            wr32(at, static_cast<uint32_t>(in.points[i].x_mm));
            wr32(at + 4, static_cast<uint32_t>(in.points[i].y_mm));
        }
        break;
    default: break;
    }
    return finishLinkFrame(kFrameBrainRequest, n, buf);
}

bool decodeBrainRequest(const uint8_t* buf, uint16_t len, BrainRequest& out) {
    const uint8_t* p = linkPayload(kFrameBrainRequest, buf, len);
    if (p == nullptr) {
        return false;
    }
    const uint8_t n = buf[3];

    out            = BrainRequest{};
    out.version    = p[0];
    out.op         = p[1];
    out.session    = rd32(p + 2);
    out.request_id = rd16(p + 6);
    if (out.version != kBrainLinkVersion) {
        return true;
    }

    const uint8_t lo = brainRequestMinLen(out.op);
    if (lo == 0) {
        return true;
    }
    if (n < lo || n > brainRequestMaxLen(out.op)) {
        return false;
    }
    const uint8_t* body = p + kBrainRequestHeaderLen;
    switch (out.op) {
    case kOpHello: out.nonce = rd32(body); break;
    case kOpSetPose:
        out.x_mm         = static_cast<int32_t>(rd32(body));
        out.y_mm         = static_cast<int32_t>(rd32(body + 4));
        out.heading_cdeg = static_cast<int32_t>(rd32(body + 8));
        break;
    case kOpGetState:
        out.imu_flags         = body[0];
        out.imu_stamp_ms      = rd32(body + 1);
        out.imu_rotation_mdeg = static_cast<int32_t>(rd32(body + 5));
        break;
    case kOpProfileWrite:
        out.profile_id = rd32(body);
        out.total_len  = rd16(body + 4);
        out.offset     = rd16(body + 6);
        out.data_len = static_cast<uint8_t>(n - kBrainRequestHeaderLen - kProfileWriteHeaderLen);
        memcpy(out.data, body + kProfileWriteHeaderLen, out.data_len);
        break;
    case kOpProfileApply:
        out.profile_id = rd32(body);
        out.total_len  = rd16(body + 4);
        break;
    case kOpReadDoc:
        out.doc_kind   = body[0];
        out.doc_id     = rd32(body + 1);
        out.doc_offset = rd16(body + 5);
        out.max_len    = body[7];
        break;
    case kOpControl:
        out.action     = body[0];
        out.action_arg = body[1];
        break;
    case kOpPathReport:
        out.command_id  = rd32(body);
        out.path_mode   = body[4];
        out.point_count = body[5];
        if (out.point_count > kPathReportMaxPoints ||
            n != kBrainRequestHeaderLen + kPathReportHeaderLen + 8 * out.point_count) {
            return false;
        }
        for (uint8_t i = 0; i < out.point_count; ++i) {
            const uint8_t* at  = body + kPathReportHeaderLen + 8 * i;
            out.points[i].x_mm = static_cast<int32_t>(rd32(at));
            out.points[i].y_mm = static_cast<int32_t>(rd32(at + 4));
        }
        break;
    default: break;
    }
    return true;
}

uint16_t encodeBrainReply(const BrainReply& in, uint8_t* buf, uint16_t cap) {
    uint8_t n = kBrainReplyHeaderLen;
    if (brainReplyMinLen(in.op, in.result) != 0) {
        n = replyLen(in);
        if (n == 0) {
            return 0;
        }
    }
    if (n + kLinkEnvelopeLen > cap) {
        return 0;
    }

    uint8_t* p = buf + 4;
    p[0]       = in.version;
    p[1]       = in.op;
    wr32(p + 2, in.session);
    wr16(p + 6, in.request_id);
    p[8] = in.result;
    wr32(p + 9, in.pi_instance);

    uint8_t* body = p + kBrainReplyHeaderLen;
    if (n > kBrainReplyHeaderLen) {
        switch (in.op) {
        case kOpHello: wr32(body, in.nonce); break;
        case kOpSetPose:
            wr32(body, in.odometry_epoch);
            wr32(body + 4, in.anchor_revision);
            break;
        case kOpGetState: wrState(body, in.state); break;
        case kOpProfileWrite:
            wr32(body, in.profile_id);
            wr16(body + 4, in.received);
            break;
        case kOpProfileApply:
            wr32(body, in.profile_id);
            body[4] = in.profile_state;
            body[5] = in.profile_reason;
            body[6] = in.profile_detail;
            break;
        case kOpReadDoc:
            body[0] = in.doc_kind;
            wr32(body + 1, in.doc_id);
            wr16(body + 5, in.doc_total_len);
            wr32(body + 7, in.doc_crc32);
            wr16(body + 11, in.doc_offset);
            memcpy(body + kReadDocReplyHeaderLen, in.data, in.data_len);
            break;
        case kOpControl:
            body[0] = in.action;
            body[1] = in.calibration;
            body[2] = in.control_detail;
            break;
        case kOpReadWheels:
            body[0] = in.wheel_count;
            for (uint8_t i = 0; i < in.wheel_count; ++i) {
                const WheelReading& w  = in.wheels[i];
                uint8_t*            at = body + 1 + kWheelReadingLen * i;
                memset(at, 0, kWheelReadingLen);
                at[0] = w.port;
                at[1] = w.flags;
                wr16(at + 2, w.discontinuity);
                wr32(at + 4, static_cast<uint32_t>(w.counts));
                wr32(at + 8, static_cast<uint32_t>(w.travel_um));
                wr16(at + 12, w.age_ms);
            }
            break;
        default: break;
        }
    }
    return finishLinkFrame(kFrameBrainReply, n, buf);
}

bool decodeBrainReply(const uint8_t* buf, uint16_t len, BrainReply& out) {
    const uint8_t* p = linkPayload(kFrameBrainReply, buf, len);
    if (p == nullptr) {
        return false;
    }
    const uint8_t n = buf[3];

    out             = BrainReply{};
    out.version     = p[0];
    out.op          = p[1];
    out.session     = rd32(p + 2);
    out.request_id  = rd16(p + 6);
    out.result      = p[8];
    out.pi_instance = rd32(p + 9);
    if (out.version != kBrainLinkVersion) {
        return true;
    }

    const uint8_t lo = brainReplyMinLen(out.op, out.result);
    if (lo == 0) {
        return true;
    }
    if (n < lo || n > brainReplyMaxLen(out.op, out.result)) {
        return false;
    }
    if (n == kBrainReplyHeaderLen) {
        return true;
    }
    const uint8_t* body = p + kBrainReplyHeaderLen;
    switch (out.op) {
    case kOpHello: out.nonce = rd32(body); break;
    case kOpSetPose:
        out.odometry_epoch  = rd32(body);
        out.anchor_revision = rd32(body + 4);
        break;
    case kOpGetState: rdState(body, out.state); break;
    case kOpProfileWrite:
        out.profile_id = rd32(body);
        out.received   = rd16(body + 4);
        break;
    case kOpProfileApply:
        out.profile_id     = rd32(body);
        out.profile_state  = body[4];
        out.profile_reason = body[5];
        out.profile_detail = body[6];
        break;
    case kOpReadDoc:
        out.doc_kind      = body[0];
        out.doc_id        = rd32(body + 1);
        out.doc_total_len = rd16(body + 5);
        out.doc_crc32     = rd32(body + 7);
        out.doc_offset    = rd16(body + 11);
        out.data_len = static_cast<uint8_t>(n - kBrainReplyHeaderLen - kReadDocReplyHeaderLen);
        memcpy(out.data, body + kReadDocReplyHeaderLen, out.data_len);
        break;
    case kOpControl:
        out.action         = body[0];
        out.calibration    = body[1];
        out.control_detail = body[2];
        break;
    case kOpReadWheels:
        out.wheel_count = body[0];
        if (out.wheel_count > kWheelReadingsMax ||
            n != kBrainReplyHeaderLen + 1 + kWheelReadingLen * out.wheel_count) {
            return false;
        }
        for (uint8_t i = 0; i < out.wheel_count; ++i) {
            const uint8_t* at      = body + 1 + kWheelReadingLen * i;
            WheelReading&  w       = out.wheels[i];
            w.port                 = at[0];
            w.flags                = at[1];
            w.discontinuity        = rd16(at + 2);
            w.counts               = static_cast<int32_t>(rd32(at + 4));
            w.travel_um            = static_cast<int32_t>(rd32(at + 8));
            w.age_ms               = rd16(at + 12);
        }
        break;
    default: break;
    }
    return true;
}

uint16_t encodePicoCommand(const PicoCommand& in, uint8_t* buf, uint16_t cap) {
    const uint8_t known = picoCommandLen(in.op);
    const uint8_t n     = known != 0 ? known : kPicoCommandHeaderLen;
    if (n + kLinkEnvelopeLen > cap) {
        return 0;
    }
    uint8_t* p = buf + 4;
    p[0]       = in.version;
    p[1]       = in.op;
    wr16(p + 2, in.request_id);
    wr16(p + 4, in.target_boot_id);
    if (in.op == kPicoOpConfigure) {
        p[6] = in.imu_enabled;
    } else if (in.op == kPicoOpReinitImu) {
        p[6] = in.imu_port;
    }
    return finishLinkFrame(kFramePicoCommand, n, buf);
}

bool decodePicoCommand(const uint8_t* buf, uint16_t len, PicoCommand& out) {
    const uint8_t* p = linkPayload(kFramePicoCommand, buf, len);
    if (p == nullptr || p[0] != kPicoLinkVersion) {
        return false;
    }
    const uint8_t n    = buf[3];
    out                = PicoCommand{};
    out.version        = p[0];
    out.op             = p[1];
    out.request_id     = rd16(p + 2);
    out.target_boot_id = rd16(p + 4);
    const uint8_t want = picoCommandLen(out.op);
    if (want == 0) {
        return true;
    }
    if (n != want) {
        return false;
    }
    if (out.op == kPicoOpConfigure) {
        out.imu_enabled = p[6];
    } else if (out.op == kPicoOpReinitImu) {
        out.imu_port = p[6];
    }
    return true;
}

uint16_t encodePicoStatus(const PicoStatus& in, uint8_t* buf, uint16_t cap) {
    if (kPicoStatusLen + kLinkEnvelopeLen > cap) {
        return 0;
    }
    uint8_t* p = buf + 4;
    p[0]       = in.version;
    wr16(p + 1, in.boot_id);
    p[3] = in.acq_epoch;
    p[4] = in.imu_epoch;
    wr32(p + 5, in.uptime_ms);
    p[9]  = in.imu_state;
    p[10] = in.imu_reason;
    wr16(p + 11, in.imu_attempts);
    p[13] = in.flags;
    wr16(p + 14, in.last_request_id);
    p[16] = in.last_op;
    p[17] = in.last_status;
    p[18] = in.last_detail;
    p[19] = in.firmware;
    return finishLinkFrame(kFramePicoStatus, kPicoStatusLen, buf);
}

bool decodePicoStatus(const uint8_t* buf, uint16_t len, PicoStatus& out) {
    const uint8_t* p = linkPayload(kFramePicoStatus, buf, len);
    if (p == nullptr || p[0] != kPicoLinkVersion) {
        return false;
    }
    out                 = PicoStatus{};
    out.version         = p[0];
    out.boot_id         = rd16(p + 1);
    out.acq_epoch       = p[3];
    out.imu_epoch       = p[4];
    out.uptime_ms       = rd32(p + 5);
    out.imu_state       = p[9];
    out.imu_reason      = p[10];
    out.imu_attempts    = rd16(p + 11);
    out.flags           = p[13];
    out.last_request_id = rd16(p + 14);
    out.last_op         = p[16];
    out.last_status     = p[17];
    out.last_detail     = p[18];
    out.firmware        = p[19];
    return true;
}

void FrameReader::reset() {
    len_       = 0;
    frame_len_ = 0;
}

uint16_t FrameReader::expectedLen() const {
    const uint16_t reject = kMaxFrameLen + 1;
    if (len_ < 3) {
        return 0;
    }
    const uint8_t type = buf_[2];
    if (type == kFrameSensor) {
        if (len_ < kSensorHeaderLen) {
            return 0;
        }
        const uint16_t mask = rd16(buf_ + 8);
        return sensorMaskValid(mask) ? sensorFrameLen(mask) : reject;
    }
    if (type == kFrameSensorV2) {
        if (len_ < kSensorV2HeaderLen) {
            return 0;
        }
        const uint16_t mask = rd16(buf_ + 12);
        return sensorMaskValid(mask) ? sensorV2FrameLen(mask) : reject;
    }
    if (linkType(type)) {
        if (len_ < 4) {
            return 0;
        }
        const uint8_t n = buf_[3];
        return linkLenValid(type, n) ? static_cast<uint16_t>(n + kLinkEnvelopeLen) : reject;
    }
    return reject;
}

bool FrameReader::frameValid(uint16_t len) const {
    if (buf_[2] == kFrameSensor || buf_[2] == kFrameSensorV2) {
        return checksum(buf_, len) == 0;
    }
    return crc16(buf_ + 2, static_cast<uint16_t>(len - 4)) == rd16(buf_ + len - 2);
}

void FrameReader::drop(uint16_t n) {
    memmove(buf_, buf_ + n, len_ - n);
    len_ = static_cast<uint16_t>(len_ - n);
}

bool FrameReader::push(uint8_t b) {
    if (frame_len_ > 0) {
        drop(frame_len_);
        frame_len_ = 0;
    }
    if (len_ >= kMaxFrameLen) {
        drop(1);
    }
    buf_[len_++] = b;
    return scan();
}

bool FrameReader::next() {
    if (frame_len_ > 0) {
        drop(frame_len_);
        frame_len_ = 0;
    }
    return scan();
}

bool FrameReader::scan() {
    // A rejected candidate drops only its sync0, so the bytes after it are rescanned.
    while (len_ > 0) {
        if (buf_[0] != kSync0 || (len_ > 1 && buf_[1] != kSync1)) {
            drop(1);
            continue;
        }
        const uint16_t want = expectedLen();
        if (want > kMaxFrameLen) {
            drop(1);
            continue;
        }
        if (want == 0 || len_ < want) {
            return false;
        }
        if (!frameValid(want)) {
            drop(1);
            continue;
        }
        frame_len_ = want;
        return true;
    }
    return false;
}

} // namespace gatr2
