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
    if (type == kFrameBrainRequest) {
        return len >= kBrainRequestHeaderLen && len <= kBrainRequestMaxLen;
    }
    if (type == kFrameBrainReply) {
        return len >= kBrainReplyHeaderLen && len <= kBrainReplyMaxLen;
    }
    return false;
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
    p[24] = s.landmark_id;
    p[25] = s.landmark_source;
    wr32(p + 26, static_cast<uint32_t>(s.lm_x_mm));
    wr32(p + 30, static_cast<uint32_t>(s.lm_y_mm));
    wr32(p + 34, static_cast<uint32_t>(s.lm_heading_cdeg));
    wr16(p + 38, s.landmark_age_ms);
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
    s.landmark_id     = p[24];
    s.landmark_source = p[25];
    s.lm_x_mm         = static_cast<int32_t>(rd32(p + 26));
    s.lm_y_mm         = static_cast<int32_t>(rd32(p + 30));
    s.lm_heading_cdeg = static_cast<int32_t>(rd32(p + 34));
    s.landmark_age_ms = rd16(p + 38);
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

uint8_t brainRequestLen(uint8_t op) {
    switch (op) {
    case kOpHello: return kBrainRequestHeaderLen + 4;
    case kOpSetPose: return kBrainRequestHeaderLen + 12;
    case kOpSelectLandmark: return kBrainRequestHeaderLen + 2;
    case kOpGetState: return kBrainRequestHeaderLen;
    default: return 0;
    }
}

uint8_t brainReplyLen(uint8_t op, uint8_t result) {
    if (op == kOpHello) {
        return kBrainReplyHeaderLen + 4;
    }
    if (result > kResultStale) {
        return 0;
    }
    const bool ok = result == kResultOk;
    switch (op) {
    case kOpSetPose:
        return (ok || result == kResultPending) ? kBrainReplyHeaderLen + 8 : kBrainReplyHeaderLen;
    case kOpSelectLandmark: return ok ? kBrainReplyHeaderLen + 2 : kBrainReplyHeaderLen;
    case kOpGetState: return ok ? kBrainReplyHeaderLen + kBrainStateLen : kBrainReplyHeaderLen;
    default: return 0;
    }
}

uint16_t encodeSensorFrame(const SensorSample& in, uint8_t* buf, uint16_t cap) {
    if (!sensorMaskValid(in.mask)) {
        return 0;
    }
    const uint16_t total = sensorFrameLen(in.mask);
    if (total > cap || total > kMaxFrameLen) {
        return 0;
    }

    buf[0] = kSync0;
    buf[1] = kSync1;
    buf[2] = kFrameSensor;
    buf[3] = in.seq;
    wr32(buf + 4, in.stamp_ms);
    wr16(buf + 8, in.mask);

    uint16_t at = kSensorHeaderLen;
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
    if (buf[0] != kSync0 || buf[1] != kSync1 || buf[2] != kFrameSensor) {
        return false;
    }
    const uint16_t mask = rd16(buf + 8);
    if (!sensorMaskValid(mask) || sensorFrameLen(mask) != len) {
        return false;
    }
    if (checksum(buf, len) != 0) {
        return false;
    }

    out          = SensorSample{};
    out.seq      = buf[3];
    out.stamp_ms = rd32(buf + 4);
    out.mask     = mask;

    uint16_t at = kSensorHeaderLen;
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
    const uint8_t known = brainRequestLen(in.op);
    const uint8_t n     = known != 0 ? known : kBrainRequestHeaderLen;
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
    case kOpSelectLandmark:
        body[0] = in.landmark_id;
        body[1] = in.select_flags;
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

    const uint8_t want = brainRequestLen(out.op);
    if (want == 0) {
        return true;
    }
    if (n != want) {
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
    case kOpSelectLandmark:
        out.landmark_id  = body[0];
        out.select_flags = body[1];
        break;
    default: break;
    }
    return true;
}

uint16_t encodeBrainReply(const BrainReply& in, uint8_t* buf, uint16_t cap) {
    const uint8_t known = brainReplyLen(in.op, in.result);
    const uint8_t n     = known != 0 ? known : kBrainReplyHeaderLen;
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
        case kOpSelectLandmark:
            body[0] = in.landmark_id;
            body[1] = in.select_flags;
            break;
        case kOpGetState: wrState(body, in.state); break;
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

    const uint8_t want = brainReplyLen(out.op, out.result);
    if (want == 0) {
        return true;
    }
    if (n != want) {
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
    case kOpSelectLandmark:
        out.landmark_id  = body[0];
        out.select_flags = body[1];
        break;
    case kOpGetState: rdState(body, out.state); break;
    default: break;
    }
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
    if (type == kFrameBrainRequest || type == kFrameBrainReply) {
        if (len_ < 4) {
            return 0;
        }
        const uint8_t n = buf_[3];
        return linkLenValid(type, n) ? static_cast<uint16_t>(n + kLinkEnvelopeLen) : reject;
    }
    return reject;
}

bool FrameReader::frameValid(uint16_t len) const {
    if (buf_[2] == kFrameSensor) {
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
