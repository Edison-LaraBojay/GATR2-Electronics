// frame_codec.cpp

#include "frame_codec.h"

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

} // namespace

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

uint16_t poseFrameLen(uint8_t n_landmarks) {
    return static_cast<uint16_t>(kPoseHeaderLen + n_landmarks * kLandmarkLen + 1);
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

uint16_t encodePoseFrame(const PoseFrame& in, uint8_t* buf, uint16_t cap) {
    if (in.n_landmarks > kMaxLandmarks) {
        return 0;
    }
    const uint16_t total = poseFrameLen(in.n_landmarks);
    if (total > cap || total > kMaxFrameLen) {
        return 0;
    }

    buf[0] = kSync0;
    buf[1] = kSync1;
    buf[2] = kFramePose;
    buf[3] = in.seq;
    wr32(buf + 4, in.stamp_ms);
    wr32(buf + 8, static_cast<uint32_t>(in.x_mm));
    wr32(buf + 12, static_cast<uint32_t>(in.y_mm));
    wr32(buf + 16, static_cast<uint32_t>(in.heading_cdeg));
    buf[20] = in.status;
    buf[21] = in.n_landmarks;

    uint16_t at = kPoseHeaderLen;
    for (uint8_t i = 0; i < in.n_landmarks; ++i) {
        const LandmarkObs& L = in.landmarks[i];
        buf[at]              = L.id;
        wr16(buf + at + 1, static_cast<uint16_t>(L.dx_mm));
        wr16(buf + at + 3, static_cast<uint16_t>(L.dy_mm));
        wr16(buf + at + 5, static_cast<uint16_t>(L.bearing_cdeg));
        buf[at + 7] = L.quality;
        at          = static_cast<uint16_t>(at + kLandmarkLen);
    }

    buf[at] = checksum(buf, at);
    return total;
}

bool decodePoseFrame(const uint8_t* buf, uint16_t len, PoseFrame& out) {
    if (len < kPoseHeaderLen + 1 || len > kMaxFrameLen) {
        return false;
    }
    if (buf[0] != kSync0 || buf[1] != kSync1 || buf[2] != kFramePose) {
        return false;
    }
    const uint8_t n = buf[21];
    if (n > kMaxLandmarks || poseFrameLen(n) != len) {
        return false;
    }
    if (checksum(buf, len) != 0) {
        return false;
    }

    out              = PoseFrame{};
    out.seq          = buf[3];
    out.stamp_ms     = rd32(buf + 4);
    out.x_mm         = static_cast<int32_t>(rd32(buf + 8));
    out.y_mm         = static_cast<int32_t>(rd32(buf + 12));
    out.heading_cdeg = static_cast<int32_t>(rd32(buf + 16));
    out.status       = buf[20];
    out.n_landmarks  = n;

    uint16_t at = kPoseHeaderLen;
    for (uint8_t i = 0; i < n; ++i) {
        LandmarkObs& L = out.landmarks[i];
        L.id           = buf[at];
        L.dx_mm        = static_cast<int16_t>(rd16(buf + at + 1));
        L.dy_mm        = static_cast<int16_t>(rd16(buf + at + 3));
        L.bearing_cdeg = static_cast<int16_t>(rd16(buf + at + 5));
        L.quality      = buf[at + 7];
        at             = static_cast<uint16_t>(at + kLandmarkLen);
    }
    return true;
}

void FrameReader::reset() {
    len_   = 0;
    ready_ = false;
}

uint16_t FrameReader::expectedLen() const {
    const uint16_t reject = kMaxFrameLen + 1;
    if (len_ < 3) {
        return 0;
    }
    if (buf_[2] == kFrameSensor) {
        if (len_ < kSensorHeaderLen) {
            return 0;
        }
        const uint16_t mask = rd16(buf_ + 8);
        return sensorMaskValid(mask) ? sensorFrameLen(mask) : reject;
    }
    if (buf_[2] == kFramePose) {
        if (len_ < kPoseHeaderLen) {
            return 0;
        }
        const uint8_t n = buf_[21];
        return n <= kMaxLandmarks ? poseFrameLen(n) : reject;
    }
    return reject;
}

bool FrameReader::push(uint8_t b) {
    if (ready_) {
        reset();
    }

    if (len_ == 0) {
        if (b == kSync0) {
            buf_[len_++] = b;
        }
        return false;
    }
    if (len_ == 1) {
        if (b == kSync1) {
            buf_[len_++] = b;
        } else if (b != kSync0) {
            len_ = 0;
        }
        return false;
    }

    if (len_ >= kMaxFrameLen) {
        len_ = 0;
        return false;
    }
    buf_[len_++] = b;

    const uint16_t want = expectedLen();
    if (want == 0) {
        return false;
    }
    if (want > kMaxFrameLen) {
        len_ = 0;
        return false;
    }
    if (len_ < want) {
        return false;
    }
    if (checksum(buf_, want) != 0) {
        len_ = 0;
        return false;
    }
    ready_ = true;
    return true;
}

} // namespace gatr2
