// link_documents_gtest.cpp
// Robot profile, field map and field estimate documents: layouts, structural
// decode and the shared validators. Wheel and footprint values are arbitrary
// test geometry, not robot measurements.

#include <gtest/gtest.h>

#include <stddef.h>
#include <stdint.h>

#include <utility>
#include <vector>

#include "link_documents.h"

using namespace gatr2;

namespace
{

using Bytes = std::vector<uint8_t>;

// CRC-32/ISO-HDLC with a runtime table, independent of the codec's bit loop.
uint32_t refCrc32(const uint8_t* data, size_t len) {
    uint32_t table[256];
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) {
            c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        }
        table[i] = c;
    }
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

// Little endian writer, independent of the codec.
struct Le {
    Bytes b;
    Le&   u8(uint32_t v) {
        b.push_back(static_cast<uint8_t>(v));
        return *this;
    }
    Le& u16(uint32_t v) { return u8(v).u8(v >> 8); }
    Le& u32(uint32_t v) { return u16(v).u16(v >> 16); }
    Le& i16(int16_t v) { return u16(static_cast<uint16_t>(v)); }
    Le& i32(int32_t v) { return u32(static_cast<uint32_t>(v)); }
    Le& zeros(size_t n) {
        b.insert(b.end(), n, 0);
        return *this;
    }
};

// ---------------------------------------------------------------------------
// Profile helpers
// ---------------------------------------------------------------------------

ProfileWheel wheel(uint8_t port, int32_t x_um, int32_t y_um, int32_t angle_mdeg,
                   uint8_t flags = 0) {
    ProfileWheel w;
    w.encoder_port   = port;
    w.flags          = flags;
    w.counts_per_rev = 4000;
    w.radius_um      = 24000;
    w.x_um           = x_um;
    w.y_um           = y_um;
    w.angle_mdeg     = angle_mdeg;
    return w;
}

// A profile the shared rules accept for every topology and IMU source pair
// they allow.
RobotProfileDoc profile(uint8_t topology, uint8_t imu_source) {
    RobotProfileDoc p;
    p.topology   = topology;
    p.imu_source = imu_source;
    if (imu_source == kImuSourceBrainVex) {
        p.vex_smart_port = 1;
    }
    p.footprint_front_um = 200000;
    p.footprint_back_um  = 200000;
    p.footprint_left_um  = 180000;
    p.footprint_right_um = 180000;
    switch (topology) {
    case kTopologyTwoWheelImu:
        p.wheel_count = 2;
        p.wheels[0]   = wheel(0, 0, 100000, 0);
        p.wheels[1]   = wheel(1, -50000, 0, 90000);
        break;
    case kTopologyTwoForwardWheelImu:
        p.wheel_count = 2;
        p.wheels[0]   = wheel(0, 0, 150000, 0);
        p.wheels[1]   = wheel(1, 0, -150000, 180000);
        break;
    case kTopologyThreeWheel:
        p.wheel_count = 3;
        p.wheels[0]   = wheel(0, 0, 150000, 0);
        p.wheels[1]   = wheel(1, 0, -150000, 0);
        p.wheels[2]   = wheel(2, -100000, 0, 90000);
        break;
    default: break;
    }
    return p;
}

ProfileCamera camera(uint8_t slot) {
    ProfileCamera c;
    c.slot       = slot;
    c.x_um       = 120000;
    c.y_um       = -30000;
    c.z_um       = 250000;
    c.roll_mdeg  = 0;
    c.pitch_mdeg = -15000;
    c.yaw_mdeg   = 180000;
    return c;
}

Bytes encodeProfile(const RobotProfileDoc& p) {
    Bytes buf(kProfileMaxLen);
    buf.resize(encodeRobotProfile(p, buf.data(), kProfileMaxLen));
    return buf;
}

struct Verdict {
    bool    ok;
    uint8_t reason;
    uint8_t detail;
};

Verdict validate(const RobotProfileDoc& p) {
    uint8_t reason = 0xEE;
    uint8_t detail = 0xEE;
    const bool ok  = validateRobotProfile(p, reason, detail);
    return {ok, reason, detail};
}

void expectAccepted(const RobotProfileDoc& p) {
    const Verdict v = validate(p);
    EXPECT_TRUE(v.ok);
    EXPECT_EQ(v.reason, kProfileReasonNone);
    EXPECT_EQ(v.detail, 0);
}

void expectRejected(const RobotProfileDoc& p, uint8_t reason, uint8_t detail = 0) {
    const Verdict v = validate(p);
    EXPECT_FALSE(v.ok);
    EXPECT_EQ(v.reason, reason);
    EXPECT_EQ(v.detail, detail);
}

void expectSameProfile(const RobotProfileDoc& a, const RobotProfileDoc& b) {
    EXPECT_EQ(a.format, b.format);
    EXPECT_EQ(a.topology, b.topology);
    EXPECT_EQ(a.wheel_count, b.wheel_count);
    EXPECT_EQ(a.camera_count, b.camera_count);
    EXPECT_EQ(a.imu_source, b.imu_source);
    EXPECT_EQ(a.imu_port, b.imu_port);
    EXPECT_EQ(a.vex_smart_port, b.vex_smart_port);
    EXPECT_EQ(a.imu_flags, b.imu_flags);
    EXPECT_EQ(a.footprint_front_um, b.footprint_front_um);
    EXPECT_EQ(a.footprint_back_um, b.footprint_back_um);
    EXPECT_EQ(a.footprint_left_um, b.footprint_left_um);
    EXPECT_EQ(a.footprint_right_um, b.footprint_right_um);
    EXPECT_EQ(a.calibration_window_ms, b.calibration_window_ms);
    EXPECT_EQ(a.still_rate_cdps, b.still_rate_cdps);
    EXPECT_EQ(a.still_travel_um, b.still_travel_um);
    for (uint8_t i = 0; i < a.wheel_count && i < kProfileMaxWheels; ++i) {
        EXPECT_EQ(a.wheels[i].encoder_port, b.wheels[i].encoder_port) << "wheel " << int(i);
        EXPECT_EQ(a.wheels[i].flags, b.wheels[i].flags);
        EXPECT_EQ(a.wheels[i].counts_per_rev, b.wheels[i].counts_per_rev);
        EXPECT_EQ(a.wheels[i].radius_um, b.wheels[i].radius_um);
        EXPECT_EQ(a.wheels[i].x_um, b.wheels[i].x_um);
        EXPECT_EQ(a.wheels[i].y_um, b.wheels[i].y_um);
        EXPECT_EQ(a.wheels[i].angle_mdeg, b.wheels[i].angle_mdeg);
        EXPECT_EQ(a.wheels[i].gear_micro, b.wheels[i].gear_micro);
        EXPECT_EQ(a.wheels[i].travel_scale_ppm, b.wheels[i].travel_scale_ppm);
    }
    for (uint8_t i = 0; i < a.camera_count && i < kProfileMaxCameras; ++i) {
        EXPECT_EQ(a.cameras[i].slot, b.cameras[i].slot) << "camera " << int(i);
        EXPECT_EQ(a.cameras[i].x_um, b.cameras[i].x_um);
        EXPECT_EQ(a.cameras[i].y_um, b.cameras[i].y_um);
        EXPECT_EQ(a.cameras[i].z_um, b.cameras[i].z_um);
        EXPECT_EQ(a.cameras[i].roll_mdeg, b.cameras[i].roll_mdeg);
        EXPECT_EQ(a.cameras[i].pitch_mdeg, b.cameras[i].pitch_mdeg);
        EXPECT_EQ(a.cameras[i].yaw_mdeg, b.cameras[i].yaw_mdeg);
    }
}

// Profile packed by an independent generator (struct.pack, binascii.crc32):
// two wheels, VEX IMU on port 1, calibration settings, a geared and scaled
// reversed wheel. See knownProfile().
const Bytes kProfileVector = {
    0x01, 0x01, 0x02, 0x00, 0x02, 0x00, 0x01, 0x00, 0x40, 0x0D, 0x03, 0x00,
    0xF0, 0x49, 0x02, 0x00, 0x20, 0xBF, 0x02, 0x00, 0x20, 0xBF, 0x02, 0x00,
    0xD0, 0x07, 0x64, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xA0, 0x0F, 0x00, 0x00, 0xC0, 0x5D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x42, 0x0F, 0x00,
    0x40, 0x42, 0x0F, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00,
    0xC0, 0x5D, 0x00, 0x00, 0xB0, 0x3C, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
    0x90, 0x5F, 0x01, 0x00, 0xA0, 0x25, 0x26, 0x00, 0x79, 0x72, 0x0F, 0x00,
};
constexpr uint32_t kProfileVectorCrc = 0x6482E4C9u;

RobotProfileDoc knownProfile() {
    RobotProfileDoc p            = profile(kTopologyTwoWheelImu, kImuSourceBrainVex);
    p.footprint_back_um          = 150000;
    p.calibration_window_ms      = 2000;
    p.still_rate_cdps            = 100;
    p.still_travel_um            = 1000;
    p.wheels[1].flags            = kWheelReversed;
    p.wheels[1].counts_per_rev   = 8192;
    p.wheels[1].gear_micro       = 2500000;
    p.wheels[1].travel_scale_ppm = 1012345;
    return p;
}

// ---------------------------------------------------------------------------
// Field map and estimate helpers
// ---------------------------------------------------------------------------

FieldMapHeader mapHeader(uint16_t count) {
    FieldMapHeader h;
    h.revision     = 3;
    h.object_count = count;
    h.min_x_mm     = 0;
    h.min_y_mm     = 0;
    h.max_x_mm     = 3658;
    h.max_y_mm     = 3658;
    return h;
}

// Kinds cycle through an estimated obstacle landmark, a fixed obstacle and a
// landmark that is only a reference.
FieldObjectRecord object(uint16_t index, uint16_t id) {
    FieldObjectRecord r;
    r.object_id    = id;
    r.x_mm         = 100 + 20 * index;
    r.y_mm         = 3500 - 20 * index;
    r.heading_cdeg = static_cast<int32_t>((index * 4500) % 36000) - 17999;
    switch (index % 3) {
    case 0:
        r.kind  = kObjectLandmark;
        r.flags = kObjectObstacle | kObjectEstimated | kObjectReference;
        break;
    case 1:
        r.kind  = kObjectFixed;
        r.flags = kObjectObstacle;
        break;
    default:
        r.kind  = kObjectLandmark;
        r.flags = kObjectReference;
        break;
    }
    if (r.flags & kObjectObstacle) {
        r.box_x_mm         = static_cast<int16_t>(10 - index);
        r.box_y_mm         = -5;
        r.box_heading_cdeg = 4500;
        r.box_length_mm    = 154;
        r.box_width_mm     = static_cast<uint16_t>(100 + index);
    }
    return r;
}

std::vector<FieldObjectRecord> objects(uint16_t count) {
    std::vector<FieldObjectRecord> out;
    for (uint16_t i = 0; i < count; ++i) {
        out.push_back(object(i, static_cast<uint16_t>(10 + 7 * i)));
    }
    return out;
}

Bytes buildMap(const FieldMapHeader& h, const std::vector<FieldObjectRecord>& records) {
    Bytes doc(fieldMapLen(h.object_count));
    EXPECT_TRUE(encodeFieldMapHeader(h, doc.data(), static_cast<uint16_t>(doc.size())));
    for (size_t i = 0; i < records.size(); ++i) {
        EXPECT_TRUE(encodeFieldObjectRecord(records[i], static_cast<uint16_t>(i), doc.data(),
                                            static_cast<uint16_t>(doc.size())));
    }
    return doc;
}

Bytes buildMap(uint16_t count) {
    return buildMap(mapHeader(count), objects(count));
}

uint32_t mapIdOf(const Bytes& map) {
    return crc32(map.data(), static_cast<uint32_t>(map.size()));
}

DocError validateMap(const Bytes& doc) {
    return validateFieldMap(doc.data(), static_cast<uint16_t>(doc.size()));
}

// Estimated objects observed, others nominal, per the map's flags.
std::vector<FieldEstimateRecord> estimates(const std::vector<FieldObjectRecord>& objs) {
    std::vector<FieldEstimateRecord> out;
    for (size_t i = 0; i < objs.size(); ++i) {
        FieldEstimateRecord e;
        e.object_id = objs[i].object_id;
        e.flags     = kEstimateValid;
        if (objs[i].flags & kObjectEstimated) {
            e.source       = kEstimateSourceObserved;
            e.x_mm         = objs[i].x_mm + 3;
            e.y_mm         = objs[i].y_mm - 4;
            e.heading_cdeg = 18000;
            e.age_ms       = static_cast<uint16_t>(40 + i);
        } else {
            e.source       = kEstimateSourceNominal;
            e.x_mm         = objs[i].x_mm;
            e.y_mm         = objs[i].y_mm;
            e.heading_cdeg = objs[i].heading_cdeg;
        }
        out.push_back(e);
    }
    return out;
}

FieldEstimateHeader estimateHeader(uint16_t count, uint32_t map_id) {
    FieldEstimateHeader h;
    h.object_count    = count;
    h.map_id          = map_id;
    h.estimate_id     = 17;
    h.odometry_epoch  = 3;
    h.anchor_revision = 5;
    return h;
}

Bytes buildEstimate(const FieldEstimateHeader& h, const std::vector<FieldEstimateRecord>& records) {
    Bytes doc(fieldEstimateLen(h.object_count));
    EXPECT_TRUE(encodeFieldEstimateHeader(h, doc.data(), static_cast<uint16_t>(doc.size())));
    for (size_t i = 0; i < records.size(); ++i) {
        EXPECT_TRUE(encodeFieldEstimateRecord(records[i], static_cast<uint16_t>(i), doc.data(),
                                              static_cast<uint16_t>(doc.size())));
    }
    return doc;
}

DocError validateEstimate(const Bytes& doc, const Bytes& map) {
    return validateFieldEstimate(doc.data(), static_cast<uint16_t>(doc.size()), map.data(),
                                 static_cast<uint16_t>(map.size()), mapIdOf(map));
}

// A valid map and matching estimate of count objects.
struct Field {
    std::vector<FieldObjectRecord>   objs;
    Bytes                            map;
    std::vector<FieldEstimateRecord> recs;
    FieldEstimateHeader              header;

    explicit Field(uint16_t count)
        : objs(objects(count)), map(buildMap(mapHeader(count), objs)), recs(estimates(objs)),
          header(estimateHeader(count, mapIdOf(map))) {}

    Bytes estimate() const { return buildEstimate(header, recs); }
    DocError check() const { return validateEstimate(estimate(), map); }
};

} // namespace

// ---------------------------------------------------------------------------
// CRC-32
// ---------------------------------------------------------------------------

TEST(Crc32, CheckValue) {
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(crc32(text, sizeof(text)), 0xCBF43926u);
    EXPECT_EQ(refCrc32(text, sizeof(text)), 0xCBF43926u);
}

TEST(Crc32, EmptyIsZero) {
    EXPECT_EQ(crc32(nullptr, 0), 0u);
}

TEST(Crc32, MatchesReferenceOnPseudoRandomData) {
    uint32_t seed = 777;
    Bytes    data;
    for (size_t len = 0; len <= kFieldMapMaxLen; ++len) {
        if (len <= 300 || len == kProfileMaxLen || len == kFieldEstimateMaxLen ||
            len == kFieldMapMaxLen) {
            ASSERT_EQ(crc32(data.data(), static_cast<uint32_t>(data.size())),
                      refCrc32(data.data(), data.size()))
                << "len " << len;
        }
        seed = seed * 1664525u + 1013904223u;
        data.push_back(static_cast<uint8_t>(seed >> 24));
    }
}

TEST(Crc32, EverySingleBitFlipChangesTheId) {
    const Bytes    map = buildMap(3);
    const uint32_t id  = mapIdOf(map);
    for (size_t i = 0; i < map.size(); ++i) {
        for (int bit = 0; bit < 8; ++bit) {
            Bytes bad = map;
            bad[i] ^= static_cast<uint8_t>(1u << bit);
            ASSERT_NE(mapIdOf(bad), id) << "byte " << i << " bit " << bit;
        }
    }
}

// ---------------------------------------------------------------------------
// Robot profile layout and codec
// ---------------------------------------------------------------------------

TEST(Profile, LayoutConstants) {
    EXPECT_EQ(kProfileFormat, 1);
    EXPECT_EQ(kProfileMaxWheels, 3);
    EXPECT_EQ(kProfileMaxCameras, 4);
    EXPECT_EQ(kProfileHeaderLen, 32);
    EXPECT_EQ(kProfileWheelLen, 32);
    EXPECT_EQ(kProfileCameraLen, 28);
    EXPECT_EQ(kProfileMaxLen, 240);
    EXPECT_EQ(kUnitMicro, 1000000u);
    // Three PROFILE_WRITE chunks carry the largest profile, two the largest
    // without cameras.
    EXPECT_GT(kProfileMaxLen, 2 * kProfileChunkMax);
    EXPECT_LE(kProfileMaxLen, 3 * kProfileChunkMax);
    EXPECT_LE(kProfileHeaderLen + kProfileMaxWheels * kProfileWheelLen, 2 * kProfileChunkMax);

    const ProfileWheel w{};
    EXPECT_EQ(w.gear_micro, kUnitMicro);
    EXPECT_EQ(w.travel_scale_ppm, kUnitMicro);
    const RobotProfileDoc d{};
    EXPECT_EQ(d.calibration_window_ms, 0);
    EXPECT_EQ(d.still_rate_cdps, 0);
    EXPECT_EQ(d.still_travel_um, 0);

    EXPECT_EQ(kTopologyTwoWheelImu, 1);
    EXPECT_EQ(kTopologyThreeWheel, 2);
    EXPECT_EQ(kTopologyTwoForwardWheelImu, 3);
    EXPECT_EQ(kImuSourceNone, 0);
    EXPECT_EQ(kImuSourcePico, 1);
    EXPECT_EQ(kImuSourceBrainVex, 2);
    EXPECT_EQ(kWheelReversed, 0x01);
    EXPECT_EQ(kImuInvert, 0x01);
}

TEST(Profile, KnownBytes) {
    const RobotProfileDoc p = knownProfile();

    Le e;
    e.u8(1).u8(kTopologyTwoWheelImu).u8(2).u8(0).u8(kImuSourceBrainVex).u8(0).u8(1).u8(0);
    e.i32(200000).i32(150000).i32(180000).i32(180000);
    e.u16(2000).u16(100).u16(1000).u16(0);
    e.u8(0).u8(0).u16(0).u32(4000).u32(24000).i32(0).i32(100000).i32(0);
    e.u32(1000000).u32(1000000);
    e.u8(1).u8(kWheelReversed).u16(0).u32(8192).u32(24000).i32(-50000).i32(0).i32(90000);
    e.u32(2500000).u32(1012345);
    ASSERT_EQ(e.b.size(), 96u);
    ASSERT_EQ(e.b, kProfileVector);
    ASSERT_EQ(refCrc32(kProfileVector.data(), kProfileVector.size()), kProfileVectorCrc);

    const Bytes doc = encodeProfile(p);
    EXPECT_EQ(doc, kProfileVector);
    EXPECT_EQ(robotProfileLen(p), 96);
    EXPECT_EQ(crc32(doc.data(), static_cast<uint32_t>(doc.size())), kProfileVectorCrc);

    RobotProfileDoc out;
    ASSERT_TRUE(decodeRobotProfile(kProfileVector.data(),
                                   static_cast<uint16_t>(kProfileVector.size()), out));
    expectSameProfile(out, p);
    expectAccepted(out);
}

TEST(Profile, HeaderAndWheelOffsets) {
    RobotProfileDoc p            = profile(kTopologyTwoWheelImu, kImuSourcePico);
    p.calibration_window_ms      = 0x1A19;
    p.still_rate_cdps            = 0x1C1B;
    p.still_travel_um            = 0x1E1D;
    p.wheels[0].gear_micro       = 0x24232221;
    p.wheels[0].travel_scale_ppm = 0x28272625;
    const Bytes doc              = encodeProfile(p);
    ASSERT_EQ(doc.size(), 96u);
    EXPECT_EQ(doc[24], 0x19);
    EXPECT_EQ(doc[25], 0x1A);
    EXPECT_EQ(doc[26], 0x1B);
    EXPECT_EQ(doc[27], 0x1C);
    EXPECT_EQ(doc[28], 0x1D);
    EXPECT_EQ(doc[29], 0x1E);
    EXPECT_EQ(doc[30], 0x00);
    EXPECT_EQ(doc[31], 0x00);
    EXPECT_EQ(Bytes(doc.begin() + 32 + 24, doc.begin() + 32 + 32),
              (Bytes{0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28}));
    // The second wheel starts right after the first.
    EXPECT_EQ(doc[64], 1);
}

TEST(Profile, RoundTripWithMaxCounts) {
    RobotProfileDoc p      = profile(kTopologyThreeWheel, kImuSourcePico);
    p.imu_flags            = kImuInvert;
    p.wheels[0].x_um       = -1000000;
    p.wheels[1].y_um       = 1000000;
    p.wheels[2].angle_mdeg = -270000;
    p.wheels[0].gear_micro       = 100000;
    p.wheels[1].gear_micro       = 10000000;
    p.wheels[0].travel_scale_ppm = 900000;
    p.wheels[2].travel_scale_ppm = 1100000;
    p.calibration_window_ms      = 20000;
    p.still_rate_cdps            = 2000;
    p.still_travel_um            = 5000;
    p.camera_count               = kProfileMaxCameras;
    for (uint8_t i = 0; i < kProfileMaxCameras; ++i) {
        p.cameras[i]      = camera(static_cast<uint8_t>(3 - i));
        p.cameras[i].z_um = -2000000 + i;
    }

    const Bytes doc = encodeProfile(p);
    ASSERT_EQ(doc.size(), kProfileMaxLen);
    RobotProfileDoc out;
    ASSERT_TRUE(decodeRobotProfile(doc.data(), static_cast<uint16_t>(doc.size()), out));
    expectSameProfile(out, p);
    expectAccepted(out);
}

TEST(Profile, LengthFollowsCounts) {
    RobotProfileDoc p;
    for (uint8_t w = 0; w <= kProfileMaxWheels + 1; ++w) {
        for (uint8_t c = 0; c <= kProfileMaxCameras + 1; ++c) {
            p.wheel_count  = w;
            p.camera_count = c;
            const uint16_t expected =
                (w > kProfileMaxWheels || c > kProfileMaxCameras) ? 0 : 32 + 32 * w + 28 * c;
            EXPECT_EQ(robotProfileLen(p), expected) << int(w) << " wheels " << int(c);
        }
    }
}

TEST(Profile, EncoderRefusesOverCapsAndSmallBuffers) {
    uint8_t         buf[kProfileMaxLen + 32];
    RobotProfileDoc p = profile(kTopologyThreeWheel, kImuSourceNone);
    EXPECT_EQ(encodeRobotProfile(p, buf, 127), 0);
    EXPECT_EQ(encodeRobotProfile(p, buf, 128), 128);

    p.wheel_count = kProfileMaxWheels + 1;
    EXPECT_EQ(encodeRobotProfile(p, buf, sizeof(buf)), 0);
    p.wheel_count  = 3;
    p.camera_count = kProfileMaxCameras + 1;
    EXPECT_EQ(encodeRobotProfile(p, buf, sizeof(buf)), 0);
}

TEST(Profile, DecodeRejectsMalformedDocuments) {
    RobotProfileDoc p = profile(kTopologyThreeWheel, kImuSourcePico);
    p.camera_count    = 2;
    p.cameras[0]      = camera(0);
    p.cameras[1]      = camera(1);
    const Bytes good  = encodeProfile(p);
    ASSERT_EQ(good.size(), 32u + 96u + 56u);

    RobotProfileDoc out;
    for (size_t n = 0; n < good.size(); ++n) {
        EXPECT_FALSE(decodeRobotProfile(good.data(), static_cast<uint16_t>(n), out)) << n;
    }
    Bytes padded = good;
    padded.push_back(0);
    EXPECT_FALSE(decodeRobotProfile(padded.data(), static_cast<uint16_t>(padded.size()), out));

    const auto rejects = [&](size_t at, uint8_t value) {
        Bytes bad = good;
        bad[at]   = value;
        return !decodeRobotProfile(bad.data(), static_cast<uint16_t>(bad.size()), out);
    };
    EXPECT_TRUE(rejects(0, 0));  // format
    EXPECT_TRUE(rejects(0, 2));  // format
    EXPECT_TRUE(rejects(2, 2));  // wheel count no longer matches the length
    EXPECT_TRUE(rejects(2, 4));  // wheel count over the cap
    EXPECT_TRUE(rejects(3, 3));  // camera count no longer matches the length
    EXPECT_TRUE(rejects(3, 5));  // camera count over the cap
    EXPECT_TRUE(rejects(30, 1));    // header reserved
    EXPECT_TRUE(rejects(31, 0x80)); // header reserved
    for (size_t w = 0; w < 3; ++w) {
        EXPECT_TRUE(rejects(32 + 32 * w + 2, 1)) << "wheel reserved " << w;
        EXPECT_TRUE(rejects(32 + 32 * w + 3, 1)) << "wheel reserved " << w;
    }
    for (size_t c = 0; c < 2; ++c) {
        for (size_t k = 1; k <= 3; ++k) {
            EXPECT_TRUE(rejects(128 + 28 * c + k, 0x80)) << "camera reserved " << c;
        }
    }
    // Every other byte from the IMU source on is a value, not a structural field.
    for (size_t at = 4; at < good.size(); ++at) {
        const bool wheel_reserved =
            at >= 32 && at < 128 && ((at - 32) % 32 == 2 || (at - 32) % 32 == 3);
        const bool camera_reserved = at >= 128 && (at - 128) % 28 >= 1 && (at - 128) % 28 <= 3;
        if (at != 30 && at != 31 && !wheel_reserved && !camera_reserved) {
            EXPECT_FALSE(rejects(at, static_cast<uint8_t>(good[at] ^ 0x10))) << "byte " << at;
        }
    }
    EXPECT_TRUE(decodeRobotProfile(good.data(), static_cast<uint16_t>(good.size()), out));
}

TEST(Profile, DecodeLeavesSemanticChecksToTheValidator) {
    RobotProfileDoc p = profile(kTopologyTwoWheelImu, kImuSourcePico);
    p.topology        = 99;
    p.imu_source      = 7;
    p.wheels[0].radius_um = 0;
    p.wheels[1].gear_micro       = 0;
    p.wheels[1].travel_scale_ppm = 0xFFFFFFFF;
    p.calibration_window_ms      = 1;
    const Bytes doc   = encodeProfile(p);
    RobotProfileDoc out;
    ASSERT_TRUE(decodeRobotProfile(doc.data(), static_cast<uint16_t>(doc.size()), out));
    EXPECT_EQ(out.topology, 99);
    EXPECT_EQ(out.imu_source, 7);
    EXPECT_EQ(out.wheels[1].gear_micro, 0u);
    EXPECT_EQ(out.wheels[1].travel_scale_ppm, 0xFFFFFFFFu);
    EXPECT_EQ(out.calibration_window_ms, 1);
    expectRejected(out, kProfileReasonTopology);
}

// ---------------------------------------------------------------------------
// Robot profile validation
// ---------------------------------------------------------------------------

TEST(ProfileValidate, EveryTopologyAndImuSourceCombination) {
    const struct {
        uint8_t topology, imu_source, reason;
    } cases[] = {
        {kTopologyTwoWheelImu, kImuSourceNone, kProfileReasonImuCombination},
        {kTopologyTwoWheelImu, kImuSourcePico, kProfileReasonNone},
        {kTopologyTwoWheelImu, kImuSourceBrainVex, kProfileReasonNone},
        {kTopologyTwoForwardWheelImu, kImuSourceNone, kProfileReasonImuCombination},
        {kTopologyTwoForwardWheelImu, kImuSourcePico, kProfileReasonNone},
        {kTopologyTwoForwardWheelImu, kImuSourceBrainVex, kProfileReasonNone},
        {kTopologyThreeWheel, kImuSourceNone, kProfileReasonNone},
        {kTopologyThreeWheel, kImuSourcePico, kProfileReasonNone},
        {kTopologyThreeWheel, kImuSourceBrainVex, kProfileReasonImuCombination},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(testing::Message() << "topology " << int(c.topology) << " imu "
                                        << int(c.imu_source));
        const RobotProfileDoc p   = profile(c.topology, c.imu_source);
        const Bytes           doc = encodeProfile(p);
        ASSERT_EQ(doc.size(), 32u + 32u * p.wheel_count);
        RobotProfileDoc out;
        ASSERT_TRUE(decodeRobotProfile(doc.data(), static_cast<uint16_t>(doc.size()), out));
        expectSameProfile(out, p);
        if (c.reason == kProfileReasonNone) {
            expectAccepted(out);
        } else {
            expectRejected(out, c.reason);
        }
    }
}

TEST(ProfileValidate, SuccessClearsReasonAndDetail) {
    uint8_t reason = 7;
    uint8_t detail = 7;
    EXPECT_TRUE(
        validateRobotProfile(profile(kTopologyThreeWheel, kImuSourceNone), reason, detail));
    EXPECT_EQ(reason, kProfileReasonNone);
    EXPECT_EQ(detail, 0);
}

TEST(ProfileValidate, Format) {
    RobotProfileDoc p = profile(kTopologyTwoWheelImu, kImuSourcePico);
    p.format          = 2;
    expectRejected(p, kProfileReasonFormat);
}

TEST(ProfileValidate, Topology) {
    for (uint8_t t : {0, 4, 255}) {
        RobotProfileDoc p = profile(kTopologyTwoWheelImu, kImuSourcePico);
        p.topology        = t;
        expectRejected(p, kProfileReasonTopology);
    }
}

TEST(ProfileValidate, WheelCount) {
    RobotProfileDoc two = profile(kTopologyTwoWheelImu, kImuSourcePico);
    two.wheel_count     = 3;
    two.wheels[2]       = wheel(2, 0, 0, 45000);
    expectRejected(two, kProfileReasonWheelCount);
    two.wheel_count = 1;
    expectRejected(two, kProfileReasonWheelCount);

    RobotProfileDoc fwd = profile(kTopologyTwoForwardWheelImu, kImuSourcePico);
    fwd.wheel_count     = 0;
    expectRejected(fwd, kProfileReasonWheelCount);

    RobotProfileDoc three = profile(kTopologyThreeWheel, kImuSourceNone);
    three.wheel_count     = 2;
    expectRejected(three, kProfileReasonWheelCount);
    three.wheel_count = 9;
    expectRejected(three, kProfileReasonWheelCount);
}

TEST(ProfileValidate, EncoderPortUsedTwice) {
    RobotProfileDoc two        = profile(kTopologyTwoWheelImu, kImuSourcePico);
    two.wheels[1].encoder_port = 0;
    expectRejected(two, kProfileReasonEncoderPort, 1);

    RobotProfileDoc three        = profile(kTopologyThreeWheel, kImuSourcePico);
    three.wheels[2].encoder_port = 0;
    expectRejected(three, kProfileReasonEncoderPort, 2);
    three.wheels[2].encoder_port = 1;
    expectRejected(three, kProfileReasonEncoderPort, 2);

    // Which ports this Pi has wired is its own check.
    three.wheels[2].encoder_port = 7;
    expectAccepted(three);
}

TEST(ProfileValidate, WheelGeometryRangesAndFlags) {
    const struct {
        const char* what;
        void (*set)(ProfileWheel&);
        bool ok;
    } cases[] = {
        {"counts 0", [](ProfileWheel& w) { w.counts_per_rev = 0; }, false},
        {"counts 1", [](ProfileWheel& w) { w.counts_per_rev = 1; }, true},
        {"counts max", [](ProfileWheel& w) { w.counts_per_rev = 1000000; }, true},
        {"counts over", [](ProfileWheel& w) { w.counts_per_rev = 1000001; }, false},
        {"radius under", [](ProfileWheel& w) { w.radius_um = 999; }, false},
        {"radius min", [](ProfileWheel& w) { w.radius_um = 1000; }, true},
        {"radius max", [](ProfileWheel& w) { w.radius_um = 200000; }, true},
        {"radius over", [](ProfileWheel& w) { w.radius_um = 200001; }, false},
        {"x max", [](ProfileWheel& w) { w.x_um = 1000000; }, true},
        {"x over", [](ProfileWheel& w) { w.x_um = 1000001; }, false},
        {"x under", [](ProfileWheel& w) { w.x_um = -1000001; }, false},
        {"y min", [](ProfileWheel& w) { w.y_um = -1000000; }, true},
        {"y over", [](ProfileWheel& w) { w.y_um = 1000001; }, false},
        {"y int min", [](ProfileWheel& w) { w.y_um = INT32_MIN; }, false},
        {"angle over", [](ProfileWheel& w) { w.angle_mdeg = 360001; }, false},
        {"angle under", [](ProfileWheel& w) { w.angle_mdeg = -360001; }, false},
        {"reversed", [](ProfileWheel& w) { w.flags = kWheelReversed; }, true},
        {"unknown flag", [](ProfileWheel& w) { w.flags = 0x02; }, false},
        {"unknown flag high", [](ProfileWheel& w) { w.flags = 0x81; }, false},
        {"gear 0", [](ProfileWheel& w) { w.gear_micro = 0; }, false},
        {"gear under", [](ProfileWheel& w) { w.gear_micro = 99999; }, false},
        {"gear min", [](ProfileWheel& w) { w.gear_micro = 100000; }, true},
        {"gear max", [](ProfileWheel& w) { w.gear_micro = 10000000; }, true},
        {"gear over", [](ProfileWheel& w) { w.gear_micro = 10000001; }, false},
        {"gear u32 max", [](ProfileWheel& w) { w.gear_micro = 0xFFFFFFFF; }, false},
        {"scale 0", [](ProfileWheel& w) { w.travel_scale_ppm = 0; }, false},
        {"scale under", [](ProfileWheel& w) { w.travel_scale_ppm = 899999; }, false},
        {"scale min", [](ProfileWheel& w) { w.travel_scale_ppm = 900000; }, true},
        {"scale max", [](ProfileWheel& w) { w.travel_scale_ppm = 1100000; }, true},
        {"scale over", [](ProfileWheel& w) { w.travel_scale_ppm = 1100001; }, false},
        {"scale u32 max", [](ProfileWheel& w) { w.travel_scale_ppm = 0xFFFFFFFF; }, false},
    };
    for (const auto& c : cases) {
        for (uint8_t i = 0; i < 3; ++i) {
            SCOPED_TRACE(testing::Message() << c.what << ", wheel " << int(i));
            RobotProfileDoc p = profile(kTopologyThreeWheel, kImuSourcePico);
            c.set(p.wheels[i]);
            if (c.ok) {
                expectAccepted(p);
            } else {
                expectRejected(p, kProfileReasonWheelGeometry, i);
            }
        }
    }
}

TEST(ProfileValidate, AnglesWithinRangeKeepTheirObservability) {
    // A full turn either way is the same direction.
    RobotProfileDoc p      = profile(kTopologyTwoWheelImu, kImuSourcePico);
    p.wheels[0].angle_mdeg = 360000;
    p.wheels[1].angle_mdeg = -270000;
    expectAccepted(p);
}

TEST(ProfileValidate, TwoWheelObservabilityEdges) {
    const struct {
        int32_t a0, a1;
        bool    ok;
    } cases[] = {
        {0, 90000, true},      {0, -90000, true},      {45000, 135000, true},
        {0, 0, false},         {0, 180000, false},     {90000, -90000, false},
        {0, 58, true},         {0, 57, false},         {90000, 90058, true},
        {90000, 90057, false}, {0, 179942, true},      {0, 179943, false},
        {-45000, -44942, true}, {-45000, -44943, false},
    };
    for (const auto& c : cases) {
        for (uint8_t imu : {kImuSourcePico, kImuSourceBrainVex}) {
            for (uint8_t flags : {0, int(kWheelReversed)}) {
                SCOPED_TRACE(testing::Message() << c.a0 << " " << c.a1 << " imu " << int(imu)
                                                << " flags " << int(flags));
                RobotProfileDoc p      = profile(kTopologyTwoWheelImu, imu);
                p.wheels[0].angle_mdeg = c.a0;
                p.wheels[1].angle_mdeg = c.a1;
                p.wheels[1].flags      = flags;
                if (c.ok) {
                    expectAccepted(p);
                } else {
                    expectRejected(p, kProfileReasonObservability);
                }
            }
        }
    }
}

TEST(ProfileValidate, TwoWheelObservabilityIgnoresOffsets) {
    RobotProfileDoc p = profile(kTopologyTwoWheelImu, kImuSourcePico);
    p.wheels[0].x_um  = 0;
    p.wheels[0].y_um  = 0;
    p.wheels[1].x_um  = 0;
    p.wheels[1].y_um  = 0;
    expectAccepted(p);
}

TEST(ProfileValidate, TwoForwardWheelAnglesOnly) {
    const struct {
        int32_t a0, a1;
        bool    ok;
    } cases[] = {
        {0, 0, true},          {0, 180000, true},  {-180000, 0, true}, {180000, -180000, true},
        {0, 1, false},         {0, 179999, false}, {90000, 0, false},  {0, -90000, false},
        {360000, 0, false},
    };
    for (const auto& c : cases) {
        for (uint8_t imu : {kImuSourcePico, kImuSourceBrainVex}) {
            SCOPED_TRACE(testing::Message() << c.a0 << " " << c.a1 << " imu " << int(imu));
            RobotProfileDoc p      = profile(kTopologyTwoForwardWheelImu, imu);
            p.wheels[0].angle_mdeg = c.a0;
            p.wheels[1].angle_mdeg = c.a1;
            if (c.ok) {
                expectAccepted(p);
            } else {
                expectRejected(p, kProfileReasonObservability);
            }
        }
    }
}

TEST(ProfileValidate, ThreeWheelObservabilityEdges) {
    // Two forward wheels at y0, y1 and a sideways wheel: det [ux uy k] = y1 - y0 in m.
    const struct {
        int32_t y0, y1;
        bool    ok;
    } forward_pairs[] = {
        {0, 33, true}, {0, 31, false}, {150000, 150000, false}, {-150000, 150000, true},
        {10, -23, true}, {10, -21, false},
    };
    for (const auto& c : forward_pairs) {
        for (uint8_t imu : {kImuSourceNone, kImuSourcePico}) {
            SCOPED_TRACE(testing::Message() << c.y0 << " " << c.y1 << " imu " << int(imu));
            RobotProfileDoc p = profile(kTopologyThreeWheel, imu);
            p.wheels[0]       = wheel(0, 0, c.y0, 0);
            p.wheels[1]       = wheel(1, 0, c.y1, 0);
            p.wheels[2]       = wheel(2, -100000, 0, 90000);
            if (c.ok) {
                expectAccepted(p);
            } else {
                expectRejected(p, kProfileReasonObservability);
            }
        }
    }

    // All wheels along one axis never see the other.
    RobotProfileDoc forward = profile(kTopologyThreeWheel, kImuSourcePico);
    forward.wheels[2]       = wheel(2, -100000, 0, 0);
    expectRejected(forward, kProfileReasonObservability);

    // Two sideways wheels at the same x cannot separate rotation from sideways travel.
    RobotProfileDoc sideways = profile(kTopologyThreeWheel, kImuSourcePico);
    sideways.wheels[1]       = wheel(1, -100000, 50000, 90000);
    expectRejected(sideways, kProfileReasonObservability);
    sideways.wheels[1].x_um = 100000;
    expectAccepted(sideways);
}

TEST(ProfileValidate, ImuSource) {
    RobotProfileDoc unknown = profile(kTopologyTwoWheelImu, kImuSourcePico);
    unknown.imu_source      = 3;
    expectRejected(unknown, kProfileReasonImuSource);
    unknown.imu_source = 255;
    expectRejected(unknown, kProfileReasonImuSource);

    RobotProfileDoc pico = profile(kTopologyTwoWheelImu, kImuSourcePico);
    pico.imu_flags       = kImuInvert;
    expectAccepted(pico);
    pico.imu_flags = 0x02;
    expectRejected(pico, kProfileReasonImuSource);

    // The VEX sign is converted on the Brain, so no invert flag.
    RobotProfileDoc vex = profile(kTopologyTwoWheelImu, kImuSourceBrainVex);
    vex.imu_flags       = kImuInvert;
    expectRejected(vex, kProfileReasonImuSource);

    RobotProfileDoc none = profile(kTopologyThreeWheel, kImuSourceNone);
    none.imu_flags       = kImuInvert;
    expectRejected(none, kProfileReasonImuSource);
}

TEST(ProfileValidate, ImuPort) {
    RobotProfileDoc none = profile(kTopologyThreeWheel, kImuSourceNone);
    none.imu_port        = 1;
    expectRejected(none, kProfileReasonImuPort);
    none.imu_port       = 0;
    none.vex_smart_port = 1;
    expectRejected(none, kProfileReasonImuPort);

    RobotProfileDoc pico = profile(kTopologyThreeWheel, kImuSourcePico);
    pico.vex_smart_port  = 1;
    expectRejected(pico, kProfileReasonImuPort);
    // Which Pico IMU ports exist is the Pi's check.
    pico.vex_smart_port = 0;
    pico.imu_port       = 3;
    expectAccepted(pico);

    for (uint8_t port : {1, 21}) {
        RobotProfileDoc vex = profile(kTopologyTwoForwardWheelImu, kImuSourceBrainVex);
        vex.vex_smart_port  = port;
        expectAccepted(vex);
    }
    for (uint8_t port : {0, 22, 255}) {
        RobotProfileDoc vex = profile(kTopologyTwoForwardWheelImu, kImuSourceBrainVex);
        vex.vex_smart_port  = port;
        expectRejected(vex, kProfileReasonImuPort);
    }
    RobotProfileDoc vex = profile(kTopologyTwoWheelImu, kImuSourceBrainVex);
    vex.imu_port        = 1;
    expectRejected(vex, kProfileReasonImuPort);
}

TEST(ProfileValidate, ImuCombination) {
    expectRejected(profile(kTopologyTwoWheelImu, kImuSourceNone), kProfileReasonImuCombination);
    expectRejected(profile(kTopologyTwoForwardWheelImu, kImuSourceNone),
                   kProfileReasonImuCombination);
    // The VEX IMU runs on the Brain clock; three wheel fusion needs Pico timing.
    expectRejected(profile(kTopologyThreeWheel, kImuSourceBrainVex),
                   kProfileReasonImuCombination);
}

TEST(ProfileValidate, Camera) {
    RobotProfileDoc p = profile(kTopologyTwoWheelImu, kImuSourcePico);
    p.camera_count    = kProfileMaxCameras;
    for (uint8_t i = 0; i < kProfileMaxCameras; ++i) {
        p.cameras[i] = camera(i);
    }
    expectAccepted(p);

    RobotProfileDoc over = p;
    over.camera_count    = kProfileMaxCameras + 1;
    expectRejected(over, kProfileReasonCamera);

    RobotProfileDoc dup = p;
    dup.cameras[3].slot = 1;
    expectRejected(dup, kProfileReasonCamera, 3);

    const struct {
        const char* what;
        void (*set)(ProfileCamera&);
        bool ok;
    } cases[] = {
        {"x max", [](ProfileCamera& c) { c.x_um = 2000000; }, true},
        {"x over", [](ProfileCamera& c) { c.x_um = 2000001; }, false},
        {"y under", [](ProfileCamera& c) { c.y_um = -2000001; }, false},
        {"z over", [](ProfileCamera& c) { c.z_um = 2000001; }, false},
        {"roll max", [](ProfileCamera& c) { c.roll_mdeg = -360000; }, true},
        {"roll over", [](ProfileCamera& c) { c.roll_mdeg = 360001; }, false},
        {"pitch over", [](ProfileCamera& c) { c.pitch_mdeg = -360001; }, false},
        {"yaw over", [](ProfileCamera& c) { c.yaw_mdeg = 360001; }, false},
    };
    for (const auto& c : cases) {
        for (uint8_t i = 0; i < kProfileMaxCameras; ++i) {
            SCOPED_TRACE(testing::Message() << c.what << ", camera " << int(i));
            RobotProfileDoc q = p;
            c.set(q.cameras[i]);
            if (c.ok) {
                expectAccepted(q);
            } else {
                expectRejected(q, kProfileReasonCamera, i);
            }
        }
    }
}

TEST(ProfileValidate, Footprint) {
    const struct {
        int32_t front, back, left, right;
        bool    ok;
    } cases[] = {
        {200000, 200000, 180000, 180000, true},
        {1, 0, 0, 1, true},
        {2000000, 2000000, 2000000, 2000000, true},
        {0, 0, 180000, 180000, false},
        {200000, 200000, 0, 0, false},
        {-1, 200000, 180000, 180000, false},
        {200000, -1, 180000, 180000, false},
        {200000, 200000, -1, 180000, false},
        {200000, 200000, 180000, -1, false},
        {2000001, 0, 180000, 180000, false},
        {200000, 200000, 180000, 2000001, false},
        {INT32_MIN, INT32_MAX, 1, 1, false},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(testing::Message()
                     << c.front << " " << c.back << " " << c.left << " " << c.right);
        RobotProfileDoc p    = profile(kTopologyTwoWheelImu, kImuSourceBrainVex);
        p.footprint_front_um = c.front;
        p.footprint_back_um  = c.back;
        p.footprint_left_um  = c.left;
        p.footprint_right_um = c.right;
        if (c.ok) {
            expectAccepted(p);
        } else {
            expectRejected(p, kProfileReasonFootprint);
        }
    }
}

TEST(ProfileValidate, CalibrationSettings) {
    const struct {
        const char* what;
        uint16_t    window, rate, travel;
        bool        ok;
    } cases[] = {
        {"defaults", 0, 0, 0, true},
        {"window min", 500, 0, 0, true},
        {"window under", 499, 0, 0, false},
        {"window 1", 1, 0, 0, false},
        {"window max", 20000, 0, 0, true},
        {"window over", 20001, 0, 0, false},
        {"window u16 max", 65535, 0, 0, false},
        {"rate min", 0, 10, 0, true},
        {"rate under", 0, 9, 0, false},
        {"rate max", 0, 2000, 0, true},
        {"rate over", 0, 2001, 0, false},
        {"travel min", 0, 0, 20, true},
        {"travel under", 0, 0, 19, false},
        {"travel max", 0, 0, 5000, true},
        {"travel over", 0, 0, 5001, false},
        {"all set", 2000, 100, 1000, true},
        {"one bad of three", 2000, 100, 5001, false},
    };
    for (const auto& c : cases) {
        for (uint8_t topology :
             {kTopologyTwoWheelImu, kTopologyTwoForwardWheelImu, kTopologyThreeWheel}) {
            SCOPED_TRACE(testing::Message() << c.what << ", topology " << int(topology));
            RobotProfileDoc p       = profile(topology, kImuSourcePico);
            p.calibration_window_ms = c.window;
            p.still_rate_cdps       = c.rate;
            p.still_travel_um       = c.travel;
            if (c.ok) {
                expectAccepted(p);
            } else {
                expectRejected(p, kProfileReasonCalibration);
            }
            // The settings survive the document.
            const Bytes     doc = encodeProfile(p);
            RobotProfileDoc out;
            ASSERT_TRUE(decodeRobotProfile(doc.data(), static_cast<uint16_t>(doc.size()), out));
            EXPECT_EQ(out.calibration_window_ms, c.window);
            EXPECT_EQ(out.still_rate_cdps, c.rate);
            EXPECT_EQ(out.still_travel_um, c.travel);
        }
    }
}

TEST(ProfileValidate, CalibrationSettingsApplyToEveryImuSource) {
    // The VEX and no-IMU profiles still carry the wheel stillness settings.
    for (uint8_t imu : {kImuSourceBrainVex, kImuSourceNone}) {
        const uint8_t topology = imu == kImuSourceNone ? kTopologyThreeWheel : kTopologyTwoWheelImu;
        RobotProfileDoc p      = profile(topology, imu);
        p.still_travel_um      = 19;
        expectRejected(p, kProfileReasonCalibration);
        p.still_travel_um = 20;
        expectAccepted(p);
    }
}

TEST(ProfileValidate, PolarityIsNotDirection) {
    // kWheelReversed flips the count sign only; the measuring direction and
    // so observability do not change.
    RobotProfileDoc p = profile(kTopologyTwoForwardWheelImu, kImuSourceBrainVex);
    p.wheels[0].flags = kWheelReversed;
    p.wheels[1].flags = kWheelReversed;
    expectAccepted(p);
    p.wheels[1].angle_mdeg = 90000;
    expectRejected(p, kProfileReasonObservability);
}

TEST(ProfileValidate, PiOnlyReasonsAreNeverSharedVerdicts) {
    // Build and NotAccepted come from the Pi alone; the shared rules above
    // cover every other reason.
    const uint8_t topologies[] = {kTopologyTwoWheelImu, kTopologyThreeWheel,
                                  kTopologyTwoForwardWheelImu};
    for (uint8_t topology : topologies) {
        for (uint8_t imu = 0; imu < 4; ++imu) {
            const Verdict v = validate(profile(topology, imu));
            EXPECT_NE(v.reason, kProfileReasonBuild);
            EXPECT_NE(v.reason, kProfileReasonNotAccepted);
        }
    }
}

// ---------------------------------------------------------------------------
// Field map
// ---------------------------------------------------------------------------

TEST(FieldMap, LayoutConstants) {
    EXPECT_EQ(kFieldMapFormat, 1);
    EXPECT_EQ(kFieldMaxObjects, 128);
    EXPECT_EQ(kFieldMapHeaderLen, 24);
    EXPECT_EQ(kFieldMapRecordLen, 28);
    EXPECT_EQ(kFieldMapMaxLen, 3608);
    EXPECT_EQ(kObjectLandmark, 1);
    EXPECT_EQ(kObjectFixed, 2);
    EXPECT_EQ(kObjectObstacle, 0x01);
    EXPECT_EQ(kObjectEstimated, 0x02);
    EXPECT_EQ(kObjectReference, 0x04);
    EXPECT_EQ(fieldMapLen(0), 24);
    EXPECT_EQ(fieldMapLen(1), 52);
    EXPECT_EQ(fieldMapLen(kFieldMaxObjects), kFieldMapMaxLen);
    EXPECT_EQ(fieldMapLen(kFieldMaxObjects + 1), 0);
    // Three objects already need more than one READ_DOC chunk.
    EXPECT_GT(fieldMapLen(3), kDocChunkMax);
}

TEST(FieldMap, KnownBytes) {
    FieldObjectRecord r;
    r.object_id        = 0x0102;
    r.kind             = kObjectLandmark;
    r.flags            = kObjectObstacle | kObjectEstimated | kObjectReference;
    r.x_mm             = 1829;
    r.y_mm             = -3;
    r.heading_cdeg     = -9000;
    r.box_x_mm         = -12;
    r.box_y_mm         = 7;
    r.box_heading_cdeg = 4500;
    r.box_length_mm    = 154;
    r.box_width_mm     = 60000;
    FieldMapHeader h   = mapHeader(1);
    h.min_x_mm         = -10;

    Le e;
    e.u8(1).u8(28).u16(3).u16(1).u16(0).i32(-10).i32(0).i32(3658).i32(3658);
    e.u16(0x0102).u8(kObjectLandmark).u8(0x07).i32(1829).i32(-3).i32(-9000);
    e.i16(-12).i16(7).i16(4500).u16(154).u16(60000).u16(0);

    const Bytes doc = buildMap(h, {r});
    EXPECT_EQ(doc, e.b);
    EXPECT_EQ(validateMap(doc), DocError::kNone);
    EXPECT_EQ(mapIdOf(doc), refCrc32(e.b.data(), e.b.size()));

    FieldMapHeader hd;
    ASSERT_TRUE(decodeFieldMapHeader(doc.data(), static_cast<uint16_t>(doc.size()), hd));
    EXPECT_EQ(hd.format, 1);
    EXPECT_EQ(hd.revision, 3);
    EXPECT_EQ(hd.object_count, 1);
    EXPECT_EQ(hd.min_x_mm, -10);
    EXPECT_EQ(hd.max_y_mm, 3658);

    FieldObjectRecord o;
    ASSERT_TRUE(decodeFieldObjectRecord(doc.data(), static_cast<uint16_t>(doc.size()), 0, o));
    EXPECT_EQ(o.object_id, 0x0102);
    EXPECT_EQ(o.kind, kObjectLandmark);
    EXPECT_EQ(o.flags, 0x07);
    EXPECT_EQ(o.x_mm, 1829);
    EXPECT_EQ(o.y_mm, -3);
    EXPECT_EQ(o.heading_cdeg, -9000);
    EXPECT_EQ(o.box_x_mm, -12);
    EXPECT_EQ(o.box_y_mm, 7);
    EXPECT_EQ(o.box_heading_cdeg, 4500);
    EXPECT_EQ(o.box_length_mm, 154);
    EXPECT_EQ(o.box_width_mm, 60000);
}

TEST(FieldMap, RoundTripForEveryCountShape) {
    for (uint16_t count : {0, 1, 2, 7, 37, 127, 128}) {
        SCOPED_TRACE(testing::Message() << count << " objects");
        const std::vector<FieldObjectRecord> objs = objects(count);
        const Bytes                          doc  = buildMap(mapHeader(count), objs);
        ASSERT_EQ(doc.size(), fieldMapLen(count));
        EXPECT_EQ(validateMap(doc), DocError::kNone);

        FieldMapHeader h;
        ASSERT_TRUE(decodeFieldMapHeader(doc.data(), static_cast<uint16_t>(doc.size()), h));
        EXPECT_EQ(h.object_count, count);
        for (uint16_t i = 0; i < count; ++i) {
            FieldObjectRecord o;
            ASSERT_TRUE(
                decodeFieldObjectRecord(doc.data(), static_cast<uint16_t>(doc.size()), i, o));
            EXPECT_EQ(o.object_id, objs[i].object_id);
            EXPECT_EQ(o.kind, objs[i].kind);
            EXPECT_EQ(o.flags, objs[i].flags);
            EXPECT_EQ(o.x_mm, objs[i].x_mm);
            EXPECT_EQ(o.y_mm, objs[i].y_mm);
            EXPECT_EQ(o.heading_cdeg, objs[i].heading_cdeg);
            EXPECT_EQ(o.box_x_mm, objs[i].box_x_mm);
            EXPECT_EQ(o.box_width_mm, objs[i].box_width_mm);
        }
        FieldObjectRecord past;
        EXPECT_FALSE(
            decodeFieldObjectRecord(doc.data(), static_cast<uint16_t>(doc.size()), count, past));
    }
}

TEST(FieldMap, IdsCoverTheWholeRange) {
    std::vector<FieldObjectRecord> objs = objects(2);
    objs[0].object_id                   = 1;
    objs[1].object_id                   = 65535;
    EXPECT_EQ(validateMap(buildMap(mapHeader(2), objs)), DocError::kNone);
}

TEST(FieldMap, WritersRefuseBadCountIndexOrCapacity) {
    Bytes doc(kFieldMapMaxLen + 28);
    EXPECT_FALSE(encodeFieldMapHeader(mapHeader(kFieldMaxObjects + 1), doc.data(),
                                      static_cast<uint16_t>(doc.size())));
    EXPECT_FALSE(encodeFieldMapHeader(mapHeader(2), doc.data(), fieldMapLen(2) - 1));
    EXPECT_TRUE(encodeFieldMapHeader(mapHeader(2), doc.data(), fieldMapLen(2)));

    const FieldObjectRecord r = object(0, 1);
    EXPECT_FALSE(encodeFieldObjectRecord(r, kFieldMaxObjects, doc.data(),
                                         static_cast<uint16_t>(doc.size())));
    EXPECT_FALSE(encodeFieldObjectRecord(r, 1, doc.data(), fieldMapLen(2) - 1));
    EXPECT_TRUE(encodeFieldObjectRecord(r, 1, doc.data(), fieldMapLen(2)));
}

TEST(FieldMap, HeaderDecodeRejectsMalformedCounts) {
    const Bytes    good = buildMap(3);
    FieldMapHeader h;
    for (size_t n = 0; n < good.size(); ++n) {
        EXPECT_FALSE(decodeFieldMapHeader(good.data(), static_cast<uint16_t>(n), h)) << n;
    }
    Bytes longer = good;
    longer.push_back(0);
    EXPECT_FALSE(decodeFieldMapHeader(longer.data(), static_cast<uint16_t>(longer.size()), h));

    for (uint16_t count : {0, 2, 4, 129, 0xFFFF}) {
        Bytes bad = good;
        bad[4]    = static_cast<uint8_t>(count);
        bad[5]    = static_cast<uint8_t>(count >> 8);
        EXPECT_FALSE(decodeFieldMapHeader(bad.data(), static_cast<uint16_t>(bad.size()), h))
            << count;
    }
}

TEST(FieldMap, ValidateLengthAndFormat) {
    const Bytes good = buildMap(3);
    EXPECT_EQ(validateFieldMap(nullptr, 0), DocError::kLength);
    EXPECT_EQ(validateFieldMap(good.data(), 23), DocError::kLength);
    EXPECT_EQ(validateFieldMap(good.data(), static_cast<uint16_t>(good.size() - 1)),
              DocError::kLength);
    EXPECT_EQ(validateFieldMap(good.data(), static_cast<uint16_t>(good.size() - 28)),
              DocError::kLength);
    Bytes longer = good;
    longer.insert(longer.end(), 28, 0);
    EXPECT_EQ(validateMap(longer), DocError::kLength);

    Bytes format = good;
    format[0]    = 2;
    EXPECT_EQ(validateMap(format), DocError::kFormat);
    Bytes record = good;
    record[1]    = 27;
    EXPECT_EQ(validateMap(record), DocError::kFormat);
}

TEST(FieldMap, ValidateCount) {
    Bytes bad = buildMap(3);
    bad[4]    = 129;
    EXPECT_EQ(validateMap(bad), DocError::kCount);
    bad[4] = 0xFF;
    bad[5] = 0xFF;
    EXPECT_EQ(validateMap(bad), DocError::kCount);
    // A count within the cap that disagrees with the length.
    bad[4] = 2;
    bad[5] = 0;
    EXPECT_EQ(validateMap(bad), DocError::kLength);
}

TEST(FieldMap, ValidateOrderAndDuplicateIds) {
    std::vector<FieldObjectRecord> zero = objects(3);
    zero[0].object_id                   = 0;
    EXPECT_EQ(validateMap(buildMap(mapHeader(3), zero)), DocError::kOrder);

    for (uint16_t i = 0; i + 1 < 10; ++i) {
        std::vector<FieldObjectRecord> dup = objects(10);
        dup[i + 1].object_id               = dup[i].object_id;
        EXPECT_EQ(validateMap(buildMap(mapHeader(10), dup)), DocError::kOrder) << i;

        std::vector<FieldObjectRecord> swapped = objects(10);
        std::swap(swapped[i].object_id, swapped[i + 1].object_id);
        EXPECT_EQ(validateMap(buildMap(mapHeader(10), swapped)), DocError::kOrder) << i;
    }
}

TEST(FieldMap, ValidateKind) {
    for (uint8_t kind : {0, 3, 255}) {
        std::vector<FieldObjectRecord> objs = objects(3);
        objs[2].kind                        = kind;
        EXPECT_EQ(validateMap(buildMap(mapHeader(3), objs)), DocError::kKind) << int(kind);
    }
}

TEST(FieldMap, ValidateFlags) {
    for (uint8_t extra : {0x08, 0x40, 0x80}) {
        std::vector<FieldObjectRecord> objs = objects(3);
        objs[0].flags |= extra;
        EXPECT_EQ(validateMap(buildMap(mapHeader(3), objs)), DocError::kFlags) << int(extra);
    }
    // Fixed elements are never estimated.
    std::vector<FieldObjectRecord> fixed = objects(3);
    ASSERT_EQ(fixed[1].kind, kObjectFixed);
    fixed[1].flags |= kObjectEstimated;
    EXPECT_EQ(validateMap(buildMap(mapHeader(3), fixed)), DocError::kFlags);

    // A fixed reference and an estimated landmark that is not an obstacle are fine.
    std::vector<FieldObjectRecord> ok = objects(3);
    ok[1].flags |= kObjectReference;
    ok[2].flags = kObjectEstimated;
    EXPECT_EQ(validateMap(buildMap(mapHeader(3), ok)), DocError::kNone);
}

TEST(FieldMap, ValidateBoxAgreesWithObstacleFlag) {
    std::vector<FieldObjectRecord> len0 = objects(3);
    len0[0].box_length_mm               = 0;
    EXPECT_EQ(validateMap(buildMap(mapHeader(3), len0)), DocError::kBox);
    std::vector<FieldObjectRecord> wid0 = objects(3);
    wid0[1].box_width_mm                = 0;
    EXPECT_EQ(validateMap(buildMap(mapHeader(3), wid0)), DocError::kBox);

    // An element with any box field but no obstacle flag.
    void (*sets[])(FieldObjectRecord&) = {
        [](FieldObjectRecord& r) { r.box_x_mm = 1; },
        [](FieldObjectRecord& r) { r.box_y_mm = -1; },
        [](FieldObjectRecord& r) { r.box_heading_cdeg = 100; },
        [](FieldObjectRecord& r) { r.box_length_mm = 1; },
        [](FieldObjectRecord& r) { r.box_width_mm = 1; },
    };
    for (auto set : sets) {
        std::vector<FieldObjectRecord> objs = objects(3);
        ASSERT_EQ(objs[2].flags & kObjectObstacle, 0);
        set(objs[2]);
        EXPECT_EQ(validateMap(buildMap(mapHeader(3), objs)), DocError::kBox);
    }
}

TEST(FieldMap, ValidateBounds) {
    FieldMapHeader h = mapHeader(2);
    h.max_x_mm       = h.min_x_mm;
    EXPECT_EQ(validateMap(buildMap(h, objects(2))), DocError::kBounds);
    h          = mapHeader(2);
    h.min_y_mm = 4000;
    EXPECT_EQ(validateMap(buildMap(h, objects(2))), DocError::kBounds);
    h          = mapHeader(0);
    h.min_x_mm = -1;
    h.max_x_mm = 0;
    h.min_y_mm = -2;
    h.max_y_mm = -1;
    EXPECT_EQ(validateMap(buildMap(h, {})), DocError::kNone);
}

TEST(FieldMap, ValidateRangeAndReservedBytes) {
    Bytes header_reserved = buildMap(3);
    header_reserved[6]    = 1;
    EXPECT_EQ(validateMap(header_reserved), DocError::kRange);
    header_reserved[6] = 0;
    header_reserved[7] = 0x80;
    EXPECT_EQ(validateMap(header_reserved), DocError::kRange);

    for (uint16_t i = 0; i < 3; ++i) {
        for (size_t k : {26, 27}) {
            Bytes bad                          = buildMap(3);
            bad[kFieldMapHeaderLen + 28 * i + k] = 1;
            EXPECT_EQ(validateMap(bad), DocError::kRange) << i << " " << k;
        }
    }

    const struct {
        int32_t heading;
        int16_t box_heading;
        bool    ok;
    } cases[] = {
        {18000, 0, true},    {-17999, 0, true},  {-18000, 0, false}, {18001, 0, false},
        {INT32_MIN, 0, false}, {0, 18000, true}, {0, -17999, true},  {0, -18000, false},
        {0, 18001, false},   {0, -32768, false},
    };
    for (const auto& c : cases) {
        std::vector<FieldObjectRecord> objs = objects(3);
        objs[0].heading_cdeg                = c.heading;
        objs[0].box_heading_cdeg            = c.box_heading;
        EXPECT_EQ(validateMap(buildMap(mapHeader(3), objs)),
                  c.ok ? DocError::kNone : DocError::kRange)
            << c.heading << " " << c.box_heading;
    }
}

// ---------------------------------------------------------------------------
// Field estimate
// ---------------------------------------------------------------------------

TEST(FieldEstimate, LayoutConstants) {
    EXPECT_EQ(kFieldEstimateFormat, 1);
    EXPECT_EQ(kFieldEstimateHeaderLen, 24);
    EXPECT_EQ(kFieldEstimateRecordLen, 20);
    EXPECT_EQ(kFieldEstimateMaxLen, 2584);
    EXPECT_EQ(kEstimateValid, 0x01);
    EXPECT_EQ(fieldEstimateLen(0), 24);
    EXPECT_EQ(fieldEstimateLen(kFieldMaxObjects), kFieldEstimateMaxLen);
    EXPECT_EQ(fieldEstimateLen(kFieldMaxObjects + 1), 0);
    EXPECT_GT(fieldEstimateLen(4), kDocChunkMax);
}

TEST(FieldEstimate, KnownBytes) {
    FieldEstimateHeader h;
    h.object_count    = 1;
    h.map_id          = 0x89ABCDEF;
    h.estimate_id     = 0x102;
    h.odometry_epoch  = 3;
    h.anchor_revision = 0xFFFFFFFF;
    FieldEstimateRecord r;
    r.object_id    = 0x0102;
    r.source       = kEstimateSourceObserved;
    r.flags        = kEstimateValid;
    r.x_mm         = -1;
    r.y_mm         = 2000;
    r.heading_cdeg = 18000;
    r.age_ms       = 65535;

    Le e;
    e.u8(1).u8(20).u16(1).u32(0x89ABCDEF).u32(0x102).u32(3).u32(0xFFFFFFFF).u32(0);
    e.u16(0x0102).u8(2).u8(1).i32(-1).i32(2000).i32(18000).u16(65535).u16(0);
    const Bytes doc = buildEstimate(h, {r});
    EXPECT_EQ(doc, e.b);

    FieldEstimateHeader hd;
    ASSERT_TRUE(decodeFieldEstimateHeader(doc.data(), static_cast<uint16_t>(doc.size()), hd));
    EXPECT_EQ(hd.format, 1);
    EXPECT_EQ(hd.object_count, 1);
    EXPECT_EQ(hd.map_id, 0x89ABCDEFu);
    EXPECT_EQ(hd.estimate_id, 0x102u);
    EXPECT_EQ(hd.odometry_epoch, 3u);
    EXPECT_EQ(hd.anchor_revision, 0xFFFFFFFFu);

    FieldEstimateRecord o;
    ASSERT_TRUE(decodeFieldEstimateRecord(doc.data(), static_cast<uint16_t>(doc.size()), 0, o));
    EXPECT_EQ(o.object_id, 0x0102);
    EXPECT_EQ(o.source, kEstimateSourceObserved);
    EXPECT_EQ(o.flags, kEstimateValid);
    EXPECT_EQ(o.x_mm, -1);
    EXPECT_EQ(o.y_mm, 2000);
    EXPECT_EQ(o.heading_cdeg, 18000);
    EXPECT_EQ(o.age_ms, 65535);
}

TEST(FieldEstimate, RoundTripForEveryCountShape) {
    for (uint16_t count : {0, 1, 2, 7, 37, 127, 128}) {
        SCOPED_TRACE(testing::Message() << count << " objects");
        const Field f(count);
        const Bytes doc = f.estimate();
        ASSERT_EQ(doc.size(), fieldEstimateLen(count));
        EXPECT_EQ(f.check(), DocError::kNone);

        FieldEstimateHeader h;
        ASSERT_TRUE(decodeFieldEstimateHeader(doc.data(), static_cast<uint16_t>(doc.size()), h));
        EXPECT_EQ(h.object_count, count);
        EXPECT_EQ(h.map_id, mapIdOf(f.map));
        EXPECT_EQ(h.estimate_id, 17u);
        for (uint16_t i = 0; i < count; ++i) {
            FieldEstimateRecord o;
            ASSERT_TRUE(
                decodeFieldEstimateRecord(doc.data(), static_cast<uint16_t>(doc.size()), i, o));
            EXPECT_EQ(o.object_id, f.recs[i].object_id);
            EXPECT_EQ(o.source, f.recs[i].source);
            EXPECT_EQ(o.x_mm, f.recs[i].x_mm);
            EXPECT_EQ(o.age_ms, f.recs[i].age_ms);
        }
        FieldEstimateRecord past;
        EXPECT_FALSE(decodeFieldEstimateRecord(doc.data(), static_cast<uint16_t>(doc.size()),
                                               count, past));
    }
}

TEST(FieldEstimate, WritersRefuseBadCountIndexOrCapacity) {
    Bytes doc(kFieldEstimateMaxLen + 20);
    EXPECT_FALSE(encodeFieldEstimateHeader(estimateHeader(kFieldMaxObjects + 1, 1), doc.data(),
                                           static_cast<uint16_t>(doc.size())));
    EXPECT_FALSE(
        encodeFieldEstimateHeader(estimateHeader(2, 1), doc.data(), fieldEstimateLen(2) - 1));
    EXPECT_TRUE(encodeFieldEstimateHeader(estimateHeader(2, 1), doc.data(), fieldEstimateLen(2)));

    const FieldEstimateRecord r;
    EXPECT_FALSE(encodeFieldEstimateRecord(r, kFieldMaxObjects, doc.data(),
                                           static_cast<uint16_t>(doc.size())));
    EXPECT_FALSE(encodeFieldEstimateRecord(r, 1, doc.data(), fieldEstimateLen(2) - 1));
    EXPECT_TRUE(encodeFieldEstimateRecord(r, 1, doc.data(), fieldEstimateLen(2)));
}

TEST(FieldEstimate, HeaderDecodeRejectsMalformedCounts) {
    const Bytes         good = Field(3).estimate();
    FieldEstimateHeader h;
    for (size_t n = 0; n < good.size(); ++n) {
        EXPECT_FALSE(decodeFieldEstimateHeader(good.data(), static_cast<uint16_t>(n), h)) << n;
    }
    for (uint16_t count : {0, 2, 4, 129, 0xFFFF}) {
        Bytes bad = good;
        bad[2]    = static_cast<uint8_t>(count);
        bad[3]    = static_cast<uint8_t>(count >> 8);
        EXPECT_FALSE(decodeFieldEstimateHeader(bad.data(), static_cast<uint16_t>(bad.size()), h))
            << count;
    }
    Bytes format = good;
    format[0]    = 0;
    EXPECT_FALSE(decodeFieldEstimateHeader(format.data(), static_cast<uint16_t>(format.size()), h));
    Bytes record = good;
    record[1]    = 21;
    EXPECT_FALSE(decodeFieldEstimateHeader(record.data(), static_cast<uint16_t>(record.size()), h));
}

TEST(FieldEstimate, ValidateLengthFormatAndCount) {
    const Field f(3);
    const Bytes good = f.estimate();
    EXPECT_EQ(validateFieldEstimate(good.data(), 0, f.map.data(),
                                    static_cast<uint16_t>(f.map.size()), mapIdOf(f.map)),
              DocError::kLength);
    EXPECT_EQ(validateFieldEstimate(good.data(), static_cast<uint16_t>(good.size() - 1),
                                    f.map.data(), static_cast<uint16_t>(f.map.size()),
                                    mapIdOf(f.map)),
              DocError::kLength);

    Bytes format = good;
    format[0]    = 2;
    EXPECT_EQ(validateEstimate(format, f.map), DocError::kFormat);
    Bytes record = good;
    record[1]    = 28;
    EXPECT_EQ(validateEstimate(record, f.map), DocError::kFormat);

    Bytes over = good;
    over[2]    = 129;
    EXPECT_EQ(validateEstimate(over, f.map), DocError::kCount);

    // Complete and consistent on its own, but for fewer objects than the map.
    Field fewer(3);
    fewer.recs.pop_back();
    fewer.header.object_count = 2;
    EXPECT_EQ(validateEstimate(fewer.estimate(), f.map), DocError::kCount);
    Field more(4);
    more.header.map_id = mapIdOf(f.map);
    EXPECT_EQ(validateEstimate(more.estimate(), f.map), DocError::kCount);
}

TEST(FieldEstimate, ValidateMapIdentity) {
    const Field f(3);
    const Bytes est = f.estimate();

    // The estimate names another map.
    Field other(3);
    other.header.map_id ^= 1;
    EXPECT_EQ(other.check(), DocError::kMapMismatch);

    // The caller's map id is not the one the estimate names.
    EXPECT_EQ(validateFieldEstimate(est.data(), static_cast<uint16_t>(est.size()), f.map.data(),
                                    static_cast<uint16_t>(f.map.size()), mapIdOf(f.map) + 1),
              DocError::kMapMismatch);

    // No usable map document.
    EXPECT_EQ(validateFieldEstimate(est.data(), static_cast<uint16_t>(est.size()), nullptr, 0,
                                    mapIdOf(f.map)),
              DocError::kMapMismatch);
    EXPECT_EQ(validateFieldEstimate(est.data(), static_cast<uint16_t>(est.size()), f.map.data(),
                                    static_cast<uint16_t>(f.map.size() - 1), mapIdOf(f.map)),
              DocError::kMapMismatch);
}

TEST(FieldEstimate, ValidateAgainstAMapWithOtherIds) {
    // Same count, same map id claimed, different objects.
    const Field                    f(4);
    std::vector<FieldObjectRecord> objs = objects(4);
    objs[3].object_id += 1;
    const Bytes other = buildMap(mapHeader(4), objs);
    Field       g(4);
    g.header.map_id = mapIdOf(other);
    EXPECT_EQ(validateEstimate(g.estimate(), other), DocError::kOrder);
}

TEST(FieldEstimate, ValidateOrderAndDuplicateIds) {
    for (uint16_t i = 0; i + 1 < 6; ++i) {
        Field swapped(6);
        std::swap(swapped.recs[i], swapped.recs[i + 1]);
        EXPECT_EQ(swapped.check(), DocError::kOrder) << i;

        Field dup(6);
        dup.recs[i + 1].object_id = dup.recs[i].object_id;
        EXPECT_EQ(dup.check(), DocError::kOrder) << i;
    }
    Field zero(2);
    zero.recs[0].object_id = 0;
    EXPECT_EQ(zero.check(), DocError::kOrder);
}

TEST(FieldEstimate, ValidateSource) {
    for (uint8_t source : {3, 255}) {
        Field f(3);
        f.recs[0].source = source;
        EXPECT_EQ(f.check(), DocError::kKind) << int(source);
    }
}

TEST(FieldEstimate, ValidateValidityAndSourceAgree) {
    Field f(3);
    ASSERT_NE(f.objs[0].flags & kObjectEstimated, 0);
    ASSERT_EQ(f.objs[1].kind, kObjectFixed);
    ASSERT_EQ(f.objs[2].flags & kObjectEstimated, 0);

    // An estimated landmark with no estimate at all.
    Field none(3);
    none.recs[0].source = kEstimateSourceNone;
    none.recs[0].flags  = 0;
    none.recs[0].age_ms = 0;
    EXPECT_EQ(none.check(), DocError::kNone);
    none.recs[0].flags = kEstimateValid;
    EXPECT_EQ(none.check(), DocError::kFlags);

    // An estimated landmark back at its nominal pose.
    Field nominal(3);
    nominal.recs[0].source = kEstimateSourceNominal;
    nominal.recs[0].age_ms = 0;
    EXPECT_EQ(nominal.check(), DocError::kNone);
    nominal.recs[0].flags = 0;
    EXPECT_EQ(nominal.check(), DocError::kFlags);

    Field observed(3);
    observed.recs[0].flags = 0;
    EXPECT_EQ(observed.check(), DocError::kFlags);

    for (uint8_t extra : {0x02, 0x80}) {
        Field bits(3);
        bits.recs[1].flags |= extra;
        EXPECT_EQ(bits.check(), DocError::kFlags) << int(extra);
    }
}

TEST(FieldEstimate, ValidateObjectsNotEstimatedStayNominal) {
    for (uint16_t i : {1, 2}) {
        Field observed(3);
        observed.recs[i].source = kEstimateSourceObserved;
        observed.recs[i].age_ms = 10;
        EXPECT_EQ(observed.check(), DocError::kFlags) << i;

        Field none(3);
        none.recs[i].source = kEstimateSourceNone;
        none.recs[i].flags  = 0;
        EXPECT_EQ(none.check(), DocError::kFlags) << i;
    }
}

TEST(FieldEstimate, ValidateAgeOnlyWhenObserved) {
    Field observed(3);
    observed.recs[0].age_ms = 65535;
    EXPECT_EQ(observed.check(), DocError::kNone);
    observed.recs[0].age_ms = 0;
    EXPECT_EQ(observed.check(), DocError::kNone);

    Field nominal(3);
    nominal.recs[1].age_ms = 1;
    EXPECT_EQ(nominal.check(), DocError::kRange);

    Field none(3);
    none.recs[0].source = kEstimateSourceNone;
    none.recs[0].flags  = 0;
    none.recs[0].age_ms = 5;
    EXPECT_EQ(none.check(), DocError::kRange);
}

TEST(FieldEstimate, ValidateHeadingAndReservedBytes) {
    for (int32_t heading : {-18000, 18001, INT32_MIN, INT32_MAX}) {
        Field f(3);
        f.recs[2].heading_cdeg = heading;
        EXPECT_EQ(f.check(), DocError::kRange) << heading;
    }
    const Field good(3);
    Bytes       header = good.estimate();
    header[23]         = 1;
    EXPECT_EQ(validateEstimate(header, good.map), DocError::kRange);
    for (uint16_t i = 0; i < 3; ++i) {
        for (size_t k : {18, 19}) {
            Bytes bad = good.estimate();
            bad[kFieldEstimateHeaderLen + 20 * i + k] = 0x40;
            EXPECT_EQ(validateEstimate(bad, good.map), DocError::kRange) << i << " " << k;
        }
    }
}

TEST(FieldEstimate, EmptyFieldValidates) {
    const Field f(0);
    EXPECT_EQ(f.check(), DocError::kNone);
}
