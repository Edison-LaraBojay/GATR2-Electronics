// frame_codec_gtest.cpp
// Byte-level tests. The known vectors match the examples in docs/interfaces.md.

#include <gtest/gtest.h>

#include <vector>

#include "frame_codec.h"

using namespace gatr2;

namespace
{

// Sensor frame: seq 7, stamp 1000, mask 0x000B, enc0 1000, enc1 -500,
// gyro_z 2500.
const std::vector<uint8_t> kSensorVector = {
    0xAA, 0x55, 0x01, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x0B, 0x00, 0xE8, 0x03,
    0x00, 0x00, 0x0C, 0xFE, 0xFF, 0xFF, 0xC4, 0x09, 0x00, 0x00, 0xCD,
};

// Pose frame: seq 42, stamp 5000, x 1500, y -250, heading 9000, status 0x00FF,
// object 3 at (2000, 1000, 0), no landmarks.
const std::vector<uint8_t> kPoseVector = {
    0xAA, 0x55, 0x02, 0x2A, 0x88, 0x13, 0x00, 0x00, 0xDC, 0x05, 0x00, 0x00,
    0x06, 0xFF, 0xFF, 0xFF, 0x28, 0x23, 0x00, 0x00, 0xFF, 0x00, 0x03, 0xD0,
    0x07, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xA7,
};

// Command frame: seq 9, init pose at (610, 457, 9000).
const std::vector<uint8_t> kCommandVector = {
    0xAA, 0x55, 0x03, 0x09, 0x01, 0x62, 0x02, 0x00, 0x00, 0xC9, 0x01, 0x00,
    0x00, 0x28, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x57,
};

SensorSample makeSample() {
    SensorSample s{};
    s.seq      = 7;
    s.stamp_ms = 1000;
    s.mask     = kSensorEnc0 | kSensorEnc1 | kSensorGyroZ;
    s.enc[0]   = 1000;
    s.enc[1]   = -500;
    s.gyro_z   = 2500;
    return s;
}

} // namespace

// ---------------------------------------------------------------------------
// Length arithmetic
// ---------------------------------------------------------------------------

TEST(Codec, MaskValidity) {
    EXPECT_TRUE(sensorMaskValid(0x0000));
    EXPECT_TRUE(sensorMaskValid(kSensorEnc0 | kSensorAccelXY));
    EXPECT_FALSE(sensorMaskValid(1u << kSensorBitCount));
}

TEST(Codec, PayloadLenSumsWidths) {
    EXPECT_EQ(sensorPayloadLen(0), 0);
    EXPECT_EQ(sensorPayloadLen(kSensorEnc0), 4);
    EXPECT_EQ(sensorPayloadLen(kSensorEnc0 | kSensorEnc1 | kSensorGyroZ), 12);
    EXPECT_EQ(sensorPayloadLen(kSensorAccelXY), 8);
}

TEST(Codec, FrameLensMatchVectors) {
    EXPECT_EQ(sensorFrameLen(kSensorEnc0 | kSensorEnc1 | kSensorGyroZ),
              kSensorVector.size());
    EXPECT_EQ(poseFrameLen(0), kPoseVector.size());
    EXPECT_EQ(poseFrameLen(kMaxLandmarks), 36 + 8 * 8 + 1);
    EXPECT_EQ(kCommandFrameLen, kCommandVector.size());
}

TEST(Codec, EveryFrameFitsTheBound) {
    EXPECT_LE(sensorFrameLen(0xFFFF & ((1u << kSensorBitCount) - 1)), kMaxFrameLen);
    EXPECT_LE(poseFrameLen(kMaxLandmarks), kMaxFrameLen);
}

// ---------------------------------------------------------------------------
// Known bytes
// ---------------------------------------------------------------------------

TEST(Codec, EncodeSensorMatchesKnownBytes) {
    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeSensorFrame(makeSample(), buf, sizeof(buf));
    ASSERT_EQ(n, kSensorVector.size());
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + n), kSensorVector);
}

TEST(Codec, DecodeSensorFromKnownBytes) {
    SensorSample s{};
    ASSERT_TRUE(decodeSensorFrame(kSensorVector.data(),
                                  static_cast<uint16_t>(kSensorVector.size()), s));
    EXPECT_EQ(s.seq, 7);
    EXPECT_EQ(s.stamp_ms, 1000u);
    EXPECT_EQ(s.mask, kSensorEnc0 | kSensorEnc1 | kSensorGyroZ);
    EXPECT_EQ(s.enc[0], 1000);
    EXPECT_EQ(s.enc[1], -500);
    EXPECT_EQ(s.gyro_z, 2500);
}

TEST(Codec, AbsentSensorsAreNotInThePayload) {
    SensorSample s{};
    ASSERT_TRUE(decodeSensorFrame(kSensorVector.data(),
                                  static_cast<uint16_t>(kSensorVector.size()), s));
    // enc2 and accel bits are clear, so their fields stay zero and cost no bytes.
    EXPECT_EQ(s.enc[2], 0);
    EXPECT_EQ(s.accel[0], 0);
    EXPECT_EQ(s.accel[1], 0);
}

TEST(Codec, EncodePoseMatchesKnownBytes) {
    PoseFrame p{};
    p.seq          = 42;
    p.stamp_ms     = 5000;
    p.x_mm         = 1500;
    p.y_mm         = -250;
    p.heading_cdeg = 9000;
    p.status       = kStatusPoseValid | kStatusEncHealthy | kStatusGyroHealthy |
               kStatusVisionAlive | kStatusBiasCal | kStatusLocInit | kStatusObjRequested |
               kStatusObjValid;
    p.object_id   = 3;
    p.obj_x_mm    = 2000;
    p.obj_y_mm    = 1000;
    p.n_landmarks = 0;

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodePoseFrame(p, buf, sizeof(buf));
    ASSERT_EQ(n, kPoseVector.size());
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + n), kPoseVector);
}

TEST(Codec, DecodePoseFromKnownBytes) {
    PoseFrame p{};
    ASSERT_TRUE(
        decodePoseFrame(kPoseVector.data(), static_cast<uint16_t>(kPoseVector.size()), p));
    EXPECT_EQ(p.status, 0x00FF);
    EXPECT_EQ(p.object_id, 3);
    EXPECT_EQ(p.obj_x_mm, 2000);
    EXPECT_EQ(p.obj_y_mm, 1000);
    EXPECT_EQ(p.obj_heading_cdeg, 0);
    EXPECT_EQ(p.n_landmarks, 0);
}

TEST(Codec, EncodeCommandMatchesKnownBytes) {
    CommandFrame c{};
    c.seq          = 9;
    c.command      = kCmdInitPose;
    c.x_mm         = 610;
    c.y_mm         = 457;
    c.heading_cdeg = 9000;

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeCommandFrame(c, buf, sizeof(buf));
    ASSERT_EQ(n, kCommandVector.size());
    EXPECT_EQ(std::vector<uint8_t>(buf, buf + n), kCommandVector);
}

TEST(Codec, CommandRoundTrip) {
    CommandFrame in{};
    in.seq          = 200;
    in.command      = kCmdSelectObject;
    in.x_mm         = -1;
    in.y_mm         = 2147483647;
    in.heading_cdeg = -18000;
    in.mode         = 2;
    in.object_id    = 7;
    in.flags        = kCmdFlagObjectRequested | kCmdFlagStreamOn;

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeCommandFrame(in, buf, sizeof(buf));
    ASSERT_EQ(n, kCommandFrameLen);

    CommandFrame out{};
    ASSERT_TRUE(decodeCommandFrame(buf, n, out));
    EXPECT_EQ(out.seq, in.seq);
    EXPECT_EQ(out.command, in.command);
    EXPECT_EQ(out.x_mm, in.x_mm);
    EXPECT_EQ(out.y_mm, in.y_mm);
    EXPECT_EQ(out.heading_cdeg, in.heading_cdeg);
    EXPECT_EQ(out.mode, in.mode);
    EXPECT_EQ(out.object_id, in.object_id);
    EXPECT_EQ(out.flags, in.flags);
}

TEST(Codec, CorruptCommandRejected) {
    std::vector<uint8_t> bad = kCommandVector;
    bad[5] ^= 0x01;
    CommandFrame c{};
    EXPECT_FALSE(decodeCommandFrame(bad.data(), static_cast<uint16_t>(bad.size()), c));
}

// ---------------------------------------------------------------------------
// Round trip
// ---------------------------------------------------------------------------

TEST(Codec, SensorRoundTripAllSensors) {
    SensorSample in{};
    in.seq      = 255;
    in.stamp_ms = 0xDEADBEEF;
    in.mask     = kSensorEnc0 | kSensorEnc1 | kSensorEnc2 | kSensorGyroZ | kSensorAccelXY;
    in.enc[0]   = 2147483647;
    in.enc[1]   = -2147483648;
    in.enc[2]   = 0;
    in.gyro_z   = -1;
    in.accel[0] = 123;
    in.accel[1] = -456;

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeSensorFrame(in, buf, sizeof(buf));
    ASSERT_GT(n, 0);

    SensorSample out{};
    ASSERT_TRUE(decodeSensorFrame(buf, n, out));
    EXPECT_EQ(out.seq, in.seq);
    EXPECT_EQ(out.stamp_ms, in.stamp_ms);
    EXPECT_EQ(out.mask, in.mask);
    EXPECT_EQ(out.enc[0], in.enc[0]);
    EXPECT_EQ(out.enc[1], in.enc[1]);
    EXPECT_EQ(out.enc[2], in.enc[2]);
    EXPECT_EQ(out.gyro_z, in.gyro_z);
    EXPECT_EQ(out.accel[0], in.accel[0]);
    EXPECT_EQ(out.accel[1], in.accel[1]);
}

TEST(Codec, SensorRoundTripEmptyMask) {
    SensorSample in{};
    in.seq      = 1;
    in.stamp_ms = 42;
    in.mask     = 0;

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeSensorFrame(in, buf, sizeof(buf));
    ASSERT_EQ(n, kSensorHeaderLen + 1);

    SensorSample out{};
    ASSERT_TRUE(decodeSensorFrame(buf, n, out));
    EXPECT_EQ(out.mask, 0);
    EXPECT_EQ(out.stamp_ms, 42u);
}

TEST(Codec, PoseRoundTripWithLandmarks) {
    PoseFrame in{};
    in.seq          = 3;
    in.stamp_ms     = 77;
    in.x_mm         = -1;
    in.y_mm         = 32767;
    in.heading_cdeg     = -18000;
    in.status           = kStatusPoseValid | kStatusObjObserved;
    in.object_id        = 250;
    in.obj_x_mm         = -3000;
    in.obj_y_mm         = 3000;
    in.obj_heading_cdeg = 4500;
    in.n_landmarks      = kMaxLandmarks;
    for (uint8_t i = 0; i < kMaxLandmarks; ++i) {
        in.landmarks[i] = {static_cast<uint8_t>(i),
                           static_cast<int16_t>(i * 100),
                           static_cast<int16_t>(-i * 100),
                           static_cast<int16_t>(i * 7),
                           static_cast<uint8_t>(255 - i)};
    }

    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodePoseFrame(in, buf, sizeof(buf));
    ASSERT_GT(n, 0);

    PoseFrame out{};
    ASSERT_TRUE(decodePoseFrame(buf, n, out));
    EXPECT_EQ(out.n_landmarks, kMaxLandmarks);
    EXPECT_EQ(out.heading_cdeg, -18000);
    EXPECT_EQ(out.status, in.status);
    EXPECT_EQ(out.object_id, 250);
    EXPECT_EQ(out.obj_x_mm, -3000);
    EXPECT_EQ(out.obj_y_mm, 3000);
    EXPECT_EQ(out.obj_heading_cdeg, 4500);
    for (uint8_t i = 0; i < kMaxLandmarks; ++i) {
        EXPECT_EQ(out.landmarks[i].id, in.landmarks[i].id);
        EXPECT_EQ(out.landmarks[i].dx_mm, in.landmarks[i].dx_mm);
        EXPECT_EQ(out.landmarks[i].dy_mm, in.landmarks[i].dy_mm);
        EXPECT_EQ(out.landmarks[i].bearing_cdeg, in.landmarks[i].bearing_cdeg);
        EXPECT_EQ(out.landmarks[i].quality, in.landmarks[i].quality);
    }
}

// ---------------------------------------------------------------------------
// Rejection
// ---------------------------------------------------------------------------

TEST(Codec, CorruptChecksumRejected) {
    std::vector<uint8_t> bad = kSensorVector;
    bad.back() ^= 0xFF;
    SensorSample s{};
    EXPECT_FALSE(
        decodeSensorFrame(bad.data(), static_cast<uint16_t>(bad.size()), s));
}

TEST(Codec, CorruptPayloadRejected) {
    std::vector<uint8_t> bad = kSensorVector;
    bad[12] ^= 0x01;
    SensorSample s{};
    EXPECT_FALSE(
        decodeSensorFrame(bad.data(), static_cast<uint16_t>(bad.size()), s));
}

TEST(Codec, TruncatedFramesRejectedWithoutCrash) {
    SensorSample s{};
    for (uint16_t n = 0; n < kSensorVector.size(); ++n) {
        EXPECT_FALSE(decodeSensorFrame(kSensorVector.data(), n, s));
    }
}

TEST(Codec, BadSyncRejected) {
    std::vector<uint8_t> bad = kSensorVector;
    bad[0]                   = 0x00;
    SensorSample s{};
    EXPECT_FALSE(
        decodeSensorFrame(bad.data(), static_cast<uint16_t>(bad.size()), s));
}

TEST(Codec, WrongTypeRejected) {
    SensorSample s{};
    EXPECT_FALSE(decodeSensorFrame(kPoseVector.data(),
                                   static_cast<uint16_t>(kPoseVector.size()), s));
}

TEST(Codec, UnknownMaskBitNotEncodable) {
    SensorSample in{};
    in.mask = static_cast<uint16_t>(1u << kSensorBitCount);
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodeSensorFrame(in, buf, sizeof(buf)), 0);
}

TEST(Codec, TooManyLandmarksNotEncodable) {
    PoseFrame in{};
    in.n_landmarks = kMaxLandmarks + 1;
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodePoseFrame(in, buf, sizeof(buf)), 0);
}

TEST(Codec, EncodeRespectsCapacity) {
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodeSensorFrame(makeSample(), buf, 4), 0);
}

// ---------------------------------------------------------------------------
// Stream reader
// ---------------------------------------------------------------------------

TEST(Reader, ReadsCleanFrame) {
    FrameReader r;
    bool        got = false;
    for (uint8_t b : kSensorVector) {
        got = r.push(b);
    }
    ASSERT_TRUE(got);
    EXPECT_EQ(r.frameLen(), kSensorVector.size());
    EXPECT_EQ(r.frameType(), kFrameSensor);
}

TEST(Reader, IgnoresLeadingGarbage) {
    FrameReader r;
    for (uint8_t b : {0x00, 0x12, 0xFF, 0xAA, 0x99, 0x55}) {
        EXPECT_FALSE(r.push(b));
    }
    bool got = false;
    for (uint8_t b : kSensorVector) {
        got = r.push(b);
    }
    EXPECT_TRUE(got);
}

TEST(Reader, RelocksAfterCorruptFrame) {
    FrameReader r;

    std::vector<uint8_t> bad = kSensorVector;
    bad.back() ^= 0xFF;
    for (uint8_t b : bad) {
        EXPECT_FALSE(r.push(b));
    }

    bool got = false;
    for (uint8_t b : kSensorVector) {
        got = r.push(b);
    }
    EXPECT_TRUE(got);
}

TEST(Reader, HandlesBackToBackFrames) {
    FrameReader r;
    int         frames = 0;
    for (int i = 0; i < 3; ++i) {
        for (uint8_t b : kSensorVector) {
            if (r.push(b)) {
                ++frames;
            }
        }
    }
    EXPECT_EQ(frames, 3);
}

TEST(Reader, HandlesRepeatedSyncBytes) {
    FrameReader r;
    // A run of sync0 before the real frame must not desync the reader.
    for (int i = 0; i < 5; ++i) {
        EXPECT_FALSE(r.push(kSync0));
    }
    bool got = false;
    for (size_t i = 1; i < kSensorVector.size(); ++i) {
        got = r.push(kSensorVector[i]);
    }
    EXPECT_TRUE(got);
}

TEST(Reader, ReadsPoseFrame) {
    FrameReader r;
    bool        got = false;
    for (uint8_t b : kPoseVector) {
        got = r.push(b);
    }
    ASSERT_TRUE(got);
    EXPECT_EQ(r.frameType(), kFramePose);

    PoseFrame p{};
    ASSERT_TRUE(decodePoseFrame(r.frame(), r.frameLen(), p));
    EXPECT_EQ(p.x_mm, 1500);
    EXPECT_EQ(p.y_mm, -250);
    EXPECT_EQ(p.heading_cdeg, 9000);
}

TEST(Reader, ReadsCommandFrame) {
    FrameReader r;
    bool        got = false;
    for (uint8_t b : kCommandVector) {
        got = r.push(b);
    }
    ASSERT_TRUE(got);
    EXPECT_EQ(r.frameType(), kFrameCommand);

    CommandFrame c{};
    ASSERT_TRUE(decodeCommandFrame(r.frame(), r.frameLen(), c));
    EXPECT_EQ(c.command, kCmdInitPose);
    EXPECT_EQ(c.x_mm, 610);
    EXPECT_EQ(c.y_mm, 457);
    EXPECT_EQ(c.heading_cdeg, 9000);
}

TEST(Reader, RejectsUnknownFrameType) {
    FrameReader          r;
    std::vector<uint8_t> bad = kSensorVector;
    bad[2]                   = 0x7F;
    for (uint8_t b : bad) {
        EXPECT_FALSE(r.push(b));
    }
}
