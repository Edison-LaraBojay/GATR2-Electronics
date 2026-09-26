// frame_codec_gtest.cpp
// Byte-level tests. The known vectors match the examples in docs/interfaces.md.
// Brain link vectors were computed with an independent CRC-16/CCITT-FALSE.

#include <gtest/gtest.h>

#include <stdint.h>
#include <vector>

#include "frame_codec.h"

using namespace gatr2;

namespace
{

using Bytes = std::vector<uint8_t>;

// Sensor frame: seq 7, stamp 1000, mask 0x000B, enc0 1000, enc1 -500,
// gyro_z 2500.
const Bytes kSensorVector = {
    0xAA, 0x55, 0x01, 0x07, 0xE8, 0x03, 0x00, 0x00, 0x0B, 0x00, 0xE8, 0x03,
    0x00, 0x00, 0x0C, 0xFE, 0xFF, 0xFF, 0xC4, 0x09, 0x00, 0x00, 0xCD,
};

constexpr uint32_t kSession  = 0xA1B2C3D4;
constexpr uint32_t kInstance = 0x0BADF00D;
constexpr uint32_t kNonce    = 0x12345678;

// HELLO: session 0, rid 1, nonce 0x12345678.
const Bytes kHelloRequest = {
    0xAA, 0x55, 0x10, 0x0C, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x78, 0x56, 0x34, 0x12, 0xE6, 0x29,
};

// SET_POSE: rid 2, (610, -457, -9000).
const Bytes kSetPoseRequest = {
    0xAA, 0x55, 0x10, 0x14, 0x03, 0x02, 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00,
    0x62, 0x02, 0x00, 0x00, 0x37, 0xFE, 0xFF, 0xFF, 0xD8, 0xDC, 0xFF, 0xFF,
    0xC5, 0x85,
};

// SELECT_LANDMARK: rid 3, landmark 5, selected.
const Bytes kSelectRequest = {
    0xAA, 0x55, 0x10, 0x0A, 0x03, 0x03, 0xD4, 0xC3, 0xB2, 0xA1, 0x03, 0x00,
    0x05, 0x01, 0xD1, 0x3E,
};

// GET_STATE: rid 65535.
const Bytes kGetStateRequest = {
    0xAA, 0x55, 0x10, 0x08, 0x03, 0x04, 0xD4, 0xC3, 0xB2, 0xA1, 0xFF, 0xFF,
    0x86, 0xA1,
};

// HELLO Ok: new session, nonce echo.
const Bytes kHelloOkReply = {
    0xAA, 0x55, 0x11, 0x11, 0x03, 0x01, 0xD4, 0xC3, 0xB2, 0xA1, 0x01, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x78, 0x56, 0x34, 0x12, 0xBB, 0x6D,
};

// HELLO Stale: session 0, nonce echo.
const Bytes kHelloStaleReply = {
    0xAA, 0x55, 0x11, 0x11, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x08, 0x0D, 0xF0, 0xAD, 0x0B, 0x78, 0x56, 0x34, 0x12, 0xCB, 0xDD,
};

// SET_POSE Ok: epoch 2, anchor revision 7.
const Bytes kSetPoseOkReply = {
    0xAA, 0x55, 0x11, 0x15, 0x03, 0x02, 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x02, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
    0x00, 0xF0, 0x26,
};

// SET_POSE Pending: epoch 2, anchor revision 6.
const Bytes kSetPosePendingReply = {
    0xAA, 0x55, 0x11, 0x15, 0x03, 0x02, 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00,
    0x01, 0x0D, 0xF0, 0xAD, 0x0B, 0x02, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
    0x00, 0x27, 0x15,
};

// SET_POSE UnknownSession: header only.
const Bytes kSetPoseUnknownSessionReply = {
    0xAA, 0x55, 0x11, 0x0D, 0x03, 0x02, 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00,
    0x02, 0x0D, 0xF0, 0xAD, 0x0B, 0xA1, 0x8C,
};

// SELECT_LANDMARK Ok: landmark 5, selected.
const Bytes kSelectOkReply = {
    0xAA, 0x55, 0x11, 0x0F, 0x03, 0x03, 0xD4, 0xC3, 0xB2, 0xA1, 0x03, 0x00,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x05, 0x01, 0x22, 0xA4,
};

// GET_STATE Ok: see makeState().
const Bytes kGetStateOkReply = {
    0xAA, 0x55, 0x11, 0x35, 0x03, 0x04, 0xD4, 0xC3, 0xB2, 0xA1, 0xFF, 0xFF,
    0x00, 0x0D, 0xF0, 0xAD, 0x0B, 0x0F, 0xDC, 0x05, 0x00, 0x00, 0x06, 0xFF,
    0xFF, 0xFF, 0x50, 0x46, 0x00, 0x00, 0x23, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, 0x0B, 0x03, 0x02, 0xD0, 0x07, 0x00, 0x00, 0xE8,
    0x03, 0x00, 0x00, 0x6C, 0xEE, 0xFF, 0xFF, 0x78, 0x00, 0xFE, 0x2C,
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

BrainRequest makeRequest(uint8_t op, uint32_t session, uint16_t rid) {
    BrainRequest r{};
    r.op         = op;
    r.session    = session;
    r.request_id = rid;
    return r;
}

BrainReply makeReply(uint8_t op, uint32_t session, uint16_t rid, uint8_t result) {
    BrainReply r{};
    r.op          = op;
    r.session     = session;
    r.request_id  = rid;
    r.result      = result;
    r.pi_instance = kInstance;
    return r;
}

BrainState makeState() {
    BrainState s{};
    s.robot_flags     = kRobotPoseValid | kRobotLocalized | kRobotAgeKnown | kRobotAnchorCommand;
    s.x_mm            = 1500;
    s.y_mm            = -250;
    s.heading_cdeg    = 18000;
    s.robot_age_ms    = 35;
    s.odometry_epoch  = 2;
    s.anchor_revision = 7;
    s.health          = kHealthEncodersFresh | kHealthGyroFresh | kHealthBiasCalibrated;
    s.landmark_id     = 3;
    s.landmark_source = kLandmarkSourceObserved;
    s.lm_x_mm         = 2000;
    s.lm_y_mm         = 1000;
    s.lm_heading_cdeg = -4500;
    s.landmark_age_ms = 120;
    return s;
}

Bytes encode(const BrainRequest& r) {
    Bytes buf(kMaxFrameLen);
    buf.resize(encodeBrainRequest(r, buf.data(), kMaxFrameLen));
    return buf;
}

Bytes encode(const BrainReply& r) {
    Bytes buf(kMaxFrameLen);
    buf.resize(encodeBrainReply(r, buf.data(), kMaxFrameLen));
    return buf;
}

// Arbitrary link frame with a correct crc, for lengths the encoders never produce.
Bytes linkFrame(uint8_t type, const Bytes& payload) {
    Bytes f = {kSync0, kSync1, type, static_cast<uint8_t>(payload.size())};
    f.insert(f.end(), payload.begin(), payload.end());
    const uint16_t crc = crc16(f.data() + 2, static_cast<uint16_t>(f.size() - 2));
    f.push_back(static_cast<uint8_t>(crc));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    return f;
}

// Request payload: header plus zero body bytes.
Bytes requestPayload(uint8_t version, uint8_t op, size_t len) {
    Bytes p(len, 0);
    p[0] = version;
    p[1] = op;
    p[6] = 0x09;
    return p;
}

// Reply payload: header plus zero body bytes.
Bytes replyPayload(uint8_t version, uint8_t op, uint8_t result, size_t len) {
    Bytes p(len, 0);
    p[0] = version;
    p[1] = op;
    p[6] = 0x09;
    p[8] = result;
    return p;
}

bool decodes(const Bytes& f, BrainRequest& out) {
    return decodeBrainRequest(f.data(), static_cast<uint16_t>(f.size()), out);
}

bool decodes(const Bytes& f, BrainReply& out) {
    return decodeBrainReply(f.data(), static_cast<uint16_t>(f.size()), out);
}

// Frames completed while pushing bytes one at a time.
std::vector<Bytes> readAll(FrameReader& r, const Bytes& stream) {
    std::vector<Bytes> frames;
    for (uint8_t b : stream) {
        if (r.push(b)) {
            frames.emplace_back(r.frame(), r.frame() + r.frameLen());
        }
    }
    return frames;
}

Bytes concat(std::initializer_list<Bytes> parts) {
    Bytes out;
    for (const Bytes& p : parts) {
        out.insert(out.end(), p.begin(), p.end());
    }
    return out;
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
}

TEST(Codec, EveryFrameFitsTheBound) {
    EXPECT_LE(sensorFrameLen(0xFFFF & ((1u << kSensorBitCount) - 1)), kMaxFrameLen);
    EXPECT_LE(kBrainRequestMaxLen + kLinkEnvelopeLen, kMaxFrameLen);
    EXPECT_LE(kBrainReplyMaxLen + kLinkEnvelopeLen, kMaxFrameLen);
}

TEST(Codec, BrainRequestLens) {
    EXPECT_EQ(brainRequestLen(kOpHello), 12);
    EXPECT_EQ(brainRequestLen(kOpSetPose), 20);
    EXPECT_EQ(brainRequestLen(kOpSelectLandmark), 10);
    EXPECT_EQ(brainRequestLen(kOpGetState), 8);
    EXPECT_EQ(brainRequestLen(0), 0);
    EXPECT_EQ(brainRequestLen(5), 0);
}

TEST(Codec, BrainReplyLens) {
    for (uint8_t result = 0; result <= kResultStale; ++result) {
        EXPECT_EQ(brainReplyLen(kOpHello, result), 17);
    }
    EXPECT_EQ(brainReplyLen(kOpHello, 200), 17);

    EXPECT_EQ(brainReplyLen(kOpSetPose, kResultOk), 21);
    EXPECT_EQ(brainReplyLen(kOpSetPose, kResultPending), 21);
    EXPECT_EQ(brainReplyLen(kOpSetPose, kResultUnknownSession), 13);
    EXPECT_EQ(brainReplyLen(kOpSetPose, kResultStale), 13);

    EXPECT_EQ(brainReplyLen(kOpSelectLandmark, kResultOk), 15);
    EXPECT_EQ(brainReplyLen(kOpSelectLandmark, kResultUnknownLandmark), 13);
    EXPECT_EQ(brainReplyLen(kOpSelectLandmark, kResultLandmarkUnsupported), 13);

    EXPECT_EQ(brainReplyLen(kOpGetState, kResultOk), 53);
    EXPECT_EQ(brainReplyLen(kOpGetState, kResultUnknownSession), 13);

    EXPECT_EQ(brainReplyLen(kOpSetPose, 9), 0);
    EXPECT_EQ(brainReplyLen(9, kResultUnsupportedOp), 0);
}

// ---------------------------------------------------------------------------
// Sensor frame
// ---------------------------------------------------------------------------

TEST(Codec, EncodeSensorMatchesKnownBytes) {
    uint8_t        buf[kMaxFrameLen];
    const uint16_t n = encodeSensorFrame(makeSample(), buf, sizeof(buf));
    ASSERT_EQ(n, kSensorVector.size());
    EXPECT_EQ(Bytes(buf, buf + n), kSensorVector);
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

TEST(Codec, CorruptChecksumRejected) {
    Bytes bad = kSensorVector;
    bad.back() ^= 0xFF;
    SensorSample s{};
    EXPECT_FALSE(decodeSensorFrame(bad.data(), static_cast<uint16_t>(bad.size()), s));
}

TEST(Codec, CorruptPayloadRejected) {
    Bytes bad = kSensorVector;
    bad[12] ^= 0x01;
    SensorSample s{};
    EXPECT_FALSE(decodeSensorFrame(bad.data(), static_cast<uint16_t>(bad.size()), s));
}

TEST(Codec, TruncatedFramesRejectedWithoutCrash) {
    SensorSample s{};
    for (uint16_t n = 0; n < kSensorVector.size(); ++n) {
        EXPECT_FALSE(decodeSensorFrame(kSensorVector.data(), n, s));
    }
}

TEST(Codec, BadSyncRejected) {
    Bytes bad = kSensorVector;
    bad[0]    = 0x00;
    SensorSample s{};
    EXPECT_FALSE(decodeSensorFrame(bad.data(), static_cast<uint16_t>(bad.size()), s));
}

TEST(Codec, WrongTypeRejected) {
    SensorSample s{};
    EXPECT_FALSE(decodeSensorFrame(kGetStateRequest.data(),
                                   static_cast<uint16_t>(kGetStateRequest.size()), s));
    BrainRequest req{};
    EXPECT_FALSE(decodes(kSensorVector, req));
    EXPECT_FALSE(decodes(kHelloOkReply, req));
    BrainReply rep{};
    EXPECT_FALSE(decodes(kHelloRequest, rep));
}

TEST(Codec, UnknownMaskBitNotEncodable) {
    SensorSample in{};
    in.mask = static_cast<uint16_t>(1u << kSensorBitCount);
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodeSensorFrame(in, buf, sizeof(buf)), 0);
}

TEST(Codec, EncodeRespectsCapacity) {
    uint8_t buf[kMaxFrameLen];
    EXPECT_EQ(encodeSensorFrame(makeSample(), buf, 4), 0);

    BrainRequest req = makeRequest(kOpGetState, kSession, 1);
    EXPECT_EQ(encodeBrainRequest(req, buf, 13), 0);
    EXPECT_EQ(encodeBrainRequest(req, buf, 14), 14);

    BrainReply rep = makeReply(kOpGetState, kSession, 1, kResultOk);
    EXPECT_EQ(encodeBrainReply(rep, buf, 58), 0);
    EXPECT_EQ(encodeBrainReply(rep, buf, 59), 59);
}

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------

TEST(Crc, CheckValue) {
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(crc16(text, sizeof(text)), 0x29B1);
}

TEST(Crc, EmptyIsInit) {
    EXPECT_EQ(crc16(nullptr, 0), 0xFFFF);
}

TEST(Crc, CoversTypeLenAndPayloadNotSync) {
    // GET_STATE request: crc over bytes 2..11, stored little endian.
    const uint16_t crc = crc16(kGetStateRequest.data() + 2, 10);
    EXPECT_EQ(crc, 0xA186);

    Bytes other_sync = kGetStateRequest;
    other_sync[0]    = 0x00;
    other_sync[1]    = 0x00;
    EXPECT_EQ(crc16(other_sync.data() + 2, 10), crc);
}

// ---------------------------------------------------------------------------
// Brain requests, known bytes
// ---------------------------------------------------------------------------

TEST(BrainRequest, HelloKnownBytes) {
    BrainRequest r = makeRequest(kOpHello, 0, 1);
    r.nonce        = kNonce;
    EXPECT_EQ(encode(r), kHelloRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kHelloRequest, out));
    EXPECT_EQ(out.version, 3);
    EXPECT_EQ(out.op, kOpHello);
    EXPECT_EQ(out.session, 0u);
    EXPECT_EQ(out.request_id, 1);
    EXPECT_EQ(out.nonce, kNonce);
}

TEST(BrainRequest, SetPoseKnownBytes) {
    BrainRequest r = makeRequest(kOpSetPose, kSession, 2);
    r.x_mm         = 610;
    r.y_mm         = -457;
    r.heading_cdeg = -9000;
    EXPECT_EQ(encode(r), kSetPoseRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kSetPoseRequest, out));
    EXPECT_EQ(out.op, kOpSetPose);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 2);
    EXPECT_EQ(out.x_mm, 610);
    EXPECT_EQ(out.y_mm, -457);
    EXPECT_EQ(out.heading_cdeg, -9000);
}

TEST(BrainRequest, SelectKnownBytes) {
    BrainRequest r = makeRequest(kOpSelectLandmark, kSession, 3);
    r.landmark_id  = 5;
    r.select_flags = kSelectFlagSelected;
    EXPECT_EQ(encode(r), kSelectRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kSelectRequest, out));
    EXPECT_EQ(out.op, kOpSelectLandmark);
    EXPECT_EQ(out.request_id, 3);
    EXPECT_EQ(out.landmark_id, 5);
    EXPECT_EQ(out.select_flags, kSelectFlagSelected);
}

TEST(BrainRequest, GetStateKnownBytes) {
    EXPECT_EQ(encode(makeRequest(kOpGetState, kSession, 0xFFFF)), kGetStateRequest);

    BrainRequest out{};
    ASSERT_TRUE(decodes(kGetStateRequest, out));
    EXPECT_EQ(out.op, kOpGetState);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 0xFFFF);
}

TEST(BrainRequest, BodyFieldsOfOtherOpsAreNotEncoded) {
    BrainRequest r = makeRequest(kOpGetState, kSession, 0xFFFF);
    r.nonce        = 99;
    r.x_mm         = 99;
    r.landmark_id  = 99;
    EXPECT_EQ(encode(r), kGetStateRequest);
}

TEST(BrainRequest, RoundTripExtremes) {
    BrainRequest r = makeRequest(kOpSetPose, 0xFFFFFFFF, 0x8000);
    r.x_mm         = INT32_MIN;
    r.y_mm         = INT32_MAX;
    r.heading_cdeg = 18000;

    BrainRequest out{};
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.session, 0xFFFFFFFFu);
    EXPECT_EQ(out.request_id, 0x8000);
    EXPECT_EQ(out.x_mm, INT32_MIN);
    EXPECT_EQ(out.y_mm, INT32_MAX);
    EXPECT_EQ(out.heading_cdeg, 18000);
}

TEST(BrainRequest, RequestIdZeroDecodes) {
    // The Pi answers it with kResultInvalidArgument, so it must reach the Pi.
    BrainRequest out{};
    ASSERT_TRUE(decodes(encode(makeRequest(kOpGetState, kSession, 0)), out));
    EXPECT_EQ(out.request_id, 0);
}

// ---------------------------------------------------------------------------
// Brain requests, rejection and versioning
// ---------------------------------------------------------------------------

TEST(BrainRequest, WrongBodyLengthForKnownOpRejected) {
    BrainRequest out{};
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, kOpHello, 8)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, kOpHello, 13)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, kOpSetPose, 12)), out));
    EXPECT_FALSE(
        decodes(linkFrame(kFrameBrainRequest, requestPayload(3, kOpSelectLandmark, 9)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, kOpGetState, 10)), out));

    EXPECT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, kOpGetState, 8)), out));
}

TEST(BrainRequest, UnknownOpDecodesHeaderOnly) {
    BrainRequest out{};
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, 0x7F, 13)), out));
    EXPECT_EQ(out.version, 3);
    EXPECT_EQ(out.op, 0x7F);
    EXPECT_EQ(out.request_id, 9);

    // Encoding an unknown op writes the header only.
    const Bytes f = encode(makeRequest(0x7F, kSession, 4));
    ASSERT_EQ(f.size(), 14u);
    EXPECT_EQ(f[3], 8);
}

TEST(BrainRequest, OtherVersionDecodesHeaderOnly) {
    BrainRequest r = makeRequest(kOpSetPose, kSession, 7);
    r.version      = 4;
    r.x_mm         = 1234;

    BrainRequest out{};
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.version, 4);
    EXPECT_EQ(out.op, kOpSetPose);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 7);
    EXPECT_EQ(out.x_mm, 0);

    // A length that is wrong for v3 is fine for another version.
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(2, kOpHello, 30)), out));
    EXPECT_EQ(out.version, 2);
    EXPECT_EQ(out.nonce, 0u);
}

TEST(BrainRequest, LenBounds) {
    BrainRequest out{};
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, Bytes(7, 0x03)), out));
    EXPECT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, 0x7F, 8)), out));
    EXPECT_TRUE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, 0x7F, 32)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainRequest, requestPayload(3, 0x7F, 33)), out));
}

TEST(BrainRequest, CorruptionRejected) {
    BrainRequest out{};
    for (size_t i = 0; i < kSetPoseRequest.size(); ++i) {
        Bytes bad = kSetPoseRequest;
        bad[i] ^= 0x01;
        EXPECT_FALSE(decodes(bad, out)) << "byte " << i;
    }
}

TEST(BrainRequest, TruncatedAndPaddedRejected) {
    BrainRequest out{};
    for (uint16_t n = 0; n < kSetPoseRequest.size(); ++n) {
        EXPECT_FALSE(decodeBrainRequest(kSetPoseRequest.data(), n, out));
    }
    Bytes padded = kSetPoseRequest;
    padded.push_back(0x00);
    EXPECT_FALSE(decodes(padded, out));
}

// ---------------------------------------------------------------------------
// Brain replies, known bytes
// ---------------------------------------------------------------------------

TEST(BrainReply, HelloOkKnownBytes) {
    BrainReply r = makeReply(kOpHello, kSession, 1, kResultOk);
    r.nonce      = kNonce;
    EXPECT_EQ(encode(r), kHelloOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kHelloOkReply, out));
    EXPECT_EQ(out.version, 3);
    EXPECT_EQ(out.op, kOpHello);
    EXPECT_EQ(out.session, kSession);
    EXPECT_EQ(out.request_id, 1);
    EXPECT_EQ(out.result, kResultOk);
    EXPECT_EQ(out.pi_instance, kInstance);
    EXPECT_EQ(out.nonce, kNonce);
}

TEST(BrainReply, HelloStaleKnownBytes) {
    BrainReply r = makeReply(kOpHello, 0, 1, kResultStale);
    r.nonce      = kNonce;
    EXPECT_EQ(encode(r), kHelloStaleReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kHelloStaleReply, out));
    EXPECT_EQ(out.result, kResultStale);
    EXPECT_EQ(out.session, 0u);
    EXPECT_EQ(out.nonce, kNonce);
}

TEST(BrainReply, SetPoseOkKnownBytes) {
    BrainReply r      = makeReply(kOpSetPose, kSession, 2, kResultOk);
    r.odometry_epoch  = 2;
    r.anchor_revision = 7;
    EXPECT_EQ(encode(r), kSetPoseOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kSetPoseOkReply, out));
    EXPECT_EQ(out.op, kOpSetPose);
    EXPECT_EQ(out.result, kResultOk);
    EXPECT_EQ(out.odometry_epoch, 2u);
    EXPECT_EQ(out.anchor_revision, 7u);
}

TEST(BrainReply, SetPosePendingKnownBytes) {
    BrainReply r      = makeReply(kOpSetPose, kSession, 2, kResultPending);
    r.odometry_epoch  = 2;
    r.anchor_revision = 6;
    EXPECT_EQ(encode(r), kSetPosePendingReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kSetPosePendingReply, out));
    EXPECT_EQ(out.result, kResultPending);
    EXPECT_EQ(out.odometry_epoch, 2u);
    EXPECT_EQ(out.anchor_revision, 6u);
}

TEST(BrainReply, ErrorResultHasNoBody) {
    BrainReply r      = makeReply(kOpSetPose, kSession, 2, kResultUnknownSession);
    r.odometry_epoch  = 2;
    r.anchor_revision = 7;
    EXPECT_EQ(encode(r), kSetPoseUnknownSessionReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kSetPoseUnknownSessionReply, out));
    EXPECT_EQ(out.result, kResultUnknownSession);
    EXPECT_EQ(out.pi_instance, kInstance);
    EXPECT_EQ(out.odometry_epoch, 0u);
    EXPECT_EQ(out.anchor_revision, 0u);
}

TEST(BrainReply, SelectOkKnownBytes) {
    BrainReply r   = makeReply(kOpSelectLandmark, kSession, 3, kResultOk);
    r.landmark_id  = 5;
    r.select_flags = kSelectFlagSelected;
    EXPECT_EQ(encode(r), kSelectOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kSelectOkReply, out));
    EXPECT_EQ(out.op, kOpSelectLandmark);
    EXPECT_EQ(out.landmark_id, 5);
    EXPECT_EQ(out.select_flags, kSelectFlagSelected);
}

TEST(BrainReply, GetStateOkKnownBytes) {
    BrainReply r = makeReply(kOpGetState, kSession, 0xFFFF, kResultOk);
    r.state      = makeState();
    EXPECT_EQ(encode(r), kGetStateOkReply);

    BrainReply out{};
    ASSERT_TRUE(decodes(kGetStateOkReply, out));
    EXPECT_EQ(out.op, kOpGetState);
    EXPECT_EQ(out.request_id, 0xFFFF);
    const BrainState& s = out.state;
    EXPECT_EQ(s.robot_flags, 0x0F);
    EXPECT_EQ(s.x_mm, 1500);
    EXPECT_EQ(s.y_mm, -250);
    EXPECT_EQ(s.heading_cdeg, 18000);
    EXPECT_EQ(s.robot_age_ms, 35);
    EXPECT_EQ(s.odometry_epoch, 2u);
    EXPECT_EQ(s.anchor_revision, 7u);
    EXPECT_EQ(s.health, 0x0B);
    EXPECT_EQ(s.landmark_id, 3);
    EXPECT_EQ(s.landmark_source, kLandmarkSourceObserved);
    EXPECT_EQ(s.lm_x_mm, 2000);
    EXPECT_EQ(s.lm_y_mm, 1000);
    EXPECT_EQ(s.lm_heading_cdeg, -4500);
    EXPECT_EQ(s.landmark_age_ms, 120);
}

TEST(BrainReply, GetStateRoundTripExtremes) {
    BrainReply r              = makeReply(kOpGetState, 1, 1, kResultOk);
    r.state.x_mm              = INT32_MIN;
    r.state.y_mm              = INT32_MAX;
    r.state.heading_cdeg      = -17999;
    r.state.robot_age_ms      = 65535;
    r.state.odometry_epoch    = 0xFFFFFFFF;
    r.state.anchor_revision   = 0x80000000;
    r.state.lm_heading_cdeg   = INT32_MIN;
    r.state.landmark_age_ms   = 65535;
    r.state.robot_flags       = 0xFF;
    r.state.landmark_source   = 0xFE;

    BrainReply out{};
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.state.x_mm, INT32_MIN);
    EXPECT_EQ(out.state.y_mm, INT32_MAX);
    EXPECT_EQ(out.state.heading_cdeg, -17999);
    EXPECT_EQ(out.state.robot_age_ms, 65535);
    EXPECT_EQ(out.state.odometry_epoch, 0xFFFFFFFFu);
    EXPECT_EQ(out.state.anchor_revision, 0x80000000u);
    EXPECT_EQ(out.state.lm_heading_cdeg, INT32_MIN);
    EXPECT_EQ(out.state.landmark_age_ms, 65535);
    EXPECT_EQ(out.state.robot_flags, 0xFF);
    EXPECT_EQ(out.state.landmark_source, 0xFE);
}

// ---------------------------------------------------------------------------
// Brain replies, rejection and versioning
// ---------------------------------------------------------------------------

TEST(BrainReply, WrongBodyLengthForKnownPairRejected) {
    BrainReply out{};
    const auto frame = [](uint8_t op, uint8_t result, size_t len) {
        return linkFrame(kFrameBrainReply, replyPayload(3, op, result, len));
    };
    EXPECT_FALSE(decodes(frame(kOpHello, kResultOk, 13), out));
    EXPECT_FALSE(decodes(frame(kOpHello, kResultStale, 13), out));
    EXPECT_FALSE(decodes(frame(kOpSetPose, kResultOk, 13), out));
    EXPECT_FALSE(decodes(frame(kOpSetPose, kResultUnknownSession, 21), out));
    EXPECT_FALSE(decodes(frame(kOpSelectLandmark, kResultOk, 13), out));
    EXPECT_FALSE(decodes(frame(kOpGetState, kResultOk, 13), out));
    EXPECT_FALSE(decodes(frame(kOpGetState, kResultOk, 52), out));
    EXPECT_FALSE(decodes(frame(kOpGetState, kResultStale, 53), out));

    EXPECT_TRUE(decodes(frame(kOpGetState, kResultStale, 13), out));
    EXPECT_TRUE(decodes(frame(kOpGetState, kResultOk, 53), out));
}

TEST(BrainReply, UnknownOpOrResultDecodesHeaderOnly) {
    BrainReply out{};
    ASSERT_TRUE(decodes(encode(makeReply(0x7F, kSession, 5, kResultUnsupportedOp)), out));
    EXPECT_EQ(out.op, 0x7F);
    EXPECT_EQ(out.result, kResultUnsupportedOp);
    EXPECT_EQ(out.request_id, 5);

    ASSERT_TRUE(decodes(linkFrame(kFrameBrainReply, replyPayload(3, kOpGetState, 99, 53)), out));
    EXPECT_EQ(out.result, 99);
    EXPECT_EQ(out.state.x_mm, 0);
}

TEST(BrainReply, OtherVersionDecodesHeaderOnly) {
    // A future Pi answering UnsupportedVersion with its own version and body.
    Bytes p = replyPayload(9, kOpGetState, kResultUnsupportedVersion, 40);
    p[2]    = 0x44;
    p[9]    = 0x0D;
    p[20]   = 0x77;
    BrainReply out{};
    ASSERT_TRUE(decodes(linkFrame(kFrameBrainReply, p), out));
    EXPECT_EQ(out.version, 9);
    EXPECT_EQ(out.op, kOpGetState);
    EXPECT_EQ(out.session, 0x44u);
    EXPECT_EQ(out.request_id, 9);
    EXPECT_EQ(out.result, kResultUnsupportedVersion);
    EXPECT_EQ(out.pi_instance, 0x0Du);
    EXPECT_EQ(out.state.robot_flags, 0);
    EXPECT_EQ(out.state.robot_age_ms, 0);

    BrainReply r = makeReply(kOpSetPose, kSession, 2, kResultOk);
    r.version    = 2;
    r.odometry_epoch = 5;
    ASSERT_TRUE(decodes(encode(r), out));
    EXPECT_EQ(out.version, 2);
    EXPECT_EQ(out.odometry_epoch, 0u);
}

TEST(BrainReply, LenBounds) {
    BrainReply out{};
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainReply, Bytes(12, 0x03)), out));
    EXPECT_TRUE(decodes(linkFrame(kFrameBrainReply, replyPayload(3, 0x7F, 0, 13)), out));
    EXPECT_TRUE(decodes(linkFrame(kFrameBrainReply, replyPayload(3, 0x7F, 0, 64)), out));
    EXPECT_FALSE(decodes(linkFrame(kFrameBrainReply, replyPayload(3, 0x7F, 0, 65)), out));
}

TEST(BrainReply, CorruptionRejected) {
    BrainReply out{};
    for (size_t i = 0; i < kGetStateOkReply.size(); ++i) {
        Bytes bad = kGetStateOkReply;
        bad[i] ^= 0x80;
        EXPECT_FALSE(decodes(bad, out)) << "byte " << i;
    }
}

TEST(BrainReply, TruncatedRejected) {
    BrainReply out{};
    for (uint16_t n = 0; n < kGetStateOkReply.size(); ++n) {
        EXPECT_FALSE(decodeBrainReply(kGetStateOkReply.data(), n, out));
    }
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

    Bytes bad = kSensorVector;
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

TEST(Reader, RejectsUnknownFrameType) {
    FrameReader r;
    Bytes       bad = kSensorVector;
    bad[2]          = 0x7F;
    for (uint8_t b : bad) {
        EXPECT_FALSE(r.push(b));
    }
}

TEST(Reader, RetiredTypesRejected) {
    FrameReader r;
    for (uint8_t type : {0x02, 0x03}) {
        Bytes bad = kSensorVector;
        bad[2]    = type;
        EXPECT_TRUE(readAll(r, bad).empty());
    }
}

TEST(Reader, LinkFrameCompletesOnLastByteOnly) {
    FrameReader r;
    for (size_t i = 0; i + 1 < kGetStateOkReply.size(); ++i) {
        EXPECT_FALSE(r.push(kGetStateOkReply[i]));
    }
    ASSERT_TRUE(r.push(kGetStateOkReply.back()));
    EXPECT_EQ(r.frameType(), kFrameBrainReply);
    EXPECT_EQ(r.frameLen(), kGetStateOkReply.size());

    BrainReply out{};
    ASSERT_TRUE(decodeBrainReply(r.frame(), r.frameLen(), out));
    EXPECT_EQ(out.state.x_mm, 1500);
}

TEST(Reader, InterleavedSensorAndLinkFrames) {
    const Bytes stream = concat({kSensorVector, kHelloRequest, {0x00, 0xAA, 0x13},
                                 kGetStateOkReply, kSensorVector, kSetPoseRequest,
                                 kSelectOkReply, {0x55}, kGetStateRequest});
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, stream);
    ASSERT_EQ(frames.size(), 7u);
    EXPECT_EQ(frames[0], kSensorVector);
    EXPECT_EQ(frames[1], kHelloRequest);
    EXPECT_EQ(frames[2], kGetStateOkReply);
    EXPECT_EQ(frames[3], kSensorVector);
    EXPECT_EQ(frames[4], kSetPoseRequest);
    EXPECT_EQ(frames[5], kSelectOkReply);
    EXPECT_EQ(frames[6], kGetStateRequest);
}

TEST(Reader, CrcErrorThenRelock) {
    Bytes bad = kSetPoseRequest;
    bad[14] ^= 0x10;
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({bad, kGetStateRequest}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kGetStateRequest);
}

TEST(Reader, OutOfRangeLenRejectedAtOnce) {
    // len 7 and 33 for a request, 12 and 65 for a reply.
    for (const Bytes& head : {Bytes{0xAA, 0x55, 0x10, 7}, Bytes{0xAA, 0x55, 0x10, 33},
                              Bytes{0xAA, 0x55, 0x11, 12}, Bytes{0xAA, 0x55, 0x11, 65}}) {
        FrameReader r;
        const std::vector<Bytes> frames = readAll(r, concat({head, kSelectRequest}));
        ASSERT_EQ(frames.size(), 1u);
        EXPECT_EQ(frames[0], kSelectRequest);
    }
}

TEST(Reader, RescansAfterCorruptedLen) {
    // SELECT with len 10 corrupted to 32 claims 38 bytes and swallows the next
    // GET_STATE request and part of a HELLO. Both must still be read.
    Bytes bad = kSelectRequest;
    bad[3]    = 32;
    FrameReader r;
    const std::vector<Bytes> frames =
        readAll(r, concat({bad, kGetStateRequest, kHelloRequest}));
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0], kGetStateRequest);
    EXPECT_EQ(frames[1], kHelloRequest);
}

TEST(Reader, RescanFindsFrameInsideRejectedCandidate) {
    // Cut-off sensor header: the reply after it is read as an invalid mask.
    FrameReader              r;
    const std::vector<Bytes> frames = readAll(r, concat({{0xAA, 0x55, 0x01}, kHelloOkReply}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kHelloOkReply);
}

TEST(Reader, ShortenedLenDoesNotLoseNextFrame) {
    // HELLO with len 12 corrupted to 8: the crc check lands inside the body.
    Bytes bad = kHelloRequest;
    bad[3]    = 8;
    FrameReader r;
    const std::vector<Bytes> frames = readAll(r, concat({bad, kSetPoseOkReply}));
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kSetPoseOkReply);
}

TEST(Reader, ResetDropsPartialFrame) {
    FrameReader r;
    for (size_t i = 0; i < 10; ++i) {
        r.push(kGetStateOkReply[i]);
    }
    r.reset();
    const std::vector<Bytes> frames = readAll(r, kSelectRequest);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0], kSelectRequest);
}
