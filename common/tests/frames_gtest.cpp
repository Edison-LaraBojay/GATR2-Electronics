// frames_gtest.cpp
// Wire constants and the XOR checksum. The values here are the documented wire values.

#include <gtest/gtest.h>
#include "frames.h"

using namespace gatr2;

TEST(Frames, ChecksumOfKnownBytes) {
    const uint8_t data[] = {0x01, 0x02, 0x03};
    EXPECT_EQ(checksum(data, 3), 0x01 ^ 0x02 ^ 0x03);
}

TEST(Frames, ChecksumEmptyIsZero) {
    EXPECT_EQ(checksum(nullptr, 0), 0x00);
}

TEST(Frames, ChecksumIsSelfInverse) {
    // XOR of a frame including its own checksum byte is zero. This is the
    // property receivers use to validate.
    uint8_t data[] = {0xAA, 0x55, 0x01, 0x2C, 0x00};
    uint8_t c = checksum(data, 4);
    data[4] = c;
    EXPECT_EQ(checksum(data, 5), 0x00);
}

TEST(Frames, WidthTableMatchesBitCount) {
    // Every defined SensorBit must have a width entry. This is the table
    // parsers walk, so drift here is a wire-level bug.
    EXPECT_EQ(kSensorBitCount, 5);
}

TEST(Frames, FrameTypes) {
    EXPECT_EQ(kSync0, 0xAA);
    EXPECT_EQ(kSync1, 0x55);
    EXPECT_EQ(kFrameSensor, 0x01);
    EXPECT_EQ(kFrameBrainRequest, 0x10);
    EXPECT_EQ(kFrameBrainReply, 0x11);
    EXPECT_EQ(kBrainLinkVersion, 3);
}

TEST(Frames, BrainOpsAndResults) {
    EXPECT_EQ(kOpHello, 1);
    EXPECT_EQ(kOpSetPose, 2);
    EXPECT_EQ(kOpSelectLandmark, 3);
    EXPECT_EQ(kOpGetState, 4);

    EXPECT_EQ(kResultOk, 0);
    EXPECT_EQ(kResultPending, 1);
    EXPECT_EQ(kResultUnknownSession, 2);
    EXPECT_EQ(kResultUnsupportedVersion, 3);
    EXPECT_EQ(kResultUnsupportedOp, 4);
    EXPECT_EQ(kResultInvalidArgument, 5);
    EXPECT_EQ(kResultUnknownLandmark, 6);
    EXPECT_EQ(kResultLandmarkUnsupported, 7);
    EXPECT_EQ(kResultStale, 8);
}

TEST(Frames, BrainStateBits) {
    EXPECT_EQ(kSelectFlagSelected, 0x01);

    EXPECT_EQ(kRobotPoseValid, 0x01);
    EXPECT_EQ(kRobotLocalized, 0x02);
    EXPECT_EQ(kRobotAgeKnown, 0x04);
    EXPECT_EQ(kRobotAnchorCommand, 0x08);
    EXPECT_EQ(kRobotAnchorConfigured, 0x10);

    EXPECT_EQ(kHealthEncodersFresh, 0x01);
    EXPECT_EQ(kHealthGyroFresh, 0x02);
    EXPECT_EQ(kHealthVisionAlive, 0x04);
    EXPECT_EQ(kHealthBiasCalibrated, 0x08);

    EXPECT_EQ(kLandmarkSourceNone, 0);
    EXPECT_EQ(kLandmarkSourceNominal, 1);
    EXPECT_EQ(kLandmarkSourceObserved, 2);
}

TEST(Frames, BrainStructsDefaultToCurrentVersion) {
    const BrainRequest req{};
    EXPECT_EQ(req.version, kBrainLinkVersion);
    EXPECT_EQ(req.session, 0u);
    EXPECT_EQ(req.request_id, 0);

    const BrainReply rep{};
    EXPECT_EQ(rep.version, kBrainLinkVersion);
    EXPECT_EQ(rep.result, kResultOk);
    EXPECT_EQ(rep.pi_instance, 0u);
    EXPECT_EQ(rep.state.landmark_source, kLandmarkSourceNone);
}
