// frames_gtest.cpp
// Wire constants and the XOR checksum. The values here are the documented wire values.

#include <gtest/gtest.h>

#include <set>

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
    uint8_t c      = checksum(data, 4);
    data[4]        = c;
    EXPECT_EQ(checksum(data, 5), 0x00);
}

TEST(Frames, WidthTableMatchesBitCount) {
    // Every defined SensorBit must have a width entry. This is the table
    // parsers walk, so drift here is a wire-level bug.
    EXPECT_EQ(kSensorBitCount, 5);
    EXPECT_EQ(kSensorEnc0, 0x01);
    EXPECT_EQ(kSensorEnc1, 0x02);
    EXPECT_EQ(kSensorEnc2, 0x04);
    EXPECT_EQ(kSensorGyroZ, 0x08);
    EXPECT_EQ(kSensorAccelXY, 0x10);
}

TEST(Frames, FrameTypes) {
    EXPECT_EQ(kSync0, 0xAA);
    EXPECT_EQ(kSync1, 0x55);
    EXPECT_EQ(kMaxFrameLen, 128);
    EXPECT_EQ(kFrameSensor, 0x01);
    EXPECT_EQ(kFrameSensorV2, 0x04);
    EXPECT_EQ(kFrameBrainRequest, 0x10);
    EXPECT_EQ(kFrameBrainReply, 0x11);
    EXPECT_EQ(kFramePicoCommand, 0x12);
    EXPECT_EQ(kFramePicoStatus, 0x13);
    EXPECT_EQ(kBrainLinkVersion, 4);
    EXPECT_EQ(kPicoLinkVersion, 1);
}

TEST(Frames, RetiredFrameTypesAreNotReused) {
    const std::set<int> types = {kFrameSensor,      kFrameSensorV2,    kFrameBrainRequest,
                                 kFrameBrainReply,  kFramePicoCommand, kFramePicoStatus};
    EXPECT_EQ(types.size(), 6u);
    EXPECT_EQ(types.count(0x02), 0u); // one-way pose frame
    EXPECT_EQ(types.count(0x03), 0u); // one-way command frame
}

TEST(Frames, BrainOps) {
    EXPECT_EQ(kOpHello, 1);
    EXPECT_EQ(kOpSetPose, 2);
    EXPECT_EQ(kOpGetState, 4);
    EXPECT_EQ(kOpProfileWrite, 6);
    EXPECT_EQ(kOpProfileApply, 7);
    EXPECT_EQ(kOpReadDoc, 8);
    EXPECT_EQ(kOpControl, 9);
    EXPECT_EQ(kOpPathReport, 10);
    EXPECT_EQ(kOpReadWheels, 11);
}

TEST(Frames, RetiredOpNumbersAreNotReused) {
    const std::set<int> ops = {kOpHello,   kOpSetPose,    kOpGetState,  kOpProfileWrite,
                               kOpProfileApply, kOpReadDoc, kOpControl, kOpPathReport,
                               kOpReadWheels};
    EXPECT_EQ(ops.size(), 9u);
    EXPECT_EQ(ops.count(0), 0u);
    EXPECT_EQ(ops.count(3), 0u); // SELECT_LANDMARK
    EXPECT_EQ(ops.count(5), 0u); // GET_STATE_WITH_IMU
}

TEST(Frames, BrainResults) {
    EXPECT_EQ(kResultOk, 0);
    EXPECT_EQ(kResultPending, 1);
    EXPECT_EQ(kResultUnknownSession, 2);
    EXPECT_EQ(kResultUnsupportedVersion, 3);
    EXPECT_EQ(kResultUnsupportedOp, 4);
    EXPECT_EQ(kResultInvalidArgument, 5);
    EXPECT_EQ(kResultStale, 8);
    EXPECT_EQ(kResultNotReady, 9);
    EXPECT_EQ(kResultProfileRejected, 10);
    EXPECT_EQ(kResultUnavailable, 11);
    EXPECT_EQ(kResultNotStationary, 12);
    EXPECT_EQ(kResultFailed, 13);
}

TEST(Frames, RetiredResultNumbersAreNotReused) {
    const std::set<int> results = {kResultOk,
                                   kResultPending,
                                   kResultUnknownSession,
                                   kResultUnsupportedVersion,
                                   kResultUnsupportedOp,
                                   kResultInvalidArgument,
                                   kResultStale,
                                   kResultNotReady,
                                   kResultProfileRejected,
                                   kResultUnavailable,
                                   kResultNotStationary,
                                   kResultFailed};
    EXPECT_EQ(results.size(), 12u);
    EXPECT_EQ(results.count(6), 0u); // UnknownLandmark
    EXPECT_EQ(results.count(7), 0u); // LandmarkUnsupported
}

TEST(Frames, StateBits) {
    EXPECT_EQ(kBenchImuValid, 0x01);

    EXPECT_EQ(kRobotPoseValid, 0x01);
    EXPECT_EQ(kRobotLocalized, 0x02);
    EXPECT_EQ(kRobotAgeKnown, 0x04);
    EXPECT_EQ(kRobotAnchorCommand, 0x08);
    EXPECT_EQ(kRobotAnchorConfigured, 0x10);

    EXPECT_EQ(kHealthEncodersFresh, 0x01);
    EXPECT_EQ(kHealthGyroFresh, 0x02);
    EXPECT_EQ(kHealthVisionAlive, 0x04);
    EXPECT_EQ(kHealthBiasCalibrated, 0x08);
    EXPECT_EQ(kHealthPicoLink, 0x10);
    EXPECT_EQ(kHealthImuInitializing, 0x20);
    EXPECT_EQ(kHealthImuFailed, 0x40);
    EXPECT_EQ(kHealthStationary, 0x80);
}

TEST(Frames, ProfileStatesAndReasons) {
    EXPECT_EQ(kProfileNone, 0);
    EXPECT_EQ(kProfileApplying, 1);
    EXPECT_EQ(kProfileApplied, 2);
    EXPECT_EQ(kProfileRejected, 3);

    EXPECT_EQ(kProfileReasonNone, 0);
    EXPECT_EQ(kProfileReasonFormat, 1);
    EXPECT_EQ(kProfileReasonTopology, 2);
    EXPECT_EQ(kProfileReasonWheelCount, 3);
    EXPECT_EQ(kProfileReasonEncoderPort, 4);
    EXPECT_EQ(kProfileReasonWheelGeometry, 5);
    EXPECT_EQ(kProfileReasonObservability, 6);
    EXPECT_EQ(kProfileReasonImuSource, 7);
    EXPECT_EQ(kProfileReasonImuPort, 8);
    EXPECT_EQ(kProfileReasonImuCombination, 9);
    EXPECT_EQ(kProfileReasonCamera, 10);
    EXPECT_EQ(kProfileReasonFootprint, 11);
    EXPECT_EQ(kProfileReasonBuild, 12);
    EXPECT_EQ(kProfileReasonNotAccepted, 13);
    EXPECT_EQ(kProfileReasonCalibration, 14);
}

TEST(Frames, ControlDocsPathsAndSources) {
    EXPECT_EQ(kCalibrationNone, 0);
    EXPECT_EQ(kCalibrationRunning, 1);
    EXPECT_EQ(kCalibrationDone, 2);
    EXPECT_EQ(kCalibrationWaitingStill, 3);
    EXPECT_EQ(kCalibrationWaitingData, 4);
    EXPECT_EQ(kCalibrationFailed, 5);

    EXPECT_EQ(kControlRecalibrate, 1);
    EXPECT_EQ(kControlReinitialize, 2);
    EXPECT_EQ(kControlReinitImu, 3);
    EXPECT_EQ(kControlRestartAcquisition, 4);

    EXPECT_EQ(kControlDetailNone, 0);
    EXPECT_EQ(kControlDetailPicoLink, 1);
    EXPECT_EQ(kControlDetailImuAbsent, 2);
    EXPECT_EQ(kControlDetailImuUnused, 3);
    EXPECT_EQ(kControlDetailPicoRefused, 4);
    EXPECT_EQ(kControlDetailTimedOut, 5);
    EXPECT_EQ(kControlDetailCalibration, 6);

    EXPECT_EQ(kWheelFresh, 0x01);
    EXPECT_EQ(kWheelValid, 0x02);

    EXPECT_EQ(kDocFieldMap, 1);
    EXPECT_EQ(kDocFieldEstimate, 2);

    EXPECT_EQ(kPathNone, 0);
    EXPECT_EQ(kPathDirect, 1);
    EXPECT_EQ(kPathAvoiding, 2);

    EXPECT_EQ(kEstimateSourceNone, 0);
    EXPECT_EQ(kEstimateSourceNominal, 1);
    EXPECT_EQ(kEstimateSourceObserved, 2);
}

TEST(Frames, ChunkCapacities) {
    EXPECT_EQ(kProfileChunkMax, 106);
    EXPECT_EQ(kDocChunkMax, 96);
    EXPECT_EQ(kPathReportMaxPoints, 13);
    EXPECT_EQ(kWheelReadingsMax, 4);
}

TEST(Frames, PicoLinkCodes) {
    EXPECT_EQ(kPicoOpConfigure, 1);
    EXPECT_EQ(kPicoOpReinitImu, 2);
    EXPECT_EQ(kPicoOpRestartAcquisition, 3);

    EXPECT_EQ(kPicoCommandNone, 0);
    EXPECT_EQ(kPicoCommandRunning, 1);
    EXPECT_EQ(kPicoCommandCompleted, 2);
    EXPECT_EQ(kPicoCommandFailed, 3);

    EXPECT_EQ(kPicoDetailNone, 0);
    EXPECT_EQ(kPicoDetailWrongTarget, 1);
    EXPECT_EQ(kPicoDetailUnknownOp, 2);
    EXPECT_EQ(kPicoDetailBadBody, 3);
    EXPECT_EQ(kPicoDetailImuAbsent, 4);
    EXPECT_EQ(kPicoDetailImuDisabled, 5);
    EXPECT_EQ(kPicoDetailNoSuchPort, 6);

    EXPECT_EQ(kPicoImuDisabled, 0);
    EXPECT_EQ(kPicoImuInitializing, 1);
    EXPECT_EQ(kPicoImuAligning, 2);
    EXPECT_EQ(kPicoImuReady, 3);
    EXPECT_EQ(kPicoImuRetrying, 4);
    EXPECT_EQ(kPicoImuFailed, 5);

    EXPECT_EQ(kPicoImuReasonNone, 0);
    EXPECT_EQ(kPicoImuReasonNoResponse, 1);
    EXPECT_EQ(kPicoImuReasonBoot, 2);
    EXPECT_EQ(kPicoImuReasonFeatures, 3);
    EXPECT_EQ(kPicoImuReasonStream, 4);

    EXPECT_EQ(kPicoImuEnabled, 0x01);

    EXPECT_EQ(kPicoFirmwareUnknown, 0);
    EXPECT_EQ(kPicoFirmwareBno08x, 1);
    EXPECT_EQ(kPicoFirmwareAsm330, 2);
}

TEST(Frames, BrainStructsDefaultToCurrentVersion) {
    const BrainRequest req{};
    EXPECT_EQ(req.version, kBrainLinkVersion);
    EXPECT_EQ(req.session, 0u);
    EXPECT_EQ(req.request_id, 0);
    EXPECT_EQ(req.imu_flags, 0);
    EXPECT_EQ(req.data_len, 0);
    EXPECT_EQ(req.path_mode, kPathNone);
    EXPECT_EQ(req.point_count, 0);
    EXPECT_EQ(req.points[kPathReportMaxPoints - 1].x_mm, 0);

    const BrainReply rep{};
    EXPECT_EQ(rep.version, kBrainLinkVersion);
    EXPECT_EQ(rep.result, kResultOk);
    EXPECT_EQ(rep.pi_instance, 0u);
    EXPECT_EQ(rep.data_len, 0);
    EXPECT_EQ(rep.state.profile_state, kProfileNone);
    EXPECT_EQ(rep.state.profile_reason, kProfileReasonNone);
    EXPECT_EQ(rep.state.profile_id, 0u);
    EXPECT_EQ(rep.state.map_id, 0u);
    EXPECT_EQ(rep.state.estimate_id, 0u);
    EXPECT_EQ(rep.state.calibration, kCalibrationNone);
    EXPECT_EQ(rep.action, 0);
    EXPECT_EQ(rep.calibration, kCalibrationNone);
    EXPECT_EQ(rep.control_detail, kControlDetailNone);
    EXPECT_EQ(rep.wheel_count, 0);
    EXPECT_EQ(rep.wheels[kWheelReadingsMax - 1].counts, 0);
}

TEST(Frames, WheelAndPicoStructDefaults) {
    const WheelReading w{};
    EXPECT_EQ(w.port, 0);
    EXPECT_EQ(w.flags, 0);
    EXPECT_EQ(w.discontinuity, 0);
    EXPECT_EQ(w.counts, 0);
    EXPECT_EQ(w.travel_um, 0);
    EXPECT_EQ(w.age_ms, 0);

    const PicoCommand c{};
    EXPECT_EQ(c.version, kPicoLinkVersion);
    EXPECT_EQ(c.op, 0);
    EXPECT_EQ(c.request_id, 0);
    EXPECT_EQ(c.target_boot_id, 0);

    const PicoStatus s{};
    EXPECT_EQ(s.version, kPicoLinkVersion);
    EXPECT_EQ(s.boot_id, 0);
    EXPECT_EQ(s.imu_state, kPicoImuDisabled);
    EXPECT_EQ(s.imu_reason, kPicoImuReasonNone);
    EXPECT_EQ(s.last_request_id, 0);
    EXPECT_EQ(s.last_status, kPicoCommandNone);
    EXPECT_EQ(s.last_detail, kPicoDetailNone);
    EXPECT_EQ(s.firmware, kPicoFirmwareUnknown);

    const SensorSample v{};
    EXPECT_FALSE(v.identity);
    EXPECT_EQ(v.boot_id, 0);
}
