// fake_pi_gtest.cpp
// The fake Pi follows the brain link v4 Pi rules the client tests rely on.

#include "sim/fake_pi.h"

#include <gtest/gtest.h>

#include "communigatr/robot_profile.h"

using namespace communigatr;

namespace
{

translagatr::BrainRequest hello(uint16_t id, uint32_t nonce) {
    translagatr::BrainRequest q;
    q.op         = translagatr::kOpHello;
    q.request_id = id;
    q.nonce      = nonce;
    return q;
}

translagatr::BrainRequest request(uint8_t op, uint32_t session, uint16_t id) {
    translagatr::BrainRequest q;
    q.op         = op;
    q.session    = session;
    q.request_id = id;
    return q;
}

translagatr::BrainRequest setPose(uint32_t session, uint16_t id, int32_t x) {
    translagatr::BrainRequest q = request(translagatr::kOpSetPose, session, id);
    q.x_mm                = x;
    return q;
}

translagatr::BrainRequest readDoc(uint32_t session, uint16_t id, uint8_t kind, uint32_t doc_id,
                            uint16_t offset, uint8_t max_len = translagatr::kDocChunkMax) {
    translagatr::BrainRequest q = request(translagatr::kOpReadDoc, session, id);
    q.doc_kind            = kind;
    q.doc_id              = doc_id;
    q.doc_offset          = offset;
    q.max_len             = max_len;
    return q;
}

translagatr::BrainRequest control(uint32_t session, uint16_t id, uint8_t action) {
    translagatr::BrainRequest q = request(translagatr::kOpControl, session, id);
    q.action              = action;
    return q;
}

ProfileDocument testProfile() {
    RobotProfile p;
    p.topology       = LocalizationTopology::kTwoWheelImu;
    p.wheels         = {{0, 0.024, 2048, 0.0, 0.05, 0.0, false},
                        {1, 0.024, 2048, -0.05, 0.0, investigatr::kPi / 2, false}};
    p.imu_source     = ImuSource::kBrainVex;
    p.vex_smart_port = 1;
    p.footprint      = {0.2, 0.2, 0.2, 0.2};
    return makeProfileDocument(p);
}

// Writes the document in chunks of chunk bytes with ids from *id.
void stage(FakePi& pi, uint32_t s, uint16_t& id, const ProfileDocument& doc,
           uint8_t chunk = translagatr::kProfileChunkMax) {
    const uint32_t profile_id = profileId(doc);
    for (uint16_t offset = 0; offset < doc.len; offset = static_cast<uint16_t>(offset + chunk)) {
        translagatr::BrainRequest q = request(translagatr::kOpProfileWrite, s, id++);
        q.profile_id          = profile_id;
        q.total_len           = doc.len;
        q.offset              = offset;
        q.data_len            = static_cast<uint8_t>(std::min<int>(chunk, doc.len - offset));
        std::copy(doc.bytes + offset, doc.bytes + offset + q.data_len, q.data);
        ASSERT_EQ(pi.answer(q).result, translagatr::kResultOk);
    }
}

translagatr::BrainReply apply(FakePi& pi, uint32_t s, uint16_t id, const ProfileDocument& doc) {
    translagatr::BrainRequest q = request(translagatr::kOpProfileApply, s, id);
    q.profile_id          = profileId(doc);
    q.total_len           = doc.len;
    return pi.answer(q);
}

} // namespace

TEST(FakePi, HelloRetryIsIdempotentAndRecentNonceIsStale) {
    FakePi                  pi;
    const translagatr::BrainReply first = pi.answer(hello(1, 77));
    ASSERT_EQ(first.result, translagatr::kResultOk);
    ASSERT_NE(first.session, 0u);
    EXPECT_EQ(first.nonce, 77u);

    const translagatr::BrainReply retry = pi.answer(hello(1, 77));
    EXPECT_EQ(retry.result, translagatr::kResultOk);
    EXPECT_EQ(retry.session, first.session);
    EXPECT_EQ(pi.sessionsOpened(), 1);

    EXPECT_EQ(pi.answer(request(translagatr::kOpGetState, first.session, 2)).result, translagatr::kResultOk);
    EXPECT_EQ(pi.answer(hello(1, 77)).result, translagatr::kResultStale);
    EXPECT_EQ(pi.session(), first.session);

    const translagatr::BrainReply second = pi.answer(hello(1, 78));
    EXPECT_EQ(second.result, translagatr::kResultOk);
    EXPECT_NE(second.session, first.session);
    EXPECT_EQ(pi.answer(request(translagatr::kOpGetState, first.session, 3)).result,
              translagatr::kResultUnknownSession);
}

TEST(FakePi, PlacementDedupedPerSessionAndAppliedOnce) {
    FakePi         pi;
    const uint32_t s = pi.answer(hello(1, 5)).session;

    const translagatr::BrainReply ok = pi.answer(setPose(s, 2, 610));
    EXPECT_EQ(ok.result, translagatr::kResultOk);
    EXPECT_EQ(ok.anchor_revision, 1u);
    EXPECT_EQ(pi.answer(request(translagatr::kOpGetState, s, 3)).result, translagatr::kResultOk);

    const translagatr::BrainReply dup = pi.answer(setPose(s, 2, 610));
    EXPECT_EQ(dup.result, translagatr::kResultOk);
    EXPECT_EQ(pi.placementsApplied(), 1);
    EXPECT_EQ(pi.answer(setPose(s, 2, 611)).result, translagatr::kResultInvalidArgument);
    EXPECT_EQ(pi.answer(request(translagatr::kOpGetState, s, 1)).result, translagatr::kResultStale);

    // Same id and pose in a new session is a new placement.
    const uint32_t s2 = pi.answer(hello(1, 6)).session;
    EXPECT_EQ(pi.answer(setPose(s2, 2, 610)).result, translagatr::kResultOk);
    EXPECT_EQ(pi.placementsApplied(), 2);
    EXPECT_EQ(pi.robot().anchor_revision, 2u);
}

TEST(FakePi, PendingUntilApplied) {
    FakePi pi;
    pi.setApplyDelay(2);
    const uint32_t s = pi.answer(hello(1, 5)).session;
    EXPECT_EQ(pi.answer(setPose(s, 2, 100)).result, translagatr::kResultPending);
    EXPECT_EQ(pi.answer(setPose(s, 2, 100)).result, translagatr::kResultPending);
    EXPECT_EQ(pi.answer(setPose(s, 2, 100)).result, translagatr::kResultOk);
    EXPECT_EQ(pi.placementsApplied(), 1);
}

TEST(FakePi, NewestIdRepeatsReadOnlyOpsWithoutReaccepting) {
    FakePi              pi;
    const uint32_t      s     = pi.answer(hello(1, 5)).session;
    translagatr::BrainRequest state = request(translagatr::kOpGetState, s, 2);
    state.imu_flags           = translagatr::kBenchImuValid;
    state.imu_stamp_ms        = 10;
    EXPECT_EQ(pi.answer(state).result, translagatr::kResultOk);
    EXPECT_EQ(pi.answer(state).result, translagatr::kResultOk);
    EXPECT_EQ(pi.imuSamples(), 1);
    state.imu_stamp_ms = 11; // same id, other body
    EXPECT_EQ(pi.answer(state).result, translagatr::kResultInvalidArgument);
    state.imu_flags = 0x02; // unknown bit
    state.request_id = 3;
    EXPECT_EQ(pi.answer(state).result, translagatr::kResultInvalidArgument);
    EXPECT_EQ(pi.answer(request(translagatr::kOpGetState, s, 0)).result,
              translagatr::kResultInvalidArgument);
}

TEST(FakePi, ProfileStagingApplyAndIdempotentRetry) {
    FakePi pi;
    pi.setProfileMode(true);
    const ProfileDocument doc = testProfile();
    ASSERT_EQ(doc.reason, translagatr::kProfileReasonNone);
    ASSERT_GT(doc.len, translagatr::kProfileChunkMax / 2);
    uint16_t       id = 2;
    const uint32_t s  = pi.answer(hello(1, 5)).session;

    // No profile: no pose, SET_POSE and CONTROL NotReady.
    translagatr::BrainReply state = pi.answer(request(translagatr::kOpGetState, s, id++));
    EXPECT_EQ(state.state.robot_flags, 0);
    EXPECT_EQ(state.state.profile_state, translagatr::kProfileNone);
    EXPECT_EQ(pi.answer(setPose(s, id++, 5)).result, translagatr::kResultNotReady);
    EXPECT_EQ(pi.answer(control(s, id++, translagatr::kControlRecalibrate)).result,
              translagatr::kResultNotReady);

    // Incomplete staging cannot be applied.
    EXPECT_EQ(apply(pi, s, id++, doc).result, translagatr::kResultInvalidArgument);
    stage(pi, s, id, doc, 40);
    EXPECT_EQ(pi.stagingReceived(), doc.len);

    const uint32_t epoch = pi.robot().odometry_epoch;
    translagatr::BrainReply r  = apply(pi, s, id++, doc);
    EXPECT_EQ(r.result, translagatr::kResultPending);
    EXPECT_EQ(r.profile_state, translagatr::kProfileApplying);
    state = pi.answer(request(translagatr::kOpGetState, s, id++)); // boundary
    EXPECT_EQ(state.state.profile_state, translagatr::kProfileApplied);
    EXPECT_EQ(state.state.profile_id, profileId(doc));
    EXPECT_EQ(state.state.odometry_epoch, epoch + 1);
    EXPECT_NE(state.state.robot_flags & translagatr::kRobotPoseValid, 0);
    EXPECT_EQ(state.state.robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_EQ(apply(pi, s, id++, doc).result, translagatr::kResultOk);

    // A new Brain session applies the same profile again: nothing resets.
    const uint32_t s2 = pi.answer(hello(1, 6)).session;
    id                = 2;
    stage(pi, s2, id, doc);
    EXPECT_EQ(apply(pi, s2, id++, doc).result, translagatr::kResultOk);
    EXPECT_EQ(pi.robot().odometry_epoch, epoch + 1);
    EXPECT_EQ(pi.profilesApplied(), 1);
}

TEST(FakePi, ProfileWriteRules) {
    FakePi                pi;
    const ProfileDocument doc = testProfile();
    const uint32_t        s   = pi.answer(hello(1, 5)).session;
    translagatr::BrainRequest   q   = request(translagatr::kOpProfileWrite, s, 2);
    q.profile_id              = profileId(doc);
    q.total_len               = doc.len;
    q.offset                  = 10; // gap
    q.data_len                = 4;
    EXPECT_EQ(pi.answer(q).result, translagatr::kResultInvalidArgument);

    q.request_id = 3;
    q.offset     = 0;
    q.data_len   = 20;
    std::copy(doc.bytes, doc.bytes + 20, q.data);
    translagatr::BrainReply r = pi.answer(q);
    EXPECT_EQ(r.result, translagatr::kResultOk);
    EXPECT_EQ(r.received, 20);

    // Resend with the same bytes is fine; other bytes are not.
    q.request_id = 4;
    EXPECT_EQ(pi.answer(q).received, 20);
    q.request_id = 5;
    q.data[3] ^= 0xFF;
    EXPECT_EQ(pi.answer(q).result, translagatr::kResultInvalidArgument);

    // total_len out of range.
    q.request_id = 6;
    q.total_len  = translagatr::kProfileHeaderLen - 1;
    EXPECT_EQ(pi.answer(q).result, translagatr::kResultInvalidArgument);
}

TEST(FakePi, ProfileRejectionsAreRemembered) {
    FakePi pi;
    pi.setProfileMode(false);
    const ProfileDocument doc = testProfile();
    uint16_t              id  = 2;
    const uint32_t        s   = pi.answer(hello(1, 5)).session;
    stage(pi, s, id, doc);
    translagatr::BrainReply r = apply(pi, s, id++, doc);
    EXPECT_EQ(r.result, translagatr::kResultProfileRejected);
    EXPECT_EQ(r.profile_reason, translagatr::kProfileReasonNotAccepted);

    FakePi capable;
    capable.setProfileMode(true);
    capable.setProfileRejection(translagatr::kProfileReasonEncoderPort, 1);
    const uint32_t s2 = capable.answer(hello(1, 5)).session;
    id                = 2;
    stage(capable, s2, id, doc);
    r = apply(capable, s2, id++, doc);
    EXPECT_EQ(r.result, translagatr::kResultProfileRejected);
    EXPECT_EQ(r.profile_reason, translagatr::kProfileReasonEncoderPort);
    EXPECT_EQ(r.profile_detail, 1);
    capable.setProfileRejection(translagatr::kProfileReasonNone);
    EXPECT_EQ(apply(capable, s2, id++, doc).profile_reason, translagatr::kProfileReasonEncoderPort);
    const translagatr::BrainReply state = capable.answer(request(translagatr::kOpGetState, s2, id++));
    EXPECT_EQ(state.state.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(state.state.profile_id, profileId(doc));
}

TEST(FakePi, DocumentsServeChunksAndReportStaleAndUnavailable) {
    FakePi         pi;
    const uint32_t s  = pi.answer(hello(1, 5)).session;
    uint16_t       id = 2;
    EXPECT_EQ(pi.answer(readDoc(s, id++, translagatr::kDocFieldMap, 0, 0)).result,
              translagatr::kResultUnavailable);
    EXPECT_EQ(pi.answer(readDoc(s, id++, translagatr::kDocFieldEstimate, 0, 0)).result,
              translagatr::kResultUnavailable);

    pi.setField(makeFakeField(7));
    const uint16_t map_len = translagatr::fieldMapLen(7);
    EXPECT_EQ(translagatr::validateFieldMap(pi.mapDocument().data(), map_len), translagatr::DocError::kNone);
    translagatr::BrainReply r = pi.answer(readDoc(s, id++, translagatr::kDocFieldMap, pi.mapId(), 0));
    ASSERT_EQ(r.result, translagatr::kResultOk);
    EXPECT_EQ(r.doc_id, pi.mapId());
    EXPECT_EQ(r.doc_total_len, map_len);
    EXPECT_EQ(r.doc_crc32, pi.mapId());
    EXPECT_EQ(r.data_len, translagatr::kDocChunkMax);
    r = pi.answer(readDoc(s, id++, translagatr::kDocFieldMap, pi.mapId(), 192, 50));
    EXPECT_EQ(r.data_len, map_len - 192);
    EXPECT_EQ(pi.answer(readDoc(s, id++, translagatr::kDocFieldMap, pi.mapId(), map_len)).result,
              translagatr::kResultInvalidArgument);
    EXPECT_EQ(pi.answer(readDoc(s, id++, translagatr::kDocFieldMap, pi.mapId() + 1, 0)).result,
              translagatr::kResultStale);

    const uint32_t first = pi.newestEstimate();
    EXPECT_EQ(pi.answer(readDoc(s, id++, translagatr::kDocFieldEstimate, first, 0)).result,
              translagatr::kResultOk);
    pi.publishNominalEstimate();
    pi.publishNominalEstimate();
    EXPECT_EQ(pi.answer(readDoc(s, id++, translagatr::kDocFieldEstimate, first, 0)).result,
              translagatr::kResultOk);
    pi.publishNominalEstimate(); // fourth: the first is no longer kept
    EXPECT_EQ(pi.answer(readDoc(s, id++, translagatr::kDocFieldEstimate, first, 0)).result,
              translagatr::kResultStale);
    r = pi.answer(readDoc(s, id++, translagatr::kDocFieldEstimate, 0, 0));
    EXPECT_EQ(r.doc_id, first + 3);

    const translagatr::BrainReply state = pi.answer(request(translagatr::kOpGetState, s, id++));
    EXPECT_EQ(state.state.map_id, pi.mapId());
    EXPECT_EQ(state.state.estimate_id, first + 3);
}

TEST(FakePi, ControlStationaryCheckAndReinitialize) {
    FakePi         pi;
    const uint32_t s  = pi.answer(hello(1, 5)).session;
    uint16_t       id = 2;
    ASSERT_EQ(pi.answer(setPose(s, id++, 100)).result, translagatr::kResultOk);
    const uint32_t epoch = pi.robot().odometry_epoch;

    pi.setMoving(true);
    EXPECT_EQ(pi.answer(control(s, id++, translagatr::kControlReinitialize)).result,
              translagatr::kResultNotStationary);
    EXPECT_EQ(pi.robot().odometry_epoch, epoch);
    pi.setMoving(false);

    const translagatr::BrainRequest reinit = control(s, id++, translagatr::kControlReinitialize);
    translagatr::BrainReply         r      = pi.answer(reinit);
    EXPECT_EQ(r.result, translagatr::kResultOk);
    EXPECT_EQ(r.action, translagatr::kControlReinitialize);
    EXPECT_EQ(r.calibration, translagatr::kCalibrationRunning);
    EXPECT_EQ(pi.robot().odometry_epoch, epoch + 1);
    EXPECT_EQ(pi.robot().robot_flags & translagatr::kRobotLocalized, 0);

    // A duplicate is answered from the record, never executed again.
    EXPECT_EQ(pi.answer(request(translagatr::kOpGetState, s, id++)).result, translagatr::kResultOk);
    r = pi.answer(reinit);
    EXPECT_EQ(r.result, translagatr::kResultOk);
    EXPECT_EQ(pi.controlsExecuted(), 1);
    EXPECT_EQ(pi.robot().odometry_epoch, epoch + 1);
}

TEST(FakePi, PathReportLatestWinsAndSessionClears) {
    FakePi              pi;
    const uint32_t      s = pi.answer(hello(1, 5)).session;
    translagatr::BrainRequest q = request(translagatr::kOpPathReport, s, 2);
    q.command_id          = 9;
    q.path_mode           = translagatr::kPathAvoiding;
    q.point_count         = 2;
    q.points[1].x_mm      = 500;
    EXPECT_EQ(pi.answer(q).result, translagatr::kResultOk);
    ASSERT_TRUE(pi.path().have);
    EXPECT_EQ(pi.path().points.size(), 2u);
    EXPECT_EQ(pi.path().points[1].x_mm, 500);

    q.request_id  = 3;
    q.path_mode   = translagatr::kPathNone;
    q.point_count = 0;
    EXPECT_EQ(pi.answer(q).result, translagatr::kResultOk);
    EXPECT_FALSE(pi.path().have);

    q.request_id = 4;
    q.path_mode  = 3;
    EXPECT_EQ(pi.answer(q).result, translagatr::kResultInvalidArgument);
}

TEST(FakePi, RestartForgetsSessionAnchorAndProfile) {
    FakePi pi(0x11);
    pi.setProfileMode(true);
    pi.setField(makeFakeField(3));
    const ProfileDocument doc = testProfile();
    uint16_t              id  = 2;
    const uint32_t        s   = pi.answer(hello(1, 5)).session;
    stage(pi, s, id, doc);
    apply(pi, s, id++, doc);
    pi.answer(request(translagatr::kOpGetState, s, id++));
    ASSERT_EQ(pi.appliedProfile(), profileId(doc));
    pi.answer(setPose(s, id++, 100));
    const uint32_t map = pi.mapId();
    pi.restart(0x22);

    const translagatr::BrainReply reply = pi.answer(request(translagatr::kOpGetState, s, id++));
    EXPECT_EQ(reply.result, translagatr::kResultUnknownSession);
    EXPECT_EQ(reply.pi_instance, 0x22u);
    EXPECT_EQ(pi.robot().anchor_revision, 0u);
    EXPECT_EQ(pi.robot().robot_flags & translagatr::kRobotLocalized, 0);
    EXPECT_EQ(pi.appliedProfile(), 0u);
    EXPECT_EQ(pi.stagingReceived(), 0);
    EXPECT_EQ(pi.mapId(), map);
    EXPECT_EQ(pi.newestEstimate(), 1u);
}

TEST(FakePi, VersionAndOpErrors) {
    FakePi pi;
    pi.setVersion(5);
    translagatr::BrainReply reply = pi.answer(hello(1, 5));
    EXPECT_EQ(reply.result, translagatr::kResultUnsupportedVersion);
    EXPECT_EQ(reply.version, 5);

    FakePi other;
    other.setUnsupportedOp(translagatr::kOpGetState);
    const uint32_t s = other.answer(hello(1, 5)).session;
    EXPECT_EQ(other.answer(request(translagatr::kOpGetState, s, 2)).result, translagatr::kResultUnsupportedOp);
    EXPECT_EQ(other.answer(request(3, s, 3)).result, translagatr::kResultUnsupportedOp); // retired
    EXPECT_EQ(other.answer(request(5, s, 4)).result, translagatr::kResultUnsupportedOp); // retired
}
