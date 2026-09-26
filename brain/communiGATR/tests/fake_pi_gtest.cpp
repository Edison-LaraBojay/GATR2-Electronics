// fake_pi_gtest.cpp
// The fake Pi follows the brain link v3 Pi rules the client tests rely on.

#include "sim/fake_pi.h"

#include <gtest/gtest.h>

using namespace communigatr;

namespace
{

gatr2::BrainRequest hello(uint16_t id, uint32_t nonce) {
    gatr2::BrainRequest q;
    q.op         = gatr2::kOpHello;
    q.request_id = id;
    q.nonce      = nonce;
    return q;
}

gatr2::BrainRequest request(uint8_t op, uint32_t session, uint16_t id) {
    gatr2::BrainRequest q;
    q.op         = op;
    q.session    = session;
    q.request_id = id;
    return q;
}

gatr2::BrainRequest setPose(uint32_t session, uint16_t id, int32_t x) {
    gatr2::BrainRequest q = request(gatr2::kOpSetPose, session, id);
    q.x_mm                = x;
    return q;
}

} // namespace

TEST(FakePi, HelloRetryIsIdempotentAndRecentNonceIsStale) {
    FakePi                  pi;
    const gatr2::BrainReply first = pi.answer(hello(1, 77));
    ASSERT_EQ(first.result, gatr2::kResultOk);
    ASSERT_NE(first.session, 0u);
    EXPECT_EQ(first.nonce, 77u);

    const gatr2::BrainReply retry = pi.answer(hello(1, 77));
    EXPECT_EQ(retry.result, gatr2::kResultOk);
    EXPECT_EQ(retry.session, first.session);
    EXPECT_EQ(pi.sessionsOpened(), 1);

    EXPECT_EQ(pi.answer(request(gatr2::kOpGetState, first.session, 2)).result, gatr2::kResultOk);
    EXPECT_EQ(pi.answer(hello(1, 77)).result, gatr2::kResultStale);
    EXPECT_EQ(pi.session(), first.session);

    const gatr2::BrainReply second = pi.answer(hello(1, 78));
    EXPECT_EQ(second.result, gatr2::kResultOk);
    EXPECT_NE(second.session, first.session);
    EXPECT_EQ(pi.answer(request(gatr2::kOpGetState, first.session, 3)).result,
              gatr2::kResultUnknownSession);
}

TEST(FakePi, PlacementDedupedPerSessionAndAppliedOnce) {
    FakePi         pi;
    const uint32_t s = pi.answer(hello(1, 5)).session;

    const gatr2::BrainReply ok = pi.answer(setPose(s, 2, 610));
    EXPECT_EQ(ok.result, gatr2::kResultOk);
    EXPECT_EQ(ok.anchor_revision, 1u);
    EXPECT_EQ(pi.answer(request(gatr2::kOpGetState, s, 3)).result, gatr2::kResultOk);

    const gatr2::BrainReply dup = pi.answer(setPose(s, 2, 610));
    EXPECT_EQ(dup.result, gatr2::kResultOk);
    EXPECT_EQ(pi.placementsApplied(), 1);
    EXPECT_EQ(pi.answer(setPose(s, 2, 611)).result, gatr2::kResultInvalidArgument);
    EXPECT_EQ(pi.answer(request(gatr2::kOpGetState, s, 1)).result, gatr2::kResultStale);

    // Same id and pose in a new session is a new placement.
    const uint32_t s2 = pi.answer(hello(1, 6)).session;
    EXPECT_EQ(pi.answer(setPose(s2, 2, 610)).result, gatr2::kResultOk);
    EXPECT_EQ(pi.placementsApplied(), 2);
    EXPECT_EQ(pi.robot().anchor_revision, 2u);
}

TEST(FakePi, PendingUntilApplied) {
    FakePi pi;
    pi.setApplyDelay(2);
    const uint32_t s = pi.answer(hello(1, 5)).session;
    EXPECT_EQ(pi.answer(setPose(s, 2, 100)).result, gatr2::kResultPending);
    EXPECT_EQ(pi.answer(setPose(s, 2, 100)).result, gatr2::kResultPending);
    EXPECT_EQ(pi.answer(setPose(s, 2, 100)).result, gatr2::kResultOk);
    EXPECT_EQ(pi.placementsApplied(), 1);
}

TEST(FakePi, SelectionResultsAndStateLandmark) {
    FakePi       pi;
    FakeLandmark lm;
    lm.source = gatr2::kLandmarkSourceNominal;
    lm.x_mm   = 2000;
    lm.age_ms = 99;
    pi.setLandmark(4, lm);
    const uint32_t s = pi.answer(hello(1, 5)).session;

    gatr2::BrainRequest select = request(gatr2::kOpSelectLandmark, s, 2);
    select.landmark_id         = 4;
    select.select_flags        = gatr2::kSelectFlagSelected;
    EXPECT_EQ(pi.answer(select).result, gatr2::kResultOk);
    gatr2::BrainReply state = pi.answer(request(gatr2::kOpGetState, s, 3));
    EXPECT_EQ(state.state.landmark_id, 4);
    EXPECT_EQ(state.state.landmark_source, gatr2::kLandmarkSourceNominal);
    EXPECT_EQ(state.state.lm_x_mm, 2000);
    EXPECT_EQ(state.state.landmark_age_ms, 0);

    select.request_id  = 4;
    select.landmark_id = 9;
    EXPECT_EQ(pi.answer(select).result, gatr2::kResultUnknownLandmark);
    EXPECT_EQ(pi.answer(request(gatr2::kOpGetState, s, 5)).state.landmark_source,
              gatr2::kLandmarkSourceNone);

    pi.setWorldEstimationNoop(true);
    select.request_id  = 6;
    select.landmark_id = 4;
    EXPECT_EQ(pi.answer(select).result, gatr2::kResultLandmarkUnsupported);

    // A new session releases the selection.
    const uint32_t s2 = pi.answer(hello(1, 6)).session;
    EXPECT_FALSE(pi.landmarkRequested());
    EXPECT_EQ(pi.answer(request(gatr2::kOpGetState, s2, 2)).state.landmark_id, 0);
}

TEST(FakePi, RestartForgetsSessionAndAnchor) {
    FakePi         pi(0x11);
    const uint32_t s = pi.answer(hello(1, 5)).session;
    pi.answer(setPose(s, 2, 100));
    pi.restart(0x22);

    const gatr2::BrainReply reply = pi.answer(request(gatr2::kOpGetState, s, 3));
    EXPECT_EQ(reply.result, gatr2::kResultUnknownSession);
    EXPECT_EQ(reply.pi_instance, 0x22u);
    EXPECT_EQ(pi.robot().anchor_revision, 0u);
    EXPECT_EQ(pi.robot().robot_flags & gatr2::kRobotLocalized, 0);
}

TEST(FakePi, VersionAndOpErrors) {
    FakePi pi;
    pi.setVersion(4);
    gatr2::BrainReply reply = pi.answer(hello(1, 5));
    EXPECT_EQ(reply.result, gatr2::kResultUnsupportedVersion);
    EXPECT_EQ(reply.version, 4);

    FakePi other;
    other.setUnsupportedOp(gatr2::kOpGetState);
    const uint32_t s = other.answer(hello(1, 5)).session;
    EXPECT_EQ(other.answer(request(gatr2::kOpGetState, s, 2)).result, gatr2::kResultUnsupportedOp);
    EXPECT_EQ(other.answer(request(9, s, 3)).result, gatr2::kResultUnsupportedOp);
}
