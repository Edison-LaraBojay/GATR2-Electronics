// driver_gtest.cpp
// Driver over the client, fake bus and fake Pi: unit conversion,
// robot validity and anchor rules, placement gating, ages, frame generations,
// landmark statuses, and what the Navigator does with stale or lost input.

#include "communigatr/driver.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "investigatr/navigator.h"
#include "sim/link_rig.h"

using namespace communigatr;
using investigatr::InputRequest;
using investigatr::InputSnapshot;
using investigatr::kPi;
using investigatr::LandmarkEstimate;
using investigatr::LandmarkId;
using investigatr::LandmarkSource;
using investigatr::LandmarkStatus;
using investigatr::MotionReason;
using investigatr::MotionState;
using investigatr::Navigator;
using investigatr::Pose;

namespace
{

constexpr Seconds kLimit = 3.0;
constexpr double  kEps   = 1e-9;
constexpr uint8_t kLocalized =
    gatr2::kRobotPoseValid | gatr2::kRobotLocalized | gatr2::kRobotAgeKnown;

void openSession(LinkRig& rig) {
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
}

void place(LinkRig& rig, Driver& driver, const Pose& pose) {
    const PlacementTicket ticket = driver.submitPlacement(pose);
    ASSERT_NE(ticket, 0u);
    ASSERT_TRUE(rig.runUntil(
        [&] { return driver.placementResult(ticket) == PlacementResult::kApplied; }, kLimit));
}

// Latest SET_POSE the fake Pi decoded.
gatr2::BrainRequest lastSetPose(const FakePi& pi) {
    gatr2::BrainRequest last;
    for (const gatr2::BrainRequest& r : pi.requests()) {
        if (r.op == gatr2::kOpSetPose) {
            last = r;
        }
    }
    return last;
}

// Calls request() every step, as the Navigator does, until the status settles.
LandmarkEstimate awaitLandmark(LinkRig& rig, Driver& driver, LandmarkId id) {
    const InputRequest request{true, id};
    LandmarkEstimate   estimate;
    rig.runUntil(
        [&] {
            driver.request(request);
            estimate = driver.latest(rig.now()).landmark;
            return estimate.status != LandmarkStatus::kPending;
        },
        kLimit);
    return estimate;
}

void expectPose(const Pose& actual, const Pose& expected) {
    EXPECT_NEAR(actual.x, expected.x, kEps);
    EXPECT_NEAR(actual.y, expected.y, kEps);
    EXPECT_NEAR(investigatr::wrapAngle(actual.heading - expected.heading), 0.0, kEps);
}

// Navigator every 10 ms on the rig clock, link every 1 ms. The Pi pose does
// not move. Returns true if every demand while stepping was exactly zero.
bool runNavigator(LinkRig& rig, Navigator& navigator, Seconds duration) {
    bool          zero = true;
    const Seconds end  = rig.now() + duration;
    int           tick = 0;
    while (rig.now() < end && !investigatr::isTerminal(navigator.status().state)) {
        rig.step();
        if (++tick % 10 == 0) {
            const investigatr::DriveCommand demand = navigator.update(rig.now());
            zero = zero && demand.forward == 0.0 && demand.turn == 0.0;
        }
    }
    return zero;
}

} // namespace

// ---------------------------------------------------------------------------
// Robot estimate
// ---------------------------------------------------------------------------

TEST(Driver, StartsEmptyAndWaitsForCorrelatedState) {
    LinkRig rig;
    Driver  driver(rig.client());
    // Localized by an earlier Brain boot through SET_POSE.
    rig.pi.robot().robot_flags = kLocalized | gatr2::kRobotAnchorCommand;

    InputSnapshot s = driver.latest(0.0);
    EXPECT_EQ(s.frame, 0u);
    EXPECT_FALSE(s.connected);
    EXPECT_TRUE(std::isinf(s.link_age));
    EXPECT_FALSE(s.robot.valid);
    EXPECT_EQ(s.landmark.status, LandmarkStatus::kNotRequested);
    EXPECT_EQ(s.landmark.id, 0u);

    // Session open, no state reply yet.
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().session() != 0; }, kLimit));
    s = driver.latest(rig.now());
    EXPECT_EQ(s.frame, 0u);
    EXPECT_FALSE(s.connected);
    EXPECT_FALSE(s.robot.valid);

    // The earlier boot's command anchor stays valid; no new placement needed.
    openSession(rig);
    s = driver.latest(rig.now());
    EXPECT_NE(s.frame, 0u);
    EXPECT_TRUE(s.connected);
    EXPECT_TRUE(s.robot.valid);
    EXPECT_LT(s.link_age, 0.03);
}

TEST(Driver, ConvertsWireUnits) {
    LinkRig rig;
    Driver  driver(rig.client());
    openSession(rig);
    place(rig, driver, Pose{1.2344, -0.5676, kPi / 2.0});
    const gatr2::BrainRequest sent = lastSetPose(rig.pi);
    EXPECT_EQ(sent.x_mm, 1234);
    EXPECT_EQ(sent.y_mm, -568);
    EXPECT_EQ(sent.heading_cdeg, 9000);
    expectPose(driver.latest(rig.now()).robot.pose, Pose{1.234, -0.568, kPi / 2.0});

    rig.pi.robot().x_mm         = 1500;
    rig.pi.robot().y_mm         = -250;
    rig.pi.robot().heading_cdeg = -4500;
    rig.run(0.1);
    expectPose(driver.latest(rig.now()).robot.pose, Pose{1.5, -0.25, -kPi / 4.0});

    rig.pi.robot().heading_cdeg = 18000;
    rig.run(0.1);
    EXPECT_NEAR(driver.latest(rig.now()).robot.pose.heading, kPi, kEps);
}

TEST(Driver, PlacementHeadingNormalizedAndRangeChecked) {
    LinkRig      rig;
    Driver       driver(rig.client());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_EQ(driver.submitPlacement(Pose{nan, 0.0, 0.0}), 0u);
    EXPECT_EQ(driver.submitPlacement(Pose{0.0, 0.0, inf}), 0u);
    EXPECT_EQ(driver.submitPlacement(Pose{3.0e6, 0.0, 0.0}), 0u); // beyond int32 mm
    EXPECT_FALSE(driver.placementPending());

    openSession(rig);
    place(rig, driver, Pose{0.0, 0.0, -kPi + 1e-9});
    EXPECT_EQ(lastSetPose(rig.pi).heading_cdeg, 18000);
    place(rig, driver, Pose{0.0, 0.0, 1.5 * kPi});
    EXPECT_EQ(lastSetPose(rig.pi).heading_cdeg, -9000);
}

TEST(Driver, RobotValidNeedsFlagsAndAcceptedAnchor) {
    LinkRig      rig;
    Driver       driver(rig.client());
    DriverConfig accept;
    accept.accept_configured_anchor = true;
    Driver configured(rig.client(), accept);
    openSession(rig);
    place(rig, driver, Pose{});
    EXPECT_TRUE(driver.latest(rig.now()).robot.valid);
    EXPECT_TRUE(configured.latest(rig.now()).robot.valid);

    for (const uint8_t flag :
         {gatr2::kRobotPoseValid, gatr2::kRobotLocalized, gatr2::kRobotAgeKnown}) {
        rig.pi.robot().robot_flags =
            static_cast<uint8_t>((kLocalized | gatr2::kRobotAnchorCommand) & ~flag);
        rig.run(0.1);
        EXPECT_FALSE(driver.latest(rig.now()).robot.valid) << "flag " << int(flag);
        EXPECT_FALSE(configured.latest(rig.now()).robot.valid) << "flag " << int(flag);
    }

    // Anchor from <InitialPlacement>: only when accepted.
    rig.pi.robot().robot_flags = kLocalized | gatr2::kRobotAnchorConfigured;
    rig.run(0.1);
    EXPECT_FALSE(driver.latest(rig.now()).robot.valid);
    EXPECT_TRUE(configured.latest(rig.now()).robot.valid);

    // No anchor origin at all.
    rig.pi.robot().robot_flags = kLocalized;
    rig.run(0.1);
    EXPECT_FALSE(driver.latest(rig.now()).robot.valid);
    EXPECT_FALSE(configured.latest(rig.now()).robot.valid);
}

TEST(Driver, PlacementGatesPoseUntilItsAnchorIsReported) {
    LinkRig rig;
    Driver  driver(rig.client());
    openSession(rig);
    place(rig, driver, Pose{0.5, 0.5, 0.0});
    const InputSnapshot before = driver.latest(rig.now());
    ASSERT_TRUE(before.robot.valid);

    rig.pi.setApplyDelay(10);
    const Pose            target{1.0, -1.0, kPi / 2.0};
    const PlacementTicket ticket = driver.submitPlacement(target);
    ASSERT_NE(ticket, 0u);
    EXPECT_FALSE(driver.latest(rig.now()).robot.valid);

    bool          valid_seen   = false;
    bool          ok_unapplied = false;
    const Seconds end          = rig.now() + kLimit;
    while (driver.placementResult(ticket) == PlacementResult::kPending && rig.now() < end) {
        rig.step();
        const PlacementStatus status = driver.placementStatus(ticket);
        if (status.state == PlacementResult::kPending) {
            valid_seen   = valid_seen || driver.latest(rig.now()).robot.valid;
            ok_unapplied = ok_unapplied || status.result == gatr2::kResultOk;
        }
    }
    ASSERT_EQ(driver.placementResult(ticket), PlacementResult::kApplied);
    EXPECT_FALSE(valid_seen);
    EXPECT_TRUE(ok_unapplied); // Ok received, still waiting for a state with its anchor

    const InputSnapshot after = driver.latest(rig.now());
    ASSERT_TRUE(after.robot.valid);
    expectPose(after.robot.pose, target);
    EXPECT_NE(after.frame, before.frame);
    EXPECT_EQ(rig.pi.placementsApplied(), 2);
}

TEST(Driver, TimedOutPlacementKeepsFrame) {
    LinkRig rig;
    Driver  driver(rig.client());
    openSession(rig);
    place(rig, driver, Pose{});
    const investigatr::FrameGeneration frame = driver.latest(rig.now()).frame;

    rig.pi.setApplyDelay(-1);
    const PlacementTicket ticket = driver.submitPlacement(Pose{1.0, 0.0, 0.0});
    ASSERT_TRUE(rig.runUntil(
        [&] { return driver.placementResult(ticket) == PlacementResult::kTimedOut; }, kLimit));
    rig.run(0.1);
    const InputSnapshot s = driver.latest(rig.now());
    EXPECT_TRUE(s.robot.valid);
    EXPECT_EQ(s.frame, frame);
}

TEST(Driver, AgesAddRoundTripAndTimeSinceReceipt) {
    FakeBusConfig bus;
    bus.reply_delay = 0.010;
    LinkRig      rig({}, bus);
    Driver       driver(rig.client());
    FakeLandmark landmark;
    landmark.age_ms = 250;
    rig.pi.setLandmark(5, landmark);
    openSession(rig);
    place(rig, driver, Pose{});
    rig.pi.robot().robot_age_ms = 40;
    ASSERT_EQ(awaitLandmark(rig, driver, 5).status, LandmarkStatus::kAvailable);
    ASSERT_TRUE(
        rig.runUntil([&] { return rig.client().state().state.robot_age_ms == 40; }, kLimit));

    const StateSample sample = rig.client().state();
    EXPECT_GT(sample.round_trip, bus.reply_delay);
    const Seconds       later = rig.now() + 0.05;
    const InputSnapshot s     = driver.latest(later);
    const Seconds       delay = sample.round_trip + (later - sample.received_at);
    EXPECT_NEAR(s.robot.age, 0.040 + delay, kEps);
    EXPECT_TRUE(s.landmark.age_known);
    EXPECT_NEAR(s.landmark.age, 0.250 + delay, kEps);
}

TEST(Driver, FrameGenerationFollowsInstanceSessionEpochAndAnchor) {
    LinkRig rig;
    Driver  driver(rig.client());
    auto    frame = [&] { return driver.latest(rig.now()).frame; };
    EXPECT_EQ(frame(), 0u);

    openSession(rig);
    const investigatr::FrameGeneration g1 = frame();
    EXPECT_NE(g1, 0u);
    rig.run(0.2);
    EXPECT_EQ(frame(), g1);

    place(rig, driver, Pose{0.1, 0.2, kPi / 6.0});
    const investigatr::FrameGeneration g2 = frame();
    EXPECT_GT(g2, g1);

    // Pico restart: new odometry epoch, same anchor.
    rig.pi.robot().odometry_epoch += 1;
    rig.run(0.1);
    const investigatr::FrameGeneration g3 = frame();
    EXPECT_GT(g3, g2);
    EXPECT_TRUE(driver.latest(rig.now()).robot.valid);

    // Another HELLO takes the Pi's session. The pose carries over, the frame
    // does not.
    gatr2::BrainRequest hello;
    hello.op         = gatr2::kOpHello;
    hello.request_id = 1;
    hello.nonce      = 0xABCDEF;
    ASSERT_EQ(rig.pi.answer(hello).result, gatr2::kResultOk);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().session_losses == 1; }, kLimit));
    EXPECT_EQ(frame(), 0u);
    EXPECT_FALSE(driver.latest(rig.now()).robot.valid);
    openSession(rig);
    const investigatr::FrameGeneration g4 = frame();
    EXPECT_GT(g4, g3);
    EXPECT_TRUE(driver.latest(rig.now()).robot.valid);
    expectPose(driver.latest(rig.now()).robot.pose, Pose{0.1, 0.2, kPi / 6.0});

    // Pi restart: new instance, anchor gone.
    rig.pi.restart(0x1234);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().pi_restarts == 1; }, kLimit));
    EXPECT_EQ(frame(), 0u);
    openSession(rig);
    EXPECT_GT(frame(), g4);
    EXPECT_FALSE(driver.latest(rig.now()).robot.valid);
}

// ---------------------------------------------------------------------------
// Landmarks
// ---------------------------------------------------------------------------

TEST(Driver, LandmarkStatuses) {
    LinkRig      rig;
    Driver       driver(rig.client());
    FakeLandmark observed;
    observed.x_mm         = 2000;
    observed.y_mm         = 1000;
    observed.heading_cdeg = 9000;
    observed.age_ms       = 100;
    FakeLandmark nominal;
    nominal.source       = gatr2::kLandmarkSourceNominal;
    nominal.x_mm         = -500;
    nominal.y_mm         = 250;
    nominal.heading_cdeg = 18000;
    FakeLandmark none;
    none.source = gatr2::kLandmarkSourceNone;
    rig.pi.setLandmark(1, observed);
    rig.pi.setLandmark(2, nominal);
    rig.pi.setLandmark(3, none);
    openSession(rig);
    place(rig, driver, Pose{});

    driver.request(InputRequest{});
    EXPECT_EQ(driver.latest(rig.now()).landmark.status, LandmarkStatus::kNotRequested);

    driver.request(InputRequest{true, 1});
    EXPECT_EQ(driver.latest(rig.now()).landmark.status, LandmarkStatus::kPending);
    LandmarkEstimate e = awaitLandmark(rig, driver, 1);
    EXPECT_EQ(e.status, LandmarkStatus::kAvailable);
    EXPECT_EQ(e.id, 1u);
    EXPECT_EQ(e.source, LandmarkSource::kObserved);
    EXPECT_TRUE(e.age_known);
    expectPose(e.pose, Pose{2.0, 1.0, kPi / 2.0});
    EXPECT_EQ(rig.pi.landmarkWireId(), 1u);

    driver.request(InputRequest{true, 2});
    EXPECT_EQ(driver.latest(rig.now()).landmark.status, LandmarkStatus::kPending);
    e = awaitLandmark(rig, driver, 2);
    EXPECT_EQ(e.status, LandmarkStatus::kAvailable);
    EXPECT_EQ(e.source, LandmarkSource::kNominal);
    EXPECT_FALSE(e.age_known);
    EXPECT_EQ(e.age, 0.0);
    expectPose(e.pose, Pose{-0.5, 0.25, kPi});

    e = awaitLandmark(rig, driver, 3);
    EXPECT_EQ(e.status, LandmarkStatus::kUnavailable);
    EXPECT_EQ(e.source, LandmarkSource::kNone);

    e = awaitLandmark(rig, driver, 9);
    EXPECT_EQ(e.status, LandmarkStatus::kUnknownLandmark);
    EXPECT_EQ(e.id, 9u);

    // No wire id: settled at once, and the Pi selection is released.
    driver.request(InputRequest{true, 300});
    e = driver.latest(rig.now()).landmark;
    EXPECT_EQ(e.status, LandmarkStatus::kUnknownLandmark);
    EXPECT_EQ(e.id, 300u);
    EXPECT_EQ(rig.client().wantedLandmark(), 0u);

    ASSERT_EQ(awaitLandmark(rig, driver, 1).status, LandmarkStatus::kAvailable);
    driver.request(InputRequest{});
    rig.run(0.1);
    EXPECT_FALSE(rig.pi.landmarkRequested());
    EXPECT_EQ(driver.latest(rig.now()).landmark.status, LandmarkStatus::kNotRequested);

    // Link down: stale.
    ASSERT_EQ(awaitLandmark(rig, driver, 1).status, LandmarkStatus::kAvailable);
    rig.bus.setPiPresent(false);
    ASSERT_TRUE(rig.runUntil([&] { return !rig.client().connected(rig.now()); }, kLimit));
    e = driver.latest(rig.now()).landmark;
    EXPECT_EQ(e.status, LandmarkStatus::kStale);
    EXPECT_EQ(e.id, 1u);
}

TEST(Driver, LandmarkUnsupportedWhenWorldEstimationNoop) {
    LinkRig rig;
    Driver  driver(rig.client());
    rig.pi.setLandmark(1, FakeLandmark{});
    rig.pi.setWorldEstimationNoop(true);
    openSession(rig);
    EXPECT_EQ(awaitLandmark(rig, driver, 1).status, LandmarkStatus::kUnsupported);
}

// ---------------------------------------------------------------------------
// Navigator on the driver
// ---------------------------------------------------------------------------

TEST(Driver, StaleMeasurementNeverBecomesUsable) {
    LinkRig rig;
    Driver  driver(rig.client());
    openSession(rig);
    place(rig, driver, Pose{});
    rig.pi.robot().robot_age_ms = 400; // older than max_pose_age
    rig.run(0.1);
    const InputSnapshot s = driver.latest(rig.now());
    ASSERT_TRUE(s.robot.valid);
    ASSERT_TRUE(s.connected);
    EXPECT_GT(s.robot.age, 0.4);

    Navigator navigator(driver);
    navigator.goTo(Pose{1.0, 1.0, 0.0});
    EXPECT_TRUE(runNavigator(rig, navigator, 5.0));
    EXPECT_EQ(navigator.status().state, MotionState::kFailed);
    EXPECT_EQ(navigator.status().reason, MotionReason::kInputUnavailable);
}

TEST(Driver, StaleMeasurementMidCommandStopsThenFails) {
    LinkRig rig;
    Driver  driver(rig.client());
    openSession(rig);
    place(rig, driver, Pose{});
    Navigator navigator(driver);
    navigator.goTo(Pose{1.0, 1.0, 0.0});
    EXPECT_FALSE(runNavigator(rig, navigator, 0.3));
    ASSERT_FALSE(investigatr::isTerminal(navigator.status().state));

    rig.pi.robot().robot_age_ms = 400;
    ASSERT_TRUE(
        rig.runUntil([&] { return rig.client().state().state.robot_age_ms == 400; }, kLimit));
    const Seconds stale = rig.now();
    EXPECT_TRUE(runNavigator(rig, navigator, 2.0));
    EXPECT_EQ(navigator.status().state, MotionState::kFailed);
    EXPECT_EQ(navigator.status().reason, MotionReason::kInputLost);
    EXPECT_LT(rig.now() - stale, navigator.config().input_loss_timeout + 0.05);
}

TEST(Driver, LinkLossFailsCommandAndNothingResumes) {
    LinkRig rig;
    Driver  driver(rig.client());
    openSession(rig);
    place(rig, driver, Pose{});
    Navigator navigator(driver);
    navigator.goTo(Pose{1.0, 1.0, 0.0});
    runNavigator(rig, navigator, 0.3);
    ASSERT_FALSE(investigatr::isTerminal(navigator.status().state));

    rig.bus.setPiPresent(false);
    runNavigator(rig, navigator, 2.0);
    EXPECT_EQ(navigator.status().state, MotionState::kFailed);
    EXPECT_EQ(navigator.status().reason, MotionReason::kInputLost);

    rig.bus.setPiPresent(true);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().connected(rig.now()); }, kLimit));
    for (int i = 0; i < 50; ++i) {
        rig.run(0.01);
        const investigatr::DriveCommand demand = navigator.update(rig.now());
        EXPECT_EQ(demand.forward, 0.0);
        EXPECT_EQ(demand.turn, 0.0);
    }
    EXPECT_EQ(navigator.status().state, MotionState::kFailed);
    EXPECT_TRUE(driver.latest(rig.now()).robot.valid);
}
