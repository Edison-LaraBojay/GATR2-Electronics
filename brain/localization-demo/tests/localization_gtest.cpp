// localization_gtest.cpp
// EncoderTracker and applyRecord: baselines, duplicates, restarts, gaps,
// wraps, IMU faults.

#include "localization.h"

#include <gtest/gtest.h>

#include <climits>
#include <cmath>

namespace localization
{
namespace
{

using encoder_record::Record;
using odometry::kPi;

constexpr double kTol = 1e-9;

// 1000 counts per meter, channel 1 forward, channel 0 sideways, no offsets.
odometry::WheelSetup testWheels() {
    odometry::WheelSetup w;
    w.forward.channel         = 1;
    w.forward.diameter        = 1.0 / kPi;
    w.forward.counts_per_rev  = 1000;
    w.sideways.channel        = 0;
    w.sideways.diameter       = 1.0 / kPi;
    w.sideways.counts_per_rev = 1000;
    return w;
}

TrackerLimits testLimits() {
    TrackerLimits l;
    l.stale_ms        = 250;
    l.max_step_counts = 20000;
    return l;
}

Record rec(uint16_t run, uint16_t seq, int32_t ch0, int32_t ch1) {
    Record r;
    r.run_id   = run;
    r.seq      = seq;
    r.channel0 = ch0;
    r.channel1 = ch1;
    return r;
}

ImuReading imu(double rotation_degrees) {
    ImuReading r;
    r.valid            = true;
    r.rotation_degrees = rotation_degrees;
    return r;
}

TEST(EncoderTracker, FirstRecordIsBaselineThenAdvances) {
    EncoderTracker t(testLimits());
    EXPECT_TRUE(t.stale(0));
    EXPECT_EQ(t.accept(rec(1, 5, 100, 200), 1000).kind, StepKind::kBaseline);
    EXPECT_FALSE(t.stale(1000));
    const Step s = t.accept(rec(1, 6, 90, 260), 1020);
    EXPECT_EQ(s.kind, StepKind::kAdvance);
    EXPECT_EQ(s.channel0, -10);
    EXPECT_EQ(s.channel1, 60);
}

TEST(EncoderTracker, DuplicateIsIgnoredAndDoesNotRefresh) {
    EncoderTracker t(testLimits());
    t.accept(rec(1, 5, 100, 200), 1000);
    EXPECT_EQ(t.accept(rec(1, 5, 100, 200), 1200).kind, StepKind::kDuplicate);
    EXPECT_TRUE(t.stale(1300));
}

TEST(EncoderTracker, SequenceAndCounterWrap) {
    EncoderTracker t(testLimits());
    t.accept(rec(1, 65535, INT_MAX, INT_MIN), 0);
    const Step s = t.accept(rec(1, 0, INT_MIN, INT_MAX), 20);
    EXPECT_EQ(s.kind, StepKind::kAdvance);
    EXPECT_EQ(s.channel0, 1);
    EXPECT_EQ(s.channel1, -1);
}

TEST(EncoderTracker, RestartGapBackwardsAndJumpRebase) {
    EncoderTracker t(testLimits());
    t.accept(rec(1, 10, 5000, 5000), 0);
    EXPECT_EQ(t.accept(rec(2, 0, 0, 0), 20).kind, StepKind::kRestart);
    EXPECT_EQ(t.accept(rec(2, 1, 10, 10), 40).kind, StepKind::kAdvance);
    EXPECT_EQ(t.accept(rec(2, 50, 30, 30), 400).kind, StepKind::kResume);
    EXPECT_EQ(t.accept(rec(2, 40, 30, 30), 410).kind, StepKind::kDuplicate); // late, ignored
    EXPECT_EQ(t.accept(rec(2, 64586, 30, 30), 420).kind, StepKind::kJump);  // 1000 back
    EXPECT_EQ(t.accept(rec(2, 41, 90000, 30), 440).kind, StepKind::kJump); // too far
    EXPECT_EQ(t.accept(rec(2, 42, 90010, 30), 460).kind, StepKind::kAdvance);
}

TEST(EncoderTracker, RebaseMakesNextABaseline) {
    EncoderTracker t(testLimits());
    t.accept(rec(1, 1, 0, 0), 0);
    t.rebase();
    EXPECT_EQ(t.accept(rec(1, 2, 5, 5), 20).kind, StepKind::kBaseline);
}

TEST(Localization, FirstSampleIsBaselineNotTravel) {
    State s(odometry::Pose{1.0, 2.0, 0.5}, testLimits());
    EXPECT_EQ(applyRecord(s, testWheels(), rec(1, 0, 123456, -98765), 0, imu(37.0)),
              StepKind::kBaseline);
    EXPECT_TRUE(s.have_baseline);
    EXPECT_NEAR(s.pose.x, 1.0, kTol);
    EXPECT_NEAR(s.pose.y, 2.0, kTol);
    EXPECT_NEAR(s.pose.heading, 0.5, kTol); // IMU reading 37 deg is the baseline
    EXPECT_FALSE(s.health.uncertain());
}

TEST(Localization, ForwardSidewaysAndTurnFromRecords) {
    const odometry::WheelSetup w = testWheels();
    State s(odometry::Pose{}, testLimits());
    applyRecord(s, w, rec(1, 0, 0, 0), 0, imu(0.0));
    applyRecord(s, w, rec(1, 1, 0, 500), 20, imu(0.0)); // forward 0.5 m on channel 1
    EXPECT_NEAR(s.pose.x, 0.5, kTol);
    applyRecord(s, w, rec(1, 2, 200, 500), 40, imu(0.0)); // left 0.2 m on channel 0
    EXPECT_NEAR(s.pose.y, 0.2, kTol);
    applyRecord(s, w, rec(1, 3, 200, 500), 60, imu(-90.0)); // VEX -90: CCW quarter turn
    EXPECT_NEAR(s.pose.heading, kPi / 2, kTol);
    EXPECT_NEAR(s.pose.x, 0.5, kTol);
    EXPECT_NEAR(s.pose.y, 0.2, kTol);
    EXPECT_EQ(s.health.advances, 3u);
}

TEST(Localization, DuplicateIsNotIntegratedTwice) {
    const odometry::WheelSetup w = testWheels();
    State s(odometry::Pose{}, testLimits());
    applyRecord(s, w, rec(1, 0, 0, 0), 0, imu(0.0));
    applyRecord(s, w, rec(1, 1, 0, 100), 20, imu(0.0));
    EXPECT_EQ(applyRecord(s, w, rec(1, 1, 0, 100), 30, imu(0.0)), StepKind::kDuplicate);
    EXPECT_NEAR(s.pose.x, 0.1, kTol);
    EXPECT_EQ(s.health.duplicates, 1u);
}

TEST(Localization, LateOlderRecordIsNotIntegratedTwice) {
    const odometry::WheelSetup w = testWheels();
    State s(odometry::Pose{}, testLimits());
    applyRecord(s, w, rec(1, 10, 0, 0), 0, imu(0.0));
    applyRecord(s, w, rec(1, 11, 0, 200), 20, imu(0.0));
    EXPECT_EQ(applyRecord(s, w, rec(1, 9, 0, 100), 30, imu(0.0)), StepKind::kDuplicate);
    applyRecord(s, w, rec(1, 12, 0, 300), 40, imu(0.0));
    EXPECT_NEAR(s.pose.x, 0.3, kTol);
    EXPECT_FALSE(s.health.uncertain());
}

TEST(Localization, PicoRestartCausesNoPhantomMovement) {
    const odometry::WheelSetup w = testWheels();
    State s(odometry::Pose{}, testLimits());
    applyRecord(s, w, rec(7, 0, 0, 0), 0, imu(0.0));
    applyRecord(s, w, rec(7, 1, 4000, 9000), 20, imu(0.0));
    const odometry::Pose before = s.pose;
    // Counts start over from zero under a new run id.
    EXPECT_EQ(applyRecord(s, w, rec(8, 0, 0, 0), 40, imu(-10.0)), StepKind::kRestart);
    EXPECT_NEAR(s.pose.x, before.x, kTol);
    EXPECT_NEAR(s.pose.y, before.y, kTol);
    EXPECT_NEAR(s.pose.heading, odometry::degreesToRadians(10.0), kTol); // IMU still valid
    applyRecord(s, w, rec(8, 1, 0, 100), 60, imu(-10.0));
    EXPECT_NEAR(std::hypot(s.pose.x - before.x, s.pose.y - before.y), 0.1, kTol);
    EXPECT_TRUE(s.health.uncertain());
    EXPECT_EQ(s.health.restarts, 1u);
}

TEST(Localization, GapRebasesAndIsReported) {
    const odometry::WheelSetup w = testWheels();
    State s(odometry::Pose{}, testLimits());
    applyRecord(s, w, rec(1, 0, 0, 0), 0, imu(0.0));
    EXPECT_EQ(applyRecord(s, w, rec(1, 100, 0, 3000), 2000, imu(0.0)), StepKind::kResume);
    EXPECT_NEAR(s.pose.x, 0.0, kTol); // travel during the gap is not invented
    EXPECT_EQ(s.health.gaps, 1u);
    EXPECT_TRUE(s.health.uncertain());
}

TEST(Localization, InvalidImuHoldsPoseThenReanchors) {
    const odometry::WheelSetup w = testWheels();
    State s(odometry::Pose{}, testLimits());
    applyRecord(s, w, rec(1, 0, 0, 0), 0, imu(0.0));
    applyRecord(s, w, rec(1, 1, 0, 0), 20, imu(-30.0));
    const double heading = s.pose.heading;
    EXPECT_EQ(applyRecord(s, w, rec(1, 2, 0, 500), 40, ImuReading{}), StepKind::kImuInvalid);
    EXPECT_NEAR(s.pose.x, 0.0, kTol);
    // IMU back, rotation reading restarted at 0: heading continues from the
    // last known value and the next record is a baseline.
    EXPECT_EQ(applyRecord(s, w, rec(1, 3, 0, 600), 60, imu(0.0)), StepKind::kBaseline);
    EXPECT_NEAR(s.pose.heading, heading, kTol);
    EXPECT_NEAR(s.pose.x, 0.0, kTol);
    applyRecord(s, w, rec(1, 4, 0, 700), 80, imu(0.0));
    EXPECT_NEAR(s.pose.x, 0.1 * std::cos(heading), kTol);
    EXPECT_EQ(s.health.imu_faults, 1u);
}

TEST(Localization, RotationInPlaceWithRealOffsetsStaysPut) {
    odometry::WheelSetup w = testWheels();
    w.forward.offset       = -0.1115;
    w.sideways.offset      = 0.011;
    State s(odometry::Pose{0.5, 0.5, 0}, testLimits());
    applyRecord(s, w, rec(1, 0, 0, 0), 0, imu(0.0));
    // Spin a full CCW turn in 36 steps of 10 deg.
    int32_t fwd = 0, side = 0;
    for (uint16_t i = 1; i <= 36; ++i) {
        const double turn = odometry::degreesToRadians(10.0 * i);
        fwd               = static_cast<int32_t>(std::lround(0.1115 * turn * 1000));
        side              = static_cast<int32_t>(std::lround(0.011 * turn * 1000));
        applyRecord(s, w, rec(1, i, side, fwd), 20u * i, imu(-10.0 * i));
    }
    EXPECT_NEAR(s.pose.x, 0.5, 2e-3);
    EXPECT_NEAR(s.pose.y, 0.5, 2e-3);
    EXPECT_NEAR(s.pose.heading, 0.0, 1e-9);
}

} // namespace
} // namespace localization
