// pose_history_gtest.cpp
// The localization history ring: ordering, revision, wrap, expiry, epoch
// changes, interpolation with shortest yaw, gap gates, pending futures,
// rate lookups, attitude slerp, and the feed's append rule.

#include <gtest/gtest.h>
#include <atomic>
#include <thread>

#include "math/angles.h"
#include "state/pose_history.h"
#include "state/robot_state_feed.h"

using namespace navigatr;

namespace
{

PoseHistoryEntry entry(int64_t at_ms, double x, double y, double heading, uint64_t epoch = 0) {
    PoseHistoryEntry e;
    e.at             = hostTime(at_ms);
    e.odom_pose      = Pose2D{x, y, heading};
    e.odometry_epoch = epoch;
    return e;
}

Attitude measured(double roll, double pitch, double yaw, int64_t at_ms, uint64_t epoch = 0) {
    Attitude a;
    a.valid            = true;
    a.q_reference_body = quaternionFromEuler(roll, pitch, yaw);
    a.reference        = "odometry";
    a.measuredAt       = hostTime(at_ms);
    a.epoch            = epoch;
    a.quality          = 1.0;
    return a;
}

} // namespace

TEST(PoseHistory, AppendOrderingRevisionAndUnset) {
    PoseHistory h;
    EXPECT_EQ(h.append(entry(10, 0, 0, 0)), AppendResult::kAppended);
    EXPECT_EQ(h.append(entry(20, 1, 0, 0)), AppendResult::kAppended);
    EXPECT_EQ(h.append(entry(20, 2, 0, 0)), AppendResult::kRevised);   // same time revises
    EXPECT_EQ(h.size(), 2u);
    EXPECT_NEAR(h.newest().odom_pose.x_m, 2.0, 1e-12);
    EXPECT_EQ(h.append(entry(15, 3, 0, 0)), AppendResult::kOutOfOrder);
    EXPECT_EQ(h.size(), 2u);
    PoseHistoryEntry unset = entry(30, 0, 0, 0);
    unset.at               = deviceTime(30);
    EXPECT_EQ(h.append(unset), AppendResult::kUnset);
}

TEST(PoseHistory, RingWrapKeepsLogicalOrderAndSearch) {
    PoseHistoryConfig config;
    config.capacity     = 4;
    config.retention_ms = 100000;
    PoseHistory h(config);
    for (int i = 0; i < 9; ++i) {
        h.append(entry(10 * i, static_cast<double>(i), 0, 0));
    }
    ASSERT_EQ(h.size(), 4u);
    EXPECT_EQ(h.oldest().at.ms, 50);
    EXPECT_EQ(h.newest().at.ms, 80);
    for (std::size_t i = 1; i < h.size(); ++i) {
        EXPECT_LT(h.entry(i - 1).at.ms, h.entry(i).at.ms);
    }
    // interpolation across the physical wrap point
    const PoseLookupResult r = h.poseAt(hostTime(65));
    ASSERT_EQ(r.status, LookupStatus::kOk);
    EXPECT_NEAR(r.odom_pose.x_m, 6.5, 1e-12);
    EXPECT_FALSE(r.exact);
    EXPECT_EQ(h.poseAt(hostTime(40)).status, LookupStatus::kExpired);
}

TEST(PoseHistory, ExactHitAndShortestYawInterpolation) {
    PoseHistory h;
    h.append(entry(0, 0, 0, degToRad(170.0)));
    h.append(entry(10, 1, 2, degToRad(-170.0)));

    const PoseLookupResult exact = h.poseAt(hostTime(10));
    ASSERT_EQ(exact.status, LookupStatus::kOk);
    EXPECT_TRUE(exact.exact);
    EXPECT_NEAR(exact.odom_pose.x_m, 1.0, 1e-12);

    const PoseLookupResult mid = h.poseAt(hostTime(5));
    ASSERT_EQ(mid.status, LookupStatus::kOk);
    EXPECT_NEAR(mid.odom_pose.x_m, 0.5, 1e-12);
    EXPECT_NEAR(mid.odom_pose.y_m, 1.0, 1e-12);
    // the short way round through 180, never the long way through zero
    EXPECT_NEAR(std::fabs(mid.odom_pose.heading_rad), kPi, 1e-9);
}

TEST(PoseHistory, GapGateExpiredPendingAndRetention) {
    PoseHistoryConfig config;
    config.max_interpolation_gap_ms = 100;
    config.retention_ms             = 1000;
    PoseHistory h(config);
    h.append(entry(0, 0, 0, 0));
    h.append(entry(500, 1, 0, 0));
    EXPECT_EQ(h.poseAt(hostTime(250)).status, LookupStatus::kGap);   // bracket too wide
    EXPECT_EQ(h.poseAt(hostTime(600)).status, LookupStatus::kPending);
    EXPECT_EQ(h.poseAt(deviceTime(100)).status, LookupStatus::kUnset);

    h.append(entry(1500, 2, 0, 0));   // the entry at 0 expires
    EXPECT_EQ(h.oldest().at.ms, 500);
    EXPECT_EQ(h.poseAt(hostTime(0)).status, LookupStatus::kExpired);
}

TEST(PoseHistory, EpochChangeClearsTheRing) {
    PoseHistory h;
    h.append(entry(0, 0, 0, 0, 0));
    h.append(entry(10, 1, 0, 0, 0));
    EXPECT_EQ(h.append(entry(20, 5, 0, 0, 1)), AppendResult::kEpochChange);
    EXPECT_EQ(h.size(), 1u);
    EXPECT_EQ(h.poseAt(hostTime(20)).odometry_epoch, 1u);
    EXPECT_EQ(h.poseAt(hostTime(5)).status, LookupStatus::kExpired);
}

TEST(PoseHistory, YawRateFromBracketUnderGates) {
    PoseHistoryConfig config;
    config.max_interpolation_gap_ms = 100;
    PoseHistory h(config);
    EXPECT_EQ(h.yawRateAt(hostTime(0)).status, LookupStatus::kEmpty);
    h.append(entry(0, 0, 0, 0.0));
    h.append(entry(100, 0, 0, 0.1));
    const RateLookupResult r = h.yawRateAt(hostTime(50));
    ASSERT_EQ(r.status, LookupStatus::kOk);
    EXPECT_NEAR(r.yaw_rate_rad_s, 1.0, 1e-9);
    EXPECT_EQ(h.yawRateAt(hostTime(150)).status, LookupStatus::kPending);
    h.append(entry(400, 0, 0, 0.2));
    EXPECT_EQ(h.yawRateAt(hostTime(300)).status, LookupStatus::kGap);
}

TEST(PoseHistory, AttitudeInterpolatesBySlerpAndRespectsGapsAndEpochs) {
    PoseHistoryConfig config;
    config.attitude_gap_ms = 100;
    PoseHistory h(config);

    PoseHistoryEntry a = entry(0, 0, 0, 0);
    a.attitude         = measured(0.0, 0.0, 0.0, 0);
    PoseHistoryEntry b = entry(100, 0, 0, 0);
    b.attitude         = measured(degToRad(20.0), 0.0, 0.0, 100);
    h.append(a);
    h.append(b);

    const AttitudeLookupResult mid = h.attitudeAt(hostTime(50));
    ASSERT_EQ(mid.status, LookupStatus::kOk);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    attitudeEuler(mid.attitude, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), 10.0, 1e-6);
    EXPECT_EQ(mid.attitude.measuredAt.ms, 50);

    // A newer pose without new attitude does not extend attitude support.
    PoseHistoryEntry c = entry(150, 0, 0, 0);   // no attitude
    h.append(c);
    EXPECT_EQ(h.attitudeAt(hostTime(125)).status, LookupStatus::kPending);

    // a source epoch change is a discontinuity, not something to blend
    PoseHistoryEntry d = entry(200, 0, 0, 0);
    d.attitude         = measured(0.0, 0.0, 0.0, 200, 7);
    h.append(d);
    EXPECT_EQ(h.attitudeAt(hostTime(175)).status, LookupStatus::kUnavailable);

    PoseHistoryEntry e = entry(600, 0, 0, 0);
    e.attitude         = measured(0.0, 0.0, 0.0, 600, 7);
    h.append(e);
    EXPECT_EQ(h.attitudeAt(hostTime(400)).status, LookupStatus::kGap);
    EXPECT_EQ(h.attitudeAt(hostTime(600)).status, LookupStatus::kOk);
}

TEST(RobotStateFeed, AppendsHistoryOnlyForAdvancesWithHostTime) {
    RobotStateFeed     feed;
    LocalizationStatus status;
    RobotState         r;
    r.valid     = true;
    r.odom_pose = Pose2D{1.0, 0.0, 0.0};

    feed.publish(r, status, true, 1);   // advanced but no host time: nothing appended
    EXPECT_EQ(feed.historySize(), 0u);

    r.measuredAtHost = hostTime(100);
    feed.publish(r, status, true, 2);
    EXPECT_EQ(feed.historySize(), 1u);

    r.odom_pose.x_m = 2.0;
    feed.publish(r, status, false, 3);   // a hold never appends a retained pose
    EXPECT_EQ(feed.historySize(), 1u);
    EXPECT_EQ(feed.publication(), 3u);
    EXPECT_NEAR(feed.latest().odom_pose.x_m, 2.0, 1e-12);
    EXPECT_EQ(feed.poseAt(hostTime(100)).status, LookupStatus::kOk);

    feed.resetHistory();
    EXPECT_EQ(feed.historySize(), 0u);
}

TEST(PoseHistory, AttitudeUsesItsOwnTimestampsAndCaptureTimeHeading) {
    PoseHistory h;
    PoseHistoryEntry a = entry(100, 0, 0, 0.2);
    a.attitude = measured(0.0, 0, 0.2, 80);
    PoseHistoryEntry b = entry(200, 1, 0, 0.6);
    b.attitude = measured(0.2, 0, 0.6, 180);
    h.append(a);
    h.append(b);
    const auto sample = h.sampleAt(hostTime(130));
    ASSERT_EQ(sample.pose.status, LookupStatus::kOk);
    ASSERT_EQ(sample.attitude.status, LookupStatus::kOk);
    double roll = 0, pitch = 0, yaw = 0;
    attitudeEuler(sample.attitude.attitude, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.1, 1e-9); // 80..180, not the carrying poses' 100..200
    EXPECT_NEAR(yaw, 0.32, 1e-9); // heading interpolated at capture time
    EXPECT_EQ(sample.attitude.attitude.measuredAt.ms, 130);
    EXPECT_EQ(h.attitudeAt(hostTime(200)).status, LookupStatus::kPending);
}

TEST(PoseHistory, IndependentAttitudeRingWrapAndSourceEpochGates) {
    PoseHistoryConfig config;
    config.capacity = 3;
    PoseHistory h(config);
    for (int i = 0; i < 8; ++i) {
        h.appendAttitude(measured(0.01 * i, 0, 0, i * 10), 4);
    }
    EXPECT_TRUE(h.empty()); // attitude alone does not invent planar poses
    EXPECT_EQ(h.attitudeAt(hostTime(40)).status, LookupStatus::kExpired);
    const auto sample = h.attitudeAt(hostTime(65));
    ASSERT_EQ(sample.status, LookupStatus::kOk);
    double roll = 0, pitch = 0, yaw = 0;
    attitudeEuler(sample.attitude, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.065, 1e-9);
    h.appendAttitude(measured(0.08, 0, 0, 80, 1), 4);
    EXPECT_EQ(h.attitudeAt(hostTime(75)).status, LookupStatus::kUnavailable);
    h.appendAttitude(measured(0.09, 0, 0, 90, 1), 5);
    EXPECT_EQ(h.attitudeAt(hostTime(80)).status, LookupStatus::kExpired);
}

TEST(PoseHistory, CombinedRollPitchInterpolationNeverAddsASecondYaw) {
    PoseHistory h;
    PoseHistoryEntry a = entry(0, 0, 0, 0.7);
    a.attitude = measured(0.4, 0.0, 0.7, 0);
    PoseHistoryEntry b = entry(100, 0, 0, 0.7);
    b.attitude = measured(0.0, 0.4, 0.7, 100);
    h.append(a);
    h.append(b);
    const auto sample = h.attitudeAt(hostTime(50));
    ASSERT_EQ(sample.status, LookupStatus::kOk);
    double roll = 0, pitch = 0, yaw = 0;
    attitudeEuler(sample.attitude, roll, pitch, yaw);
    EXPECT_NEAR(yaw, 0.7, 1e-9);
    EXPECT_GT(roll, 0.1);
    EXPECT_GT(pitch, 0.1);
}

TEST(PoseHistory, SameTimeAttitudeRevisionChangesPayloadWithoutRefreshingTime) {
    PoseHistory h;
    h.appendAttitude(measured(0.1, 0, 0, 100), 4);
    EXPECT_EQ(h.appendAttitude(measured(0.2, 0, 1.5, 100, 1), 4),
              AppendResult::kRevised);
    const auto sample = h.attitudeAt(hostTime(100));
    ASSERT_EQ(sample.status, LookupStatus::kOk);
    EXPECT_EQ(sample.attitude.epoch, 1u);
    EXPECT_EQ(sample.attitude.measuredAt.ms, 100);
    double roll = 0, pitch = 0, yaw = 0;
    attitudeEuler(sample.attitude, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.2, 1e-9);
    EXPECT_NEAR(yaw, 0.0, 1e-9); // no pose is available to supply heading
    EXPECT_EQ(h.attitudeAt(hostTime(101)).status, LookupStatus::kPending);
}

TEST(RobotStateFeed, AttitudeOnlyUpdatesDoNotRefreshPoseTime) {
    RobotStateFeed feed;
    RobotState state;
    state.valid = true;
    state.measuredAtHost = hostTime(100);
    state.attitude = measured(0.0, 0, 0, 100);
    feed.publish(state, {}, true, 1);
    state.attitude = measured(0.2, 0, 0, 120);
    feed.publish(state, {}, false, 2);
    EXPECT_EQ(feed.historySize(), 1u);
    EXPECT_EQ(feed.latest().measuredAtHost.ms, 100);
    const auto lookup = feed.sampleAt(hostTime(110));
    EXPECT_EQ(lookup.pose.status, LookupStatus::kPending);
    ASSERT_EQ(lookup.attitude.status, LookupStatus::kOk);
    double roll = 0, pitch = 0, yaw = 0;
    attitudeEuler(lookup.attitude.attitude, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.1, 1e-9);

    // A newer pose carrying exactly this old attitude must not extend it.
    state.measuredAtHost = hostTime(200);
    feed.publish(state, {}, true, 3);
    EXPECT_EQ(feed.attitudeAt(hostTime(150)).status, LookupStatus::kPending);
    state.odometry_epoch = 1;
    state.measuredAtHost = {};
    state.attitude = {};
    feed.publish(state, {}, false, 4);
    EXPECT_EQ(feed.poseAt(hostTime(100)).status, LookupStatus::kEmpty);
    EXPECT_EQ(feed.attitudeAt(hostTime(100)).status, LookupStatus::kEmpty);
}

TEST(RobotStateFeed, ConcurrentSnapshotNeverMixesEpochStateHistoryOrStatus) {
    RobotStateFeed feed;
    std::atomic<bool> done{false};
    std::thread writer([&] {
        for (uint64_t epoch = 1; epoch <= 2000; ++epoch) {
            RobotState state;
            state.valid = true;
            state.odometry_epoch = epoch;
            state.anchor_revision = epoch;
            state.odom_pose.x_m = static_cast<double>(epoch);
            state.measuredAtHost = hostTime(100);
            state.attitude = measured(0.1, 0, 0, 100, epoch);
            LocalizationStatus status;
            status.updates = epoch;
            feed.publish(state, status, true, epoch);
        }
        done = true;
    });
    bool consistent = true;
    do {
        const auto combined = feed.sampleSnapshotAt(hostTime(100));
        const auto& snapshot = combined.current;
        if (snapshot.robot.valid) {
            consistent = consistent && combined.sample.pose.status == LookupStatus::kOk &&
                combined.sample.attitude.status == LookupStatus::kOk &&
                combined.sample.pose.odometry_epoch == snapshot.robot.odometry_epoch &&
                combined.sample.attitude.attitude.epoch == snapshot.robot.odometry_epoch &&
                snapshot.status.updates == snapshot.publication;
        }
        const auto with_trail = feed.snapshot(4);
        if (!with_trail.trail.empty()) {
            consistent = consistent && with_trail.trail.front().odometry_epoch ==
                with_trail.robot.odometry_epoch;
        }
    } while (!done);
    writer.join();
    EXPECT_TRUE(consistent);
}
