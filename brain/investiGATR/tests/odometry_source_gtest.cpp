// odometry_source_gtest.cpp

#include "investigatr/odometry_source.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "investigatr/navigator.h"
#include "sim/differential_drive_sim.h"
#include "sim/simulated_source.h"

using namespace investigatr;

namespace
{

constexpr Seconds kPeriod = 0.01;
constexpr Meters  kTrack  = 0.30;

void expectPose(const Pose& actual, const Pose& expected, double tolerance) {
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(wrapAngle(actual.heading - expected.heading), 0.0, tolerance);
}

InputSnapshot freshSnapshot(const Pose& pose) {
    InputSnapshot snapshot;
    snapshot.frame       = 4;
    snapshot.connected   = true;
    snapshot.robot.valid = true;
    snapshot.robot.pose  = pose;
    return snapshot;
}

// Navigator, drivetrain sim, a Navigatr stand in reporting truth, and
// odometry fed from the sim's wheel travel.
struct FallbackRig {
    FallbackRig()
        : drive(DifferentialDriveConfig{}, Pose{0.2, 0.3, 0.4}), odometry(kTrack),
          navigator(navigatr) {
        navigatr.setFrame(5);
    }

    void step() {
        navigatr.setRobot(drive.pose(), now);
        odometry.update(now, drive.leftTravel(), drive.rightTravel());
        drive.step(mixTank(navigator.update(now)), kPeriod);
        now += kPeriod;
    }

    MotionState run(Seconds limit) {
        const Seconds end = now + limit;
        while (now < end && !isTerminal(navigator.status().state)) {
            step();
        }
        return navigator.status().state;
    }

    DifferentialDriveSim drive;
    SimulatedSource      navigatr;
    OdometrySource       odometry;
    Navigator            navigator;
    Seconds              now = 0;
};

} // namespace

TEST(OdometrySource, NoPoseBeforeAlign) {
    OdometrySource odometry(kTrack);
    odometry.update(0.0, 0.0, 0.0);
    odometry.update(0.1, 0.2, 0.2);
    const InputSnapshot snapshot = odometry.latest(0.1);
    EXPECT_FALSE(odometry.aligned());
    EXPECT_EQ(snapshot.frame, 0u);
    EXPECT_FALSE(snapshot.connected);
    EXPECT_FALSE(snapshot.robot.valid);
}

TEST(OdometrySource, EachAlignStartsNewGeneration) {
    OdometrySource odometry(kTrack);
    odometry.align(Pose{1.0, 2.0, 0.5}, 0.0);
    EXPECT_EQ(odometry.frame(), 1u);
    InputSnapshot snapshot = odometry.latest(0.0);
    EXPECT_EQ(snapshot.frame, 1u);
    EXPECT_TRUE(snapshot.connected);
    EXPECT_TRUE(snapshot.robot.valid);
    expectPose(snapshot.robot.pose, Pose{1.0, 2.0, 0.5}, 1e-12);

    odometry.align(Pose{0.0, 0.0, 0.0}, 1.0);
    EXPECT_EQ(odometry.latest(1.0).frame, 2u);

    odometry.invalidate();
    snapshot = odometry.latest(1.0);
    EXPECT_EQ(snapshot.frame, 0u);
    EXPECT_FALSE(snapshot.connected);
    EXPECT_FALSE(snapshot.robot.valid);

    odometry.align(Pose{0.0, 0.0, 0.0}, 2.0);
    EXPECT_EQ(odometry.latest(2.0).frame, 3u);
}

TEST(OdometrySource, StraightAndTurnInPlace) {
    OdometrySource odometry(kTrack);
    odometry.update(0.0, 10.0, 20.0);
    odometry.align(Pose{0.0, 0.0, 0.0}, 0.0);
    odometry.update(0.1, 10.5, 20.5);
    expectPose(odometry.pose(), Pose{0.5, 0.0, 0.0}, 1e-12);
    // 0.15 m each way on a 0.30 m track is a 1 rad turn.
    odometry.update(0.2, 10.35, 20.65);
    expectPose(odometry.pose(), Pose{0.5, 0.0, 1.0}, 1e-12);
}

TEST(OdometrySource, IntegratesLikeTheDriveSim) {
    DifferentialDriveSim drive(DifferentialDriveConfig{}, Pose{1.0, -0.5, 2.0});
    OdometrySource       odometry(kTrack);
    Seconds              now = 0;
    odometry.update(now, drive.leftTravel(), drive.rightTravel());
    odometry.align(drive.pose(), now);
    const TankOutput outputs[] = {{0.5, 0.5}, {0.2, 0.8}, {-0.6, 0.6}, {0.9, 0.3}, {-0.4, -0.4}};
    for (const TankOutput& output : outputs) {
        for (int i = 0; i < 100; ++i) {
            drive.step(output, kPeriod);
            now += kPeriod;
            odometry.update(now, drive.leftTravel(), drive.rightTravel());
        }
    }
    expectPose(odometry.latest(now).robot.pose, drive.pose(), 1e-9);
}

TEST(OdometrySource, AbsoluteHeadingDrivesTurn) {
    OdometrySource odometry(kTrack);
    odometry.update(0.0, 0.0, 0.0, 0.2);
    odometry.align(Pose{0.0, 0.0, 1.0}, 0.0);
    // Wheels say straight, the heading sensor says 0.3 rad left.
    odometry.update(0.1, 0.1, 0.1, 0.5);
    expectPose(odometry.pose(), Pose{0.1 * std::cos(1.15), 0.1 * std::sin(1.15), 1.3}, 1e-12);
    // Sensor wrapping across pi.
    odometry.update(0.2, 0.1, 0.1, 3.1);
    odometry.update(0.3, 0.1, 0.1, -3.1);
    EXPECT_NEAR(odometry.pose().heading, wrapAngle(1.0 + (-3.1 - 0.2)), 1e-12);
}

TEST(OdometrySource, NonFiniteReadingsIgnored) {
    const double   inf = std::numeric_limits<double>::infinity();
    OdometrySource odometry(kTrack);
    odometry.update(0.0, 0.0, 0.0, 0.0);
    odometry.align(Pose{1.0, 1.0, 0.0}, 0.0);
    odometry.update(0.1, 0.1, 0.1, inf);
    odometry.update(0.2, std::numeric_limits<double>::quiet_NaN(), 0.1, 0.0);
    expectPose(odometry.pose(), Pose{1.0, 1.0, 0.0}, 1e-12);
    EXPECT_NEAR(odometry.latest(0.2).robot.age, 0.2, 1e-12);
    odometry.update(0.3, 0.1, 0.1, 0.0);
    expectPose(odometry.pose(), Pose{1.1, 1.0, 0.0}, 1e-12);
}

TEST(OdometrySource, AgeFromLastUpdate) {
    OdometrySource odometry(kTrack);
    odometry.align(Pose{}, 1.0);
    InputSnapshot snapshot = odometry.latest(1.2);
    EXPECT_NEAR(snapshot.robot.age, 0.2, 1e-12);
    EXPECT_NEAR(snapshot.link_age, 0.2, 1e-12);
    odometry.update(1.3, 0.0, 0.0);
    snapshot = odometry.latest(1.3);
    EXPECT_EQ(snapshot.robot.age, 0.0);
}

TEST(OdometrySource, LandmarksUnsupported) {
    OdometrySource odometry(kTrack);
    odometry.align(Pose{}, 0.0);
    odometry.request(InputRequest{true, 3});
    const InputSnapshot snapshot = odometry.latest(0.0);
    EXPECT_EQ(snapshot.landmark.id, 3);
    EXPECT_EQ(snapshot.landmark.status, LandmarkStatus::kUnsupported);

    Navigator navigator(odometry);
    navigator.goToRelative(3, Pose{});
    navigator.update(0.0);
    EXPECT_EQ(navigator.status().state, MotionState::kFailed);
    EXPECT_EQ(navigator.status().reason, MotionReason::kLandmarkUnsupported);
}

TEST(OdometrySource, AlignFromRefusesUnusableSnapshots) {
    OdometrySource odometry(kTrack);
    const Pose     pose{1.0, 1.0, 0.0};

    InputSnapshot snapshot = freshSnapshot(pose);
    snapshot.robot.valid   = false;
    EXPECT_FALSE(odometry.alignFrom(snapshot, 0.25, 0.0));

    snapshot       = freshSnapshot(pose);
    snapshot.frame = 0;
    EXPECT_FALSE(odometry.alignFrom(snapshot, 0.25, 0.0));

    snapshot           = freshSnapshot(pose);
    snapshot.connected = false;
    EXPECT_FALSE(odometry.alignFrom(snapshot, 0.25, 0.0));

    snapshot           = freshSnapshot(pose);
    snapshot.robot.age = 0.3;
    EXPECT_FALSE(odometry.alignFrom(snapshot, 0.25, 0.0));

    EXPECT_FALSE(odometry.aligned());
    EXPECT_EQ(odometry.frame(), 0u);

    snapshot           = freshSnapshot(pose);
    snapshot.robot.age = 0.1;
    EXPECT_TRUE(odometry.alignFrom(snapshot, 0.25, 5.0));
    EXPECT_EQ(odometry.frame(), 1u);
    const InputSnapshot aligned = odometry.latest(5.0);
    expectPose(aligned.robot.pose, pose, 1e-12);
    EXPECT_NEAR(aligned.robot.age, 0.1, 1e-12);
}

TEST(OdometrySource, ExplicitSwitchCancelsActiveCommand) {
    FallbackRig rig;
    rig.step();
    ASSERT_TRUE(rig.odometry.alignFrom(rig.navigatr.latest(rig.now), 0.25, rig.now));
    rig.navigator.goTo(Pose{1.5, 0.3, 0.0});
    rig.run(0.5);
    ASSERT_FALSE(isTerminal(rig.navigator.status().state));

    rig.navigator.setSource(rig.odometry);
    EXPECT_EQ(rig.navigator.status().state, MotionState::kCancelled);
    EXPECT_EQ(rig.navigator.status().reason, MotionReason::kSourceChanged);
    rig.step();
    EXPECT_EQ(rig.navigator.status().state, MotionState::kCancelled);
}

TEST(OdometrySource, FallbackAfterNavigatrLoss) {
    const NavigatorConfig config;
    FallbackRig           rig;
    const Pose            goal{1.5, 0.3, 0.0};

    // Stationary, fresh Navigatr pose: align odometry to it.
    rig.step();
    ASSERT_TRUE(rig.odometry.alignFrom(rig.navigatr.latest(rig.now), config.max_pose_age, rig.now));
    rig.navigator.goTo(goal);
    rig.run(1.0);
    ASSERT_FALSE(isTerminal(rig.navigator.status().state));

    // Navigatr drops out mid command.
    rig.navigatr.setConnected(false);
    rig.step();
    EXPECT_EQ(rig.navigator.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.navigator.status().reason, MotionReason::kInputLost);
    EXPECT_FALSE(
        rig.odometry.alignFrom(rig.navigatr.latest(rig.now), config.max_pose_age, rig.now));

    // Explicit switch and a new command. Nothing resumes on its own.
    rig.navigator.setSource(rig.odometry);
    EXPECT_EQ(rig.navigator.status().reason, MotionReason::kInputLost);
    rig.step();
    EXPECT_EQ(rig.navigator.status().state, MotionState::kFailed);
    const CommandId id = rig.navigator.goTo(goal);
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    EXPECT_EQ(rig.navigator.status().command_id, id);
    EXPECT_LE(std::hypot(rig.drive.pose().x - goal.x, rig.drive.pose().y - goal.y),
              config.position_tolerance);
    EXPECT_LE(std::fabs(wrapAngle(rig.drive.pose().heading - goal.heading)),
              config.heading_tolerance);
}
