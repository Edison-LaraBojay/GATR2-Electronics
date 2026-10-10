// sim_gtest.cpp
// The deterministic sim: exact poses for known ticks, identical bytes on every run, the restart
// and dropout schedules, the field bounds, and the bytes going through the real feed path.

#include "receiver/sim/trajectory.h"

#include <gtest/gtest.h>

#include <cmath>

#include "receiver/feed/pose_feed.h"
#include "spectagatr/protocol/units.h"

namespace spectagatr::receiver::sim
{
namespace
{

constexpr double kPi = protocol::kPi;

SimConfig config(SimMode mode, Hertz rate = 10.0, uint32_t seed = 1) {
    SimConfig c;
    c.mode    = mode;
    c.rate_hz = rate;
    c.seed    = seed;
    return c;
}

TEST(Sim, Figure8StartsAtTheCenterHeadingUpRight) {
    const std::optional<protocol::PoseRecord> first = simRecord(config(SimMode::kFigure8), 0);
    ASSERT_TRUE(first);
    EXPECT_EQ(first->x_mm, 1783); // 3.5664 / 2 = 1.7832 m
    EXPECT_EQ(first->y_mm, 1783);
    EXPECT_EQ(first->yaw_cdeg, 4500); // dx = dy at t = 0
    EXPECT_EQ(first->seq, 0u);
    EXPECT_EQ(first->t_ms, 1000u);
    EXPECT_EQ(first->epoch, 0);
    EXPECT_EQ(first->flags, 0);
    EXPECT_EQ(first->session, simSession(1, 0));
    // A quarter period (5 s = tick 50): the right lobe's tip, heading straight down.
    const std::optional<protocol::PoseRecord> tip = simRecord(config(SimMode::kFigure8), 50);
    ASSERT_TRUE(tip);
    EXPECT_EQ(tip->x_mm, 2983);
    EXPECT_EQ(tip->y_mm, 1783);
    EXPECT_EQ(tip->yaw_cdeg, -9000);
    EXPECT_EQ(tip->t_ms, 6000u);
}

TEST(Sim, CircleIsCounterclockwiseWithTangentHeading) {
    const SimPose start   = simMotion(SimMode::kCircle, 0.0);
    const SimPose quarter = simMotion(SimMode::kCircle, 3.0);
    EXPECT_NEAR(start.x, kFieldCenter + 1.0, 1e-9);
    EXPECT_NEAR(start.y, kFieldCenter, 1e-9);
    EXPECT_NEAR(start.yaw, kPi / 2.0, 1e-9); // moving +y
    EXPECT_NEAR(quarter.x, kFieldCenter, 1e-9);
    EXPECT_NEAR(quarter.y, kFieldCenter + 1.0, 1e-9);
}

TEST(Sim, SquareDrivesThenTurnsInPlace) {
    const SimPose start = simMotion(SimMode::kSquare, 0.0);
    EXPECT_NEAR(start.x, kFieldCenter - 1.0, 1e-9);
    EXPECT_NEAR(start.y, kFieldCenter - 1.0, 1e-9);
    EXPECT_NEAR(start.yaw, 0.0, 1e-9);
    const SimPose turning = simMotion(SimMode::kSquare, 4.5); // half way through the first turn
    EXPECT_NEAR(turning.x, kFieldCenter + 1.0, 1e-9);
    EXPECT_NEAR(turning.y, kFieldCenter - 1.0, 1e-9);
    EXPECT_NEAR(turning.yaw, kPi / 4.0, 1e-9);
    const SimPose second = simMotion(SimMode::kSquare, 7.0); // second edge, heading +y
    EXPECT_NEAR(second.x, kFieldCenter + 1.0, 1e-9);
    EXPECT_NEAR(second.y, kFieldCenter, 1e-9);
    EXPECT_NEAR(second.yaw, kPi / 2.0, 1e-9);
}

TEST(Sim, AttitudeSweepsEachAxisPositiveOneAtATime) {
    const SimConfig attitude = config(SimMode::kAttitude);
    const auto      at       = [&](Tick k) { return *simRecord(attitude, k); };
    // 3 s into each 6 s sweep is its peak.
    EXPECT_EQ(at(30).yaw_cdeg, 9000);
    EXPECT_EQ(at(30).roll_cdeg, 0);
    EXPECT_EQ(at(90).roll_cdeg, 2000); // left side up
    EXPECT_EQ(at(90).yaw_cdeg, 0);
    EXPECT_EQ(at(150).pitch_cdeg, 2000); // nose down
    EXPECT_EQ(at(190).yaw_cdeg + at(190).roll_cdeg + at(190).pitch_cdeg, 0);
    for (const Tick k : {Tick{0}, Tick{30}, Tick{90}, Tick{150}}) {
        EXPECT_EQ(at(k).x_mm, 1783);
        EXPECT_EQ(at(k).y_mm, 1783);
        EXPECT_EQ(at(k).flags, protocol::kFullAttitude);
    }
}

TEST(Sim, RestartChangesSessionEvery15sAndEpochHalfWay) {
    const SimConfig restart = config(SimMode::kRestart);
    const auto      at      = [&](Tick k) { return *simRecord(restart, k); };
    EXPECT_EQ(at(0).session, simSession(1, 0));
    EXPECT_EQ(at(74).epoch, 0);
    EXPECT_EQ(at(75).epoch, 1);          // 7.5 s
    EXPECT_EQ(at(75).seq, 75u);          // seq runs on across the epoch bump
    EXPECT_NE(at(74).x_mm, at(75).x_mm); // the pose jumps at the reset
    EXPECT_EQ(at(149).session, simSession(1, 0));
    EXPECT_EQ(at(150).session, simSession(1, 1)); // 15 s: a new publisher run
    EXPECT_EQ(at(150).seq, 0u);
    EXPECT_EQ(at(150).epoch, 0);
    EXPECT_EQ(at(150).t_ms, 1000u);
    EXPECT_NE(simSession(1, 0), simSession(1, 1));
}

TEST(Sim, DropoutIsSilentForThreeSecondsOfTen) {
    const SimConfig dropout = config(SimMode::kDropout);
    EXPECT_TRUE(simRecord(dropout, 69));
    EXPECT_FALSE(simRecord(dropout, 70));
    EXPECT_FALSE(simRecord(dropout, 99));
    const std::optional<protocol::PoseRecord> back = simRecord(dropout, 100);
    ASSERT_TRUE(back);
    EXPECT_EQ(back->seq, 100u); // the silent seqs are lost, as on a radio
}

TEST(Sim, SameSeedSameBytesOtherSeedOtherSession) {
    for (const SimMode mode : {SimMode::kFigure8, SimMode::kCircle, SimMode::kSquare,
                               SimMode::kAttitude, SimMode::kRestart, SimMode::kDropout}) {
        for (Tick k = 0; k < 400; k += 7) {
            EXPECT_EQ(simBytes(config(mode), k), simBytes(config(mode), k));
        }
    }
    EXPECT_NE(simRecord(config(SimMode::kCircle, 10, 1), 0)->session,
              simRecord(config(SimMode::kCircle, 10, 2), 0)->session);
    EXPECT_EQ(simTime(config(SimMode::kCircle, 25), 50), 2.0);
}

TEST(Sim, EveryModeStaysInsideTheField) {
    for (const SimMode mode : {SimMode::kFigure8, SimMode::kCircle, SimMode::kSquare,
                               SimMode::kAttitude, SimMode::kRestart}) {
        for (double t = 0; t < 60.0; t += 0.05) {
            const SimPose pose = simMotion(mode, t);
            EXPECT_GT(pose.x, 0.1);
            EXPECT_LT(pose.x, kFieldSize - 0.1);
            EXPECT_GT(pose.y, 0.1);
            EXPECT_LT(pose.y, kFieldSize - 0.1);
            EXPECT_TRUE(std::isfinite(pose.yaw));
        }
    }
}

TEST(Sim, BytesGoThroughTheRealFeedPath) {
    feed::PoseFeed  feed;
    const SimConfig restart  = config(SimMode::kRestart, 10.0, 7);
    std::size_t     restarts = 0;
    for (Tick k = 0; k < 400; ++k) {
        const sources::Bytes bytes = simBytes(restart, k);
        feed.push(bytes.data(), bytes.size(), static_cast<double>(k) * 100.0,
                  [&](const feed::TrackedPose& pose) {
                      restarts += pose.restart ? 1 : 0;
                      EXPECT_EQ(pose.record, *simRecord(restart, k));
                  });
    }
    // Ticks 0, 75, 150, 225, 300, 375: first pose, then an epoch or a session each.
    EXPECT_EQ(restarts, 6u);
    EXPECT_EQ(feed.counts().poses, 400u);
    EXPECT_EQ(feed.counts().sessions, 3u);
    EXPECT_EQ(feed.counts().malformed + feed.counts().bad_crc + feed.counts().truncated, 0u);
}

TEST(Sim, ModeNames) {
    for (const SimMode mode : {SimMode::kFigure8, SimMode::kCircle, SimMode::kSquare,
                               SimMode::kAttitude, SimMode::kRestart, SimMode::kDropout}) {
        EXPECT_EQ(simModeOf(simModeName(mode)), mode);
    }
    EXPECT_FALSE(simModeOf("spiral"));
}

} // namespace
} // namespace spectagatr::receiver::sim
