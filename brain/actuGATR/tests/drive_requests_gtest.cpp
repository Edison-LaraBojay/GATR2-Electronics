// drive_requests_gtest.cpp
// Requests to the drive task: latest wins, goTo ids become Motion ids, and a
// goTo not yet taken never reads as idle.

#include "actugatr/drive_requests.h"

#include <gtest/gtest.h>

#include "sim/sim_motor_output.h"
#include "sim/simulated_state.h"
#include "sim/straight_planner.h"

using namespace actugatr;

namespace
{

WheelDrive wheels() {
    WheelDrive w;
    w.wheel_diameter  = 0.1;
    w.usable_fraction = 1.0;
    return w;
}

MotionConfig config() {
    MotionConfig c;
    c.model.footprint = {0.2, 0.2, 0.2, 0.2};
    return c;
}

struct Task {
    Task() : out(2), drive(tank, wheels(), out), motion(state, planner, follower, config()),
             owner(motion, drive) {
        state.setRobot(Pose{1, 1, 0}, 0);
    }

    // One drive task cycle.
    void step() {
        apply(requests.take(), owner);
        owner.step(t);
        DriveSnapshot s;
        s.motion = owner.motion();
        s.drive  = owner.drive();
        s.mode   = owner.mode();
        requests.publish(s);
        t += 0.01;
        state.setRobot(Pose{1, 1, 0}, t);
    }

    SimulatedState       state;
    StraightPlanner      planner;
    DifferentialFollower follower;
    TankKinematics       tank{0.3};
    SimMotorOutput       out;
    Drive                drive;
    Motion               motion;
    DriveOwner           owner;
    DriveRequests        requests;
    Seconds              t = 0;
};

} // namespace

TEST(DriveRequests, GoToIdIsTheMotionIdAndShowsWaitingUntilTaken) {
    Task            k;
    const CommandId id = k.requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    EXPECT_EQ(id, 1u);
    DriveSnapshot s = k.requests.snapshot();
    EXPECT_EQ(s.motion.command_id, id);
    EXPECT_EQ(s.motion.state, MotionState::kWaiting);
    EXPECT_EQ(s.mode, DriveMode::kNavigate);

    k.step();
    EXPECT_EQ(k.owner.motion().command_id, id);
    s = k.requests.snapshot();
    EXPECT_EQ(s.motion.command_id, id);
    EXPECT_FALSE(isTerminal(s.motion.state));
    EXPECT_NE(s.motion.state, MotionState::kIdle);
}

TEST(DriveRequests, LatestRequestWins) {
    Task k;
    k.requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    ManualDemand d;
    d.forward = 0.3;
    k.requests.manual(d, 0.0);
    EXPECT_EQ(k.requests.snapshot().motion.command_id, 0u); // the goTo never started
    k.step();
    EXPECT_EQ(k.owner.mode(), DriveMode::kManual);
    EXPECT_EQ(k.owner.motion().command_id, 0u);

    const CommandId id = k.requests.goTo(PlanMode::kAvoiding, {1.5, 1, 0}, Reference::origin(), {});
    EXPECT_EQ(id, 2u);
    k.requests.stop();
    k.step();
    EXPECT_EQ(k.owner.mode(), DriveMode::kDisabled);
    EXPECT_EQ(k.owner.motion().command_id, 0u);
}

TEST(DriveRequests, StopCancelsTheRunningCommand) {
    Task            k;
    const CommandId id = k.requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    k.step();
    k.step();
    k.requests.stop();
    k.step();
    const DriveSnapshot s = k.requests.snapshot();
    EXPECT_EQ(s.motion.command_id, id);
    EXPECT_EQ(s.motion.state, MotionState::kCancelled);
    EXPECT_EQ(s.mode, DriveMode::kDisabled);
    EXPECT_TRUE(k.out.stopped());
}

TEST(DriveRequests, ManualKeepsTheApplicationTimeForStaleness) {
    Task         k;
    ManualDemand d;
    d.forward = 0.5;
    k.requests.manual(d, -1.0); // sent long before this cycle
    k.step();
    EXPECT_EQ(k.owner.drive().fault, DriveFault::kStale);
}

// The operator loop reads the snapshot, then a button requests a test, then
// the manual rule runs with centered sticks. The test must stay active.
TEST(DriveRequests, CenteredSticksNeverCancelANewCommand) {
    Task k;
    k.step();
    const DriveSnapshot before = k.requests.snapshot(); // read at the top of the loop
    ASSERT_FALSE(moving(before));

    const CommandId id = k.requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    const ManualDemand centered;
    EXPECT_FALSE(sendManual(centered, true, before)); // stale idle snapshot, same cycle
    EXPECT_TRUE(moving(k.requests.snapshot()));        // a fresh read shows it waiting
    EXPECT_FALSE(sendManual(centered, false, k.requests.snapshot()));

    for (int i = 0; i < 20; ++i) {
        k.step();
        if (sendManual(centered, false, k.requests.snapshot())) {
            k.requests.manual(centered, k.t);
        }
    }
    const DriveSnapshot s = k.requests.snapshot();
    EXPECT_EQ(s.motion.command_id, id);
    EXPECT_TRUE(moving(s));
    EXPECT_EQ(s.mode, DriveMode::kNavigate);
}

TEST(DriveRequests, SticksTakeOverAndIdleSendsZero) {
    Task         k;
    ManualDemand push;
    push.forward = 0.4;
    k.requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    k.step();
    EXPECT_TRUE(sendManual(push, false, k.requests.snapshot())); // over a running command
    EXPECT_TRUE(sendManual(push, true, k.requests.snapshot()));
    k.requests.stop();
    k.step();
    EXPECT_TRUE(sendManual(ManualDemand{}, false, k.requests.snapshot())); // idle: keep fresh
}

// Why the rule exists: a zero demand posted after a request in the same
// cycle replaces it, so the command never starts.
TEST(DriveRequests, ZeroDemandAfterARequestReplacesIt) {
    Task            k;
    const CommandId id = k.requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    k.requests.manual(ManualDemand{}, k.t);
    k.step();
    EXPECT_NE(k.owner.motion().command_id, id);
    EXPECT_EQ(k.owner.mode(), DriveMode::kManual);
}

TEST(DriveRequests, AvoidingModeReachesMotion) {
    Task k;
    k.requests.goTo(PlanMode::kAvoiding, {1.5, 1, 0}, Reference::origin(), {});
    EXPECT_EQ(k.requests.snapshot().motion.mode, PlanMode::kAvoiding);
    k.step();
    EXPECT_EQ(k.owner.motion().mode, PlanMode::kAvoiding);
}
