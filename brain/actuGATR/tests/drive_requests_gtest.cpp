// drive_requests_gtest.cpp
// Requests to the drive task: latest wins, goTo ids become Motion ids, and a
// goTo awaiting task publication never reads as idle.

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

// The drive task releases the request mutex between take() and publish().
// Another operator cycle with centered sticks must not cancel that command.
TEST(DriveRequests, TakenCommandStaysWaitingUntilItsStatusIsPublished) {
    Task k;
    k.step();
    const CommandId id = k.requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    const DriveRequest request = k.requests.take();

    const DriveSnapshot during = k.requests.snapshot();
    EXPECT_EQ(during.motion.command_id, id);
    EXPECT_EQ(during.motion.state, MotionState::kWaiting);
    EXPECT_EQ(during.mode, DriveMode::kNavigate);
    EXPECT_FALSE(sendManual(ManualDemand{}, false, during));

    apply(request, k.owner);
    k.owner.step(k.t);
    k.requests.publish({k.owner.motion(), k.owner.drive(), k.owner.mode()});
    const DriveSnapshot after = k.requests.snapshot();
    EXPECT_EQ(after.motion.command_id, id);
    EXPECT_EQ(after.motion.state, k.owner.motion().state);
    EXPECT_TRUE(moving(after));
}

TEST(DriveRequests, EmptyTakeAfterMissedPublicationKeepsCommandVisible) {
    DriveRequests requests;
    const CommandId id = requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    requests.take();
    // Simulate the drive task failing to acquire the publication mutex.
    EXPECT_EQ(requests.take().kind, DriveRequest::Kind::kNone);
    EXPECT_EQ(requests.snapshot().motion.command_id, id);
    EXPECT_FALSE(sendManual(ManualDemand{}, false, requests.snapshot()));

    DriveSnapshot result;
    result.motion.command_id = id;
    result.motion.state = MotionState::kRunning;
    requests.publish(result);
    EXPECT_EQ(requests.snapshot().motion.state, MotionState::kRunning);
}

TEST(DriveRequests, PublishedTerminalResultClearsTakenCommandWaitingState) {
    for (const MotionState terminal : {MotionState::kFailed, MotionState::kCompleted,
                                       MotionState::kCancelled}) {
        DriveRequests requests;
        const CommandId id = requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
        requests.take();
        DriveSnapshot result;
        result.motion.command_id = id;
        result.motion.state = terminal;
        requests.publish(result);
        EXPECT_EQ(requests.snapshot().motion.state, terminal);
        EXPECT_TRUE(sendManual(ManualDemand{}, false, requests.snapshot()));
    }
}

TEST(DriveRequests, NewerPendingCommandSurvivesOlderPublicationAndItsOwnTake) {
    DriveRequests requests;
    const CommandId first = requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
    requests.take();
    const CommandId second = requests.goTo(PlanMode::kAvoiding, {2, 1, 0}, Reference::origin(), {});
    EXPECT_EQ(requests.snapshot().motion.command_id, second);

    DriveSnapshot result;
    result.motion.command_id = first;
    result.motion.state = MotionState::kFailed;
    requests.publish(result);
    EXPECT_EQ(requests.snapshot().motion.command_id, second);
    EXPECT_EQ(requests.take().id, second);
    EXPECT_EQ(requests.snapshot().motion.command_id, second);
    EXPECT_EQ(requests.snapshot().motion.mode, PlanMode::kAvoiding);
    EXPECT_FALSE(sendManual(ManualDemand{}, false, requests.snapshot()));
}

TEST(DriveRequests, SticksAndStopStillOverrideATakenCommand) {
    for (const bool stop : {false, true}) {
        Task k;
        const CommandId id = k.requests.goTo(PlanMode::kDirect, {1.5, 1, 0}, Reference::origin(), {});
        const DriveRequest request = k.requests.take();
        if (stop) {
            k.requests.stop();
        } else {
            ManualDemand push;
            push.forward = 0.4;
            EXPECT_TRUE(sendManual(push, false, k.requests.snapshot()));
            k.requests.manual(push, k.t);
        }
        apply(request, k.owner);
        k.owner.step(k.t);
        k.requests.publish({k.owner.motion(), k.owner.drive(), k.owner.mode()});
        k.step();
        EXPECT_EQ(k.requests.snapshot().motion.command_id, id);
        EXPECT_EQ(k.requests.snapshot().motion.state, MotionState::kCancelled);
        EXPECT_EQ(k.requests.snapshot().mode, stop ? DriveMode::kDisabled : DriveMode::kManual);
    }
}

TEST(DriveRequests, UnavailableStatusNeverAllowsCenteredSticksToCancelACommand) {
    for (const MotionState cached : {MotionState::kIdle, MotionState::kCompleted,
                                    MotionState::kFailed, MotionState::kCancelled}) {
        DriveSnapshot old;
        old.motion.state = cached;
        old.motion.command_id = cached == MotionState::kIdle ? 0 : 1;
        EXPECT_TRUE(sendManual(ManualDemand{}, false, old, true));
        // A newer command may have started since this cached status. A failed
        // status read must not let centered sticks overwrite that command.
        EXPECT_FALSE(sendManual(ManualDemand{}, false, old, false));

        ManualDemand sticks;
        sticks.turn = 0.4;
        EXPECT_TRUE(sendManual(sticks, false, old, false));
        EXPECT_TRUE(sendManual(sticks, true, old, false));
    }
}
