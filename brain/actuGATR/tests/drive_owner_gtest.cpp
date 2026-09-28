// drive_owner_gtest.cpp
// Mode ownership: manual, navigation and disabled, one writer, stale manual
// demands, and no resuming after a mode change.

#include "actugatr/drive_owner.h"

#include <cmath>
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

struct Owner {
    Owner() : out(2), drive(tank, wheels(), out), motion(state, planner, follower, config()),
              owner(motion, drive) {
        state.setRobot(Pose{1, 1, 0}, 0);
    }
    SimulatedState       state;
    StraightPlanner      planner;
    DifferentialFollower follower;
    TankKinematics       tank{0.3};
    SimMotorOutput       out;
    Drive                drive;
    Motion               motion;
    DriveOwner           owner;
};

} // namespace

TEST(DriveOwner, StartsDisabledAndStopped) {
    Owner o;
    o.owner.step(0.0);
    EXPECT_EQ(o.owner.mode(), DriveMode::kDisabled);
    EXPECT_TRUE(o.out.stopped());
}

TEST(DriveOwner, ManualDemandDrivesInPhysicalUnits) {
    Owner          o;
    ManualDemand d;
    d.forward = 0.5;
    d.turn    = 0.0;
    o.owner.manual(d, 1.0);
    o.owner.step(1.0);
    EXPECT_EQ(o.owner.mode(), DriveMode::kManual);
    EXPECT_NEAR(o.owner.drive().command.vx, 0.5, 1e-9); // half of manual_speed 1.0 m/s
    EXPECT_FALSE(o.out.stopped());
}

TEST(DriveOwner, TankIgnoresStrafeInsteadOfStopping) {
    Owner          o;
    ManualDemand d;
    d.forward = 0.2;
    d.strafe  = 1.0;
    o.owner.manual(d, 0.0);
    o.owner.step(0.0);
    EXPECT_EQ(o.owner.drive().fault, DriveFault::kNone);
    EXPECT_NEAR(o.owner.drive().command.vy, 0.0, 1e-12);
    EXPECT_NEAR(o.owner.drive().command.vx, 0.2, 1e-9);
}

TEST(DriveOwner, StaleManualDemandStops) {
    Owner          o;
    ManualDemand d;
    d.forward = 0.5;
    o.owner.manual(d, 1.0);
    o.owner.step(1.1);
    EXPECT_FALSE(o.out.stopped());
    o.owner.step(1.3);
    EXPECT_TRUE(o.out.stopped());
    EXPECT_EQ(o.owner.drive().fault, DriveFault::kStale);
}

TEST(DriveOwner, NonFiniteManualDemandIsZero) {
    Owner          o;
    ManualDemand d;
    d.forward = std::nan("");
    d.turn    = 2.0;
    o.owner.manual(d, 0.0);
    o.owner.step(0.0);
    EXPECT_NEAR(o.owner.drive().command.vx, 0.0, 1e-12);
    EXPECT_NEAR(o.owner.drive().command.omega, 3.0, 1e-9); // clamped to one full manual_omega
}

TEST(DriveOwner, ManualCancelsNavigationAndNothingResumes) {
    Owner           o;
    const CommandId id = o.owner.goToDirect({2.0, 1.0, 0});
    EXPECT_EQ(o.owner.mode(), DriveMode::kNavigate);
    o.owner.step(0.0);
    o.owner.manual(ManualDemand{}, 0.01);
    EXPECT_EQ(o.owner.motion().command_id, id);
    EXPECT_EQ(o.owner.motion().state, MotionState::kCancelled);
    o.owner.step(0.02);
    EXPECT_EQ(o.owner.mode(), DriveMode::kManual);
    EXPECT_EQ(o.owner.motion().state, MotionState::kCancelled);
}

TEST(DriveOwner, DisableCancelsAndStops) {
    Owner o;
    o.owner.goToDirect({2.0, 1.0, 0});
    o.state.setRobot(Pose{1, 1, 0}, 0.1);
    o.owner.step(0.1);
    o.owner.disable();
    o.owner.step(0.11);
    EXPECT_EQ(o.owner.mode(), DriveMode::kDisabled);
    EXPECT_EQ(o.owner.motion().state, MotionState::kCancelled);
    EXPECT_TRUE(o.out.stopped());
}
