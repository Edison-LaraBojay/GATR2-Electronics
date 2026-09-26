// equivalence_gtest.cpp
// One Navigator scenario on the drivetrain sim, run twice: once on
// SimulatedSource fed with the sim truth, once on Driver with the fake
// Pi reporting that truth over the fake bus. Both must complete at the same
// pose.

#include "communigatr/driver.h"

#include <cmath>
#include <cstdint>
#include <functional>
#include <gtest/gtest.h>

#include "investigatr/navigator.h"
#include "sim/differential_drive_sim.h"
#include "sim/link_rig.h"
#include "sim/simulated_source.h"

using namespace communigatr;
using investigatr::CommandId;
using investigatr::DifferentialDriveSim;
using investigatr::DriveCommand;
using investigatr::isTerminal;
using investigatr::kPi;
using investigatr::LandmarkId;
using investigatr::Meters;
using investigatr::MotionReason;
using investigatr::MotionState;
using investigatr::Navigator;
using investigatr::NavigatorConfig;
using investigatr::Pose;
using investigatr::wrapAngle;

namespace
{

constexpr Seconds    kTick         = 0.001; // sim and link step
constexpr int        kControlTicks = 10;    // Navigator period 10 ms
constexpr Seconds    kLimit        = 15.0;
constexpr LandmarkId kLandmark     = 4;

using Command = std::function<CommandId(Navigator&)>;

struct Scenario {
    Pose    start;
    Pose    landmark; // physical landmark pose, field frame
    Command command;
};

struct Outcome {
    MotionState  state  = MotionState::kIdle;
    MotionReason reason = MotionReason::kNone;
    Pose         pose; // sim truth at the end
    Seconds      elapsed = 0;
};

Outcome outcome(const Navigator& navigator, const DifferentialDriveSim& drive) {
    return {navigator.status().state, navigator.status().reason, drive.pose(),
            navigator.status().elapsed};
}

int32_t toMm(double meters) { return static_cast<int32_t>(std::lround(meters * 1000.0)); }

int32_t toCdeg(double radians) {
    return static_cast<int32_t>(std::lround(wrapAngle(radians) * 18000.0 / kPi));
}

Outcome runSimulated(const Scenario& scenario) {
    DifferentialDriveSim           drive({}, scenario.start);
    investigatr::SimulatedSource   source;
    investigatr::SimulatedLandmark landmark;
    landmark.pose = scenario.landmark;
    source.setLandmark(kLandmark, landmark);

    Navigator navigator(source);
    scenario.command(navigator);
    DriveCommand demand;
    for (int tick = 0; tick * kTick < kLimit && !isTerminal(navigator.status().state); ++tick) {
        const Seconds now = tick * kTick;
        source.setRobot(drive.pose(), now);
        if (tick % kControlTicks == 0) {
            demand = navigator.update(now);
        }
        drive.step(investigatr::mixTank(demand), kTick);
    }
    return outcome(navigator, drive);
}

// The Pi's estimate is the sim truth, reported fresh at every request.
void report(FakePi& pi, const Pose& pose) {
    pi.robot().x_mm         = toMm(pose.x);
    pi.robot().y_mm         = toMm(pose.y);
    pi.robot().heading_cdeg = toCdeg(pose.heading);
    pi.robot().robot_age_ms = 0;
}

Outcome runNavigatr(const Scenario& scenario) {
    LinkRig              rig;
    Driver               driver(rig.client());
    DifferentialDriveSim drive({}, scenario.start);
    FakeLandmark         landmark;
    landmark.x_mm         = toMm(scenario.landmark.x);
    landmark.y_mm         = toMm(scenario.landmark.y);
    landmark.heading_cdeg = toCdeg(scenario.landmark.heading);
    rig.pi.setLandmark(kLandmark, landmark);

    // Initialization as the application does it: connect, then place.
    report(rig.pi, drive.pose());
    EXPECT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, 3.0));
    const PlacementTicket ticket = driver.submitPlacement(scenario.start);
    EXPECT_TRUE(rig.runUntil(
        [&] { return driver.placementResult(ticket) == PlacementResult::kApplied; }, 3.0));

    Navigator navigator(driver);
    scenario.command(navigator);
    DriveCommand demand;
    for (int tick = 0; tick * kTick < kLimit && !isTerminal(navigator.status().state); ++tick) {
        report(rig.pi, drive.pose());
        rig.step(kTick);
        if (tick % kControlTicks == 0) {
            demand = navigator.update(rig.now());
        }
        drive.step(investigatr::mixTank(demand), kTick);
    }
    EXPECT_FALSE(rig.bus.collision());
    EXPECT_EQ(rig.client().stats().session_losses, 0u);
    return outcome(navigator, drive);
}

Meters distance(const Pose& a, const Pose& b) { return std::hypot(a.x - b.x, a.y - b.y); }

void expectEquivalent(const Scenario& scenario, const Pose& goal) {
    const NavigatorConfig config;
    const Outcome         simulated = runSimulated(scenario);
    const Outcome         navigatr  = runNavigatr(scenario);
    ASSERT_EQ(simulated.state, MotionState::kCompleted) << investigatr::toString(simulated.reason);
    ASSERT_EQ(navigatr.state, MotionState::kCompleted) << investigatr::toString(navigatr.reason);

    for (const Outcome* o : {&simulated, &navigatr}) {
        EXPECT_LE(distance(o->pose, goal), config.position_tolerance);
        EXPECT_LE(std::fabs(wrapAngle(o->pose.heading - goal.heading)), config.heading_tolerance);
    }
    EXPECT_LE(distance(simulated.pose, navigatr.pose), config.position_tolerance);
    EXPECT_LE(std::fabs(wrapAngle(simulated.pose.heading - navigatr.pose.heading)),
              config.heading_tolerance);
}

} // namespace

TEST(Equivalence, AbsoluteGoal) {
    const Pose goal{1.0, 0.5, 1.0};
    Scenario   scenario;
    scenario.start   = Pose{0.0, 0.0, 0.0};
    scenario.command = [&](Navigator& n) { return n.goTo(goal); };
    expectEquivalent(scenario, goal);
}

TEST(Equivalence, LandmarkRelativeGoal) {
    Scenario scenario;
    scenario.start    = Pose{0.0, 0.0, 0.3};
    scenario.landmark = Pose{1.5, -0.5, kPi / 2.0};
    const Pose offset{-0.4, 0.0, 0.0};
    scenario.command = [&](Navigator& n) { return n.goToRelative(kLandmark, offset); };
    // compose(landmark, offset) by hand: 0.4 m behind the landmark, same heading.
    expectEquivalent(scenario, Pose{1.5, -0.9, kPi / 2.0});
}

TEST(Equivalence, WaypointPath) {
    const Pose        goal{0.3, 0.9, kPi};
    investigatr::Path path;
    path.push_back({investigatr::Destination::field(Pose{0.6, 0.0, 0.0}), false});
    path.push_back({investigatr::Destination::field(Pose{0.9, 0.6, 0.0}), false});
    path.push_back({investigatr::Destination::field(goal), true});
    Scenario scenario;
    scenario.start   = Pose{0.0, 0.0, 0.0};
    scenario.command = [&](Navigator& n) { return n.follow(path); };
    expectEquivalent(scenario, goal);
}
