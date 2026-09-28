// motion_gtest.cpp
// Motion: references applied once, direct vs avoiding, waiting and failure
// reasons, cancellation, identity, deadlines through replans, field
// corrections and path reports. Uses the host test planner; the real planner
// runs in the integration tests.

#include "actugatr/motion.h"

#include <cmath>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "actugatr/drive.h"
#include "sim/drive_sim.h"
#include "sim/simulated_state.h"
#include "sim/straight_planner.h"

using namespace actugatr;
using investigatr::EstimateSource;
using investigatr::Field;
using investigatr::FieldObject;
using investigatr::kPi;
using investigatr::ObjectKind;
using investigatr::PlanStatus;
using investigatr::RobotStatus;

namespace
{

constexpr Seconds kDt     = 0.01;
constexpr uint32_t kMapId = 0xC0FFEE01;

WheelDrive wheels() {
    WheelDrive w;
    w.wheel_diameter  = 0.1016;
    w.gear_ratio      = 0.6;
    w.usable_fraction = 0.9;
    return w;
}

MotionConfig config(bool holonomic) {
    MotionConfig c;
    c.model.holonomic       = holonomic;
    c.model.footprint       = {0.2, 0.2, 0.2, 0.2};
    c.model.clearance       = 0.05;
    c.model.limits.max_speed = 0.8;
    c.model.limits.max_accel = 2.0;
    c.model.limits.max_omega = 3.0;
    c.model.limits.max_alpha = 8.0;
    return c;
}

FieldObject landmark(uint16_t id, const Pose& pose, EstimateSource source) {
    FieldObject o;
    o.id        = id;
    o.kind      = ObjectKind::kLandmark;
    o.obstacle  = true;
    o.estimated = true;
    o.reference = true;
    o.nominal   = pose;
    o.box       = {Pose{}, 0.15, 0.15};
    o.source    = source;
    o.valid     = source != EstimateSource::kNone;
    o.pose      = pose;
    return o;
}

Field field(const std::vector<FieldObject>& objects) {
    Field f;
    f.map.id   = kMapId;
    f.bounds   = {0, 0, 3.6, 3.6};
    f.objects  = objects;
    f.frame    = 1;
    return f;
}

class RecordingSink : public investigatr::PathSink {
public:
    void reportPath(CommandId command, const investigatr::Path& path) override {
        reports.push_back({command, path.segments.size()});
    }
    struct Report {
        CommandId   command;
        std::size_t segments;
    };
    std::vector<Report> reports;
};

struct Loop {
    explicit Loop(bool holonomic = false)
        : kinematics(holonomic ? std::unique_ptr<Kinematics>(new MecanumKinematics(0.3, 0.3))
                               : std::unique_ptr<Kinematics>(new TankKinematics(0.3))),
          out(kinematics->groups()), drive(*kinematics, wheels(), out),
          sim(*kinematics, wheels(), out),
          follower(holonomic ? std::unique_ptr<Follower>(new HolonomicFollower())
                             : std::unique_ptr<Follower>(new DifferentialFollower())),
          motion(state, planner, *follower, config(holonomic)) {
        motion.setPathSink(&sink);
        sim.setPose(Pose{1.0, 1.0, 0});
    }

    void step() {
        state.setRobot(sim.pose(), t);
        const ChassisCommand c = motion.update(t);
        last                   = c;
        drive.apply(c, t, t);
        sim.step(kDt);
        t += kDt;
    }

    // Steps until the command ends or seconds pass.
    MotionState run(Seconds seconds, const std::function<void(Loop&)>& each = {}) {
        const Seconds end = t + seconds;
        while (t < end) {
            step();
            if (each) {
                each(*this);
            }
            if (isTerminal(motion.status().state)) {
                break;
            }
        }
        return motion.status().state;
    }

    SimulatedState              state;
    StraightPlanner             planner;
    std::unique_ptr<Kinematics> kinematics;
    SimMotorOutput              out;
    Drive                       drive;
    DriveSim                    sim;
    std::unique_ptr<Follower>   follower;
    Motion                      motion;
    RecordingSink               sink;
    ChassisCommand              last;
    Seconds                     t = 0;
};

} // namespace

TEST(Motion, DirectToFieldOriginByDefault) {
    Loop            l;
    const CommandId id = l.motion.goToDirect({1.6, 1.3, kPi / 2});
    EXPECT_EQ(l.motion.status().command_id, id);
    EXPECT_EQ(l.motion.status().state, MotionState::kWaiting);
    ASSERT_EQ(l.run(15.0), MotionState::kCompleted) << toString(l.motion.status().reason);
    EXPECT_NEAR(l.sim.pose().x, 1.6, 0.03);
    EXPECT_NEAR(l.sim.pose().y, 1.3, 0.03);
    EXPECT_NEAR(l.sim.pose().heading, kPi / 2, 0.05);
    EXPECT_EQ(l.planner.lastRequest().mode, PlanMode::kDirect);
    EXPECT_EQ(l.planner.lastRequest().field, nullptr);
    EXPECT_TRUE(isZero(l.last));
}

TEST(Motion, DirectNeedsNoFieldOrLandmarks) {
    Loop l;
    l.motion.goToDirect({1.4, 1.0, 0});
    EXPECT_EQ(l.run(10.0), MotionState::kCompleted);
    EXPECT_EQ(l.state.fieldCopies(), 0);
}

TEST(Motion, MecanumStrafesToTheGoal) {
    Loop l(true);
    l.motion.goToDirect({1.0, 1.6, 0});
    ASSERT_EQ(l.run(10.0), MotionState::kCompleted) << toString(l.motion.status().reason);
    EXPECT_NEAR(l.sim.pose().y, 1.6, 0.03);
    EXPECT_NEAR(l.sim.pose().heading, 0.0, 0.05);
    EXPECT_TRUE(l.planner.lastRequest().model.holonomic);
}

TEST(Motion, ObjectReferenceAppliesTranslationAndRotationOnce) {
    Loop l;
    l.state.setField(field({landmark(5, {2.0, 1.0, kPi / 2}, EstimateSource::kObserved)}));
    l.motion.goToDirect({-0.4, 0.1, kPi / 2}, Reference::object(5, kMapId));
    l.step();
    ASSERT_TRUE(l.motion.status().has_destination) << toString(l.motion.status().reason);
    // Landmark at (2, 1) facing +y: its -x is field -y, its +y is field -x.
    const Pose d = l.motion.status().destination;
    EXPECT_NEAR(d.x, 2.0 - 0.1, 1e-9);
    EXPECT_NEAR(d.y, 1.0 - 0.4, 1e-9);
    EXPECT_NEAR(d.heading, investigatr::wrapAngle(kPi), 1e-9);
    EXPECT_EQ(l.motion.status().reference_source, EstimateSource::kObserved);
}

TEST(Motion, RobotAtStartReference) {
    Loop l;
    l.sim.setPose(Pose{1.0, 1.0, kPi / 2});
    l.motion.goToDirect({0.5, 0, 0}, Reference::robotAtStart());
    l.step();
    const Pose d = l.motion.status().destination;
    EXPECT_NEAR(d.x, 1.0, 1e-9);
    EXPECT_NEAR(d.y, 1.5, 1e-9);
    EXPECT_NEAR(d.heading, kPi / 2, 1e-9);
}

TEST(Motion, ReferenceFailuresAreExplicit) {
    {
        Loop l;
        l.state.setField(field({landmark(5, {2, 1, 0}, EstimateSource::kObserved)}));
        l.motion.goToDirect({}, Reference::object(5, kMapId + 1));
        EXPECT_EQ(l.run(1.0), MotionState::kFailed);
        EXPECT_EQ(l.motion.status().reason, MotionReason::kMapMismatch);
    }
    {
        Loop l;
        l.state.setField(field({landmark(5, {2, 1, 0}, EstimateSource::kObserved)}));
        l.motion.goToDirect({}, Reference::object(6, kMapId));
        EXPECT_EQ(l.run(1.0), MotionState::kFailed);
        EXPECT_EQ(l.motion.status().reason, MotionReason::kUnknownReference);
    }
    {
        Loop        l;
        FieldObject fixed = landmark(9, {0.1, 0.1, 0}, EstimateSource::kNominal);
        fixed.reference   = false;
        l.state.setField(field({fixed}));
        l.motion.goToDirect({}, Reference::object(9, kMapId));
        EXPECT_EQ(l.run(1.0), MotionState::kFailed);
        EXPECT_EQ(l.motion.status().reason, MotionReason::kNotReference);
    }
}

TEST(Motion, NominalReferenceWaitsForAnObservationUnlessAllowed) {
    Loop l;
    l.state.setField(field({landmark(5, {2.0, 1.0, 0}, EstimateSource::kNominal)}));
    l.motion.goToDirect({-0.5, 0, 0}, Reference::object(5, kMapId));
    for (int i = 0; i < 50; ++i) {
        l.step();
    }
    EXPECT_EQ(l.motion.status().state, MotionState::kWaiting);
    EXPECT_TRUE(isZero(l.last));
    // An observation arrives inside the wait: the command proceeds.
    l.state.setField(field({landmark(5, {2.02, 1.0, 0}, EstimateSource::kObserved)}));
    EXPECT_EQ(l.run(10.0), MotionState::kCompleted);
    EXPECT_NEAR(l.sim.pose().x, 1.52, 0.03);

    Loop never;
    never.state.setField(field({landmark(5, {2.0, 1.0, 0}, EstimateSource::kNominal)}));
    never.motion.goToDirect({-0.5, 0, 0}, Reference::object(5, kMapId));
    EXPECT_EQ(never.run(3.0), MotionState::kFailed);
    EXPECT_EQ(never.motion.status().reason, MotionReason::kReferenceUnavailable);

    Loop allowed;
    allowed.state.setField(field({landmark(5, {2.0, 1.0, 0}, EstimateSource::kNominal)}));
    MoveOptions options;
    options.require_observed_reference = false;
    allowed.motion.goToDirect({-0.5, 0, 0}, Reference::object(5, kMapId), options);
    EXPECT_EQ(allowed.run(10.0), MotionState::kCompleted);
}

TEST(Motion, AvoidingNeedsAFieldAndNeverFallsBackToDirect) {
    Loop l;
    l.motion.goToAvoiding({1.5, 1.0, 0});
    EXPECT_EQ(l.run(3.0), MotionState::kFailed);
    EXPECT_EQ(l.motion.status().reason, MotionReason::kFieldUnavailable);
    EXPECT_EQ(l.planner.plans(), 0);
    EXPECT_LT(std::hypot(l.sim.pose().x - 1.0, l.sim.pose().y - 1.0), 1e-6);

    Loop ok;
    ok.state.setField(field({}));
    ok.motion.goToAvoiding({1.5, 1.0, 0});
    EXPECT_EQ(ok.run(10.0), MotionState::kCompleted);
    EXPECT_EQ(ok.planner.lastRequest().mode, PlanMode::kAvoiding);
    EXPECT_NE(ok.planner.lastRequest().field, nullptr);
}

TEST(Motion, PlannerFailuresEndTheCommandWithTheirReason) {
    Loop l;
    l.state.setField(field({}));
    l.planner.force(PlanStatus::kGoalBlocked, 7);
    l.motion.goToAvoiding({1.5, 1.0, 0});
    EXPECT_EQ(l.run(1.0), MotionState::kFailed);
    EXPECT_EQ(l.motion.status().reason, MotionReason::kGoalBlocked);
    EXPECT_EQ(l.motion.status().blocking, 7);
    EXPECT_EQ(l.planner.plans(), 1);

    const PlanStatus statuses[] = {PlanStatus::kNoPath, PlanStatus::kStartBlocked,
                                   PlanStatus::kStartOutOfBounds, PlanStatus::kUnsupportedModel};
    const MotionReason reasons[] = {MotionReason::kNoPath, MotionReason::kStartBlocked,
                                    MotionReason::kStartOutOfBounds,
                                    MotionReason::kUnsupportedModel};
    for (int i = 0; i < 4; ++i) {
        Loop f;
        f.state.setField(field({}));
        f.planner.force(statuses[i]);
        f.motion.goToAvoiding({1.5, 1.0, 0});
        EXPECT_EQ(f.run(1.0), MotionState::kFailed);
        EXPECT_EQ(f.motion.status().reason, reasons[i]);
    }
}

TEST(Motion, UnusableStateWaitsThenNamesTheReason) {
    const RobotStatus  statuses[] = {RobotStatus::kUnplaced, RobotStatus::kCalibrating,
                                     RobotStatus::kNoProfile, RobotStatus::kNoPose};
    const MotionReason reasons[]  = {MotionReason::kPlacementRequired, MotionReason::kCalibrating,
                                     MotionReason::kNoProfile, MotionReason::kInputUnavailable};
    for (int i = 0; i < 4; ++i) {
        Loop l;
        l.state.setStatus(statuses[i]);
        l.motion.goToDirect({1.5, 1.0, 0});
        for (int k = 0; k < 20; ++k) {
            l.step();
        }
        EXPECT_EQ(l.motion.status().state, MotionState::kWaiting);
        EXPECT_EQ(l.run(3.0), MotionState::kFailed);
        EXPECT_EQ(l.motion.status().reason, reasons[i]);
    }
}

TEST(Motion, LostLinkStopsAtOnceAndStaleStateAfterTheLossTimeout) {
    Loop l;
    l.motion.goToDirect({2.5, 1.0, 0});
    for (int k = 0; k < 60; ++k) {
        l.step();
    }
    ASSERT_EQ(l.motion.status().state, MotionState::kRunning);
    l.state.setConnected(false);
    l.step();
    EXPECT_EQ(l.motion.status().state, MotionState::kFailed);
    EXPECT_EQ(l.motion.status().reason, MotionReason::kInputLost);
    EXPECT_TRUE(isZero(l.last));

    Loop s;
    s.motion.goToDirect({2.5, 1.0, 0});
    for (int k = 0; k < 60; ++k) {
        s.step();
    }
    s.state.setLatency(1.0); // no sample new enough: pose goes stale
    for (int k = 0; k < 10; ++k) {
        s.step();
    }
    EXPECT_EQ(s.motion.status().state, MotionState::kWaiting);
    EXPECT_TRUE(isZero(s.last));
    EXPECT_EQ(s.run(1.0), MotionState::kFailed);
    EXPECT_EQ(s.motion.status().reason, MotionReason::kInputLost);
}

TEST(Motion, FrameChangeEndsTheCommand) {
    Loop l;
    l.motion.goToDirect({2.5, 1.0, 0});
    for (int k = 0; k < 20; ++k) {
        l.step();
    }
    l.state.setFrame(2);
    l.step();
    EXPECT_EQ(l.motion.status().state, MotionState::kFailed);
    EXPECT_EQ(l.motion.status().reason, MotionReason::kFrameChanged);
}

TEST(Motion, CancelAndReplaceKeepIdentityAndNeverResume) {
    Loop            l;
    const CommandId first = l.motion.goToDirect({2.5, 1.0, 0});
    for (int k = 0; k < 30; ++k) {
        l.step();
    }
    l.motion.cancel();
    EXPECT_EQ(l.motion.status().state, MotionState::kCancelled);
    EXPECT_EQ(l.motion.status().reason, MotionReason::kCancelledByCaller);
    l.step();
    EXPECT_TRUE(isZero(l.last));
    EXPECT_EQ(l.motion.status().command_id, first);

    const CommandId second = l.motion.goToDirect({1.5, 1.0, 0});
    EXPECT_EQ(second, first + 1);
    const CommandId third = l.motion.goToDirect({1.5, 1.2, 0});
    EXPECT_EQ(third, second + 1);
    EXPECT_EQ(l.motion.status().command_id, third);
}

TEST(Motion, InvalidCommandsFailWithAnId) {
    Loop        l;
    MoveOptions bad;
    bad.speed_scale      = 0;
    const CommandId id   = l.motion.goToDirect({1, 1, 0}, Reference::origin(), bad);
    EXPECT_EQ(l.motion.status().command_id, id);
    EXPECT_EQ(l.motion.status().state, MotionState::kFailed);
    EXPECT_EQ(l.motion.status().reason, MotionReason::kInvalidCommand);
    l.motion.goToDirect({std::nan(""), 1, 0});
    EXPECT_EQ(l.motion.status().reason, MotionReason::kInvalidCommand);
}

TEST(Motion, MismatchedFollowerIsAnInvalidConfig) {
    SimulatedState   state;
    StraightPlanner  planner;
    HolonomicFollower follower;
    Motion           motion(state, planner, follower, config(false));
    motion.goToDirect({1, 1, 0});
    EXPECT_EQ(motion.status().reason, MotionReason::kInvalidConfig);
}

TEST(Motion, TimeoutCountsFromTheFirstUpdateThroughReplans) {
    Loop l;
    l.state.setField(field({landmark(5, {3.0, 1.0, 0}, EstimateSource::kObserved)}));
    MoveOptions options;
    options.timeout = 1.0;
    l.motion.goToDirect({-0.5, 0, 0}, Reference::object(5, kMapId), options);
    int moves = 0;
    EXPECT_EQ(l.run(5.0, [&moves](Loop& loop) {
                  if (loop.t > 0.3 * (moves + 1) && moves < 2) {
                      ++moves;
                      loop.state.setField(field({landmark(
                          5, {3.0, 1.0 + 0.1 * moves, 0}, EstimateSource::kObserved)}));
                  }
              }),
              MotionState::kFailed);
    EXPECT_EQ(l.motion.status().reason, MotionReason::kTimedOut);
    EXPECT_NEAR(l.motion.status().elapsed, 1.0, 0.02);
    EXPECT_GE(l.motion.status().plans, 3u);
}

TEST(Motion, CorrectionsBeyondTheThresholdReplanAndTinyOnesDoNot) {
    Loop l;
    l.state.setField(field({landmark(5, {2.5, 1.0, 0}, EstimateSource::kObserved)}));
    const CommandId id = l.motion.goToDirect({-0.5, 0, 0}, Reference::object(5, kMapId));
    for (int k = 0; k < 30; ++k) {
        l.step();
    }
    ASSERT_EQ(l.motion.status().plans, 1u);
    l.state.setField(field({landmark(5, {2.51, 1.0, 0}, EstimateSource::kObserved)}));
    l.step();
    EXPECT_EQ(l.motion.status().plans, 1u);
    l.state.setField(field({landmark(5, {2.6, 1.0, 0}, EstimateSource::kObserved)}));
    l.step();
    // Brakes to a steady pose first, then replans from there.
    EXPECT_EQ(l.motion.status().plans, 1u);
    EXPECT_TRUE(isZero(l.last));
    for (int k = 0; k < 150 && l.motion.status().plans < 2; ++k) {
        l.step();
    }
    EXPECT_EQ(l.motion.status().plans, 2u);
    EXPECT_EQ(l.motion.status().command_id, id);
    EXPECT_NEAR(l.motion.status().destination.x, 2.1, 1e-9);
    ASSERT_EQ(l.run(15.0), MotionState::kCompleted);
    EXPECT_NEAR(l.sim.pose().x, 2.1, 0.03);
}

TEST(Motion, AvoidingRevalidatesAndReplansWhenThePathIsBlocked) {
    Loop l;
    l.state.setField(field({}));
    l.motion.goToAvoiding({2.5, 1.0, 0});
    for (int k = 0; k < 30; ++k) {
        l.step();
    }
    const int clears = l.planner.clears();
    l.state.setField(field({landmark(7, {3.3, 3.3, 0}, EstimateSource::kObserved)}));
    l.step();
    EXPECT_EQ(l.planner.clears(), clears + 1);
    EXPECT_EQ(l.motion.status().plans, 1u);

    l.planner.setClear(false);
    l.state.setField(field({landmark(7, {3.3, 3.2, 0}, EstimateSource::kObserved)}));
    l.step();
    EXPECT_TRUE(isZero(l.last));
    for (int k = 0; k < 150 && l.motion.status().plans < 2; ++k) {
        l.step();
    }
    EXPECT_EQ(l.motion.status().plans, 2u);
    // The replan starts where the robot stopped.
    EXPECT_NEAR(l.planner.lastRequest().start.x, l.sim.pose().x, 0.01);
}

TEST(Motion, PathSinkSeesEveryPlanAndTheClear) {
    Loop l;
    const CommandId id = l.motion.goToDirect({1.8, 1.0, kPi / 2});
    ASSERT_EQ(l.run(15.0), MotionState::kCompleted);
    ASSERT_GE(l.sink.reports.size(), 2u);
    EXPECT_EQ(l.sink.reports.front().command, id);
    EXPECT_GT(l.sink.reports.front().segments, 0u);
    EXPECT_EQ(l.sink.reports.back().command, id);
    EXPECT_EQ(l.sink.reports.back().segments, 0u);
}

TEST(Motion, ReplacedCommandClearsItsPath) {
    Loop            l;
    const CommandId first = l.motion.goToDirect({2.5, 1.0, 0});
    for (int k = 0; k < 30; ++k) {
        l.step();
    }
    ASSERT_FALSE(l.sink.reports.empty());
    ASSERT_GT(l.sink.reports.back().segments, 0u);

    // The replacement fails before planning; the old path is still cleared.
    l.motion.goToAvoiding({1.5, 1.0, 0});
    l.run(3.0);
    ASSERT_EQ(l.motion.status().state, MotionState::kFailed);
    EXPECT_EQ(l.sink.reports.back().command, first);
    EXPECT_EQ(l.sink.reports.back().segments, 0u);
}

TEST(Motion, NextIdContinuesFromTheOwnersId) {
    Loop l;
    l.motion.setNextId(41);
    EXPECT_EQ(l.motion.goToDirect({1.5, 1.0, 0}), 41u);
    EXPECT_EQ(l.motion.goToDirect({1.5, 1.0, 0}), 42u);
    l.motion.setNextId(0);
    EXPECT_EQ(l.motion.goToDirect({1.5, 1.0, 0}), 43u);
    l.motion.setNextId(1);
    EXPECT_EQ(l.motion.goToDirect({1.5, 1.0, 0}), 1u);
}

TEST(Motion, SourceChangeCancels) {
    Loop           l;
    SimulatedState other;
    l.motion.goToDirect({2.5, 1.0, 0});
    l.step();
    l.motion.setSource(other);
    EXPECT_EQ(l.motion.status().state, MotionState::kCancelled);
    EXPECT_EQ(l.motion.status().reason, MotionReason::kSourceChanged);
}

TEST(Motion, SpeedScaleLimitsThePlanModel) {
    Loop        l;
    MoveOptions options;
    options.speed_scale = 0.5;
    l.motion.goToDirect({2.0, 1.0, 0}, Reference::origin(), options);
    l.step();
    EXPECT_DOUBLE_EQ(l.planner.lastRequest().model.limits.max_speed, 0.4);
    double peak = 0;
    l.run(10.0, [&peak](Loop& loop) { peak = std::max(peak, std::fabs(loop.last.vx)); });
    EXPECT_LE(peak, 0.4 + 1e-9);
}
