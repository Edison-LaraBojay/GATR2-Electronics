// trajectory_gtest.cpp
// Planner, follower, drive and drivetrain sim together. The true rectangular
// footprint is checked against every obstacle at every simulated step,
// through turns and corners, for tank and mecanum drives, including a field
// correction that forces a replan.

#include <algorithm>
#include <cmath>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "actugatr/drive.h"
#include "actugatr/drivetrain.h"
#include "actugatr/motion.h"
#include "investigatr/planner.h"
#include "sim/drive_sim.h"
#include "sim/simulated_state.h"

using namespace actugatr;
using investigatr::EstimateSource;
using investigatr::Field;
using investigatr::FieldObject;
using investigatr::Footprint;
using investigatr::kPi;
using investigatr::ObjectKind;

namespace
{

constexpr Seconds       kDt     = 0.01;
constexpr uint32_t      kMapId  = 0x5EED0001;
const Footprint         kFootprint{0.23, 0.20, 0.21, 0.21};
constexpr double        kClearance = 0.08;

WheelDrive wheels() {
    WheelDrive w;
    w.wheel_diameter  = 0.1016;
    w.gear_ratio      = 0.6;
    w.usable_fraction = 0.9;
    return w;
}

FieldObject box(uint16_t id, const Pose& pose, double length, double width, bool landmark) {
    FieldObject o;
    o.id        = id;
    o.kind      = landmark ? ObjectKind::kLandmark : ObjectKind::kFixed;
    o.obstacle  = true;
    o.estimated = landmark;
    o.reference = landmark;
    o.nominal   = pose;
    o.box       = {Pose{}, length, width};
    o.source    = landmark ? EstimateSource::kObserved : EstimateSource::kNominal;
    o.valid     = true;
    o.pose      = pose;
    return o;
}

Field field(std::vector<FieldObject> objects) {
    std::sort(objects.begin(), objects.end(),
              [](const FieldObject& a, const FieldObject& b) { return a.id < b.id; });
    Field f;
    f.map.id  = kMapId;
    f.bounds  = {0, 0, 3.5664, 3.5664};
    f.objects = objects;
    f.frame   = 1;
    return f;
}

struct World {
    explicit World(bool holonomic)
        : kinematics(holonomic ? std::unique_ptr<Kinematics>(new MecanumKinematics(0.32, 0.30))
                               : std::unique_ptr<Kinematics>(new TankKinematics(0.32))),
          out(kinematics->groups()), drive(*kinematics, wheels(), out),
          sim(*kinematics, wheels(), out),
          follower(holonomic ? std::unique_ptr<Follower>(new HolonomicFollower())
                             : std::unique_ptr<Follower>(new DifferentialFollower())),
          motion(state, planner, *follower, config(holonomic)) {}

    static MotionConfig config(bool holonomic) {
        MotionConfig c;
        c.model.holonomic        = holonomic;
        c.model.footprint        = kFootprint;
        c.model.clearance        = kClearance;
        c.model.limits.max_speed = 0.8;
        c.model.limits.max_accel = 2.0;
        c.model.limits.max_omega = 3.0;
        c.model.limits.max_alpha = 8.0;
        c.default_timeout        = 30.0;
        return c;
    }

    // Runs to the end of the command; checks the true footprint every step.
    MotionState run(Seconds seconds, const std::function<void(World&)>& each = {}) {
        const Seconds end = t + seconds;
        while (t < end) {
            state.setRobot(sim.pose(), t);
            const ChassisCommand c = motion.update(t);
            drive.apply(c, t, t);
            sim.step(kDt);
            t += kDt;
            const investigatr::Clearance clear =
                investigatr::footprintClearance(sim.pose(), kFootprint, 0.0, current);
            worst = std::min(worst, clear.distance);
            if (!clear.clear) {
                ++collisions;
            }
            if (each) {
                each(*this);
            }
            if (isTerminal(motion.status().state)) {
                break;
            }
        }
        return motion.status().state;
    }

    void setField(const Field& f) {
        current = f;
        state.setField(f);
    }

    SimulatedState                 state;
    investigatr::GeometricPlanner  planner;
    std::unique_ptr<Kinematics>    kinematics;
    SimMotorOutput                 out;
    Drive                          drive;
    DriveSim                       sim;
    std::unique_ptr<Follower>      follower;
    Motion                         motion;
    Field                          current;
    Seconds                        t          = 0;
    double                         worst      = 1e9;
    int                            collisions = 0;
};

// A wall across the direct line with gaps above and below, plus a second
// box beyond it, so the route needs several corners.
std::vector<FieldObject> maze() {
    return {box(10, {1.5, 1.78, 0}, 0.2, 1.6, false), box(11, {2.4, 2.4, 0.3}, 0.5, 0.3, false),
            box(20, {3.0, 0.6, 0}, 0.1542, 0.1542, true)};
}

} // namespace

TEST(Trajectory, TankAroundObstaclesNeverTouchesThem) {
    World w(false);
    w.setField(field(maze()));
    w.sim.setPose(Pose{0.5, 1.78, 0});
    w.motion.goToAvoiding({3.0, 1.78, 0});
    ASSERT_EQ(w.run(30.0), MotionState::kCompleted) << toString(w.motion.status().reason);
    EXPECT_EQ(w.collisions, 0);
    EXPECT_GT(w.worst, 0.0);
    EXPECT_NEAR(w.sim.pose().x, 3.0, 0.03);
    EXPECT_NEAR(w.sim.pose().y, 1.78, 0.03);
    EXPECT_GT(w.motion.path().segments.size(), 3u);
}

TEST(Trajectory, MecanumAroundObstaclesNeverTouchesThem) {
    World w(true);
    w.setField(field(maze()));
    w.sim.setPose(Pose{0.5, 1.78, kPi / 2});
    w.motion.goToAvoiding({3.0, 1.78, 0});
    ASSERT_EQ(w.run(30.0), MotionState::kCompleted) << toString(w.motion.status().reason);
    EXPECT_EQ(w.collisions, 0);
    EXPECT_NEAR(w.sim.pose().x, 3.0, 0.03);
    EXPECT_NEAR(w.sim.pose().heading, 0.0, 0.05);
}

TEST(Trajectory, LandmarkRelativeAvoidingEndsBesideTheLandmark) {
    World w(false);
    w.setField(field(maze()));
    w.sim.setPose(Pose{0.5, 0.6, 0});
    w.motion.goToAvoiding({-0.5, 0, 0}, Reference::object(20, kMapId));
    ASSERT_EQ(w.run(30.0), MotionState::kCompleted) << toString(w.motion.status().reason);
    EXPECT_EQ(w.collisions, 0);
    EXPECT_NEAR(w.sim.pose().x, 2.5, 0.03);
    EXPECT_NEAR(w.sim.pose().y, 0.6, 0.03);
}

TEST(Trajectory, CorrectionOntoThePathReplansAndStaysClear) {
    for (bool holonomic : {false, true}) {
        World w(holonomic);
        w.setField(field({box(20, {3.0, 0.6, 0}, 0.1542, 0.1542, true)}));
        w.sim.setPose(Pose{0.5, 1.78, 0});
        w.motion.goToAvoiding({3.0, 1.78, 0});
        bool moved = false;
        EXPECT_EQ(w.run(30.0,
                        [&moved](World& world) {
                            if (!moved && world.sim.pose().x > 0.9) {
                                // A correction puts a box across the current path.
                                world.setField(field({box(20, {3.0, 0.6, 0}, 0.1542, 0.1542, true),
                                                      box(30, {1.9, 1.78, 0}, 0.3, 0.8, false)}));
                                moved = true;
                            }
                        }),
                  MotionState::kCompleted)
            << toString(w.motion.status().reason);
        EXPECT_TRUE(moved);
        EXPECT_GE(w.motion.status().plans, 2u);
        EXPECT_EQ(w.collisions, 0);
    }
}

TEST(Trajectory, BlockedAndUnreachableGoalsDoNotMove) {
    World w(false);
    w.setField(field(maze()));
    w.sim.setPose(Pose{0.5, 1.78, 0});
    w.motion.goToAvoiding({1.5, 1.78, 0}); // inside the wall
    EXPECT_EQ(w.run(2.0), MotionState::kFailed);
    EXPECT_EQ(w.motion.status().reason, MotionReason::kGoalBlocked);
    EXPECT_EQ(w.motion.status().blocking, 10);
    EXPECT_LT(std::hypot(w.sim.pose().x - 0.5, w.sim.pose().y - 1.78), 1e-6);

    // A closed ring around the goal.
    World r(false);
    r.setField(field({box(1, {2.5, 2.2, 0}, 1.4, 0.1, false), box(2, {2.5, 1.0, 0}, 1.4, 0.1, false),
                      box(3, {1.85, 1.6, 0}, 0.1, 1.3, false), box(4, {3.15, 1.6, 0}, 0.1, 1.3, false)}));
    r.sim.setPose(Pose{0.5, 0.5, 0});
    r.motion.goToAvoiding({2.5, 1.6, 0});
    EXPECT_EQ(r.run(2.0), MotionState::kFailed);
    EXPECT_EQ(r.motion.status().reason, MotionReason::kNoPath);
}

TEST(Trajectory, DirectModeIgnoresObstaclesByDesign) {
    World w(false);
    w.setField(field({box(20, {3.0, 0.6, 0}, 0.1542, 0.1542, true)}));
    w.sim.setPose(Pose{0.5, 0.6, 0});
    w.motion.goToDirect({2.5, 0.6, 0});
    ASSERT_EQ(w.run(15.0), MotionState::kCompleted);
    EXPECT_EQ(w.motion.path().mode, PlanMode::kDirect);
}
