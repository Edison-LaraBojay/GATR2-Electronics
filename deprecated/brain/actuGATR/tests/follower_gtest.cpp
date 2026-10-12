// follower_gtest.cpp
// Followers closed loop through Drive and the drivetrain sim: turns, cross
// track correction, reverse, strafing with a heading change, segment
// sequencing without corner cutting, and tracking failures.

#include "actugatr/follower.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "actugatr/drive.h"
#include "sim/drive_sim.h"

using namespace actugatr;
using investigatr::kPi;
using investigatr::PathSegment;
using investigatr::SegmentKind;

namespace
{

constexpr Seconds kDt = 0.01;

MotionLimits limits() {
    MotionLimits l;
    l.max_speed = 0.8;
    l.max_accel = 2.0;
    l.max_omega = 3.0;
    l.max_alpha = 8.0;
    return l;
}

WheelDrive wheels() {
    WheelDrive w;
    w.wheel_diameter  = 0.1016;
    w.gear_ratio      = 0.6;
    w.cartridge       = Cartridge::kBlue;
    w.usable_fraction = 0.9;
    return w;
}

PathSegment translate(const Pose& a, const Pose& b, bool reverse = false) {
    PathSegment s;
    s.kind    = SegmentKind::kTranslate;
    s.start   = a;
    s.end     = b;
    s.reverse = reverse;
    return s;
}

PathSegment turn(const Pose& at, double heading) {
    PathSegment s;
    s.kind        = SegmentKind::kTurn;
    s.start       = at;
    s.end         = at;
    s.end.heading = heading;
    s.turn_direction =
        investigatr::wrapAngle(heading - at.heading) >= 0 ? 1 : -1;
    return s;
}

struct Rig {
    explicit Rig(bool holonomic)
        : kinematics(holonomic ? std::unique_ptr<Kinematics>(new MecanumKinematics(0.3, 0.3))
                               : std::unique_ptr<Kinematics>(new TankKinematics(0.3))),
          out(kinematics->groups()), drive(*kinematics, wheels(), out),
          sim(*kinematics, wheels(), out) {
        if (holonomic) {
            follower.reset(new HolonomicFollower());
        } else {
            follower.reset(new DifferentialFollower());
        }
    }

    // Steps until the follower ends or seconds pass. Every pose is recorded.
    FollowOutput run(Seconds seconds, const std::function<void(Rig&)>& each = {}) {
        FollowOutput last;
        for (Seconds t = 0; t < seconds; t += kDt) {
            last = follower->update(sim.pose(), kDt);
            drive.apply(last.command, t, t);
            sim.step(kDt);
            poses.push_back(sim.pose());
            if (each) {
                each(*this);
            }
            if (last.state == FollowState::kDone || last.state == FollowState::kFailed) {
                break;
            }
        }
        return last;
    }

    std::unique_ptr<Kinematics> kinematics;
    SimMotorOutput              out;
    Drive                       drive;
    DriveSim                    sim;
    std::unique_ptr<Follower>   follower;
    std::vector<Pose>           poses;
};

double distanceToSegment(const Pose& p, const Pose& a, const Pose& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double l2 = dx * dx + dy * dy;
    double       t  = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0;
    t               = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

double distanceToPolyline(const Pose& p, const std::vector<Pose>& corners) {
    double best = 1e9;
    for (std::size_t i = 0; i + 1 < corners.size(); ++i) {
        best = std::min(best, distanceToSegment(p, corners[i], corners[i + 1]));
    }
    return best;
}

} // namespace

TEST(Follower, ConfigValidation) {
    FollowerConfig c;
    EXPECT_TRUE(valid(c));
    c.tracking_tolerance = c.position_tolerance;
    EXPECT_FALSE(valid(c));
    c                  = FollowerConfig{};
    c.along.output_limit = 0;
    EXPECT_FALSE(valid(c));
    c                   = FollowerConfig{};
    c.max_heading_error = 4.0;
    EXPECT_FALSE(valid(c));
}

TEST(Follower, EmptyPathIsDone) {
    Rig r(false);
    r.follower->start(investigatr::Path{}, limits());
    EXPECT_EQ(r.follower->update(Pose{}, kDt).state, FollowState::kDone);
}

TEST(DifferentialFollower, TurnInPlaceSettlesOnTheHeading) {
    Rig               r(false);
    investigatr::Path p;
    p.segments.push_back(turn(Pose{}, kPi / 2));
    r.follower->start(p, limits());
    const FollowOutput out = r.run(5.0);
    ASSERT_EQ(out.state, FollowState::kDone);
    EXPECT_NEAR(r.sim.pose().heading, kPi / 2, 0.035);
    EXPECT_LT(std::hypot(r.sim.pose().x, r.sim.pose().y), 0.005);
}

TEST(DifferentialFollower, TranslationCorrectsCrossTrack) {
    Rig r(false);
    r.sim.setPose(Pose{0, 0.02, 0});
    investigatr::Path p;
    p.segments.push_back(translate(Pose{0, 0, 0}, Pose{1.0, 0, 0}));
    r.follower->start(p, limits());
    const FollowOutput out = r.run(5.0);
    ASSERT_EQ(out.state, FollowState::kDone);
    EXPECT_NEAR(r.sim.pose().x, 1.0, 0.02);
    EXPECT_LT(std::fabs(r.sim.pose().y), 0.01);
    for (const Pose& q : r.poses) {
        EXPECT_LE(std::fabs(q.y), 0.021);
    }
}

TEST(DifferentialFollower, ReverseSegmentBacksUpKeepingHeading) {
    Rig               r(false);
    investigatr::Path p;
    p.segments.push_back(translate(Pose{0, 0, 0}, Pose{-0.6, 0, 0}, true));
    r.follower->start(p, limits());
    const FollowOutput out = r.run(5.0);
    ASSERT_EQ(out.state, FollowState::kDone);
    EXPECT_NEAR(r.sim.pose().x, -0.6, 0.02);
    EXPECT_NEAR(r.sim.pose().heading, 0.0, 0.03);
}

TEST(DifferentialFollower, SequencedSegmentsNeverCutTheCorner) {
    Rig                     r(false);
    const Pose              a{0, 0, 0};
    const Pose              b{1.0, 0, 0};
    const Pose              c{1.0, 1.0, kPi / 2};
    investigatr::Path       p;
    p.segments.push_back(translate(a, b));
    p.segments.push_back(turn(b, kPi / 2));
    p.segments.push_back(translate(Pose{1.0, 0, kPi / 2}, c));
    p.segments.push_back(turn(c, kPi));
    r.follower->start(p, limits());
    const FollowOutput out = r.run(15.0);
    ASSERT_EQ(out.state, FollowState::kDone);
    EXPECT_NEAR(r.sim.pose().x, 1.0, 0.03);
    EXPECT_NEAR(r.sim.pose().y, 1.0, 0.03);
    EXPECT_NEAR(r.sim.pose().heading, kPi, 0.035);

    const std::vector<Pose> corners = {a, b, c};
    double                  closest = 1e9;
    for (const Pose& q : r.poses) {
        EXPECT_LE(distanceToPolyline(q, corners), 0.04);
        closest = std::min(closest, std::hypot(q.x - b.x, q.y - b.y));
    }
    // The executed motion reaches the corner instead of shortcutting inside it.
    EXPECT_LT(closest, 0.025);
}

TEST(DifferentialFollower, PushedOffThePathFails) {
    Rig               r(false);
    investigatr::Path p;
    p.segments.push_back(translate(Pose{0, 0, 0}, Pose{2.0, 0, 0}));
    r.follower->start(p, limits());
    bool               pushed = false;
    const FollowOutput out    = r.run(5.0, [&pushed](Rig& rig) {
        if (!pushed && rig.sim.pose().x > 0.5) {
            Pose q = rig.sim.pose();
            q.y += 0.1;
            rig.sim.setPose(q);
            pushed = true;
        }
    });
    ASSERT_TRUE(pushed);
    EXPECT_EQ(out.state, FollowState::kFailed);
    EXPECT_EQ(out.fault, FollowFault::kTrackingError);
    EXPECT_TRUE(isZero(r.follower->update(r.sim.pose(), kDt).command));
}

TEST(DifferentialFollower, TurnAcceptsWhatAFinishedTranslationLeaves) {
    DifferentialFollower f;
    investigatr::Path    p;
    p.segments.push_back(translate(Pose{0, 0, 0}, Pose{1.0, 0, 0}));
    p.segments.push_back(turn(Pose{1.0, 0, 0}, kPi / 2));
    f.start(p, limits());

    // 1.9 cm short and 3.9 cm left: inside both translation tolerances.
    FollowOutput out = f.update(Pose{0.981, 0.039, 0}, kDt);
    EXPECT_EQ(out.state, FollowState::kRunning);
    EXPECT_EQ(out.segment, 1u);

    out = f.update(Pose{1.0, 0.05, 0.2}, kDt);
    EXPECT_EQ(out.state, FollowState::kFailed);
    EXPECT_EQ(out.fault, FollowFault::kTrackingError);
}

TEST(HolonomicFollower, StrafesWhileTurningToTheGoalHeading) {
    Rig               r(true);
    investigatr::Path p;
    p.segments.push_back(translate(Pose{0, 0, 0}, Pose{0, 1.0, kPi / 2}));
    r.follower->start(p, limits());
    const FollowOutput out = r.run(8.0);
    ASSERT_EQ(out.state, FollowState::kDone);
    EXPECT_NEAR(r.sim.pose().x, 0.0, 0.02);
    EXPECT_NEAR(r.sim.pose().y, 1.0, 0.02);
    EXPECT_NEAR(r.sim.pose().heading, kPi / 2, 0.035);
    for (const Pose& q : r.poses) {
        EXPECT_LE(std::fabs(q.x), 0.04);
    }
}

TEST(HolonomicFollower, CornerIsReachedBeforeTheNextSegment) {
    Rig                     r(true);
    const Pose              a{0, 0, 0};
    const Pose              b{0.8, 0, 0};
    const Pose              c{0.8, 0.8, 0};
    investigatr::Path       p;
    p.segments.push_back(translate(a, b));
    p.segments.push_back(translate(b, c));
    r.follower->start(p, limits());
    const FollowOutput out = r.run(10.0);
    ASSERT_EQ(out.state, FollowState::kDone);
    double closest = 1e9;
    for (const Pose& q : r.poses) {
        EXPECT_LE(distanceToPolyline(q, {a, b, c}), 0.04);
        closest = std::min(closest, std::hypot(q.x - b.x, q.y - b.y));
    }
    EXPECT_LT(closest, 0.025);
}
