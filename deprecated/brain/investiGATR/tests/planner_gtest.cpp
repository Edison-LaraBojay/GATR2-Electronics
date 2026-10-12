// planner_gtest.cpp

#include "investigatr/planner.h"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <set>
#include <string>
#include <vector>

using namespace investigatr;

namespace
{

MotionModel tank(Meters half = 0.15, Meters clearance = 0.02) {
    MotionModel m;
    m.holonomic     = false;
    m.turn_in_place = true;
    m.reverse       = true;
    m.footprint     = Footprint{half, half, half, half};
    m.clearance     = clearance;
    return m;
}

MotionModel mecanum(Meters half = 0.15, Meters clearance = 0.02) {
    MotionModel m = tank(half, clearance);
    m.holonomic   = true;
    return m;
}

FieldObject obstacle(ObjectId id, const Pose& pose, Meters length, Meters width,
                     const Pose& offset = {}) {
    FieldObject o;
    o.id       = id;
    o.kind     = ObjectKind::kFixed;
    o.obstacle = true;
    o.nominal  = pose;
    o.pose     = pose;
    o.valid    = true;
    o.source   = EstimateSource::kNominal;
    o.box      = Box{offset, length, width};
    return o;
}

FieldObject landmark(ObjectId id, const Pose& pose, Meters size, const Pose& offset = {}) {
    FieldObject o = obstacle(id, pose, size, size, offset);
    o.kind        = ObjectKind::kLandmark;
    o.estimated   = true;
    o.reference   = true;
    return o;
}

Field field(std::vector<FieldObject> objects, const Bounds& bounds = Bounds{0, 0, 4, 4}) {
    Field f;
    f.generation = 1;
    f.map.id     = 0xABCD1234;
    f.bounds     = bounds;
    f.objects    = std::move(objects);
    return f;
}

PlanRequest request(PlanMode mode, const Pose& start, const Pose& goal, const MotionModel& model,
                    const Field* f = nullptr) {
    PlanRequest r;
    r.mode  = mode;
    r.start = start;
    r.goal  = goal;
    r.model = model;
    r.field = f;
    return r;
}

Radians totalTurn(const Path& path) {
    Radians total = 0;
    for (const PathSegment& s : path.segments) {
        if (s.kind == SegmentKind::kTurn) {
            total += std::fabs(wrapAngle(s.end.heading - s.start.heading));
        }
    }
    return total;
}

std::size_t count(const Path& path, SegmentKind kind) {
    return static_cast<std::size_t>(
        std::count_if(path.segments.begin(), path.segments.end(),
                      [kind](const PathSegment& s) { return s.kind == kind; }));
}

// Distance from p to the real box of o, 0 inside.
double boxDistance(Point p, const FieldObject& o) {
    const Pose   center = compose(o.valid ? o.pose : o.nominal, o.box.center);
    const Pose   local  = between(center, Pose{p.x, p.y, 0});
    const double dx     = std::max(std::fabs(local.x) - 0.5 * o.box.length, 0.0);
    const double dy     = std::max(std::fabs(local.y) - 0.5 * o.box.width, 0.0);
    return std::hypot(dx, dy);
}

// Chained, executable by the model, starts at start, ends at goal. An exact
// segment is entered at its own heading.
void expectExecutable(const Path& path, const PlanRequest& r,
                      const GeometricPlannerConfig& config = {}) {
    ASSERT_FALSE(path.empty());
    const Radians tol = config.min_turn + 1e-9;
    Pose          at  = r.start;
    for (std::size_t i = 0; i < path.segments.size(); ++i) {
        const PathSegment& s = path.segments[i];
        SCOPED_TRACE(i);
        ASSERT_TRUE(finite(s.start) && finite(s.end));
        EXPECT_NEAR(s.start.x, at.x, 1e-12);
        EXPECT_NEAR(s.start.y, at.y, 1e-12);
        EXPECT_LE(std::fabs(wrapAngle(s.start.heading - at.heading)), s.exact ? 1e-9 : tol);
        const bool before_exact = i + 1 < path.segments.size() && path.segments[i + 1].exact;
        if (s.kind == SegmentKind::kTurn) {
            EXPECT_DOUBLE_EQ(s.end.x, s.start.x);
            EXPECT_DOUBLE_EQ(s.end.y, s.start.y);
            const Radians by = wrapAngle(s.end.heading - s.start.heading);
            EXPECT_GE(std::fabs(by), before_exact ? 1e-9 : config.min_turn);
            EXPECT_EQ(s.turn_direction, by > 0 ? 1 : -1);
        } else if (!r.model.holonomic) {
            const Radians travel = std::atan2(s.end.y - s.start.y, s.end.x - s.start.x);
            const Radians facing = s.reverse ? wrapAngle(travel + kPi) : travel;
            EXPECT_NEAR(wrapAngle(s.start.heading - facing), 0.0, 1e-9);
            EXPECT_NEAR(wrapAngle(s.end.heading - facing), 0.0, 1e-9);
            if (s.reverse) {
                EXPECT_TRUE(r.model.reverse && r.allow_reverse);
            }
        } else {
            EXPECT_FALSE(s.reverse);
        }
        at = s.end;
    }
    EXPECT_NEAR(at.x, r.goal.x, 1e-12);
    EXPECT_NEAR(at.y, r.goal.y, 1e-12);
    EXPECT_LE(std::fabs(wrapAngle(at.heading - r.goal.heading)), tol);
}

// Every path vertex at least R from every obstacle box.
void expectVerticesClear(const Path& path, const Field& f, const MotionModel& model) {
    const Meters radius = enclosingRadius(model);
    for (const PathSegment& s : path.segments) {
        for (const Pose& p : {s.start, s.end}) {
            for (const FieldObject& o : f.objects) {
                if (o.obstacle) {
                    EXPECT_GE(boxDistance(Point{p.x, p.y}, o), radius - 1e-9) << o.id;
                }
            }
        }
    }
}

const GeometricPlanner kPlanner;

} // namespace

TEST(Planner, StatusStrings) {
    const PlanStatus all[] = {
        PlanStatus::kOk,
        PlanStatus::kInvalidRequest,
        PlanStatus::kUnsupportedModel,
        PlanStatus::kNoField,
        PlanStatus::kStartOutOfBounds,
        PlanStatus::kStartBlocked,
        PlanStatus::kGoalOutOfBounds,
        PlanStatus::kGoalBlocked,
        PlanStatus::kNoPath,
    };
    std::set<std::string> seen;
    for (PlanStatus s : all) {
        const std::string text = toString(s);
        EXPECT_NE(text, "?");
        EXPECT_TRUE(seen.insert(text).second) << text;
    }
}

TEST(PlannerDirect, StraightAheadNeedsNoTurn) {
    const PlanRequest r = request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{1, 0, 0}, tank());
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(p.path.mode, PlanMode::kDirect);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_EQ(p.path.segments[0].kind, SegmentKind::kTranslate);
    EXPECT_FALSE(p.path.segments[0].reverse);
    EXPECT_NEAR(p.path.length(), 1.0, 1e-12);
    expectExecutable(p.path, r);
}

TEST(PlannerDirect, TankTurnsDrivesTurns) {
    const PlanRequest r = request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{0, 1, 0}, tank());
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 3u);
    const PathSegment& first = p.path.segments[0];
    const PathSegment& drive = p.path.segments[1];
    const PathSegment& last  = p.path.segments[2];
    EXPECT_EQ(first.kind, SegmentKind::kTurn);
    EXPECT_EQ(first.turn_direction, 1);
    EXPECT_NEAR(first.end.heading, kPi / 2.0, 1e-12);
    EXPECT_EQ(drive.kind, SegmentKind::kTranslate);
    EXPECT_FALSE(drive.reverse); // equal turning either way: forward
    EXPECT_EQ(last.kind, SegmentKind::kTurn);
    EXPECT_EQ(last.turn_direction, -1);
    EXPECT_NEAR(last.end.heading, 0.0, 1e-12);
    expectExecutable(p.path, r);
}

TEST(PlannerDirect, ReverseWhenItSavesTurning) {
    // Goal straight behind: back up without turning.
    PlanRequest r = request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{-1, 0, 0}, tank());
    PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_TRUE(p.path.segments[0].reverse);
    EXPECT_NEAR(p.path.segments[0].start.heading, 0.0, 1e-12);
    expectExecutable(p.path, r);

    // Behind and a little left: small turns while reversing.
    r = request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{-1, 0.2, 0}, tank());
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 3u);
    EXPECT_TRUE(p.path.segments[1].reverse);
    EXPECT_NEAR(totalTurn(p.path), 2.0 * std::atan2(0.2, 1.0), 1e-12);
    expectExecutable(p.path, r);

    // Behind, finishing turned around: turning cost is equal, so forward.
    r = request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{-1, 0, kPi}, tank());
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 2u);
    EXPECT_EQ(p.path.segments[0].kind, SegmentKind::kTurn);
    EXPECT_FALSE(p.path.segments[1].reverse);
    expectExecutable(p.path, r);
}

TEST(PlannerDirect, ReverseOnlyWhenAllowed) {
    PlanRequest r   = request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{-1, 0, 0}, tank());
    r.allow_reverse = false;
    PlanResult p    = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 3u);
    EXPECT_FALSE(p.path.segments[1].reverse);
    EXPECT_NEAR(totalTurn(p.path), 2.0 * kPi, 1e-12);
    expectExecutable(p.path, r);

    r               = request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{-1, 0, 0}, tank());
    r.model.reverse = false;
    p               = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(count(p.path, SegmentKind::kTurn), 2u);
    for (const PathSegment& s : p.path.segments) {
        EXPECT_FALSE(s.reverse);
    }
}

TEST(PlannerDirect, HolonomicSingleTranslation) {
    const PlanRequest r =
        request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{1.0, 0.5, kPi / 2.0}, mecanum());
    const PlanResult p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    const PathSegment& s = p.path.segments[0];
    EXPECT_EQ(s.kind, SegmentKind::kTranslate);
    EXPECT_NEAR(s.start.heading, 0.0, 1e-12);
    EXPECT_NEAR(s.end.heading, kPi / 2.0, 1e-12);
    expectExecutable(p.path, r);
}

TEST(PlannerDirect, HolonomicTurnInPlace) {
    const PlanRequest r =
        request(PlanMode::kDirect, Pose{1, 1, 0}, Pose{1, 1, -kPi / 2.0}, mecanum());
    const PlanResult p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_EQ(p.path.segments[0].kind, SegmentKind::kTurn);
    EXPECT_EQ(p.path.segments[0].turn_direction, -1);
    expectExecutable(p.path, r);
}

TEST(PlannerDirect, DropsTinyMoves) {
    PlanResult p =
        kPlanner.plan(request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{0.003, 0, 0.005}, tank()));
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.empty());

    p = kPlanner.plan(request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{0.003, 0, 1.0}, tank()));
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_EQ(p.path.segments[0].kind, SegmentKind::kTurn);
    EXPECT_DOUBLE_EQ(p.path.segments[0].start.x, 0.0);

    // A heading error below min_turn is left to the follower.
    const PlanRequest r = request(PlanMode::kDirect, Pose{0, 0, 0}, Pose{1, 0.004, 0.005}, tank());
    p                   = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    expectExecutable(p.path, r);
}

TEST(PlannerDirect, NeedsNoFieldAndDoesNotAvoid) {
    const PlanRequest r = request(PlanMode::kDirect, Pose{1, 2, 0}, Pose{3, 2, 0}, tank());
    ASSERT_EQ(kPlanner.plan(r).status, PlanStatus::kOk);

    // A box on the line and one holding the goal change nothing.
    const Field f =
        field({obstacle(1, Pose{2, 2, 0}, 0.5, 0.5), obstacle(2, Pose{3, 2, 0}, 0.3, 0.3)});
    PlanRequest with   = r;
    with.field         = &f;
    const PlanResult p = kPlanner.plan(with);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_NEAR(p.path.length(), 2.0, 1e-12);
    EXPECT_FALSE(kPlanner.clear(p.path, f, r.model));
}

TEST(Planner, UnsupportedModels) {
    const Field f = field({});
    for (PlanMode mode : {PlanMode::kDirect, PlanMode::kAvoiding}) {
        MotionModel m   = tank();
        m.turn_in_place = false;
        EXPECT_EQ(kPlanner.plan(request(mode, Pose{1, 1, 0}, Pose{2, 2, 0}, m, &f)).status,
                  PlanStatus::kUnsupportedModel);

        m                 = tank();
        m.min_turn_radius = 0.3;
        EXPECT_EQ(kPlanner.plan(request(mode, Pose{1, 1, 0}, Pose{2, 2, 0}, m, &f)).status,
                  PlanStatus::kUnsupportedModel);

        m                 = mecanum();
        m.min_turn_radius = 0.3;
        EXPECT_EQ(kPlanner.plan(request(mode, Pose{1, 1, 0}, Pose{2, 2, 0}, m, &f)).status,
                  PlanStatus::kUnsupportedModel);

        // Holonomic translation does not need a turn in place.
        m               = mecanum();
        m.turn_in_place = false;
        EXPECT_EQ(kPlanner.plan(request(mode, Pose{1, 1, 0}, Pose{2, 2, 0}, m, &f)).status,
                  PlanStatus::kOk);
    }
}

TEST(Planner, InvalidRequests) {
    const Field f   = field({});
    auto        run = [&](PlanRequest r) { return kPlanner.plan(r).status; };
    for (PlanMode mode : {PlanMode::kDirect, PlanMode::kAvoiding}) {
        EXPECT_EQ(run(request(mode, Pose{NAN, 1, 0}, Pose{2, 2, 0}, tank(), &f)),
                  PlanStatus::kInvalidRequest);
        EXPECT_EQ(run(request(mode, Pose{1, 1, 0}, Pose{2, 2, INFINITY}, tank(), &f)),
                  PlanStatus::kInvalidRequest);
        EXPECT_EQ(run(request(mode, Pose{1, 1, 0}, Pose{2, 2, 0}, MotionModel{}, &f)),
                  PlanStatus::kInvalidRequest);
    }
    EXPECT_EQ(run(request(static_cast<PlanMode>(7), Pose{1, 1, 0}, Pose{2, 2, 0}, tank(), &f)),
              PlanStatus::kInvalidRequest);

    GeometricPlannerConfig bad;
    bad.min_segment = -1;
    EXPECT_EQ(GeometricPlanner(bad)
                  .plan(request(PlanMode::kDirect, Pose{1, 1, 0}, Pose{2, 2, 0}, tank()))
                  .status,
              PlanStatus::kInvalidRequest);

    // A dropped turn leaves at most 0.1 rad of heading error.
    GeometricPlannerConfig coarse;
    coarse.min_turn = 0.1;
    EXPECT_EQ(GeometricPlanner(coarse)
                  .plan(request(PlanMode::kDirect, Pose{1, 1, 0}, Pose{2, 2, 0}, tank()))
                  .status,
              PlanStatus::kOk);
    coarse.min_turn = 0.11;
    EXPECT_EQ(GeometricPlanner(coarse)
                  .plan(request(PlanMode::kDirect, Pose{1, 1, 0}, Pose{2, 2, 0}, tank()))
                  .status,
              PlanStatus::kInvalidRequest);
}

TEST(PlannerAvoiding, NeedsAUsableField) {
    const Pose a{1, 1, 0};
    const Pose b{3, 3, 0};
    EXPECT_EQ(kPlanner.plan(request(PlanMode::kAvoiding, a, b, tank())).status,
              PlanStatus::kNoField);

    Field f      = field({obstacle(1, Pose{2, 1, 0}, 0.2, 0.2)});
    f.generation = 0;
    EXPECT_EQ(kPlanner.plan(request(PlanMode::kAvoiding, a, b, tank(), &f)).status,
              PlanStatus::kNoField);

    f              = field({obstacle(1, Pose{2, 1, 0}, 0.2, 0.2)});
    f.bounds.max_y = f.bounds.min_y;
    EXPECT_EQ(kPlanner.plan(request(PlanMode::kAvoiding, a, b, tank(), &f)).status,
              PlanStatus::kNoField);

    f                      = field({obstacle(1, Pose{2, 1, 0}, 0.2, 0.2)});
    f.objects[0].nominal.y = NAN;
    f.objects[0].valid     = false;
    EXPECT_EQ(kPlanner.plan(request(PlanMode::kAvoiding, a, b, tank(), &f)).status,
              PlanStatus::kNoField);
}

TEST(PlannerAvoiding, StraightWhenClear) {
    const Field       f = field({obstacle(1, Pose{2, 3, 0}, 0.5, 0.5)});
    const PlanRequest r =
        request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, kPi / 2.0}, tank(), &f);
    const PlanResult p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(p.path.mode, PlanMode::kAvoiding);
    ASSERT_EQ(p.path.segments.size(), 2u);
    EXPECT_NEAR(p.path.length(), 2.0, 1e-12);
    EXPECT_TRUE(kPlanner.clear(p.path, f, r.model));
    expectExecutable(p.path, r);
}

TEST(PlannerAvoiding, TankGoesAroundABox) {
    const Field       f = field({obstacle(1, Pose{2, 2, 0}, 0.6, 0.6)});
    const PlanRequest r = request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, 0}, tank(), &f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(p.blocking, 0);
    EXPECT_EQ(count(p.path, SegmentKind::kTranslate), 3u);
    expectExecutable(p.path, r);
    expectVerticesClear(p.path, f, r.model);
    EXPECT_TRUE(kPlanner.clear(p.path, f, r.model));

    // Shortest route over the two top vertices of the grown octagon, pushed
    // out by vertex_margin: the side at R, the corner cut at 45 degrees.
    const double grow = enclosingRadius(r.model) + kPlanner.config().vertex_margin;
    const double x    = 0.3 + grow * (std::sqrt(2.0) - 1.0);
    const double y    = 0.3 + grow;
    EXPECT_NEAR(p.path.length(), 2.0 * std::hypot(1.0 - x, y) + 2.0 * x, 1e-9);
}

TEST(PlannerAvoiding, HolonomicHeadingFollowsDistance) {
    const Field       f = field({obstacle(1, Pose{2, 2, 0}, 0.6, 0.6)});
    const PlanRequest r =
        request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, kPi / 2.0}, mecanum(), &f);
    const PlanResult p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(count(p.path, SegmentKind::kTurn), 0u);
    EXPECT_EQ(count(p.path, SegmentKind::kTranslate), 3u);
    expectExecutable(p.path, r);
    EXPECT_TRUE(kPlanner.clear(p.path, f, r.model));

    const Meters total = p.path.length();
    Meters       done  = 0;
    for (const PathSegment& s : p.path.segments) {
        EXPECT_NEAR(s.start.heading, kPi / 2.0 * done / total, 1e-9);
        done += std::hypot(s.end.x - s.start.x, s.end.y - s.start.y);
        EXPECT_NEAR(s.end.heading, kPi / 2.0 * done / total, 1e-9);
    }
}

TEST(PlannerAvoiding, ReverseSavesTurning) {
    // Robot faces +x, goal behind a box, final heading +x.
    const Field f   = field({obstacle(1, Pose{2, 2, 0}, 0.6, 0.6)});
    PlanRequest r   = request(PlanMode::kAvoiding, Pose{3, 2, 0}, Pose{1, 2, 0}, tank(), &f);
    PlanResult  rev = kPlanner.plan(r);
    ASSERT_EQ(rev.status, PlanStatus::kOk);
    expectExecutable(rev.path, r);
    bool reversed = false;
    for (const PathSegment& s : rev.path.segments) {
        reversed = reversed || s.reverse;
    }
    EXPECT_TRUE(reversed);

    r.allow_reverse      = false;
    const PlanResult fwd = kPlanner.plan(r);
    ASSERT_EQ(fwd.status, PlanStatus::kOk);
    expectExecutable(fwd.path, r);
    EXPECT_LT(totalTurn(rev.path) + 1.0, totalTurn(fwd.path));
    EXPECT_NEAR(rev.path.length(), fwd.path.length(), 1e-9);
}

TEST(PlannerAvoiding, RotatedBox) {
    // Thin box along the diagonal through (2, 2).
    const Field       f = field({obstacle(5, Pose{2, 2, kPi / 4.0}, 1.2, 0.1)});
    const MotionModel m = tank();

    // Inside the box's axis aligned extent but clear of the box itself.
    PlanRequest r = request(PlanMode::kAvoiding, Pose{3.0, 1.0, 0}, Pose{2.5, 1.5, 0}, m, &f);
    PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(count(p.path, SegmentKind::kTranslate), 1u);

    // On the box.
    r = request(PlanMode::kAvoiding, Pose{3.0, 1.0, 0}, Pose{2.3, 2.3, 0}, m, &f);
    p = kPlanner.plan(r);
    EXPECT_EQ(p.status, PlanStatus::kGoalBlocked);
    EXPECT_EQ(p.blocking, 5);

    // Across the box: around one of its ends.
    r = request(PlanMode::kAvoiding, Pose{1.4, 2.6, 0}, Pose{2.6, 1.4, 0}, m, &f);
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_GT(p.path.length(), std::hypot(1.2, 1.2) + 0.1);
    expectExecutable(p.path, r);
    expectVerticesClear(p.path, f, m);
    EXPECT_TRUE(kPlanner.clear(p.path, f, m));
}

TEST(PlannerAvoiding, BoxOffsetFollowsTheLandmark) {
    // Landmark origin at (2, 2); its box sits 0.5 m ahead of it.
    Field      f = field({landmark(3, Pose{2, 2, 0}, 0.3, Pose{0.5, 0, 0})});
    const Pose start{0.5, 0.5, 0};
    EXPECT_EQ(kPlanner.plan(request(PlanMode::kAvoiding, start, Pose{2, 2, 0}, tank(), &f)).status,
              PlanStatus::kOk);
    PlanResult p = kPlanner.plan(request(PlanMode::kAvoiding, start, Pose{2.5, 2, 0}, tank(), &f));
    EXPECT_EQ(p.status, PlanStatus::kGoalBlocked);
    EXPECT_EQ(p.blocking, 3);

    // Turned to face +y, the box moves with it.
    f.objects[0].pose.heading = kPi / 2.0;
    EXPECT_EQ(
        kPlanner.plan(request(PlanMode::kAvoiding, start, Pose{2.5, 2, 0}, tank(), &f)).status,
        PlanStatus::kOk);
    p = kPlanner.plan(request(PlanMode::kAvoiding, start, Pose{2, 2.5, 0}, tank(), &f));
    EXPECT_EQ(p.status, PlanStatus::kGoalBlocked);
    EXPECT_EQ(p.blocking, 3);
}

TEST(PlannerAvoiding, Boundaries) {
    const Field       f      = field({});
    const MotionModel m      = tank();
    const Meters      radius = enclosingRadius(m);
    auto              status = [&](const Pose& a, const Pose& b) {
        return kPlanner.plan(request(PlanMode::kAvoiding, a, b, m, &f));
    };
    // Goal within R of the wall, facing it: only a straight approach gets
    // there, backing in from the open side.
    PlanResult p = status(Pose{2, 2, 0}, Pose{radius - 0.01, 2, 0});
    EXPECT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.segments.back().exact);
    EXPECT_TRUE(p.path.segments.back().reverse);
    PlanRequest forward_only =
        request(PlanMode::kAvoiding, Pose{2, 2, 0}, Pose{radius - 0.01, 2, 0}, m, &f);
    forward_only.allow_reverse = false;
    p                          = kPlanner.plan(forward_only);
    EXPECT_EQ(p.status, PlanStatus::kGoalOutOfBounds);
    EXPECT_EQ(p.blocking, 0);
    EXPECT_EQ(status(Pose{2, 2, 0}, Pose{radius + 0.01, 2, 0}).status, PlanStatus::kOk);
    EXPECT_EQ(status(Pose{2, 2, 0}, Pose{5, 2, 0}).status, PlanStatus::kGoalOutOfBounds);
    // Along the north wall a tank cannot get clear of it.
    EXPECT_EQ(status(Pose{2, 4 - radius + 0.01, 0}, Pose{2, 2, 0}).status,
              PlanStatus::kStartOutOfBounds);
    EXPECT_EQ(status(Pose{2, -1, 0}, Pose{2, 2, 0}).status, PlanStatus::kStartOutOfBounds);

    // Field narrower than the circle: no point is clear of both walls, so
    // only a straight move along the corridor is possible.
    const Field narrow = field({}, Bounds{0, 0, 4, 2 * radius - 0.01});
    p                  = kPlanner.plan(
        request(PlanMode::kAvoiding, Pose{1, radius, 0}, Pose{3, radius, 0}, m, &narrow));
    EXPECT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_TRUE(p.path.segments[0].exact);
    EXPECT_EQ(kPlanner
                  .plan(request(PlanMode::kAvoiding, Pose{1, radius, 0}, Pose{3, radius, kPi / 2},
                                m, &narrow))
                  .status,
              PlanStatus::kStartOutOfBounds);
}

TEST(PlannerAvoiding, BlockedEndsNameTheObject) {
    const Field f =
        field({obstacle(7, Pose{1, 1, 0}, 0.4, 0.4), obstacle(12, Pose{3, 3, 0}, 0.4, 0.4)});
    PlanResult p =
        kPlanner.plan(request(PlanMode::kAvoiding, Pose{1.1, 1, 0}, Pose{3, 3.3, 0}, tank(), &f));
    EXPECT_EQ(p.status, PlanStatus::kStartBlocked);
    EXPECT_EQ(p.blocking, 7);
    EXPECT_TRUE(p.path.empty());

    p = kPlanner.plan(request(PlanMode::kAvoiding, Pose{2, 2, 0}, Pose{3, 3.3, 0}, tank(), &f));
    EXPECT_EQ(p.status, PlanStatus::kGoalBlocked);
    EXPECT_EQ(p.blocking, 12);

    // Near the box but clear of it with the robot's radius.
    const Meters edge = 0.2 + enclosingRadius(tank());
    p                 = kPlanner.plan(
        request(PlanMode::kAvoiding, Pose{2, 2, 0}, Pose{3, 3 + edge + 0.001, 0}, tank(), &f));
    EXPECT_EQ(p.status, PlanStatus::kOk);
}

TEST(PlannerAvoiding, NoRouteThroughAWall) {
    const Field      f = field({obstacle(1, Pose{2, 2, 0}, 0.1, 4.0)});
    const PlanResult p =
        kPlanner.plan(request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, 0}, tank(), &f));
    EXPECT_EQ(p.status, PlanStatus::kNoPath);
    EXPECT_EQ(p.blocking, 0);
    EXPECT_TRUE(p.path.empty());
}

TEST(PlannerAvoiding, NarrowGap) {
    // Wall at x = 2 with a 0.4 m gap between y = 1.8 and 2.2.
    const Field f =
        field({obstacle(1, Pose{2, 0.9, 0}, 0.1, 1.8), obstacle(2, Pose{2, 3.1, 0}, 0.1, 1.8)});
    const Pose start{0.8, 1.0, 0};
    const Pose goal{3.2, 3.0, 0};

    const MotionModel narrow = tank(0.1, 0.02);
    ASSERT_LT(2.0 * enclosingRadius(narrow), 0.4);
    const PlanRequest r = request(PlanMode::kAvoiding, start, goal, narrow, &f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    expectExecutable(p.path, r);
    expectVerticesClear(p.path, f, narrow);
    EXPECT_TRUE(kPlanner.clear(p.path, f, narrow));
    bool through_gap = false;
    for (const PathSegment& s : p.path.segments) {
        if (s.kind == SegmentKind::kTranslate && (s.start.x - 2) * (s.end.x - 2) <= 0 &&
            s.end.x != s.start.x) {
            const double t = (2 - s.start.x) / (s.end.x - s.start.x);
            const double y = s.start.y + t * (s.end.y - s.start.y);
            through_gap    = y > 1.8 && y < 2.2;
        }
    }
    EXPECT_TRUE(through_gap);

    const MotionModel wide = tank(0.2, 0.02);
    ASSERT_GT(2.0 * enclosingRadius(wide), 0.4);
    EXPECT_EQ(kPlanner.plan(request(PlanMode::kAvoiding, start, goal, wide, &f)).status,
              PlanStatus::kNoPath);
    // The same wide robot through a straight line in the gap is refused too.
    EXPECT_EQ(
        kPlanner.plan(request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, 0}, wide, &f)).status,
        PlanStatus::kNoPath);
}

TEST(PlannerAvoiding, MaxVerticesBound) {
    // Start, goal and the eight vertices of one grown box: 10 vertices.
    const Field       f = field({obstacle(1, Pose{2, 2, 0}, 0.6, 0.6)});
    const PlanRequest r = request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, 0}, tank(), &f);

    GeometricPlannerConfig c;
    c.max_vertices = 10;
    EXPECT_EQ(GeometricPlanner(c).plan(r).status, PlanStatus::kOk);
    c.max_vertices = 9;
    EXPECT_EQ(GeometricPlanner(c).plan(r).status, PlanStatus::kNoPath);

    // Vertices outside the free region do not count: only the upper four of
    // a box standing on the south wall.
    const Field       edge = field({obstacle(1, Pose{2, 0.3, 0}, 0.6, 1.0)});
    const PlanRequest low =
        request(PlanMode::kAvoiding, Pose{1, 0.6, 0}, Pose{3, 0.6, 0}, tank(), &edge);
    c.max_vertices = 6;
    EXPECT_EQ(GeometricPlanner(c).plan(low).status, PlanStatus::kOk);
    c.max_vertices = 5;
    EXPECT_EQ(GeometricPlanner(c).plan(low).status, PlanStatus::kNoPath);

    // A clear straight line builds no graph.
    c.max_vertices         = 0;
    const PlanRequest open = request(PlanMode::kAvoiding, Pose{1, 3, 0}, Pose{3, 3, 0}, tank(), &f);
    EXPECT_EQ(GeometricPlanner(c).plan(open).status, PlanStatus::kOk);
}

TEST(PlannerAvoiding, FieldCorrectionInvalidatesPath) {
    Field             f = field({landmark(4, Pose{2, 3, 0}, 0.3)});
    const PlanRequest r = request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, 0}, tank(), &f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(count(p.path, SegmentKind::kTranslate), 1u);
    EXPECT_TRUE(kPlanner.clear(p.path, f, r.model));

    // A correction away from the path keeps it.
    Field moved             = f;
    moved.generation        = 2;
    moved.objects[0].source = EstimateSource::kObserved;
    moved.objects[0].pose   = Pose{2.1, 3.1, 0.2};
    EXPECT_TRUE(kPlanner.clear(p.path, moved, r.model));

    // One onto the path does not.
    moved.objects[0].pose = Pose{2.0, 2.15, 0.1};
    EXPECT_FALSE(kPlanner.clear(p.path, moved, r.model));

    PlanRequest again  = r;
    again.field        = &moved;
    const PlanResult q = kPlanner.plan(again);
    ASSERT_EQ(q.status, PlanStatus::kOk);
    EXPECT_GT(q.path.length(), 2.0);
    EXPECT_TRUE(kPlanner.clear(q.path, moved, r.model));
    expectVerticesClear(q.path, moved, r.model);

    // Without a valid estimate the nominal pose counts again.
    moved.objects[0].valid  = false;
    moved.objects[0].source = EstimateSource::kNone;
    EXPECT_TRUE(kPlanner.clear(p.path, moved, r.model));
}

TEST(PlannerAvoiding, NonObstaclesIgnored) {
    FieldObject marker  = landmark(9, Pose{2, 2, 0}, 0.0);
    marker.obstacle     = false;
    const Field       f = field({marker});
    const PlanRequest r = request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, 0}, tank(), &f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(p.path.segments.size(), 1u);
}

TEST(PlannerAvoiding, TurnOnlyAtStart) {
    const Field       f = field({obstacle(1, Pose{1.5, 1, 0}, 0.3, 0.3)});
    const PlanRequest r =
        request(PlanMode::kAvoiding, Pose{1, 1, 0}, Pose{1, 1, kPi / 2.0}, tank(), &f);
    const PlanResult p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_EQ(p.path.segments[0].kind, SegmentKind::kTurn);
    EXPECT_TRUE(kPlanner.clear(p.path, f, r.model));
}

TEST(PlannerClear, RejectsCollisionsAndBadInput) {
    const Field       f = field({obstacle(1, Pose{2, 2, 0}, 0.6, 0.6)});
    const MotionModel m = tank();

    PathSegment through;
    through.start = Pose{1, 2, 0};
    through.end   = Pose{3, 2, 0};
    Path path;
    path.segments = {through};
    EXPECT_FALSE(kPlanner.clear(path, f, m));

    PathSegment spin;
    spin.kind     = SegmentKind::kTurn;
    spin.start    = Pose{2, 2.4, 0};
    spin.end      = Pose{2, 2.4, 1};
    path.segments = {spin};
    EXPECT_FALSE(kPlanner.clear(path, f, m));
    spin.start.y = spin.end.y = 3;
    path.segments             = {spin};
    EXPECT_TRUE(kPlanner.clear(path, f, m));

    PathSegment out;
    out.start     = Pose{1, 3, 0};
    out.end       = Pose{1, 3.9, 0};
    path.segments = {out};
    EXPECT_FALSE(kPlanner.clear(path, f, m));

    PathSegment bad;
    bad.start     = Pose{1, 3, 0};
    bad.end       = Pose{NAN, 3, 0};
    path.segments = {bad};
    EXPECT_FALSE(kPlanner.clear(path, f, m));

    EXPECT_TRUE(kPlanner.clear(Path{}, f, m));
    EXPECT_FALSE(kPlanner.clear(Path{}, f, MotionModel{}));
    Field broken  = f;
    broken.bounds = Bounds{};
    EXPECT_FALSE(kPlanner.clear(Path{}, broken, m));
}
