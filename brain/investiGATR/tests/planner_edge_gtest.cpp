// planner_edge_gtest.cpp
// Numerical and geometric edge cases: contact, collinear corners, repeated
// points, heading wrap, boxes across the boundary, merged boxes, zero-size
// boxes, fields too small for the robot, and the graph bound.

#include "investigatr/planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <gtest/gtest.h>
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

FieldObject obstacle(ObjectId id, const Pose& pose, Meters length, Meters width) {
    FieldObject o;
    o.id       = id;
    o.kind     = ObjectKind::kFixed;
    o.obstacle = true;
    o.nominal  = pose;
    o.pose     = pose;
    o.valid    = true;
    o.source   = EstimateSource::kNominal;
    o.box      = Box{Pose{}, length, width};
    return o;
}

Field field(std::vector<FieldObject> objects, const Bounds& bounds = Bounds{0, 0, 4, 4}) {
    Field f;
    f.generation = 1;
    f.map.id     = 0xED6E;
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

std::size_t count(const Path& path, SegmentKind kind) {
    return static_cast<std::size_t>(
        std::count_if(path.segments.begin(), path.segments.end(),
                      [kind](const PathSegment& s) { return s.kind == kind; }));
}

// Chained, executable by the model, every heading in (-pi, pi], ends at goal.
void expectExecutable(const Path& path, const PlanRequest& r) {
    const GeometricPlannerConfig config;
    const Radians                tol = config.min_turn + 1e-9;
    Pose                         at  = r.start;
    for (std::size_t i = 0; i < path.segments.size(); ++i) {
        const PathSegment& s = path.segments[i];
        SCOPED_TRACE(i);
        ASSERT_TRUE(finite(s.start) && finite(s.end));
        for (Radians h : {s.start.heading, s.end.heading}) {
            EXPECT_GT(h, -kPi);
            EXPECT_LE(h, kPi);
        }
        EXPECT_NEAR(s.start.x, at.x, 1e-12);
        EXPECT_NEAR(s.start.y, at.y, 1e-12);
        EXPECT_LE(std::fabs(wrapAngle(s.start.heading - at.heading)), s.exact ? 1e-9 : tol);
        const bool before_exact = i + 1 < path.segments.size() && path.segments[i + 1].exact;
        if (s.kind == SegmentKind::kTurn) {
            const Radians by = wrapAngle(s.end.heading - s.start.heading);
            EXPECT_GE(std::fabs(by), before_exact ? 1e-9 : config.min_turn);
            EXPECT_EQ(s.turn_direction, by > 0 ? 1 : -1);
        } else {
            EXPECT_GT(std::hypot(s.end.x - s.start.x, s.end.y - s.start.y), 1e-9);
            if (!r.model.holonomic) {
                const Radians travel = std::atan2(s.end.y - s.start.y, s.end.x - s.start.x);
                const Radians facing = s.reverse ? wrapAngle(travel + kPi) : travel;
                EXPECT_NEAR(wrapAngle(s.start.heading - facing), 0.0, 1e-9);
            }
        }
        at = s.end;
    }
    EXPECT_NEAR(at.x, r.goal.x, 1e-12);
    EXPECT_NEAR(at.y, r.goal.y, 1e-12);
    EXPECT_LE(std::fabs(wrapAngle(at.heading - r.goal.heading)), tol);
}

// Every sampled pose keeps the footprint off every box by the clearance.
void expectFootprintsClear(const Path& path, const PlanRequest& r) {
    for (const PathSegment& s : path.segments) {
        const Radians turn = wrapAngle(s.end.heading - s.start.heading);
        for (int i = 0; i <= 40; ++i) {
            const double    t = i / 40.0;
            const Pose      p{s.start.x + t * (s.end.x - s.start.x),
                              s.start.y + t * (s.end.y - s.start.y), s.start.heading + t * turn};
            const Clearance c =
                footprintClearance(p, r.model.footprint, r.model.clearance, *r.field);
            ASSERT_GE(c.distance, -1e-9) << "near " << c.nearest << " at t " << t;
        }
    }
}

double highestY(const Path& path) {
    double y = -1e9;
    for (const PathSegment& s : path.segments) {
        y = std::max({y, s.start.y, s.end.y});
    }
    return y;
}

double lowestY(const Path& path) {
    double y = 1e9;
    for (const PathSegment& s : path.segments) {
        y = std::min({y, s.start.y, s.end.y});
    }
    return y;
}

const GeometricPlanner kPlanner;

} // namespace

TEST(PlannerEdge, TouchingCornersLeaveNoGap) {
    // Two boxes meeting corner to corner at (1.8, 1.8); the straight line
    // runs through the contact point.
    const Field f = field(
        {obstacle(1, Pose{1.55, 1.55, 0}, 0.5, 0.5), obstacle(2, Pose{2.05, 2.05, 0}, 0.5, 0.5)});
    GeometricPlannerConfig flush;
    flush.vertex_margin = 0;
    for (const GeometricPlanner& planner : {GeometricPlanner(), GeometricPlanner(flush)}) {
        const PlanRequest r =
            request(PlanMode::kAvoiding, Pose{1.2, 2.4, 0}, Pose{2.4, 1.2, 0}, tank(0.01, 0.0), &f);
        const PlanResult p = planner.plan(r);
        ASSERT_EQ(p.status, PlanStatus::kOk);
        EXPECT_GT(p.path.length(), std::hypot(1.2, 1.2) + 0.3);
        expectExecutable(p.path, r);
        expectFootprintsClear(p.path, r);
        EXPECT_TRUE(planner.clear(p.path, f, r.model));
    }
}

TEST(PlannerEdge, CollinearEdgesAndCorners) {
    // Three boxes in a row whose grown boxes overlap: the top and bottom
    // edges and vertices all line up.
    const Field f =
        field({obstacle(1, Pose{1.5, 2, 0}, 0.3, 0.3), obstacle(2, Pose{2.0, 2, 0}, 0.3, 0.3),
               obstacle(3, Pose{2.5, 2, 0}, 0.3, 0.3)});
    GeometricPlannerConfig flush;
    flush.vertex_margin = 0;
    const GeometricPlanner planner(flush);
    const MotionModel      m = tank(0.1, 0.0);
    const Meters           r = enclosingRadius(m);

    // Along the grown top edges: touching, not crossing.
    PlanRequest q = request(PlanMode::kAvoiding, Pose{1, 2.15 + r, 0}, Pose{3, 2.15 + r, 0}, m, &f);
    PlanResult  p = planner.plan(q);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_TRUE(planner.clear(p.path, f, m));

    // Across the row: along the bottom vertices as one straight leg.
    q = request(PlanMode::kAvoiding, Pose{1, 1.8, 0}, Pose{3, 1.8, 0}, m, &f);
    p = planner.plan(q);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(count(p.path, SegmentKind::kTranslate), 3u);
    expectExecutable(p.path, q);
    expectFootprintsClear(p.path, q);
    EXPECT_TRUE(planner.clear(p.path, f, m));
    for (const PathSegment& s : p.path.segments) {
        if (s.kind == SegmentKind::kTranslate && std::fabs(s.start.y - s.end.y) < 1e-12) {
            EXPECT_NEAR(s.start.y, 1.85 - r, 1e-12);
            EXPECT_GT(std::fabs(s.end.x - s.start.x), 1.2);
        }
    }
}

TEST(PlannerEdge, LongRowIsOneLeg) {
    // Twelve boxes in a row: their grown bottom vertices all lie on one
    // line, and the route along them is a single leg.
    std::vector<FieldObject> row;
    for (int i = 0; i < 12; ++i) {
        row.push_back(obstacle(static_cast<ObjectId>(i + 1), Pose{0.6 + 0.25 * i, 2, 0}, 0.2, 0.2));
    }
    const Field            f = field(row);
    GeometricPlannerConfig flush;
    flush.vertex_margin = 0;
    for (const GeometricPlanner& planner : {GeometricPlanner(), GeometricPlanner(flush)}) {
        for (const MotionModel& m : {tank(0.05, 0.01), mecanum(0.05, 0.01)}) {
            const PlanRequest q =
                request(PlanMode::kAvoiding, Pose{0.2, 1.97, 0}, Pose{3.8, 1.97, 0}, m, &f);
            const PlanResult p = planner.plan(q);
            ASSERT_EQ(p.status, PlanStatus::kOk);
            EXPECT_EQ(count(p.path, SegmentKind::kTranslate), 3u);
            expectExecutable(p.path, q);
            expectFootprintsClear(p.path, q);
        }
    }
}

TEST(PlannerEdge, RepeatedPointsMakeNoZeroLegs) {
    const Field       f    = field({obstacle(1, Pose{2, 2, 0}, 0.6, 0.6)});
    const MotionModel m    = tank();
    const Meters      grow = enclosingRadius(m) + kPlanner.config().vertex_margin;

    // Start on a graph vertex: the upper left one of the grown box.
    const Pose        start{2 - 0.3 - grow, 2 + 0.3 + grow * (std::sqrt(2.0) - 1.0), 0};
    const PlanRequest r = request(PlanMode::kAvoiding, start, Pose{2.6, 2.9, 0}, m, &f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    expectExecutable(p.path, r);
    EXPECT_TRUE(kPlanner.clear(p.path, f, m));

    // Zero-length moves are point checks.
    PathSegment still;
    still.start = still.end = Pose{1, 1, 0};
    Path path;
    path.segments = {still};
    EXPECT_TRUE(kPlanner.clear(path, f, m));
    path.segments[0].start = path.segments[0].end = Pose{2, 2.4, 0};
    EXPECT_FALSE(kPlanner.clear(path, f, m));
}

TEST(PlannerEdge, GoalEqualsStart) {
    const Field       f = field({obstacle(1, Pose{2, 2, 0}, 0.3, 0.3)});
    const MotionModel m = tank();
    for (PlanMode mode : {PlanMode::kDirect, PlanMode::kAvoiding}) {
        // In the open.
        PlanResult p = kPlanner.plan(request(mode, Pose{1, 1, 0.5}, Pose{1, 1, 0.5}, m, &f));
        ASSERT_EQ(p.status, PlanStatus::kOk);
        EXPECT_TRUE(p.path.empty());

        // A half turn is one turn, counterclockwise.
        p = kPlanner.plan(request(mode, Pose{1, 1, 0}, Pose{1, 1, kPi}, m, &f));
        ASSERT_EQ(p.status, PlanStatus::kOk);
        ASSERT_EQ(p.path.segments.size(), 1u);
        EXPECT_EQ(p.path.segments[0].turn_direction, 1);
    }
    // Touching a wall or a box, already there.
    PlanResult p = kPlanner.plan(
        request(PlanMode::kAvoiding, Pose{2, 0.15, kPi / 2}, Pose{2, 0.15, kPi / 2}, m, &f));
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.empty());
    p = kPlanner.plan(request(PlanMode::kAvoiding, Pose{1.7, 2, 0}, Pose{1.7, 2, 0}, m, &f));
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.empty());
    // Overlapping it is refused even with nowhere to go.
    p = kPlanner.plan(request(PlanMode::kAvoiding, Pose{1.75, 2, 0}, Pose{1.75, 2, 0}, m, &f));
    EXPECT_EQ(p.status, PlanStatus::kStartBlocked);
    EXPECT_EQ(p.blocking, 1);
}

TEST(PlannerEdge, HeadingWrap) {
    const MotionModel t = tank();

    // Facing -x as pi, -pi or 3 pi: no turn to drive toward -x.
    for (Radians h : {kPi, -kPi, 3.0 * kPi, -3.0 * kPi}) {
        const PlanRequest r = request(PlanMode::kDirect, Pose{1, 1, h}, Pose{0.5, 1, -kPi}, t);
        const PlanResult  p = kPlanner.plan(r);
        ASSERT_EQ(p.status, PlanStatus::kOk);
        ASSERT_EQ(p.path.segments.size(), 1u) << h;
        EXPECT_FALSE(p.path.segments[0].reverse);
        EXPECT_DOUBLE_EQ(p.path.segments[0].start.heading, kPi);
        expectExecutable(p.path, r);
    }

    // 3.1 to -3.1 is a short turn counterclockwise through pi.
    PlanResult p = kPlanner.plan(request(PlanMode::kDirect, Pose{1, 1, 3.1}, Pose{1, 1, -3.1}, t));
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_EQ(p.path.segments[0].turn_direction, 1);
    EXPECT_NEAR(wrapAngle(p.path.segments[0].end.heading - p.path.segments[0].start.heading),
                2.0 * kPi - 6.2, 1e-12);

    // Backing toward +x while facing just past -pi.
    const PlanRequest back =
        request(PlanMode::kDirect, Pose{1, 1, -kPi + 0.001}, Pose{2, 1, -kPi + 0.001}, t);
    p = kPlanner.plan(back);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_TRUE(p.path.segments[0].reverse);
    expectExecutable(p.path, back);

    // Holonomic around a box from heading 3.0 to -3.0: the short way, CCW.
    const Field       f = field({obstacle(1, Pose{2, 2, 0}, 0.6, 0.6)});
    const PlanRequest r =
        request(PlanMode::kAvoiding, Pose{1, 2, 3.0}, Pose{3, 2, -3.0}, mecanum(), &f);
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    expectExecutable(p.path, r);
    Radians turned = 0;
    for (const PathSegment& s : p.path.segments) {
        const Radians step = wrapAngle(s.end.heading - s.start.heading);
        EXPECT_GE(step, 0.0);
        turned += step;
    }
    EXPECT_NEAR(turned, 2.0 * kPi - 6.0, 1e-9);
}

TEST(PlannerEdge, BoxesAcrossTheBoundary) {
    const MotionModel m = tank();
    const Meters      r = enclosingRadius(m);

    // Half outside the south wall: no way under it.
    const Field wall = field({obstacle(1, Pose{2, 0, 0}, 1.0, 1.0)});
    PlanRequest q    = request(PlanMode::kAvoiding, Pose{1, 0.6, 0}, Pose{3, 0.6, 0}, m, &wall);
    PlanResult  p    = kPlanner.plan(q);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_GE(highestY(p.path), 0.5 + r);
    expectExecutable(p.path, q);
    expectFootprintsClear(p.path, q);

    // Wholly outside: no effect.
    const Field outside = field({obstacle(1, Pose{-1, 2, 0}, 1.0, 1.0)});
    p                   = kPlanner.plan(
        request(PlanMode::kAvoiding, Pose{1, 1, 0}, Pose{1, 3, kPi / 2}, m, &outside));
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_EQ(count(p.path, SegmentKind::kTranslate), 1u);

    // Across the whole field: two halves.
    const Field split = field({obstacle(1, Pose{2, 2, 0}, 6.0, 0.2)});
    p = kPlanner.plan(request(PlanMode::kAvoiding, Pose{2, 1, 0}, Pose{2, 3, 0}, m, &split));
    EXPECT_EQ(p.status, PlanStatus::kNoPath);
}

TEST(PlannerEdge, OverlappingBoxesFormWalls) {
    const MotionModel m = tank();
    const Meters      r = enclosingRadius(m);

    // A wall of three overlapping boxes from the south wall to y = 3.3.
    const Field wall =
        field({obstacle(1, Pose{2, 0.5, 0}, 0.2, 1.2), obstacle(2, Pose{2, 1.6, 0}, 0.2, 1.2),
               obstacle(3, Pose{2, 2.7, 0}, 0.2, 1.2)});
    const PlanRequest q = request(PlanMode::kAvoiding, Pose{1, 1, 0}, Pose{3, 1, 0}, m, &wall);
    PlanResult        p = kPlanner.plan(q);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_GE(highestY(p.path), 3.3 + r);
    expectExecutable(p.path, q);
    expectFootprintsClear(p.path, q);

    // A T: the narrow box's vertices inside the wide box's grown box are
    // dropped, leaving start, goal, 8 + 4 vertices.
    const Field tee =
        field({obstacle(1, Pose{2, 2, 0}, 1.0, 0.2), obstacle(2, Pose{2, 2.5, 0}, 0.2, 1.0)});
    const PlanRequest over =
        request(PlanMode::kAvoiding, Pose{1, 2.8, 0}, Pose{3, 2.8, 0}, m, &tee);
    GeometricPlannerConfig c;
    c.max_vertices = 14;
    EXPECT_EQ(GeometricPlanner(c).plan(over).status, PlanStatus::kOk);
    c.max_vertices = 13;
    EXPECT_EQ(GeometricPlanner(c).plan(over).status, PlanStatus::kNoPath);

    // A closed ring of overlapping boxes.
    const Field ring =
        field({obstacle(1, Pose{2, 2.6, 0}, 1.4, 0.2), obstacle(2, Pose{2, 1.4, 0}, 1.4, 0.2),
               obstacle(3, Pose{1.4, 2, 0}, 0.2, 1.4), obstacle(4, Pose{2.6, 2, 0}, 0.2, 1.4)});
    p = kPlanner.plan(
        request(PlanMode::kAvoiding, Pose{0.5, 0.5, 0}, Pose{2, 2, 0}, tank(0.1, 0.02), &ring));
    EXPECT_EQ(p.status, PlanStatus::kNoPath);
}

TEST(PlannerEdge, ZeroSizeBoxes) {
    // The link's map validator rejects obstacles without a size; the planner
    // still treats one as a point or a line.
    const MotionModel m = tank();
    const Meters      r = enclosingRadius(m);

    const Field point = field({obstacle(1, Pose{2, 2, 0}, 0, 0)});
    PlanRequest q     = request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, 0}, m, &point);
    PlanResult  p     = kPlanner.plan(q);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_GT(count(p.path, SegmentKind::kTranslate), 1u);
    expectFootprintsClear(p.path, q);
    // Clear of the footprint but within R: reached by a straight approach.
    EXPECT_EQ(
        kPlanner
            .plan(request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{2, 2 + r - 0.01, 0}, m, &point))
            .status,
        PlanStatus::kOk);
    // Under the footprint.
    p = kPlanner.plan(request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{2, 2.1, 0}, m, &point));
    EXPECT_EQ(p.status, PlanStatus::kGoalBlocked);
    EXPECT_EQ(p.blocking, 1);

    const Field line = field({obstacle(1, Pose{2, 2, kPi / 2}, 1.0, 0)});
    q                = request(PlanMode::kAvoiding, Pose{1, 2, 0}, Pose{3, 2, 0}, m, &line);
    p                = kPlanner.plan(q);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(highestY(p.path) >= 2.5 + r || lowestY(p.path) <= 1.5 - r);
    expectFootprintsClear(p.path, q);

    const Clearance c =
        footprintClearance(Pose{0, 0, 0}, Footprint{0.1, 0.1, 0.1, 0.1}, 0.0,
                           field({obstacle(1, Pose{0.5, 0.3, 0}, 0, 0)}, Bounds{-5, -5, 5, 5}));
    EXPECT_NEAR(c.distance, std::hypot(0.4, 0.2), 1e-12);

    Field negative = field({obstacle(1, Pose{2, 2, 0}, -0.1, 0.3)});
    EXPECT_EQ(
        kPlanner.plan(request(PlanMode::kAvoiding, Pose{1, 1, 0}, Pose{3, 3, 0}, m, &negative))
            .status,
        PlanStatus::kNoField);
}

TEST(PlannerEdge, FieldsTooSmallForTheRobot) {
    const MotionModel m = tank(); // 0.3 m square

    // Exactly the robot's size: it fits, touching all four walls, and can
    // only stay.
    const Field snug  = field({}, Bounds{0, 0, 0.3, 0.3});
    const Pose  there = {0.15, 0.15, 0};
    PlanResult  p     = kPlanner.plan(request(PlanMode::kAvoiding, there, there, m, &snug));
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.empty());
    // A quarter turn fits the square but needs room to turn.
    EXPECT_EQ(
        kPlanner.plan(request(PlanMode::kAvoiding, there, Pose{0.15, 0.15, kPi / 2}, m, &snug))
            .status,
        PlanStatus::kStartOutOfBounds);
    EXPECT_EQ(
        kPlanner.plan(request(PlanMode::kAvoiding, there, Pose{0.15, 0.15, 1}, m, &snug)).status,
        PlanStatus::kGoalOutOfBounds);
    EXPECT_EQ(
        kPlanner.plan(request(PlanMode::kAvoiding, there, Pose{0.2, 0.15, 0}, m, &snug)).status,
        PlanStatus::kGoalOutOfBounds);

    // Smaller than the robot, or no area.
    const Field tiny = field({}, Bounds{0, 0, 0.25, 0.25});
    EXPECT_EQ(kPlanner
                  .plan(request(PlanMode::kAvoiding, Pose{0.125, 0.125, 0}, Pose{0.125, 0.125, 0},
                                m, &tiny))
                  .status,
              PlanStatus::kStartOutOfBounds);
    const Field flat = field({}, Bounds{0, 0, 4, 0});
    EXPECT_EQ(
        kPlanner.plan(request(PlanMode::kAvoiding, Pose{1, 0, 0}, Pose{2, 0, 0}, m, &flat)).status,
        PlanStatus::kNoField);
}

TEST(PlannerEdge, Deterministic) {
    const Field f =
        field({obstacle(1, Pose{2, 2, 0.3}, 0.6, 0.4), obstacle(2, Pose{1.2, 2.9, 0}, 0.3, 0.3)});
    const PlanRequest r =
        request(PlanMode::kAvoiding, Pose{0.8, 1.2, 0.4}, Pose{3.1, 3.2, -2.0}, tank(), &f);
    const PlanResult a = kPlanner.plan(r);
    const PlanResult b = GeometricPlanner().plan(r);
    ASSERT_EQ(a.status, PlanStatus::kOk);
    ASSERT_EQ(a.path.segments.size(), b.path.segments.size());
    for (std::size_t i = 0; i < a.path.segments.size(); ++i) {
        const PathSegment& x = a.path.segments[i];
        const PathSegment& y = b.path.segments[i];
        EXPECT_EQ(x.kind, y.kind);
        EXPECT_EQ(x.start.x, y.start.x);
        EXPECT_EQ(x.start.y, y.start.y);
        EXPECT_EQ(x.end.x, y.end.x);
        EXPECT_EQ(x.end.y, y.end.y);
        EXPECT_EQ(x.end.heading, y.end.heading);
    }
}

TEST(PlannerCost, FullSizeMapIsBounded) {
    // 128 small boxes, the link's largest map, and a small robot: every grown
    // box keeps all 8 vertices, 1026 with start and goal.
    Field f = field({}, Bounds{0, 0, 3.5664, 3.5664});
    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 8; ++j) {
            f.objects.push_back(obstacle(static_cast<ObjectId>(f.objects.size() + 1),
                                         Pose{0.12 + i * 0.22, 0.25 + j * 0.44, 0.3 * (i + j)},
                                         0.05, 0.05));
        }
    }
    const PlanRequest r = request(PlanMode::kAvoiding, Pose{0.02, 0.25, 0}, Pose{3.54, 0.25, 0},
                                  tank(0.01, 0.005), &f);

    // Over the default bound: refused before any search.
    auto       t0 = std::chrono::steady_clock::now();
    PlanResult p  = kPlanner.plan(r);
    auto       t1 = std::chrono::steady_clock::now();
    EXPECT_EQ(p.status, PlanStatus::kNoPath);
    std::printf("128 boxes over the 512 vertex bound: %.0f us\n",
                std::chrono::duration<double, std::micro>(t1 - t0).count());

    GeometricPlannerConfig all;
    all.max_vertices = 2 + 8 * 128;
    t0               = std::chrono::steady_clock::now();
    p                = GeometricPlanner(all).plan(r);
    t1               = std::chrono::steady_clock::now();
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(kPlanner.clear(p.path, f, r.model));
    expectExecutable(p.path, r);
    std::printf("128 boxes, 1026 vertices, routed: %.0f us\n",
                std::chrono::duration<double, std::micro>(t1 - t0).count());
}
