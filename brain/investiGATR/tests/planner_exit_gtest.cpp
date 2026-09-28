// planner_exit_gtest.cpp
// Start escapes and goal approaches: path ends closer to a wall or box than
// the enclosing radius, joined to the circle-planned route by exact moves.

#include "investigatr/planner.h"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace investigatr;

namespace
{

const Footprint kSquare{0.15, 0.15, 0.15, 0.15};
const Footprint kLong{0.22, 0.08, 0.14, 0.11}; // origin off center

MotionModel tank(const Footprint& f = kSquare, Meters clearance = 0.02) {
    MotionModel m;
    m.holonomic     = false;
    m.turn_in_place = true;
    m.reverse       = true;
    m.footprint     = f;
    m.clearance     = clearance;
    return m;
}

MotionModel mecanum(const Footprint& f = kSquare, Meters clearance = 0.02) {
    MotionModel m = tank(f, clearance);
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
    f.map.id     = 0xE5CA9E;
    f.bounds     = bounds;
    f.objects    = std::move(objects);
    return f;
}

PlanRequest request(const Pose& start, const Pose& goal, const MotionModel& model, const Field& f) {
    PlanRequest r;
    r.mode  = PlanMode::kAvoiding;
    r.start = start;
    r.goal  = goal;
    r.model = model;
    r.field = &f;
    return r;
}

Point position(const Pose& p) { return Point{p.x, p.y}; }

double boundsDistance(const Field& f, Point p) {
    const Bounds& b = f.bounds;
    return std::min({p.x - b.min_x, b.max_x - p.x, p.y - b.min_y, b.max_y - p.y});
}

// Exact footprint gap at p, negative when overlapping.
double gapAt(const Pose& p, const MotionModel& m, const Field& f) {
    return footprintClearance(p, m.footprint, 0.0, f).distance;
}

// Robot touching the wall through wall_point with inward normal n.
Pose touching(Point wall_point, Point n, Radians heading, const Footprint& f) {
    const Point body[4] = {
        {f.front, f.left}, {f.front, -f.right}, {-f.back, f.left}, {-f.back, -f.right}};
    double reach = 0;
    for (const Point& c : body) {
        const Pose at = compose(Pose{0, 0, heading}, Pose{c.x, c.y, 0});
        reach         = std::max(reach, -(at.x * n.x + at.y * n.y));
    }
    return Pose{wall_point.x + reach * n.x, wall_point.y + reach * n.y, heading};
}

// Chained, executable by the model, starts at start, ends at goal. An exact
// segment is entered at its own heading: a turn before it is never dropped.
void expectExecutable(const Path& path, const PlanRequest& r) {
    const GeometricPlannerConfig config;
    const Radians                tol = config.min_turn + 1e-9;
    Pose                         at  = r.start;
    for (std::size_t i = 0; i < path.segments.size(); ++i) {
        const PathSegment& s = path.segments[i];
        SCOPED_TRACE(i);
        ASSERT_TRUE(finite(s.start) && finite(s.end));
        EXPECT_NEAR(s.start.x, at.x, 1e-12);
        EXPECT_NEAR(s.start.y, at.y, 1e-12);
        EXPECT_LE(std::fabs(wrapAngle(s.start.heading - at.heading)), s.exact ? 1e-9 : tol);
        const bool before_exact = i + 1 < path.segments.size() && path.segments[i + 1].exact;
        if (s.kind == SegmentKind::kTurn) {
            EXPECT_FALSE(s.exact);
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
        } else {
            EXPECT_FALSE(s.reverse);
        }
        if (s.exact) {
            EXPECT_NEAR(wrapAngle(s.end.heading - s.start.heading), 0.0, 1e-12);
        }
        at = s.end;
    }
    EXPECT_NEAR(at.x, r.goal.x, 1e-12);
    EXPECT_NEAR(at.y, r.goal.y, 1e-12);
    EXPECT_LE(std::fabs(wrapAngle(at.heading - r.goal.heading)), tol);
}

// Sampled footprints keep the clearance, except that within 2R of the start
// and goal poses the first and last exact moves come no closer than those
// poses already are.
void expectFootprintsClear(const Path& path, const PlanRequest& r) {
    const Field& f    = *r.field;
    const Meters zone = 2.0 * enclosingRadius(r.model);
    for (std::size_t i = 0; i < path.segments.size(); ++i) {
        const PathSegment& s     = path.segments[i];
        const bool         first = s.exact && i == 0;
        const bool         last  = s.exact && i + 1 == path.segments.size();
        const Radians      turn  = wrapAngle(s.end.heading - s.start.heading);
        const double       len   = std::hypot(s.end.x - s.start.x, s.end.y - s.start.y);
        for (int k = 0; k <= 50; ++k) {
            const double t = k / 50.0;
            const Pose   p{s.start.x + t * (s.end.x - s.start.x),
                           s.start.y + t * (s.end.y - s.start.y), s.start.heading + t * turn};
            double       floor = r.model.clearance;
            if (first && t * len <= zone) {
                floor = std::min(floor, gapAt(s.start, r.model, f));
            }
            if (last && (1 - t) * len <= zone) {
                floor = std::min(floor, gapAt(s.end, r.model, f));
            }
            ASSERT_GE(floor, -1e-9);
            ASSERT_GE(gapAt(p, r.model, f), floor - 1e-9) << "t " << t;
        }
    }
}

struct Wall {
    const char* name;
    Point       at; // middle of the wall
    Point       n;  // into the field
};

const Wall kWalls[4] = {{"south", {2, 0}, {0, 1}},
                        {"north", {2, 4}, {0, -1}},
                        {"west", {0, 2}, {1, 0}},
                        {"east", {4, 2}, {-1, 0}}};

const GeometricPlanner kPlanner;

} // namespace

TEST(PlannerExit, StartAgainstEachWallFacingAway) {
    const Field f = field({});
    for (const Wall& wall : kWalls) {
        for (const MotionModel& m : {tank(kLong), mecanum(kLong)}) {
            SCOPED_TRACE(std::string(wall.name) + (m.holonomic ? " holonomic" : " tank"));
            const Radians     away   = std::atan2(wall.n.y, wall.n.x);
            const Pose        start  = touching(wall.at, wall.n, away, m.footprint);
            const Meters      radius = enclosingRadius(m);
            const PlanRequest r      = request(start, Pose{1.2, 2.6, 0.3}, m, f);
            ASSERT_LT(boundsDistance(f, position(start)), radius);
            EXPECT_NEAR(gapAt(start, m, f), 0.0, 1e-12);

            const PlanResult p = kPlanner.plan(r);
            ASSERT_EQ(p.status, PlanStatus::kOk) << toString(p.status);
            const PathSegment& escape = p.path.segments.front();
            EXPECT_TRUE(escape.exact);
            EXPECT_EQ(escape.kind, SegmentKind::kTranslate);
            EXPECT_FALSE(escape.reverse);
            EXPECT_NEAR(wrapAngle(escape.start.heading - away), 0.0, 1e-12);
            const Point  move{escape.end.x - escape.start.x, escape.end.y - escape.start.y};
            const double length = std::hypot(move.x, move.y);
            EXPECT_GT(length, 0.0);
            EXPECT_LE(length, 2.0 * radius);
            EXPECT_NEAR((move.x * wall.n.x + move.y * wall.n.y) / length, 1.0, 1e-12);
            EXPECT_GE(boundsDistance(f, position(escape.end)), radius);
            for (std::size_t i = 1; i < p.path.segments.size(); ++i) {
                EXPECT_FALSE(p.path.segments[i].exact) << i;
            }
            expectExecutable(p.path, r);
            expectFootprintsClear(p.path, r);
            EXPECT_TRUE(kPlanner.clear(p.path, f, m));
        }
    }
}

TEST(PlannerExit, StartAlongEachWall) {
    const Field f = field({});
    for (const Wall& wall : kWalls) {
        for (const Radians side : {kPi / 2.0, -kPi / 2.0}) {
            SCOPED_TRACE(std::string(wall.name) +
                         (side > 0 ? " wall on the left" : " wall on the right"));
            const Radians along = std::atan2(wall.n.y, wall.n.x) + side;

            // A tank can only drive along the wall and never gets clear of it.
            const MotionModel t = tank(kLong);
            PlanResult        p = kPlanner.plan(
                request(touching(wall.at, wall.n, along, t.footprint), Pose{1.2, 2.6, 0.3}, t, f));
            EXPECT_EQ(p.status, PlanStatus::kStartOutOfBounds);
            EXPECT_EQ(p.blocking, 0);
            EXPECT_TRUE(p.path.empty());

            // A holonomic drive steps sideways off it.
            const MotionModel m = mecanum(kLong);
            const PlanRequest r =
                request(touching(wall.at, wall.n, along, m.footprint), Pose{1.2, 2.6, 0.3}, m, f);
            p = kPlanner.plan(r);
            ASSERT_EQ(p.status, PlanStatus::kOk);
            const PathSegment& escape = p.path.segments.front();
            EXPECT_TRUE(escape.exact);
            EXPECT_NEAR(wrapAngle(escape.start.heading - wrapAngle(along)), 0.0, 1e-12);
            EXPECT_NEAR(wrapAngle(escape.end.heading - escape.start.heading), 0.0, 1e-12);
            const Point move{escape.end.x - escape.start.x, escape.end.y - escape.start.y};
            EXPECT_NEAR((move.x * wall.n.x + move.y * wall.n.y) / std::hypot(move.x, move.y), 1.0,
                        1e-12);
            expectExecutable(p.path, r);
            expectFootprintsClear(p.path, r);
            EXPECT_TRUE(kPlanner.clear(p.path, f, m));
        }
    }
}

TEST(PlannerExit, FacingTheWallBacksOut) {
    const Field f = field({});
    for (const Wall& wall : kWalls) {
        SCOPED_TRACE(wall.name);
        const Radians     toward = std::atan2(-wall.n.y, -wall.n.x);
        const MotionModel m      = tank(kLong);
        PlanRequest       r =
            request(touching(wall.at, wall.n, toward, m.footprint), Pose{1.2, 2.6, 0.3}, m, f);
        const PlanResult p = kPlanner.plan(r);
        ASSERT_EQ(p.status, PlanStatus::kOk);
        const PathSegment& escape = p.path.segments.front();
        EXPECT_TRUE(escape.exact);
        EXPECT_TRUE(escape.reverse);
        const Point move{escape.end.x - escape.start.x, escape.end.y - escape.start.y};
        EXPECT_GT(move.x * wall.n.x + move.y * wall.n.y, 0.0);
        expectExecutable(p.path, r);
        expectFootprintsClear(p.path, r);

        r.allow_reverse = false;
        EXPECT_EQ(kPlanner.plan(r).status, PlanStatus::kStartOutOfBounds);
        r.allow_reverse = true;
        r.model.reverse = false;
        EXPECT_EQ(kPlanner.plan(r).status, PlanStatus::kStartOutOfBounds);
    }
}

TEST(PlannerExit, StartInACorner) {
    const Field       f = field({});
    const MotionModel m = tank();
    // Touching both walls of the south-west corner.
    const Pose out{0.15 * std::sqrt(2.0), 0.15 * std::sqrt(2.0), kPi / 4.0};
    ASSERT_NEAR(gapAt(out, m, f), 0.0, 1e-12);
    const PlanRequest r = request(out, Pose{2, 2, 0}, m, f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.segments.front().exact);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);

    // Square in the corner, facing along the south wall: no straight way out.
    EXPECT_EQ(kPlanner.plan(request(Pose{0.15, 0.15, 0}, Pose{2, 2, 0}, m, f)).status,
              PlanStatus::kStartOutOfBounds);
    // Facing out along the diagonal would be; so is a holonomic sideways step.
    EXPECT_EQ(kPlanner.plan(request(Pose{0.15, 0.15, 0}, Pose{2, 2, 0}, mecanum(), f)).status,
              PlanStatus::kOk);
}

TEST(PlannerExit, GoalBesideALandmark) {
    // Landmark box 0.3 m square at (2, 2); goals 1 cm from its faces.
    FieldObject goal_box = obstacle(4, Pose{2, 2, 0}, 0.3, 0.3);
    goal_box.kind        = ObjectKind::kLandmark;
    const Field f        = field({goal_box});
    const Pose  start{0.5, 0.5, kPi / 2.0}; // no straight exact move to the goals

    for (const MotionModel& m : {tank(), mecanum()}) {
        SCOPED_TRACE(m.holonomic ? "holonomic" : "tank");
        // Facing the landmark: arrive driving forward from behind the goal.
        const PlanRequest facing = request(start, Pose{1.69, 2, 0}, m, f);
        PlanResult        p      = kPlanner.plan(facing);
        ASSERT_EQ(p.status, PlanStatus::kOk) << toString(p.status);
        const PathSegment& in = p.path.segments.back();
        EXPECT_TRUE(in.exact);
        EXPECT_EQ(in.kind, SegmentKind::kTranslate);
        EXPECT_FALSE(in.reverse);
        EXPECT_DOUBLE_EQ(in.end.x, 1.69);
        EXPECT_DOUBLE_EQ(in.end.y, 2.0);
        EXPECT_NEAR(in.start.y, 2.0, 1e-12);
        EXPECT_LT(in.start.x, in.end.x);
        EXPECT_NEAR(in.start.heading, 0.0, 1e-12);
        EXPECT_GE(2.0 - 0.15 - in.start.x, enclosingRadius(m));
        expectExecutable(p.path, facing);
        expectFootprintsClear(p.path, facing);
        EXPECT_TRUE(kPlanner.clear(p.path, f, m));
    }

    // Back to the landmark: a tank reverses in, a holonomic drive slides back.
    const PlanRequest back = request(start, Pose{2.31, 2, 0}, tank(), f);
    PlanResult        p    = kPlanner.plan(back);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.segments.back().exact);
    EXPECT_TRUE(p.path.segments.back().reverse);
    expectExecutable(p.path, back);
    expectFootprintsClear(p.path, back);

    PlanRequest no_reverse   = back;
    no_reverse.allow_reverse = false;
    p                        = kPlanner.plan(no_reverse);
    EXPECT_EQ(p.status, PlanStatus::kGoalBlocked);
    EXPECT_EQ(p.blocking, 4);

    // A correction putting the landmark 1 cm under the goal footprint: the
    // approach alone is no longer clear.
    p = kPlanner.plan(back);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    const Path approach{PlanMode::kAvoiding, {p.path.segments.back()}};
    EXPECT_TRUE(kPlanner.clear(approach, f, tank()));
    Field moved             = f;
    moved.generation        = 2;
    moved.objects[0].source = EstimateSource::kObserved;
    moved.objects[0].pose.x = 2.02;
    EXPECT_FALSE(kPlanner.clear(approach, moved, tank()));

    // A holonomic drive has four approaches at heading 0; from the south-west
    // the shortest whole route slides in sideways from below, along the face.
    const PlanRequest slide = request(start, Pose{2.31, 2, 0}, mecanum(), f);
    p                       = kPlanner.plan(slide);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    const PathSegment& in = p.path.segments.back();
    EXPECT_TRUE(in.exact);
    EXPECT_FALSE(in.reverse);
    EXPECT_DOUBLE_EQ(in.start.x, in.end.x);
    EXPECT_LT(in.start.y, in.end.y);
    EXPECT_NEAR(in.start.heading, 0.0, 1e-12);
    expectExecutable(p.path, slide);
    expectFootprintsClear(p.path, slide);
    // From the north-east, turned so no single straight move gets there, it
    // slides back in from the east instead.
    const PlanRequest east = request(Pose{3.5, 3.5, kPi / 2.0}, Pose{2.31, 2, 0}, mecanum(), f);
    p                      = kPlanner.plan(east);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    const PathSegment& last = p.path.segments.back();
    EXPECT_TRUE(last.exact);
    EXPECT_GT(last.start.x, last.end.x);
    EXPECT_DOUBLE_EQ(last.start.y, last.end.y);
    expectExecutable(p.path, east);
    expectFootprintsClear(p.path, east);
}

TEST(PlannerExit, EscapeBlockedBySecondBox) {
    // Box 1 touches the robot's back, box 2 is 0.1 m ahead: no straight
    // escape reaches a point clear of both within 2R.
    const Field f =
        field({obstacle(1, Pose{2, 0.75, 0}, 0.2, 0.2), obstacle(2, Pose{2, 1.35, 0}, 0.2, 0.2)});
    const Pose start{2, 1, kPi / 2.0};
    const Pose goal{3, 3, 0};
    ASSERT_NEAR(gapAt(start, tank(), f), 0.0, 1e-12);

    PlanResult p = kPlanner.plan(request(start, goal, tank(), f));
    EXPECT_EQ(p.status, PlanStatus::kStartBlocked);
    EXPECT_EQ(p.blocking, 1);
    EXPECT_TRUE(p.path.empty());

    // Sideways is open for a holonomic drive.
    const PlanRequest r = request(start, goal, mecanum(), f);
    p                   = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    const PathSegment& escape = p.path.segments.front();
    EXPECT_TRUE(escape.exact);
    EXPECT_NEAR(escape.end.y, 1.0, 1e-12);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);

    // A post in the robot's way out: the point past it is clear, the sweep
    // to it is not.
    const Field post = field(
        {obstacle(1, Pose{2, 0.75, 0}, 0.2, 0.2), obstacle(3, Pose{2.13, 1.2, 0}, 0.02, 0.02)});
    ASSERT_GT(gapAt(start, tank(), post), 0.0 - 1e-12);
    p = kPlanner.plan(request(start, goal, tank(), post));
    EXPECT_EQ(p.status, PlanStatus::kStartBlocked);
    EXPECT_EQ(p.blocking, 1);
    // Without the post the same tank drives straight out past box 2's corner.
    const Field open =
        field({obstacle(1, Pose{2, 0.75, 0}, 0.2, 0.2), obstacle(3, Pose{2.3, 1.2, 0}, 0.2, 0.1)});
    p = kPlanner.plan(request(start, goal, tank(), open));
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.segments.front().exact);
}

TEST(PlannerExit, ExitsAreAtMostTwoRadiiLong) {
    // A tank in a corridor 5 cm wider than it on each side, facing along it.
    const Pose start{2, 2, 0};
    const Pose goal{3.5, 3.5, 0};
    for (const Meters length : {0.6, 1.2}) {
        const Field f = field(
            {obstacle(1, Pose{2, 1.7, 0}, length, 0.2), obstacle(2, Pose{2, 2.3, 0}, length, 0.2)});
        const PlanResult p = kPlanner.plan(request(start, goal, tank(), f));
        if (length < 1.0) {
            // Out of the end within 2R.
            ASSERT_EQ(p.status, PlanStatus::kOk);
            const PathSegment& escape = p.path.segments.front();
            EXPECT_TRUE(escape.exact);
            EXPECT_LE(std::hypot(escape.end.x - escape.start.x, escape.end.y - escape.start.y),
                      2.0 * enclosingRadius(tank()));
        } else {
            EXPECT_EQ(p.status, PlanStatus::kStartBlocked);
            EXPECT_EQ(p.blocking, 1);
        }
    }
}

TEST(PlannerExit, ExactFootprintMustBeClear) {
    const Field       f = field({obstacle(6, Pose{2, 2, 0}, 0.3, 0.3)});
    const MotionModel m = tank();

    // 1 mm into the south wall, and exactly touching it.
    PlanResult p = kPlanner.plan(request(Pose{2, 0.149, kPi / 2.0}, Pose{1, 3, 0}, m, f));
    EXPECT_EQ(p.status, PlanStatus::kStartOutOfBounds);
    EXPECT_EQ(p.blocking, 0);
    EXPECT_EQ(kPlanner.plan(request(Pose{2, 0.15, kPi / 2.0}, Pose{1, 3, 0}, m, f)).status,
              PlanStatus::kOk);

    // Goal footprint 1 mm into the box, and touching it.
    p = kPlanner.plan(request(Pose{0.5, 0.5, 0}, Pose{1.701, 2, 0}, m, f));
    EXPECT_EQ(p.status, PlanStatus::kGoalBlocked);
    EXPECT_EQ(p.blocking, 6);
    EXPECT_EQ(kPlanner.plan(request(Pose{0.5, 0.5, 0}, Pose{1.7, 2, 0}, m, f)).status,
              PlanStatus::kOk);

    // Start overlapping a box.
    p = kPlanner.plan(request(Pose{2, 2.29, kPi / 2.0}, Pose{1, 3, 0}, m, f));
    EXPECT_EQ(p.status, PlanStatus::kStartBlocked);
    EXPECT_EQ(p.blocking, 6);
}

TEST(PlannerExit, StraightExactMoves) {
    const Field       f = field({});
    const MotionModel t = tank();
    const Pose        start{2, 0.15, kPi / 2.0}; // touching the south wall

    // Short hop off the wall, still inside the band: one exact move.
    PlanRequest r = request(start, Pose{2, 0.2, kPi / 2.0}, t, f);
    PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_TRUE(p.path.segments[0].exact);
    expectExecutable(p.path, r);

    // Already there, next to the wall: nothing to do.
    p = kPlanner.plan(request(start, start, t, f));
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.empty());

    // Backing into the wall is refused.
    p = kPlanner.plan(request(start, Pose{2, 0.1, kPi / 2.0}, t, f));
    EXPECT_EQ(p.status, PlanStatus::kGoalOutOfBounds);

    // A holonomic drive slides along the wall within 2R of both ends.
    const MotionModel m = mecanum();
    ASSERT_LT(0.4, 2.0 * enclosingRadius(m));
    r = request(start, Pose{2.4, 0.15, kPi / 2.0}, m, f);
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 1u);
    EXPECT_TRUE(p.path.segments[0].exact);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);
    // Farther, the middle of the slide would ride the wall: out, across, in.
    ASSERT_GT(1.0, 4.0 * enclosingRadius(m));
    r = request(start, Pose{3, 0.15, kPi / 2.0}, m, f);
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 3u);
    EXPECT_TRUE(p.path.segments[0].exact);
    EXPECT_FALSE(p.path.segments[1].exact);
    EXPECT_TRUE(p.path.segments[2].exact);
    EXPECT_GE(p.path.segments[1].start.y, enclosingRadius(m));
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);

    // Turning in place by the wall needs room: out, turn, back in.
    r = request(start, Pose{2, 0.15, 0}, m, f);
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.segments.front().exact);
    EXPECT_TRUE(p.path.segments.back().exact);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);
    EXPECT_EQ(kPlanner.plan(request(start, Pose{2, 0.15, 0}, t, f)).status,
              PlanStatus::kGoalOutOfBounds);
}

TEST(PlannerExit, CollinearExitsJoinTheRoute) {
    FieldObject       goal_box = obstacle(4, Pose{2, 3, 0}, 0.3, 0.3);
    const Field       f        = field({goal_box});
    const MotionModel t        = tank();

    // Off the wall and straight on to a point in the open, then a turn.
    PlanRequest r = request(Pose{2, 0.15, kPi / 2.0}, Pose{2, 1.5, 0}, t, f);
    PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 2u);
    EXPECT_TRUE(p.path.segments[0].exact);
    EXPECT_NEAR(p.path.segments[0].end.y, 1.5, 1e-12);
    EXPECT_EQ(p.path.segments[1].kind, SegmentKind::kTurn);
    expectExecutable(p.path, r);

    // A turn, then one move from the open into the goal beside the box.
    r = request(Pose{2, 1, 0}, Pose{2, 3 - 0.15 - 0.01 - 0.15, kPi / 2.0}, t, f);
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 2u);
    EXPECT_EQ(p.path.segments[0].kind, SegmentKind::kTurn);
    EXPECT_TRUE(p.path.segments[1].exact);
    EXPECT_NEAR(p.path.segments[1].start.y, 1.0, 1e-12);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);
}

TEST(PlannerExit, ClearRevalidatesExactSegments) {
    const Field       f = field({obstacle(8, Pose{3, 3, 0}, 0.3, 0.3)});
    const MotionModel m = tank();
    const PlanRequest r = request(Pose{2, 0.15, kPi / 2.0}, Pose{1, 2.5, 0}, m, f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_TRUE(p.path.segments.front().exact);
    EXPECT_TRUE(kPlanner.clear(p.path, f, m));

    // The same escape checked with the circle would fail at the wall.
    Path circle                   = p.path;
    circle.segments.front().exact = false;
    EXPECT_FALSE(kPlanner.clear(circle, f, m));

    // Remaining path from a measured pose partway out, a little off line.
    Path rest = p.path;
    rest.segments.front().start.x += 0.005;
    rest.segments.front().start.y = 0.18;
    EXPECT_TRUE(kPlanner.clear(rest, f, m));

    // Measured 1 mm into the wall while leaving it: no deeper, still clear.
    rest.segments.front().start.y = 0.149;
    EXPECT_TRUE(kPlanner.clear(rest, f, m));
    // Ending in the wall is not.
    Path into                     = p.path;
    into.segments.front().end.y   = 0.149;
    into.segments.front().start.y = 0.149;
    EXPECT_FALSE(kPlanner.clear(into, f, m));

    // A box corrected onto the way out.
    Field moved             = f;
    moved.generation        = 2;
    moved.objects[0].source = EstimateSource::kObserved;
    moved.objects[0].pose   = Pose{2, 0.45, 0};
    EXPECT_FALSE(kPlanner.clear(p.path, moved, m));
    // Beside the robot, 1 cm from it already: moving along it keeps the gap,
    // so the escape stays clear; the route after it does not.
    moved.objects[0].pose = Pose{2.15 + 0.15 + 0.01, 0.3, 0};
    EXPECT_TRUE(kPlanner.clear(Path{p.path.mode, {p.path.segments.front()}}, moved, m));
    EXPECT_FALSE(kPlanner.clear(p.path, moved, m));

    // Two exact moves meeting 1 cm from a box: the start and goal poses are
    // clear of it, so the meeting point must keep the clearance.
    PathSegment first;
    first.start        = Pose{1, 1, 0};
    first.end          = Pose{2, 1, 0};
    first.exact        = true;
    PathSegment second = first;
    second.start       = first.end;
    second.end         = Pose{2, 2, 0};
    const Field near   = field({obstacle(8, Pose{2.35 + 0.01, 1, 0}, 0.4, 0.2)});
    EXPECT_FALSE(kPlanner.clear(Path{PlanMode::kAvoiding, {first, second}}, near, m));

    // An exact move with a heading change is not something the planner makes.
    Path turning = p.path;
    turning.segments.front().end.heading += 0.1;
    EXPECT_FALSE(kPlanner.clear(turning, f, m));
}

TEST(PlannerExit, ClearKeepsTheClearanceAwayFromTheEnds) {
    // Off the wall and on to (2, 1.5) in one exact move, then a turn.
    const Field       f = field({});
    const MotionModel m = tank();
    const PlanRequest r = request(Pose{2, 0.15, kPi / 2.0}, Pose{2, 1.5, 0}, m, f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_EQ(p.path.segments.size(), 2u);
    ASSERT_TRUE(p.path.segments[0].exact);

    // A box corrected to 1 cm beside the middle of the move: nearer than the
    // clearance where the robot was not already that near.
    Field moved      = field({obstacle(9, Pose{2.15 + 0.01 + 0.1, 0.9, 0}, 0.2, 0.2)});
    moved.generation = 2;
    EXPECT_FALSE(kPlanner.clear(p.path, moved, m));
    moved.objects[0].pose.x += 0.02;
    EXPECT_TRUE(kPlanner.clear(p.path, moved, m));
}

namespace
{

// A channel 0.4 m wide between x 1.6 and 2.0 opening east into a closed
// pocket; the open field lies west of it.
Field pocketField() {
    return field({obstacle(1, Pose{1.8, 1.65, 0}, 0.4, 0.1), obstacle(2, Pose{1.8, 2.15, 0}, 0.4, 0.1),
                  obstacle(3, Pose{2.9, 1.9, 0}, 0.1, 1.5), obstacle(4, Pose{2.275, 2.6, 0}, 1.35, 0.1),
                  obstacle(5, Pose{2.275, 1.2, 0}, 1.35, 0.1)},
                 Bounds{0, 0, 3.6, 3.6});
}

} // namespace

TEST(PlannerExit, EveryExitIsTried) {
    const Field       f = pocketField();
    const MotionModel m = tank();
    const Pose        open{0.8, 1.0, kPi / 2.0};
    ASSERT_EQ(kPlanner.plan(request(Pose{2.5, 1.9, 0}, open, m, f)).status, PlanStatus::kNoPath);

    // Facing the pocket: the forward exit is the shorter one but leads into
    // the pocket; backing out west leads to the goal.
    PlanRequest r = request(Pose{1.9, 1.9, 0}, open, m, f);
    PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk) << toString(p.status);
    EXPECT_TRUE(p.path.segments.front().exact);
    EXPECT_TRUE(p.path.segments.front().reverse);
    EXPECT_LT(p.path.segments.front().end.x, 1.6);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);
    EXPECT_TRUE(kPlanner.clear(p.path, f, m));
    // Forward only: the pocket is all there is.
    r.allow_reverse = false;
    EXPECT_EQ(kPlanner.plan(r).status, PlanStatus::kNoPath);

    // Facing out, the same exit is driven forward.
    r = request(Pose{1.9, 1.9, kPi}, open, m, f);
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.segments.front().exact);
    EXPECT_FALSE(p.path.segments.front().reverse);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);

    // Mirrored at the goal: the approach from the pocket side is shorter but
    // unreachable; backing in from the west is not.
    r = request(open, Pose{1.9, 1.9, kPi}, m, f);
    p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk) << toString(p.status);
    EXPECT_TRUE(p.path.segments.back().exact);
    EXPECT_TRUE(p.path.segments.back().reverse);
    EXPECT_LT(p.path.segments.back().start.x, 1.6);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);
    EXPECT_TRUE(kPlanner.clear(p.path, f, m));
}

TEST(PlannerExit, LaterStretchesAlongOneMove) {
    // Rows 0.075 m beside the robot's sides: box pair A along it, then a gap,
    // then a thin pair B. Driving forward, the first point clear of the
    // circle is a closed sliver between A and B; the open field starts past B.
    const Field f =
        field({obstacle(1, Pose{0.95, 2.275, 0}, 0.3, 0.1), obstacle(2, Pose{0.95, 1.725, 0}, 0.3, 0.1),
               obstacle(3, Pose{1.34, 2.275, 0}, 0.02, 0.1), obstacle(4, Pose{1.34, 1.725, 0}, 0.02, 0.1)});
    const MotionModel m = tank();
    PlanRequest       r = request(Pose{1.09, 2, 0}, Pose{3, 2, 0}, m, f);
    r.allow_reverse     = false;
    ASSERT_NEAR(gapAt(r.start, m, f), 0.075, 1e-12);

    const PlanResult p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk) << toString(p.status);
    const PathSegment& out = p.path.segments.front();
    EXPECT_TRUE(out.exact);
    EXPECT_GT(out.end.x, 1.35 + 0.1);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);
    EXPECT_TRUE(kPlanner.clear(p.path, f, m));
}

TEST(PlannerExit, AllowanceOnlyNearTheEnds) {
    const Field       f    = field({});
    const MotionModel m    = tank(kSquare, 0.05);
    const Meters      zone = 2.0 * enclosingRadius(m);
    const Pose        start{0.3, 0.15, 0}; // touching the south wall, along it

    // Along the wall: every point within 2R of one end, else refused, since a
    // tank there can only drive along it.
    for (const Meters length : {zone - 0.01, 2.0 * zone - 0.01, 2.0 * zone + 0.01, 3.0}) {
        SCOPED_TRACE(length);
        const PlanRequest r = request(start, Pose{0.3 + length, 0.15, 0}, m, f);
        const PlanResult  p = kPlanner.plan(r);
        if (length < 2.0 * zone) {
            ASSERT_EQ(p.status, PlanStatus::kOk);
            ASSERT_EQ(p.path.segments.size(), 1u);
            EXPECT_TRUE(p.path.segments[0].exact);
            expectFootprintsClear(p.path, r);
        } else {
            EXPECT_EQ(p.status, PlanStatus::kStartOutOfBounds);
            EXPECT_EQ(p.blocking, 0);
        }
    }
    PathSegment ride;
    ride.start = start;
    ride.end   = Pose{3.3, 0.15, 0};
    ride.exact = true;
    EXPECT_FALSE(kPlanner.clear(Path{PlanMode::kAvoiding, {ride}}, f, m));
    ride.end.x = 0.3 + 2.0 * zone - 0.01;
    EXPECT_TRUE(kPlanner.clear(Path{PlanMode::kAvoiding, {ride}}, f, m));

    // 5 mm above a 2 m barrier, goal clear past its far end: backing out past
    // the near end, then around with the full clearance.
    const Field bar = field({obstacle(1, Pose{1.8, 1.0, 0}, 2.0, 0.1)});
    PlanRequest r   = request(Pose{0.9, 1.205, 0}, Pose{3.2, 1.205, 0}, m, bar);
    ASSERT_NEAR(gapAt(r.start, m, bar), 0.005, 1e-12);
    PlanResult p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    EXPECT_TRUE(p.path.segments.front().exact);
    EXPECT_TRUE(p.path.segments.front().reverse);
    EXPECT_LT(p.path.segments.front().end.x, 0.8);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);
    // Forward only there is no way off the barrier.
    r.allow_reverse = false;
    EXPECT_EQ(kPlanner.plan(r).status, PlanStatus::kStartBlocked);
}

TEST(PlannerExit, TurnOntoAnApproachIsKept) {
    // The goal touches the box face; the route ends 8 mrad off the goal
    // heading, below min_turn, and the approach was swept at the goal heading.
    const Field       f = field({obstacle(4, Pose{2, 2, 0}, 0.3, 0.3)});
    const MotionModel m = tank();
    const PlanRequest r = request(Pose{0.6, 2.008, 0}, Pose{1.7, 2, 0}, m, f);
    const PlanResult  p = kPlanner.plan(r);
    ASSERT_EQ(p.status, PlanStatus::kOk);
    ASSERT_GE(p.path.segments.size(), 2u);
    const PathSegment& in   = p.path.segments.back();
    const PathSegment& turn = p.path.segments[p.path.segments.size() - 2];
    EXPECT_TRUE(in.exact);
    ASSERT_EQ(turn.kind, SegmentKind::kTurn);
    const Radians by = std::fabs(wrapAngle(turn.end.heading - turn.start.heading));
    EXPECT_GT(by, 0.0);
    EXPECT_LT(by, GeometricPlannerConfig{}.min_turn);
    expectExecutable(p.path, r);
    expectFootprintsClear(p.path, r);

    // A straight exact move keeps the start heading; the goal heading is
    // within min_turn of it.
    const Field       open = field({});
    const PlanRequest s = request(Pose{2, 0.155, kPi / 2.0 + 0.005}, Pose{2, 0.3, kPi / 2.0}, mecanum(), open);
    const PlanResult  q = kPlanner.plan(s);
    ASSERT_EQ(q.status, PlanStatus::kOk);
    ASSERT_EQ(q.path.segments.size(), 1u);
    EXPECT_TRUE(q.path.segments[0].exact);
    EXPECT_DOUBLE_EQ(q.path.segments[0].start.heading, s.start.heading);
    EXPECT_DOUBLE_EQ(q.path.segments[0].end.heading, s.start.heading);
    expectExecutable(q.path, s);
    expectFootprintsClear(q.path, s);
}
