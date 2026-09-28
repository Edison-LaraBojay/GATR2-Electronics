// planner_random_gtest.cpp
// Randomized fields with fixed seeds: every plan is checked against exact
// geometry, and every refusal against its stated reason.

#include "investigatr/planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace investigatr;

namespace
{

class Rng {
  public:
    explicit Rng(unsigned seed) : gen_(seed) {}

    double uniform(double lo, double hi) {
        return std::uniform_real_distribution<double>(lo, hi)(gen_);
    }
    int  integer(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(gen_); }
    bool chance(double p) { return uniform(0, 1) < p; }

  private:
    std::mt19937 gen_;
};

const Bounds kBounds{0, 0, 3.6, 3.6};

Field randomField(Rng& rng, int max_objects) {
    Field f;
    f.generation = 1;
    f.map.id     = 0x0D15EA5E;
    f.bounds     = kBounds;
    const int n  = rng.integer(0, max_objects);
    for (int i = 0; i < n; ++i) {
        FieldObject o;
        o.id         = static_cast<ObjectId>(i + 1);
        o.kind       = rng.chance(0.6) ? ObjectKind::kLandmark : ObjectKind::kFixed;
        o.obstacle   = rng.chance(0.85);
        o.nominal    = Pose{rng.uniform(-0.2, 3.8), rng.uniform(-0.2, 3.8), rng.uniform(-kPi, kPi)};
        o.box.length = rng.uniform(0.0, 0.7);
        o.box.width  = rng.uniform(0.0, 0.7);
        if (rng.chance(0.4)) {
            o.box.center =
                Pose{rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(-kPi, kPi)};
        }
        if (o.kind == ObjectKind::kLandmark && rng.chance(0.5)) {
            o.estimated = true;
            o.source    = EstimateSource::kObserved;
            o.valid     = true;
            o.pose      = compose(o.nominal, Pose{rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1),
                                                  rng.uniform(-0.2, 0.2)});
        } else if (o.kind == ObjectKind::kLandmark && rng.chance(0.3)) {
            // No usable estimate: the nominal pose counts; the stale pose must not.
            o.estimated = true;
            o.valid     = false;
            o.pose      = Pose{rng.uniform(0, 3.6), rng.uniform(0, 3.6), 0};
        } else {
            o.source = EstimateSource::kNominal;
            o.valid  = true;
            o.pose   = o.nominal;
        }
        f.objects.push_back(o);
    }
    return f;
}

MotionModel randomModel(Rng& rng) {
    MotionModel m;
    m.holonomic     = rng.chance(0.5);
    m.turn_in_place = true;
    m.reverse       = rng.chance(0.7);
    m.footprint = Footprint{rng.uniform(0.03, 0.3), rng.uniform(0.03, 0.3), rng.uniform(0.03, 0.25),
                            rng.uniform(0.03, 0.25)};
    m.clearance = rng.uniform(0.0, 0.05);
    return m;
}

Pose randomPose(Rng& rng) {
    return Pose{rng.uniform(0.05, 3.55), rng.uniform(0.05, 3.55), rng.uniform(-kPi, kPi)};
}

Pose obstaclePose(const FieldObject& o) { return o.valid ? o.pose : o.nominal; }

// Real box of o, local coordinates of p.
Point boxLocal(const FieldObject& o, Point p) {
    const Pose local = between(compose(obstaclePose(o), o.box.center), Pose{p.x, p.y, 0});
    return Point{local.x, local.y};
}

double boxDistance(const FieldObject& o, Point p) {
    const Point  l  = boxLocal(o, p);
    const double dx = std::max(std::fabs(l.x) - 0.5 * o.box.length, 0.0);
    const double dy = std::max(std::fabs(l.y) - 0.5 * o.box.width, 0.0);
    return std::hypot(dx, dy);
}

double pointSegment(Point p, Point a, Point b) {
    const double dx   = b.x - a.x;
    const double dy   = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    const double t =
        len2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0) : 0.0;
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

// Exact distance from segment a-b to the real box of o, 0 when they meet.
double segmentBoxDistance(const FieldObject& o, Point a, Point b) {
    const Point  la      = boxLocal(o, a);
    const Point  lb      = boxLocal(o, b);
    const double half[2] = {0.5 * o.box.length, 0.5 * o.box.width};
    const double from[2] = {la.x, la.y};
    const double step[2] = {lb.x - la.x, lb.y - la.y};
    double       lo      = 0;
    double       hi      = 1;
    bool         meets   = true;
    for (int k = 0; k < 2 && meets; ++k) {
        if (step[k] == 0) {
            meets = std::fabs(from[k]) <= half[k];
            continue;
        }
        double t0 = (-half[k] - from[k]) / step[k];
        double t1 = (half[k] - from[k]) / step[k];
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        lo = std::max(lo, t0);
        hi = std::min(hi, t1);
    }
    if (meets && lo <= hi) {
        return 0;
    }
    double      best     = std::min(boxDistance(o, a), boxDistance(o, b));
    const Point local[4] = {
        {half[0], half[1]}, {-half[0], half[1]}, {-half[0], -half[1]}, {half[0], -half[1]}};
    for (const Point& c : local) {
        best = std::min(best, pointSegment(c, la, lb));
    }
    return best;
}

double boundsDistance(Point p) {
    return std::min(
        {p.x - kBounds.min_x, kBounds.max_x - p.x, p.y - kBounds.min_y, kBounds.max_y - p.y});
}

// First reason path is not executable from request, empty when it is. A
// dropped turn may leave a heading jump below min_turn, except into an exact
// move, which was swept at its own heading only.
std::string executionProblem(const Path& path, const PlanRequest& r,
                             const GeometricPlannerConfig& c) {
    std::ostringstream why;
    const Radians      tol = c.min_turn + 1e-9;
    Pose               at  = r.start;
    for (std::size_t i = 0; i < path.segments.size(); ++i) {
        const PathSegment& s = path.segments[i];
        if (std::hypot(s.start.x - at.x, s.start.y - at.y) > 1e-12) {
            why << "segment " << i << " not chained";
            return why.str();
        }
        if (std::fabs(wrapAngle(s.start.heading - at.heading)) > (s.exact ? 1e-9 : tol)) {
            why << "segment " << i << " heading jump";
            return why.str();
        }
        const bool before_exact = i + 1 < path.segments.size() && path.segments[i + 1].exact;
        if (s.kind == SegmentKind::kTurn) {
            const Radians by = wrapAngle(s.end.heading - s.start.heading);
            if (s.end.x != s.start.x || s.end.y != s.start.y ||
                std::fabs(by) < (before_exact ? 1e-9 : c.min_turn) ||
                s.turn_direction != (by > 0 ? 1 : -1)) {
                why << "segment " << i << " bad turn";
                return why.str();
            }
        } else if (!r.model.holonomic) {
            const Radians travel = std::atan2(s.end.y - s.start.y, s.end.x - s.start.x);
            const Radians facing = s.reverse ? wrapAngle(travel + kPi) : travel;
            if (std::fabs(wrapAngle(s.start.heading - facing)) > 1e-9 ||
                std::fabs(wrapAngle(s.end.heading - facing)) > 1e-9) {
                why << "segment " << i << " not along its heading";
                return why.str();
            }
            if (s.reverse && !(r.model.reverse && r.allow_reverse)) {
                why << "segment " << i << " reverse not allowed";
                return why.str();
            }
        } else if (s.reverse) {
            why << "segment " << i << " holonomic reverse";
            return why.str();
        }
        at = s.end;
    }
    // A last leg shorter than min_segment may be dropped.
    const double miss = std::hypot(at.x - r.goal.x, at.y - r.goal.y);
    if (miss > 1e-12 && miss >= c.min_segment) {
        why << "ends " << miss << " m from the goal";
        return why.str();
    }
    if (std::fabs(wrapAngle(at.heading - r.goal.heading)) > tol) {
        why << "ends off the goal heading";
        return why.str();
    }
    return {};
}

// Robot poses along a segment, as a follower would drive it.
std::vector<Pose> samples(const PathSegment& s, int n) {
    std::vector<Pose> out;
    const Radians     turn = wrapAngle(s.end.heading - s.start.heading);
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        out.push_back(Pose{s.start.x + t * (s.end.x - s.start.x),
                           s.start.y + t * (s.end.y - s.start.y),
                           wrapAngle(s.start.heading + t * turn)});
    }
    return out;
}

// Origin R + delta clear of the bounds and of every box grown as a rectangle,
// so every point within delta of it passes the planner's circle test.
bool gridFree(const Field& f, const MotionModel& m, Point p, double delta) {
    const double grow = enclosingRadius(m) + delta;
    if (p.x < kBounds.min_x + grow || p.x > kBounds.max_x - grow || p.y < kBounds.min_y + grow ||
        p.y > kBounds.max_y - grow) {
        return false;
    }
    for (const FieldObject& o : f.objects) {
        if (!o.obstacle) {
            continue;
        }
        const Point l = boxLocal(o, p);
        if (std::fabs(l.x) < 0.5 * o.box.length + grow &&
            std::fabs(l.y) < 0.5 * o.box.width + grow) {
            return false;
        }
    }
    return true;
}

// Grid search in the configuration space grown by a further delta, from any
// of from to any of to. A route here means the planner must find one.
bool gridRoute(const Field& f, const MotionModel& m, const std::vector<Point>& from,
               const std::vector<Point>& to, double delta) {
    const double h    = 0.02;
    const int    nx   = static_cast<int>(std::floor(kBounds.max_x / h));
    const int    ny   = static_cast<int>(std::floor(kBounds.max_y / h));
    auto         cell = [&](Point p) {
        const int ix = std::clamp(static_cast<int>(std::floor(p.x / h)), 0, nx - 1);
        const int iy = std::clamp(static_cast<int>(std::floor(p.y / h)), 0, ny - 1);
        return iy * nx + ix;
    };
    auto usable = [&](int c) {
        return gridFree(f, m, Point{(c % nx + 0.5) * h, (c / nx + 0.5) * h}, delta);
    };
    std::vector<char> seen(static_cast<std::size_t>(nx * ny), 0);
    std::vector<char> target(static_cast<std::size_t>(nx * ny), 0);
    std::deque<int>   queue;
    for (const Point& p : to) {
        target[static_cast<std::size_t>(cell(p))] = 1;
    }
    for (const Point& p : from) {
        const int c = cell(p);
        if (!seen[static_cast<std::size_t>(c)] && usable(c)) {
            seen[static_cast<std::size_t>(c)] = 1;
            queue.push_back(c);
        }
    }
    while (!queue.empty()) {
        const int c = queue.front();
        queue.pop_front();
        if (target[static_cast<std::size_t>(c)]) {
            return true;
        }
        const int x          = c % nx;
        const int y          = c / nx;
        const int next[4][2] = {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
        for (const auto& n : next) {
            if (n[0] < 0 || n[0] >= nx || n[1] < 0 || n[1] >= ny) {
                continue;
            }
            const int k = n[1] * nx + n[0];
            if (!seen[static_cast<std::size_t>(k)] && usable(k)) {
                seen[static_cast<std::size_t>(k)] = 1;
                queue.push_back(k);
            }
        }
    }
    return false;
}

// Exact footprint gap at p, negative when overlapping.
double gapAt(const Pose& p, const MotionModel& m, const Field& f) {
    return footprintClearance(p, m.footprint, 0.0, f).distance;
}

// Farthest footprint corner from the origin.
Meters reach(const MotionModel& m) { return enclosingRadius(m) - m.clearance; }

// One field per obstacle that comes within limit of the segment a-b, holding
// only it, with the bounds far away: gaps to single objects.
std::vector<Field> nearObstacles(const Field& f, Point a, Point b, Meters limit) {
    std::vector<Field> out;
    for (const FieldObject& o : f.objects) {
        if (o.obstacle && segmentBoxDistance(o, a, b) < limit) {
            Field one   = f;
            one.bounds  = Bounds{-100, -100, 100, 100};
            one.objects = {o};
            out.push_back(one);
        }
    }
    return out;
}

// Exact footprint gaps at p to each listed obstacle, then to the bound sides
// min x, max x, min y, max y.
std::vector<double> gaps(const Pose& p, const MotionModel& m, const std::vector<Field>& near) {
    std::vector<double> out;
    for (const Field& one : near) {
        out.push_back(footprintClearance(p, m.footprint, 0.0, one).distance);
    }
    const Footprint& fp      = m.footprint;
    const Point      body[4] = {
        {fp.front, fp.left}, {fp.front, -fp.right}, {-fp.back, fp.left}, {-fp.back, -fp.right}};
    double side[4] = {1e9, 1e9, 1e9, 1e9};
    for (const Point& c : body) {
        const Pose at = compose(p, Pose{c.x, c.y, 0});
        side[0]       = std::min(side[0], at.x - kBounds.min_x);
        side[1]       = std::min(side[1], kBounds.max_x - at.x);
        side[2]       = std::min(side[2], at.y - kBounds.min_y);
        side[3]       = std::min(side[3], kBounds.max_y - at.y);
    }
    out.insert(out.end(), side, side + 4);
    return out;
}

// Points the planner must be able to start or end its route at, for an end
// pose it has to leave or approach by a straight exact move: along each move
// the model allows at the end heading, points at most 2R away that the grid
// counts as free, swept to without coming nearer to any box or bound side
// than the end pose already is, or than the clearance. A gap already below
// clearance + 1 mm must grow from the start (gaps are convex along a
// translation, so it then never shrinks); the others must stay above
// clearance + 1 mm at samples 2 mm apart (gaps change at most as fast as the
// robot moves). A longer sweep holds a shorter one, so the first failure
// ends a move.
std::vector<Point> exitPoints(const Field& f, const PlanRequest& r, const Pose& end, bool leaving,
                              double delta) {
    const MotionModel& m      = r.model;
    const Meters       limit  = 2.0 * enclosingRadius(m);
    const double       margin = 0.001;
    const double       step   = 0.002;
    const Point        fwd{std::cos(end.heading), std::sin(end.heading)};
    const Point        left{-fwd.y, fwd.x};
    std::vector<Point> rays{leaving ? fwd : Point{-fwd.x, -fwd.y}};
    if (m.holonomic) {
        rays = {fwd, Point{-fwd.x, -fwd.y}, left, Point{-left.x, -left.y}};
    } else if (m.reverse && r.allow_reverse) {
        rays = {fwd, Point{-fwd.x, -fwd.y}};
    }

    std::vector<Point> out;
    const Point        at{end.x, end.y};
    for (const Point& u : rays) {
        const Point               far{at.x + limit * u.x, at.y + limit * u.y};
        const std::vector<Field>  near = nearObstacles(f, at, far, reach(m) + m.clearance + margin);
        const std::vector<double> g0   = gaps(end, m, near);
        const std::vector<double> g1 =
            gaps(Pose{at.x + 1e-8 * u.x, at.y + 1e-8 * u.y, end.heading}, m, near);
        std::vector<char> sampled(g0.size(), 0);
        bool              open = true;
        for (std::size_t k = 0; k < g0.size() && open; ++k) {
            sampled[k] = g0[k] >= m.clearance + margin;
            open       = sampled[k] || g1[k] - g0[k] >= 1e-9;
        }
        for (double d = step; open && d <= limit; d += step) {
            const Pose                q{at.x + d * u.x, at.y + d * u.y, end.heading};
            const std::vector<double> g = gaps(q, m, near);
            for (std::size_t k = 0; k < g.size() && open; ++k) {
                open = !sampled[k] || g[k] >= m.clearance + margin;
            }
            if (open && gridFree(f, m, Point{q.x, q.y}, delta)) {
                out.push_back(Point{q.x, q.y});
            }
        }
    }
    return out;
}

// Robot at heading h with its footprint a gap from a random wall.
Pose byWall(Rng& rng, const MotionModel& m, Radians h) {
    const Footprint& fp      = m.footprint;
    const Point      body[4] = {
        {fp.front, fp.left}, {fp.front, -fp.right}, {-fp.back, fp.left}, {-fp.back, -fp.right}};
    const int   wall  = rng.integer(0, 3);
    const Point n     = wall == 0   ? Point{0, 1}
                        : wall == 1 ? Point{0, -1}
                        : wall == 2 ? Point{1, 0}
                                    : Point{-1, 0};
    double      reach = 0;
    for (const Point& c : body) {
        const Pose at = compose(Pose{0, 0, h}, Pose{c.x, c.y, 0});
        reach         = std::max(reach, -(at.x * n.x + at.y * n.y));
    }
    const double off   = reach + (rng.chance(0.5) ? 0.0 : rng.uniform(0.0, 0.03));
    const double along = rng.uniform(0.4, 3.2);
    const Point  at    = n.y != 0 ? Point{along, n.y > 0 ? off : kBounds.max_y - off}
                                  : Point{n.x > 0 ? off : kBounds.max_x - off, along};
    return Pose{at.x, at.y, h};
}

// Robot near a random obstacle with its footprint clear of everything.
Pose byBox(Rng& rng, const Field& f, const MotionModel& m) {
    std::vector<const FieldObject*> boxes;
    for (const FieldObject& o : f.objects) {
        if (o.obstacle) {
            boxes.push_back(&o);
        }
    }
    Pose p = randomPose(rng);
    if (boxes.empty()) {
        return p;
    }
    const FieldObject& o =
        *boxes[static_cast<std::size_t>(rng.integer(0, static_cast<int>(boxes.size()) - 1))];
    const Pose   c     = compose(obstaclePose(o), o.box.center);
    const double reach = std::hypot(o.box.length, o.box.width) / 2 + enclosingRadius(m);
    for (int attempt = 0; attempt < 50; ++attempt) {
        p = Pose{c.x + rng.uniform(-reach, reach), c.y + rng.uniform(-reach, reach),
                 rng.uniform(-kPi, kPi)};
        if (gapAt(p, m, f) >= 0 && boxDistance(o, Point{p.x, p.y}) < enclosingRadius(m)) {
            return p;
        }
    }
    return p;
}

// Poses outside every grown box, so plans get past the end checks, and some
// by a wall or a box so they need an exact exit.
Pose endPose(Rng& rng, const Field& f, const MotionModel& m) {
    const Meters radius = enclosingRadius(m);
    Pose         p      = randomPose(rng);
    const double kind   = rng.uniform(0, 1);
    if (kind < 0.1) {
        return p;
    }
    if (kind < 0.25) {
        return byWall(rng, m,
                      rng.chance(0.5) ? rng.uniform(-kPi, kPi) : rng.integer(-1, 2) * kPi / 2);
    }
    if (kind < 0.4) {
        return byBox(rng, f, m);
    }
    for (int attempt = 0; attempt < 50; ++attempt, p = randomPose(rng)) {
        bool free = boundsDistance(Point{p.x, p.y}) >= radius;
        for (const FieldObject& o : f.objects) {
            if (free && o.obstacle) {
                const Point l = boxLocal(o, Point{p.x, p.y});
                free          = std::fabs(l.x) >= 0.5 * o.box.length + radius ||
                                std::fabs(l.y) >= 0.5 * o.box.width + radius;
            }
        }
        if (free) {
            return p;
        }
    }
    return p;
}

struct Tally {
    int ok            = 0;
    int blocked       = 0;
    int bounds        = 0;
    int no_path       = 0;
    int detoured      = 0;
    int searched      = 0; // no path results confirmed by the grid search
    int exit_searched = 0; // of those, searched from or to exit points
    int exits_checked = 0; // end refusals confirmed by finding no exit
    int escaped       = 0; // plans leaving a start that fails the circle test
    int arrived       = 0; // plans reaching a goal that fails the circle test
};

// Origin at least R from the bounds and every box. The planner's grown boxes
// hold every point nearer than R, so it must exit from any end failing this.
bool circleFree(const Field& f, Point p, Meters radius) {
    if (boundsDistance(p) < radius - 1e-9) {
        return false;
    }
    for (const FieldObject& o : f.objects) {
        if (o.obstacle && boxDistance(o, p) < radius - 1e-9) {
            return false;
        }
    }
    return true;
}

enum class EndUse : uint8_t { kUnclear, kItself, kExits };

// Where the grid may begin or end a route for a plan end: the end itself when
// the grid counts it free, its exit points when the planner surely has to
// leave or approach it by an exact move.
EndUse routeEnds(const Field& f, const PlanRequest& r, const Pose& end, bool leaving, double delta,
                 std::vector<Point>& out) {
    const Point at{end.x, end.y};
    if (gridFree(f, r.model, at, delta)) {
        out = {at};
        return EndUse::kItself;
    }
    if (!circleFree(f, at, enclosingRadius(r.model))) {
        out = exitPoints(f, r, end, leaving, delta);
        return EndUse::kExits;
    }
    return EndUse::kUnclear;
}

// Checks on a successful avoiding plan.
void checkPath(const Path& path, const PlanRequest& r, const GeometricPlanner& planner,
               Tally& tally, Rng& rng) {
    const Field& f      = *r.field;
    const Meters radius = enclosingRadius(r.model);
    const Meters zone   = 2.0 * radius;
    const Point  start{r.start.x, r.start.y};
    const Point  goal{r.goal.x, r.goal.y};
    ASSERT_TRUE(planner.clear(path, f, r.model));
    ASSERT_EQ(executionProblem(path, r, planner.config()), "");
    if (path.length() > 0) {
        EXPECT_GE(path.length() + 1e-12, std::hypot(goal.x - start.x, goal.y - start.y));
    }
    const bool escaping = !circleFree(f, start, radius);
    const bool arriving = !circleFree(f, goal, radius);
    if (!path.empty()) {
        EXPECT_TRUE(!escaping || path.segments.front().exact);
        EXPECT_TRUE(!arriving || path.segments.back().exact);
        tally.escaped += escaping ? 1 : 0;
        tally.arrived += arriving ? 1 : 0;
    }

    std::size_t translations = 0;
    for (std::size_t i = 0; i < path.segments.size(); ++i) {
        const PathSegment& s = path.segments[i];
        const Point        a{s.start.x, s.start.y};
        const Point        b{s.end.x, s.end.y};
        translations += s.kind == SegmentKind::kTranslate ? 1 : 0;
        if (s.exact) {
            // Swept exact footprint: the clearance from every box and bound
            // side, except within 2R of the path's start or goal pose, where
            // it may be as near to each as that pose already is.
            ASSERT_EQ(s.kind, SegmentKind::kTranslate);
            const std::vector<Field> near =
                nearObstacles(f, a, b, reach(r.model) + r.model.clearance + 1e-6);
            const std::vector<double> at_start = gaps(path.segments.front().start, r.model, near);
            const std::vector<double> at_goal  = gaps(path.segments.back().end, r.model, near);
            const bool                first    = i == 0;
            const bool                last     = i + 1 == path.segments.size();
            for (const Pose& at : samples(s, 96)) {
                const double              from_start = std::hypot(at.x - a.x, at.y - a.y);
                const double              from_goal  = std::hypot(b.x - at.x, b.y - at.y);
                const std::vector<double> g          = gaps(at, r.model, near);
                for (std::size_t k = 0; k < g.size(); ++k) {
                    double floor = r.model.clearance;
                    if (first && from_start <= zone) {
                        floor = std::min(floor, at_start[k]);
                    }
                    if (last && from_goal <= zone) {
                        floor = std::min(floor, at_goal[k]);
                    }
                    ASSERT_GE(g[k], floor - 1e-9) << "entry " << k << " of " << g.size();
                }
            }
            ASSERT_GE(gapAt(s.end, r.model, f), -1e-9);
            continue;
        }
        ASSERT_GE(boundsDistance(a), radius - 1e-9);
        ASSERT_GE(boundsDistance(b), radius - 1e-9);
        for (const FieldObject& o : f.objects) {
            if (o.obstacle) {
                ASSERT_GE(segmentBoxDistance(o, a, b), radius - 1e-9) << o.id;
            }
        }
        for (const Pose& at : samples(s, 24)) {
            const Clearance c = footprintClearance(at, r.model.footprint, r.model.clearance, f);
            ASSERT_GE(c.distance, -1e-9) << "near " << c.nearest;
        }
    }
    tally.detoured += translations > 1 ? 1 : 0;

    // Moving an obstacle onto any point of the path must be caught, unless
    // the robot already overlaps it at the start or goal pose of an exact move.
    if (path.empty()) {
        return;
    }
    for (std::size_t k = 0; k < f.objects.size(); ++k) {
        if (!f.objects[k].obstacle) {
            continue;
        }
        const std::size_t i =
            static_cast<std::size_t>(rng.integer(0, static_cast<int>(path.segments.size()) - 1));
        const PathSegment& s = path.segments[i];
        const double       t = rng.uniform(0, 1);
        const Pose on{s.start.x + t * (s.end.x - s.start.x), s.start.y + t * (s.end.y - s.start.y),
                      rng.uniform(-kPi, kPi)};
        Field      moved = f;
        FieldObject& o   = moved.objects[k];
        o.valid          = true;
        o.source         = EstimateSource::kObserved;
        o.pose           = compose(on, inverse(o.box.center));
        moved.generation = 2;
        Field only       = moved;
        only.objects     = {o};
        const bool excused =
            s.exact && ((i == 0 && gapAt(s.start, r.model, only) <= 0) ||
                        (i + 1 == path.segments.size() && gapAt(s.end, r.model, only) <= 0));
        if (!excused) {
            EXPECT_FALSE(planner.clear(path, moved, r.model)) << o.id;
        }
        break;
    }
}

} // namespace

TEST(PlannerRandom, AvoidingPathsAreClearAndRefusalsHonest) {
    const GeometricPlanner planner;
    const auto&            config = planner.config();
    Tally                  tally;
    for (unsigned seed = 1; seed <= 300; ++seed) {
        Rng         rng(seed);
        const Field f = randomField(rng, 30);
        for (int trial = 0; trial < 8; ++trial) {
            PlanRequest r;
            r.mode          = PlanMode::kAvoiding;
            r.model         = randomModel(rng);
            r.start         = endPose(rng, f, r.model);
            r.goal          = endPose(rng, f, r.model);
            r.allow_reverse = rng.chance(0.8);
            r.field         = &f;
            SCOPED_TRACE("seed " + std::to_string(seed) + " trial " + std::to_string(trial));

            const PlanResult p      = planner.plan(r);
            const Meters     radius = enclosingRadius(r.model);
            const Point      start{r.start.x, r.start.y};
            const Point      goal{r.goal.x, r.goal.y};
            const double     delta = config.vertex_margin + 0.02;

            // A refused end whose footprint is clear had no straight exit.
            auto noExit = [&](const Pose& end, bool leaving) {
                if (gapAt(end, r.model, f) >= 1e-6) {
                    ++tally.exits_checked;
                    EXPECT_TRUE(exitPoints(f, r, end, leaving, delta).empty())
                        << "the planner missed an exit";
                }
            };

            switch (p.status) {
            case PlanStatus::kOk:
                ++tally.ok;
                checkPath(p.path, r, planner, tally, rng);
                break;
            case PlanStatus::kStartBlocked:
            case PlanStatus::kGoalBlocked: {
                // Within R of the named box: the exact footprint overlaps it,
                // or no straight exit gets clear.
                ++tally.blocked;
                const bool         at_start = p.status == PlanStatus::kStartBlocked;
                const Point        at       = at_start ? start : goal;
                const FieldObject* o        = f.find(p.blocking);
                ASSERT_NE(o, nullptr);
                ASSERT_TRUE(o->obstacle);
                const Point l = boxLocal(*o, at);
                EXPECT_LT(std::fabs(l.x), 0.5 * o->box.length + radius);
                EXPECT_LT(std::fabs(l.y), 0.5 * o->box.width + radius);
                EXPECT_TRUE(p.path.empty());
                noExit(at_start ? r.start : r.goal, at_start);
                break;
            }
            case PlanStatus::kStartOutOfBounds:
                ++tally.bounds;
                EXPECT_LT(boundsDistance(start), radius);
                EXPECT_EQ(p.blocking, 0);
                noExit(r.start, true);
                break;
            case PlanStatus::kGoalOutOfBounds:
                ++tally.bounds;
                EXPECT_LT(boundsDistance(goal), radius);
                EXPECT_EQ(p.blocking, 0);
                noExit(r.goal, false);
                break;
            case PlanStatus::kNoPath: {
                ++tally.no_path;
                EXPECT_TRUE(p.path.empty());
                std::vector<Point> from;
                std::vector<Point> to;
                const EndUse       a = routeEnds(f, r, r.start, true, delta, from);
                const EndUse       b = routeEnds(f, r, r.goal, false, delta, to);
                if (a != EndUse::kUnclear && b != EndUse::kUnclear && !from.empty() &&
                    !to.empty()) {
                    ++tally.searched;
                    tally.exit_searched += a == EndUse::kExits || b == EndUse::kExits ? 1 : 0;
                    EXPECT_FALSE(gridRoute(f, r.model, from, to, delta))
                        << "grid search found a route the planner missed";
                }
                break;
            }
            case PlanStatus::kInvalidRequest:
            case PlanStatus::kUnsupportedModel:
            case PlanStatus::kNoField:
                ADD_FAILURE() << toString(p.status);
                break;
            }
        }
    }
    // The generator must exercise every outcome.
    EXPECT_GT(tally.ok, 1000);
    EXPECT_GT(tally.detoured, 200);
    EXPECT_GT(tally.escaped, 100);
    EXPECT_GT(tally.arrived, 100);
    EXPECT_GT(tally.blocked, 100);
    EXPECT_GT(tally.bounds, 100);
    EXPECT_GT(tally.no_path, 100);
    EXPECT_GT(tally.searched, 50);
    EXPECT_GT(tally.exit_searched, 0);
    EXPECT_GT(tally.exits_checked, 0);
    std::printf("ok %d (detoured %d, escaped %d, arrived %d), blocked %d, bounds %d (no exit "
                "confirmed %d), no path %d (searched %d, from or to exits %d)\n",
                tally.ok, tally.detoured, tally.escaped, tally.arrived, tally.blocked, tally.bounds,
                tally.exits_checked, tally.no_path, tally.searched, tally.exit_searched);
}

TEST(PlannerRandom, DirectPathsAreExecutable) {
    const GeometricPlanner planner;
    for (unsigned seed = 1; seed <= 500; ++seed) {
        Rng         rng(seed);
        PlanRequest r;
        r.mode          = PlanMode::kDirect;
        r.model         = randomModel(rng);
        r.start         = randomPose(rng);
        r.goal          = rng.chance(0.1) ? Pose{r.start.x + rng.uniform(-0.004, 0.004), r.start.y,
                                                 rng.uniform(-kPi, kPi)}
                                          : randomPose(rng);
        r.allow_reverse = rng.chance(0.7);
        SCOPED_TRACE("seed " + std::to_string(seed));
        const PlanResult p = planner.plan(r);
        ASSERT_EQ(p.status, PlanStatus::kOk);
        ASSERT_EQ(executionProblem(p.path, r, planner.config()), "");
        EXPECT_LE(p.path.segments.size(), r.model.holonomic ? 1u : 3u);
        // Reverse only when it turns less than driving forward.
        if (!r.model.holonomic && r.model.reverse && r.allow_reverse) {
            PlanRequest fwd    = r;
            fwd.allow_reverse  = false;
            const PlanResult q = planner.plan(fwd);
            Radians          a = 0;
            Radians          b = 0;
            for (const PathSegment& s : p.path.segments) {
                a += s.kind == SegmentKind::kTurn
                         ? std::fabs(wrapAngle(s.end.heading - s.start.heading))
                         : 0;
            }
            for (const PathSegment& s : q.path.segments) {
                b += s.kind == SegmentKind::kTurn
                         ? std::fabs(wrapAngle(s.end.heading - s.start.heading))
                         : 0;
            }
            EXPECT_LE(a, b + 1e-9);
        }
    }
}

TEST(PlannerRandom, ClearanceMatchesBruteForce) {
    // Separating axis result against point to rectangle distances from
    // dense perimeter samples, corners included.
    for (unsigned seed = 1; seed <= 400; ++seed) {
        Rng   rng(seed);
        Field f;
        f.generation = 1;
        f.bounds     = Bounds{-10, -10, 10, 10};
        FieldObject o;
        o.id         = 1;
        o.obstacle   = true;
        o.valid      = true;
        o.nominal    = Pose{rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-kPi, kPi)};
        o.pose       = o.nominal;
        o.box.length = rng.uniform(0.0, 0.8);
        o.box.width  = rng.uniform(0.0, 0.8);
        o.box.center = Pose{rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(-kPi, kPi)};
        f.objects    = {o};
        const Footprint fp{rng.uniform(0.0, 0.4), rng.uniform(0.0, 0.4), rng.uniform(0.0, 0.3),
                           rng.uniform(0.0, 0.3)};
        const Pose      robot{rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-kPi, kPi)};
        SCOPED_TRACE("seed " + std::to_string(seed));

        const Clearance c = footprintClearance(robot, fp, 0.0, f);
        ASSERT_EQ(c.nearest, 1);

        const Pose   box   = compose(o.pose, o.box.center);
        const double hl    = 0.5 * o.box.length;
        const double hw    = 0.5 * o.box.width;
        auto         toBox = [&](Point p) {
            const Pose   l  = between(box, Pose{p.x, p.y, 0});
            const double dx = std::max(std::fabs(l.x) - hl, 0.0);
            const double dy = std::max(std::fabs(l.y) - hw, 0.0);
            return std::hypot(dx, dy);
        };
        auto toRobot = [&](Point p) {
            const Pose   l  = between(robot, Pose{p.x, p.y, 0});
            const double dx = std::max({l.x - fp.front, -fp.back - l.x, 0.0});
            const double dy = std::max({l.y - fp.left, -fp.right - l.y, 0.0});
            return std::hypot(dx, dy);
        };
        const Point ra[4] = {
            {fp.front, -fp.right}, {fp.front, fp.left}, {-fp.back, fp.left}, {-fp.back, -fp.right}};
        const Point rb[4] = {{hl, -hw}, {hl, hw}, {-hl, hw}, {-hl, -hw}};
        const int   n     = 100;
        double      gap   = 1e9;
        for (int e = 0; e < 4; ++e) {
            for (int i = 0; i < n; ++i) {
                const double t = static_cast<double>(i) / n;
                const Point  a = ra[e];
                const Point  b = ra[(e + 1) % 4];
                const Pose   wa =
                    compose(robot, Pose{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), 0});
                gap           = std::min(gap, toBox(Point{wa.x, wa.y}));
                const Point p = rb[e];
                const Point q = rb[(e + 1) % 4];
                const Pose wb = compose(box, Pose{p.x + t * (q.x - p.x), p.y + t * (q.y - p.y), 0});
                gap           = std::min(gap, toRobot(Point{wb.x, wb.y}));
            }
        }
        if (c.distance > 0) {
            EXPECT_NEAR(c.distance, gap, 1e-9);
        } else {
            // Overlap or contact: the perimeters meet within one sample spacing.
            EXPECT_LT(gap, 0.01);
        }
    }
}
