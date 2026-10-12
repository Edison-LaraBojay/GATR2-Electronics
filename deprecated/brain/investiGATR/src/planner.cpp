// planner.cpp

#include "investigatr/planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "collision.h"

namespace investigatr
{
namespace
{

using collision::kContact;
using collision::Polygon;
using collision::Rect;
using collision::Region;

constexpr double kInf = std::numeric_limits<double>::infinity();

// Legs shorter than this are repeated points.
constexpr Meters kSamePoint = 1e-9;

// Reverse must save more turning than this to win a tie.
constexpr Radians kTurnTie = 1e-9;

// Headings, and directions relative to their length, that agree this well
// are the same.
constexpr double kParallel = 1e-9;

// Largest min_turn: a dropped turn leaves at most this heading error.
constexpr Radians kMaxMinTurn = 0.1;

// Obstacles grown by the enclosing radius and where the origin may be, plus
// the real boxes and footprint for exact swept checks.
struct Space {
    std::vector<Region>   boxes;  // grown by the radius, corners cut
    std::vector<Rect>     solids; // as configured
    std::vector<ObjectId> ids;
    Bounds                bounds;
    Bounds                free; // bounds shrunk by the radius, may be empty
    Footprint             footprint;
    Meters                clearance = 0;
    Meters                radius    = 0;
};

Space makeSpace(const Field& field, const MotionModel& model) {
    Space space;
    space.radius    = enclosingRadius(model);
    space.footprint = model.footprint;
    space.clearance = model.clearance;
    space.bounds    = field.bounds;
    for (const FieldObject& o : field.objects) {
        if (o.obstacle) {
            space.solids.push_back(collision::obstacleRect(o, 0));
            space.boxes.push_back(
                collision::region(collision::grown(space.solids.back(), space.radius)));
            space.ids.push_back(o.id);
        }
    }
    const Meters r = space.radius;
    space.free     = Bounds{field.bounds.min_x + r, field.bounds.min_y + r, field.bounds.max_x - r,
                            field.bounds.max_y - r};
    return space;
}

bool inRegion(const Bounds& b, Point p) {
    return p.x >= b.min_x && p.x <= b.max_x && p.y >= b.min_y && p.y <= b.max_y;
}

// Index of the first grown box holding p, -1 when none.
int blockingBox(const Space& space, Point p) {
    for (std::size_t i = 0; i < space.boxes.size(); ++i) {
        if (collision::inside(space.boxes[i], p)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

bool segmentFree(const Space& space, Point a, Point b) {
    for (const Region& box : space.boxes) {
        if (collision::crosses(box, a, b)) {
            return false;
        }
    }
    return true;
}

// Origin path from a to b stays in the free region and out of every box.
bool moveFree(const Space& space, Point a, Point b) {
    return inRegion(space.free, a) && inRegion(space.free, b) && segmentFree(space, a, b);
}

double distance(Point a, Point b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

Point position(const Pose& p) {
    return Point{p.x, p.y};
}

bool sameHeading(Radians a, Radians b) {
    return std::fabs(wrapAngle(a - b)) <= kParallel;
}

// What a pose at a path end runs into.
struct Hit {
    bool        any   = false;
    bool        bound = false; // else a box
    std::size_t box   = 0;     // index into the space
};

// Circle test: origin outside the free region or inside a grown box.
Hit circleHit(const Space& space, Point p) {
    Hit hit;
    if (!inRegion(space.free, p)) {
        hit.any   = true;
        hit.bound = true;
        return hit;
    }
    const int box = blockingBox(space, p);
    if (box >= 0) {
        hit.any = true;
        hit.box = static_cast<std::size_t>(box);
    }
    return hit;
}

// Distance of r inside each bound side: min x, max x, min y, max y.
void sideGaps(const Bounds& b, const Rect& r, double out[4]) {
    Point c[4];
    collision::corners(r, c);
    std::fill(out, out + 4, kInf);
    for (const Point& p : c) {
        out[0] = std::min(out[0], p.x - b.min_x);
        out[1] = std::min(out[1], b.max_x - p.x);
        out[2] = std::min(out[2], p.y - b.min_y);
        out[3] = std::min(out[3], b.max_y - p.y);
    }
}

// Exact test: the footprint at p overlaps a bound or a box. Contact is clear.
Hit footprintHit(const Space& space, const Pose& p) {
    Hit        hit;
    const Rect body = collision::footprintRect(p, space.footprint);
    double     gaps[4];
    sideGaps(space.bounds, body, gaps);
    if (*std::min_element(gaps, gaps + 4) < -kContact) {
        hit.any   = true;
        hit.bound = true;
        return hit;
    }
    for (std::size_t i = 0; i < space.solids.size(); ++i) {
        if (collision::signedDistance(body, space.solids[i]) < -kContact) {
            hit.any = true;
            hit.box = i;
            return hit;
        }
    }
    return hit;
}

// Which ends of a sweep are the path's start pose and goal pose. Within 2R of
// such an end the sweep may come as near to a box or bound as that pose
// already is, when that is nearer than the clearance.
struct Ends {
    bool start = false;
    bool goal  = false;
};

// Part of a sweep with one allowance.
struct Piece {
    Polygon swept;
    double  gaps[4]; // nearest to each bound side, reached at one of its ends
    bool    near_start = false;
    bool    near_goal  = false;
};

// Footprint translated at a's heading from a to b. Clear when the footprint
// at b overlaps nothing and on the way it keeps the clearance from every box
// and bound, except within 2R of a loose end, where it comes no nearer than
// that end pose already is.
bool sweepFree(const Space& space, const Pose& a, Point b, Ends loose) {
    const Rect   from = collision::footprintRect(a, space.footprint);
    const Rect   to   = collision::footprintRect(Pose{b.x, b.y, a.heading}, space.footprint);
    const double len  = distance(position(a), b);
    const double zone = 2.0 * space.radius;

    // Cuts where an allowance starts or stops: near the start, between, near
    // the goal.
    double cut[4];
    int    cuts = 0;
    cut[cuts++] = 0;
    if (loose.start && zone < len) {
        cut[cuts++] = zone;
    }
    if (loose.goal && len - zone > 0) {
        cut[cuts++] = len - zone;
    }
    cut[cuts++] = len;
    std::sort(cut, cut + cuts);

    auto along = [&](double s) {
        if (s <= 0) {
            return position(a);
        }
        if (s >= len) {
            return b;
        }
        return Point{a.x + s * (b.x - a.x) / len, a.y + s * (b.y - a.y) / len};
    };
    Piece pieces[3];
    int   count = 0;
    for (int k = 0; k + 1 < cuts; ++k) {
        Piece&      piece = pieces[count++];
        const Point p0    = along(cut[k]);
        const Point p1    = along(cut[k + 1]);
        const Rect  r0    = collision::footprintRect(Pose{p0.x, p0.y, a.heading}, space.footprint);
        const Rect  r1    = collision::footprintRect(Pose{p1.x, p1.y, a.heading}, space.footprint);
        piece.swept       = collision::sweep(r0, Point{p1.x - p0.x, p1.y - p0.y});
        double g0[4];
        double g1[4];
        sideGaps(space.bounds, r0, g0);
        sideGaps(space.bounds, r1, g1);
        for (int i = 0; i < 4; ++i) {
            piece.gaps[i] = std::min(g0[i], g1[i]);
        }
        const double mid = 0.5 * (cut[k] + cut[k + 1]);
        piece.near_start = loose.start && mid <= zone;
        piece.near_goal  = loose.goal && mid >= len - zone;
    }
    auto allowed = [&](const Piece& piece, double at_start, double at_end) {
        return std::min(
            {space.clearance, piece.near_start ? at_start : kInf, piece.near_goal ? at_end : kInf});
    };

    double from_gaps[4];
    double to_gaps[4];
    sideGaps(space.bounds, from, from_gaps);
    sideGaps(space.bounds, to, to_gaps);
    for (int k = 0; k < 4; ++k) {
        if (to_gaps[k] < -kContact) {
            return false;
        }
        for (int i = 0; i < count; ++i) {
            if (pieces[i].gaps[k] < allowed(pieces[i], from_gaps[k], to_gaps[k]) - kContact) {
                return false;
            }
        }
    }

    for (const Rect& solid : space.solids) {
        const double at_end = collision::signedDistance(to, solid);
        if (at_end < -kContact) {
            return false;
        }
        const double  at_start = loose.start ? collision::signedDistance(from, solid) : kInf;
        const Polygon box      = collision::polygon(solid);
        for (int i = 0; i < count; ++i) {
            if (collision::signedDistance(pieces[i].swept, box) <
                allowed(pieces[i], at_start, at_end) - kContact) {
                return false;
            }
        }
    }
    return true;
}

enum class Search : uint8_t { kFound, kNone, kTooLarge };

// Drops route corners in line with their neighbours when the joined leg is
// clear, so collinear corners do not split one leg into several.
void straighten(const Space& space, std::vector<Point>& route) {
    std::vector<Point> out{route.front()};
    for (std::size_t i = 1; i + 1 < route.size(); ++i) {
        const Point  a   = out.back();
        const Point  p   = route[i];
        const Point  b   = route[i + 1];
        const double len = distance(a, b);
        const double along =
            len > 0 ? ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / len : -1;
        const double off =
            len > 0 ? std::fabs((p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x)) / len : 1;
        if (off <= kSamePoint && along >= 0 && along <= len && segmentFree(space, a, b)) {
            continue;
        }
        out.push_back(p);
    }
    out.push_back(route.back());
    route = std::move(out);
}

// A place the circle route may begin or end, and the exact exit length
// beyond it.
struct Terminal {
    Point  at;
    Meters extra = 0;
};

// Circle route from one of the from terminals to one of the to terminals.
struct Route {
    std::vector<Point> points;
    std::size_t        from = 0;
    std::size_t        to   = 0;
};

// Shortest route counting the exits beyond its terminals. A clear straight
// pair no longer than every pair's straight bound needs no graph. Otherwise
// A* over the visibility graph: the terminals and the box corners pushed out
// by margin that lie in the free region and outside every other box, from
// every from terminal at once to a virtual goal behind the to terminals.
// Memory and work are bounded by max_vertices: at most V^2 / 2 segment tests.
// When the graph would be larger, a clear straight pair still answers.
Search shortestRoute(const Space& space, const std::vector<Terminal>& from,
                     const std::vector<Terminal>& to, const GeometricPlannerConfig& config,
                     Route& route) {
    const std::size_t ns     = from.size();
    const std::size_t nt     = to.size();
    double            bound  = kInf;
    double            direct = kInf;
    for (std::size_t i = 0; i < ns; ++i) {
        for (std::size_t j = 0; j < nt; ++j) {
            const double cost = from[i].extra + distance(from[i].at, to[j].at) + to[j].extra;
            bound             = std::min(bound, cost);
            if (cost < direct && segmentFree(space, from[i].at, to[j].at)) {
                direct       = cost;
                route.points = {from[i].at, to[j].at};
                route.from   = i;
                route.to     = j;
            }
        }
    }
    if (direct <= bound) {
        return Search::kFound;
    }
    const Search too_large = direct < kInf ? Search::kFound : Search::kTooLarge;

    std::vector<Point> v;
    for (const Terminal& t : from) {
        v.push_back(t.at);
    }
    for (const Terminal& t : to) {
        v.push_back(t.at);
    }
    if (v.size() > config.max_vertices) {
        return too_large;
    }
    for (const Rect& solid : space.solids) {
        const Polygon corners = collision::grown(solid, space.radius + config.vertex_margin);
        for (int i = 0; i < corners.n; ++i) {
            const Point p = corners.p[i];
            if (inRegion(space.free, p) && blockingBox(space, p) < 0) {
                if (v.size() == config.max_vertices) {
                    return too_large;
                }
                v.push_back(p);
            }
        }
    }

    // Vertex n is the virtual goal, reached from to terminal j at cost extra.
    const std::size_t   n = v.size();
    std::vector<double> g(n + 1, kInf);
    std::vector<double> h(n + 1, 0);
    std::vector<int>    parent(n + 1, -1);
    std::vector<char>   done(n + 1, 0);
    for (std::size_t i = 0; i < n; ++i) {
        h[i] = kInf;
        for (const Terminal& t : to) {
            h[i] = std::min(h[i], distance(v[i], t.at) + t.extra);
        }
    }
    for (std::size_t i = 0; i < ns; ++i) {
        g[i] = from[i].extra;
    }
    for (;;) {
        std::size_t u    = n + 1;
        double      best = kInf;
        for (std::size_t i = 0; i <= n; ++i) {
            if (!done[i] && g[i] + h[i] < best) {
                best = g[i] + h[i];
                u    = i;
            }
        }
        if (u == n + 1) {
            return Search::kNone;
        }
        if (u == n) {
            break;
        }
        done[u] = 1;
        if (u >= ns && u < ns + nt && g[u] + to[u - ns].extra < g[n]) {
            g[n]      = g[u] + to[u - ns].extra;
            parent[n] = static_cast<int>(u);
        }
        for (std::size_t w = 0; w < n; ++w) {
            if (done[w]) {
                continue;
            }
            const double through = g[u] + distance(v[u], v[w]);
            if (through < g[w] && segmentFree(space, v[u], v[w])) {
                g[w]      = through;
                parent[w] = static_cast<int>(u);
            }
        }
    }

    // Only from terminals start with a cost and no parent.
    std::size_t i = static_cast<std::size_t>(parent[n]);
    route.to      = i - ns;
    route.points  = {v[i]};
    while (parent[i] >= 0) {
        i = static_cast<std::size_t>(parent[i]);
        route.points.push_back(v[i]);
    }
    route.from = i;
    std::reverse(route.points.begin(), route.points.end());
    straighten(space, route.points);
    return Search::kFound;
}

PathSegment turn(Point at, Radians from, Radians to) {
    PathSegment s;
    s.kind           = SegmentKind::kTurn;
    s.start          = Pose{at.x, at.y, from};
    s.end            = Pose{at.x, at.y, to};
    s.turn_direction = wrapAngle(to - from) > 0 ? 1 : -1;
    return s;
}

PathSegment translate(Point a, Radians from, Point b, Radians to, bool reverse) {
    PathSegment s;
    s.kind    = SegmentKind::kTranslate;
    s.start   = Pose{a.x, a.y, from};
    s.end     = Pose{b.x, b.y, to};
    s.reverse = reverse;
    return s;
}

// Per leg, true to drive it backward: least total turning from start heading
// through every leg to goal heading, forward on ties.
std::vector<bool> chooseReverse(const std::vector<Radians>& travel, Radians start, Radians goal) {
    const std::size_t n = travel.size();
    auto facing = [&](std::size_t leg, int back) { return wrapAngle(travel[leg] + back * kPi); };
    auto turnBy = [](Radians a, Radians b) { return std::fabs(wrapAngle(b - a)); };

    std::vector<double> cost(2 * n);
    std::vector<int>    from(2 * n, 0);
    for (int d = 0; d < 2; ++d) {
        cost[d] = turnBy(start, facing(0, d));
    }
    for (std::size_t k = 1; k < n; ++k) {
        for (int d = 0; d < 2; ++d) {
            const double fwd = cost[2 * (k - 1)] + turnBy(facing(k - 1, 0), facing(k, d));
            const double bwd = cost[2 * (k - 1) + 1] + turnBy(facing(k - 1, 1), facing(k, d));
            const bool   use = bwd < fwd - kTurnTie;
            cost[2 * k + d]  = use ? bwd : fwd;
            from[2 * k + d]  = use ? 1 : 0;
        }
    }
    const double      fwd = cost[2 * (n - 1)] + turnBy(facing(n - 1, 0), goal);
    const double      bwd = cost[2 * (n - 1) + 1] + turnBy(facing(n - 1, 1), goal);
    int               d   = bwd < fwd - kTurnTie ? 1 : 0;
    std::vector<bool> out(n);
    for (std::size_t k = n; k-- > 0;) {
        out[k] = d == 1;
        d      = from[2 * k + d];
    }
    return out;
}

// Segments along the route for the model, from heading start to heading
// goal. A single leg shorter than min_segment is dropped; longer routes keep
// every leg, since each one is needed to stay clear. Before an exact approach
// (approach) that leg is kept too, and so is the final turn however small:
// the approach was swept at the goal heading only.
std::vector<PathSegment> shape(const std::vector<Point>& route, Radians start, Radians goal,
                               const PlanRequest& request, const GeometricPlannerConfig& config,
                               bool approach) {
    std::vector<PathSegment> out;

    std::vector<Point> points{route.front()};
    for (std::size_t i = 1; i < route.size(); ++i) {
        if (distance(points.back(), route[i]) >= kSamePoint) {
            points.push_back(route[i]);
        }
    }
    if (!approach && points.size() == 2 && distance(points[0], points[1]) < config.min_segment) {
        points.pop_back();
    }

    start                       = wrapAngle(start);
    goal                        = wrapAngle(goal);
    const std::size_t legs      = points.size() - 1;
    auto              finalTurn = [&](Radians from) {
        const Radians by = std::fabs(wrapAngle(goal - from));
        return approach ? by > kParallel : by >= config.min_turn;
    };

    if (legs == 0) {
        if (finalTurn(start)) {
            out.push_back(turn(points.front(), start, goal));
        }
        return out;
    }

    if (request.model.holonomic) {
        Meters total = 0;
        for (std::size_t i = 0; i < legs; ++i) {
            total += distance(points[i], points[i + 1]);
        }
        const Radians change = wrapAngle(goal - start);
        Meters        done   = 0;
        Radians       from   = start;
        for (std::size_t i = 0; i < legs; ++i) {
            done += distance(points[i], points[i + 1]);
            const Radians to = i + 1 == legs ? goal : wrapAngle(start + change * done / total);
            out.push_back(translate(points[i], from, points[i + 1], to, false));
            from = to;
        }
        return out;
    }

    std::vector<Radians> travel(legs);
    for (std::size_t i = 0; i < legs; ++i) {
        travel[i] = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
    }
    std::vector<bool> reverse(legs, false);
    if (request.model.reverse && request.allow_reverse) {
        reverse = chooseReverse(travel, start, goal);
    }
    Radians heading = start;
    for (std::size_t i = 0; i < legs; ++i) {
        const Radians facing = reverse[i] ? wrapAngle(travel[i] + kPi) : travel[i];
        if (std::fabs(wrapAngle(facing - heading)) >= config.min_turn) {
            out.push_back(turn(points[i], heading, facing));
        }
        out.push_back(translate(points[i], facing, points[i + 1], facing, reverse[i]));
        heading = facing;
    }
    if (finalTurn(heading)) {
        out.push_back(turn(points.back(), heading, goal));
    }
    return out;
}

// Straight moves at a fixed heading the request allows, in preference
// order: forward, backward, then for holonomic models left and right.
struct Move {
    Point dir;
    bool  reverse = false;
};

int straightMoves(const PlanRequest& r, Radians heading, Move out[4]) {
    const Point fwd{std::cos(heading), std::sin(heading)};
    int         n = 0;
    out[n++]      = Move{fwd, false};
    if (r.model.holonomic) {
        out[n++] = Move{Point{-fwd.x, -fwd.y}, false};
        out[n++] = Move{Point{-fwd.y, fwd.x}, false};
        out[n++] = Move{Point{fwd.y, -fwd.x}, false};
    } else if (r.model.reverse && r.allow_reverse) {
        out[n++] = Move{Point{-fwd.x, -fwd.y}, true};
    }
    return n;
}

// Distances, at most limit, along the unit direction u from p to points that
// pass the circle test: the far side of the free region bound or of a grown
// box, plus push. Increasing; empty when there is none.
std::vector<double> freeStops(const Space& space, Point p, Point u, Meters limit, Meters push) {
    std::vector<double> out;
    const double        from[2] = {p.x, p.y};
    const double        step[2] = {u.x, u.y};
    const double        low[2]  = {space.free.min_x, space.free.min_y};
    const double        high[2] = {space.free.max_x, space.free.max_y};
    double              lo      = 0;
    double              hi      = kInf;
    for (int k = 0; k < 2; ++k) {
        if (step[k] == 0) {
            if (from[k] < low[k] || from[k] > high[k]) {
                return out;
            }
            continue;
        }
        double t0 = (low[k] - from[k]) / step[k];
        double t1 = (high[k] - from[k]) / step[k];
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        lo = std::max(lo, t0);
        hi = std::min(hi, t1);
    }
    if (lo > hi) {
        return out;
    }

    std::vector<double> candidates{lo};
    for (const Region& box : space.boxes) {
        double enter = 0;
        double leave = 0;
        if (collision::clip(box, p, u, enter, leave) && leave > 0) {
            candidates.push_back(leave);
        }
    }
    std::sort(candidates.begin(), candidates.end());
    for (const double t : candidates) {
        const double along = t + push;
        if (along > limit) {
            break;
        }
        const Point q{p.x + along * u.x, p.y + along * u.y};
        if (inRegion(space.free, q) && blockingBox(space, q) < 0) {
            out.push_back(along);
        }
    }
    return out;
}

// Exact straight translations, at most 2R long, joining a path end that fails
// the circle test to points that pass it: leaving the start, or arriving at
// the goal. Along each of the model's moves, the first point of every
// circle-free stretch; a longer sweep holds a shorter one, so the first
// unclear sweep ends that move.
void findExits(const Space& space, const PlanRequest& r, const Pose& end, bool leaving, Meters push,
               std::vector<PathSegment>& out) {
    Move        moves[4];
    const int   n  = straightMoves(r, end.heading, moves);
    const Point at = position(end);
    for (int i = 0; i < n; ++i) {
        const Point dir  = moves[i].dir;
        const Point ray  = leaving ? dir : Point{-dir.x, -dir.y};
        bool        have = false;
        Point       last{};
        for (const double length : freeStops(space, at, ray, 2.0 * space.radius, push)) {
            const Point other{at.x + length * ray.x, at.y + length * ray.y};
            if (have && moveFree(space, last, other)) {
                continue; // same stretch
            }
            const Point a = leaving ? at : other;
            const Point b = leaving ? other : at;
            if (!sweepFree(space, Pose{a.x, a.y, end.heading}, b, Ends{leaving, !leaving})) {
                break;
            }
            out.push_back(translate(a, end.heading, b, end.heading, moves[i].reverse));
            out.back().exact = true;
            have             = true;
            last             = other;
        }
    }
}

Meters segmentLength(const PathSegment& s) {
    return distance(position(s.start), position(s.end));
}

// One exact translation from start to goal at the start heading, when the
// model can drive it and its sweep is clear. Leaves segments empty when
// already there.
bool straightExact(const Space& space, const PlanRequest& r, const Pose& start, const Pose& goal,
                   const GeometricPlannerConfig& config, std::vector<PathSegment>& segments) {
    if (std::fabs(wrapAngle(goal.heading - start.heading)) >= config.min_turn) {
        return false;
    }
    const Point  a   = position(start);
    const Point  b   = position(goal);
    const double len = distance(a, b);
    segments.clear();
    if (len < config.min_segment) {
        return true;
    }
    bool reverse = false;
    if (!r.model.holonomic) {
        const Point  fwd{std::cos(start.heading), std::sin(start.heading)};
        const double along  = (b.x - a.x) * fwd.x + (b.y - a.y) * fwd.y;
        const double across = (b.y - a.y) * fwd.x - (b.x - a.x) * fwd.y;
        reverse             = along < 0;
        if (std::fabs(across) > kParallel * len ||
            (reverse && !(r.model.reverse && r.allow_reverse))) {
            return false;
        }
    }
    if (!sweepFree(space, start, b, Ends{true, true})) {
        return false;
    }
    segments.push_back(translate(a, start.heading, b, start.heading, reverse));
    segments.back().exact = true;
    return true;
}

// Consecutive translations in one direction at one heading, one of them
// exact, become one exact translation when its sweep is clear and the robot
// enters the first at that heading (entry: the heading before the path). A
// dropped turn before it would leave the exact sweep at the wrong heading.
void joinExact(const Space& space, Radians entry, std::vector<PathSegment>& segments) {
    std::size_t i = 0;
    while (i + 1 < segments.size()) {
        const PathSegment& a  = segments[i];
        const PathSegment& b  = segments[i + 1];
        const Radians      at = i == 0 ? entry : segments[i - 1].end.heading;
        const Point        da{a.end.x - a.start.x, a.end.y - a.start.y};
        const Point        db{b.end.x - b.start.x, b.end.y - b.start.y};
        const double       scale = std::hypot(da.x, da.y) * std::hypot(db.x, db.y);
        const bool         join =
            a.kind == SegmentKind::kTranslate && b.kind == SegmentKind::kTranslate &&
            (a.exact || b.exact) && a.reverse == b.reverse && sameHeading(at, a.start.heading) &&
            sameHeading(a.start.heading, a.end.heading) &&
            sameHeading(a.start.heading, b.start.heading) &&
            sameHeading(a.start.heading, b.end.heading) && da.x * db.x + da.y * db.y > 0 &&
            std::fabs(da.x * db.y - da.y * db.x) <= kParallel * scale &&
            sweepFree(space, a.start, position(b.end), Ends{i == 0, i + 2 == segments.size()});
        if (!join) {
            ++i;
            continue;
        }
        PathSegment joined = a;
        joined.end         = Pose{b.end.x, b.end.y, a.start.heading};
        joined.exact       = true;
        segments[i]        = joined;
        segments.erase(segments.begin() + static_cast<std::ptrdiff_t>(i) + 1);
    }
}

// Exact translations by their swept footprint, loose within 2R of the path's
// start and goal; everything else by the circle.
bool pathFree(const Space& space, const std::vector<PathSegment>& segments) {
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const PathSegment& s = segments[i];
        if (!finite(s.start) || !finite(s.end)) {
            return false;
        }
        const bool clear = s.exact && s.kind == SegmentKind::kTranslate
                               ? sameHeading(s.start.heading, s.end.heading) &&
                                     sweepFree(space, s.start, position(s.end),
                                               Ends{i == 0, i + 1 == segments.size()})
                               : moveFree(space, position(s.start), position(s.end));
        if (!clear) {
            return false;
        }
    }
    return true;
}

bool validConfig(const GeometricPlannerConfig& c) {
    auto ok = [](double v) { return std::isfinite(v) && v >= 0; };
    return ok(c.vertex_margin) && ok(c.min_segment) && ok(c.min_turn) && c.min_turn <= kMaxMinTurn;
}

} // namespace

const char* toString(PlanStatus status) {
    switch (status) {
    case PlanStatus::kOk: return "ok";
    case PlanStatus::kInvalidRequest: return "invalid request";
    case PlanStatus::kUnsupportedModel: return "unsupported motion model";
    case PlanStatus::kNoField: return "no usable field";
    case PlanStatus::kStartOutOfBounds: return "start out of bounds";
    case PlanStatus::kStartBlocked: return "start blocked";
    case PlanStatus::kGoalOutOfBounds: return "goal out of bounds";
    case PlanStatus::kGoalBlocked: return "goal blocked";
    case PlanStatus::kNoPath: return "no path";
    }
    return "?";
}

GeometricPlanner::GeometricPlanner(const GeometricPlannerConfig& config) : config_(config) {}

PlanResult GeometricPlanner::plan(const PlanRequest& request) const {
    PlanResult result;
    result.path.mode = request.mode;

    const bool known_mode =
        request.mode == PlanMode::kDirect || request.mode == PlanMode::kAvoiding;
    if (!known_mode || !finite(request.start) || !finite(request.goal) || !valid(request.model) ||
        !validConfig(config_)) {
        result.status = PlanStatus::kInvalidRequest;
        return result;
    }
    const MotionModel& m = request.model;
    if ((!m.holonomic && !m.turn_in_place) || m.min_turn_radius > 0) {
        result.status = PlanStatus::kUnsupportedModel;
        return result;
    }

    const Pose start{request.start.x, request.start.y, wrapAngle(request.start.heading)};
    const Pose goal{request.goal.x, request.goal.y, wrapAngle(request.goal.heading)};
    if (request.mode == PlanMode::kDirect) {
        result.status        = PlanStatus::kOk;
        result.path.segments = shape({position(start), position(goal)}, start.heading, goal.heading,
                                     request, config_, false);
        return result;
    }

    const Field* field = request.field;
    if (field == nullptr || field->generation == 0 || !collision::usable(*field)) {
        result.status = PlanStatus::kNoField;
        return result;
    }
    const Space space = makeSpace(*field, m);

    // Ends: the circle test; an end that fails it needs an exact footprint
    // clear of everything, then an exit.
    struct End {
        Pose       pose;
        PlanStatus out_of_bounds;
        PlanStatus blocked;
        Hit        circle;
    };
    End  ends[2] = {{start, PlanStatus::kStartOutOfBounds, PlanStatus::kStartBlocked, {}},
                    {goal, PlanStatus::kGoalOutOfBounds, PlanStatus::kGoalBlocked, {}}};
    auto refuse  = [&](const End& end, const Hit& hit) {
        result.status   = hit.bound ? end.out_of_bounds : end.blocked;
        result.blocking = hit.bound ? 0 : space.ids[hit.box];
        return result;
    };
    for (End& end : ends) {
        end.circle = circleHit(space, position(end.pose));
        if (end.circle.any) {
            const Hit exact = footprintHit(space, end.pose);
            if (exact.any) {
                return refuse(end, exact);
            }
        }
    }
    const bool escape   = ends[0].circle.any;
    const bool approach = ends[1].circle.any;

    std::vector<PathSegment> segments;
    if ((escape || approach) && straightExact(space, request, start, goal, config_, segments)) {
        result.status        = PlanStatus::kOk;
        result.path.segments = std::move(segments);
        return result;
    }

    // Every exit, so a short one into a closed pocket cannot hide a longer one
    // that leads out; the search picks the shortest whole route.
    const Meters             push = config_.vertex_margin + kContact;
    std::vector<PathSegment> escapes;
    std::vector<PathSegment> approaches;
    std::vector<Terminal>    from{{position(start), 0}};
    std::vector<Terminal>    to{{position(goal), 0}};
    if (escape) {
        findExits(space, request, start, true, push, escapes);
        if (escapes.empty()) {
            return refuse(ends[0], ends[0].circle);
        }
        from.clear();
        for (const PathSegment& s : escapes) {
            from.push_back(Terminal{position(s.end), segmentLength(s)});
        }
    }
    if (approach) {
        findExits(space, request, goal, false, push, approaches);
        if (approaches.empty()) {
            return refuse(ends[1], ends[1].circle);
        }
        to.clear();
        for (const PathSegment& s : approaches) {
            to.push_back(Terminal{position(s.start), segmentLength(s)});
        }
    }

    Route route;
    if (shortestRoute(space, from, to, config_, route) != Search::kFound) {
        result.status = PlanStatus::kNoPath;
        return result;
    }
    if (escape) {
        segments.push_back(escapes[route.from]);
    }
    const std::vector<PathSegment> middle =
        shape(route.points, start.heading, goal.heading, request, config_, approach);
    segments.insert(segments.end(), middle.begin(), middle.end());
    if (approach) {
        segments.push_back(approaches[route.to]);
    }
    joinExact(space, start.heading, segments);
    if (!pathFree(space, segments)) {
        result.status = PlanStatus::kNoPath;
        return result;
    }
    result.status        = PlanStatus::kOk;
    result.path.segments = std::move(segments);
    return result;
}

bool GeometricPlanner::clear(const Path& path, const Field& field, const MotionModel& model) const {
    if (!valid(model) || !collision::usable(field)) {
        return false;
    }
    return pathFree(makeSpace(field, model), path.segments);
}

} // namespace investigatr
