// collision.cpp

#include "collision.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "investigatr/planner.h"

namespace investigatr
{
namespace collision
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

bool finiteNonNegative(double v) {
    return std::isfinite(v) && v >= 0;
}

// Positive when o, a, b turn counterclockwise.
double cross(Point o, Point a, Point b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

double pointSegment(Point p, Point a, Point b) {
    const double dx   = b.x - a.x;
    const double dy   = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double       t    = 0;
    if (len2 > 0) {
        t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0);
    }
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

// Smallest vertex to edge distance; exact for disjoint convex polygons.
double gap(const Polygon& a, const Polygon& b) {
    double best = kInf;
    for (int i = 0; i < a.n; ++i) {
        for (int j = 0; j < b.n; ++j) {
            best = std::min(best, pointSegment(a.p[i], b.p[j], b.p[(j + 1) % b.n]));
            best = std::min(best, pointSegment(b.p[j], a.p[i], a.p[(i + 1) % a.n]));
        }
    }
    return best;
}

void project(const Polygon& poly, Point u, double& lo, double& hi) {
    lo = kInf;
    hi = -kInf;
    for (int i = 0; i < poly.n; ++i) {
        const double v = poly.p[i].x * u.x + poly.p[i].y * u.y;
        lo             = std::min(lo, v);
        hi             = std::max(hi, v);
    }
}

} // namespace

Pose obstaclePose(const FieldObject& object) {
    return object.valid ? object.pose : object.nominal;
}

bool usable(const Field& field) {
    const Bounds& b = field.bounds;
    if (!std::isfinite(b.min_x) || !std::isfinite(b.min_y) || !std::isfinite(b.max_x) ||
        !std::isfinite(b.max_y) || !(b.max_x > b.min_x) || !(b.max_y > b.min_y)) {
        return false;
    }
    for (const FieldObject& o : field.objects) {
        if (!o.obstacle) {
            continue;
        }
        if (!finite(obstaclePose(o)) || !finite(o.box.center) || !finiteNonNegative(o.box.length) ||
            !finiteNonNegative(o.box.width)) {
            return false;
        }
    }
    return true;
}

Rect obstacleRect(const FieldObject& object, Meters grow) {
    const Pose center = compose(obstaclePose(object), object.box.center);
    Rect       r;
    r.center = Point{center.x, center.y};
    r.c      = std::cos(center.heading);
    r.s      = std::sin(center.heading);
    r.hx     = 0.5 * object.box.length + grow;
    r.hy     = 0.5 * object.box.width + grow;
    return r;
}

Rect footprintRect(const Pose& robot, const Footprint& f) {
    const Pose offset{0.5 * (f.front - f.back), 0.5 * (f.left - f.right), 0};
    const Pose center = compose(robot, offset);
    Rect       r;
    r.center = Point{center.x, center.y};
    r.c      = std::cos(robot.heading);
    r.s      = std::sin(robot.heading);
    r.hx     = 0.5 * (f.front + f.back);
    r.hy     = 0.5 * (f.left + f.right);
    return r;
}

Point toLocal(const Rect& r, Point p) {
    const double dx = p.x - r.center.x;
    const double dy = p.y - r.center.y;
    return Point{r.c * dx + r.s * dy, -r.s * dx + r.c * dy};
}

Point toField(const Rect& r, Point local) {
    return Point{r.center.x + r.c * local.x - r.s * local.y,
                 r.center.y + r.s * local.x + r.c * local.y};
}

void corners(const Rect& r, Point out[4]) {
    const Point local[4] = {{r.hx, -r.hy}, {r.hx, r.hy}, {-r.hx, r.hy}, {-r.hx, -r.hy}};
    for (int i = 0; i < 4; ++i) {
        out[i] = toField(r, local[i]);
    }
}

Polygon polygon(const Rect& r) {
    Polygon out;
    corners(r, out.p);
    out.n = 4;
    return out;
}

Polygon grown(const Rect& r, Meters grow) {
    const double hx       = r.hx;
    const double hy       = r.hy;
    const double far      = grow;
    const double cut      = grow * (std::sqrt(2.0) - 1.0);
    const Point  local[8] = {{hx + far, -hy - cut},  {hx + far, hy + cut},  {hx + cut, hy + far},
                             {-hx - cut, hy + far},  {-hx - far, hy + cut}, {-hx - far, -hy - cut},
                             {-hx - cut, -hy - far}, {hx + cut, -hy - far}};
    Polygon      out;
    out.n = 8;
    for (int i = 0; i < 8; ++i) {
        out.p[i] = toField(r, local[i]);
    }
    return out;
}

Region region(const Polygon& poly) {
    Region out;
    out.low  = Point{kInf, kInf};
    out.high = Point{-kInf, -kInf};
    for (int i = 0; i < poly.n; ++i) {
        const Point  p0  = poly.p[i];
        const Point  p1  = poly.p[(i + 1) % poly.n];
        const double len = std::hypot(p1.x - p0.x, p1.y - p0.y);
        out.low          = Point{std::min(out.low.x, p0.x), std::min(out.low.y, p0.y)};
        out.high         = Point{std::max(out.high.x, p0.x), std::max(out.high.y, p0.y)};
        if (!(len > 0)) {
            continue;
        }
        // Outward normal of a counterclockwise edge.
        const Point n{(p1.y - p0.y) / len, (p0.x - p1.x) / len};
        out.n[out.count] = n;
        out.d[out.count] = n.x * p0.x + n.y * p0.y;
        ++out.count;
    }
    return out;
}

bool inside(const Region& r, Point p) {
    for (int i = 0; i < r.count; ++i) {
        if (!(r.n[i].x * p.x + r.n[i].y * p.y < r.d[i] - kContact)) {
            return false;
        }
    }
    return r.count > 0;
}

namespace
{

// Narrows (enter, leave) to where a + t * u is inside r; false once empty.
bool narrow(const Region& r, Point a, Point u, double& enter, double& leave) {
    for (int i = 0; i < r.count; ++i) {
        const double room = r.d[i] - kContact - (r.n[i].x * a.x + r.n[i].y * a.y);
        const double rate = r.n[i].x * u.x + r.n[i].y * u.y;
        if (rate > 0) {
            leave = std::min(leave, room / rate);
        } else if (rate < 0) {
            enter = std::max(enter, room / rate);
        } else if (!(room > 0)) {
            return false;
        }
        if (!(enter < leave)) {
            return false;
        }
    }
    return r.count > 0;
}

} // namespace

bool clip(const Region& r, Point a, Point u, double& enter, double& leave) {
    enter = -kInf;
    leave = kInf;
    return narrow(r, a, u, enter, leave);
}

bool crosses(const Region& r, Point a, Point b) {
    if (std::max(a.x, b.x) <= r.low.x || std::min(a.x, b.x) >= r.high.x ||
        std::max(a.y, b.y) <= r.low.y || std::min(a.y, b.y) >= r.high.y) {
        return false;
    }
    double enter = 0;
    double leave = 1;
    return narrow(r, a, Point{b.x - a.x, b.y - a.y}, enter, leave);
}

Polygon sweep(const Rect& r, Point move) {
    // Monotone chain over the corners at both ends.
    Point pts[8];
    corners(r, pts);
    for (int i = 0; i < 4; ++i) {
        pts[4 + i] = Point{pts[i].x + move.x, pts[i].y + move.y};
    }
    std::sort(pts, pts + 8,
              [](Point a, Point b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    Point chain[16];
    int   k = 0;
    for (int i = 0; i < 8; ++i) {
        while (k >= 2 && cross(chain[k - 2], chain[k - 1], pts[i]) <= 0) {
            --k;
        }
        chain[k++] = pts[i];
    }
    for (int i = 6, lower = k + 1; i >= 0; --i) {
        while (k >= lower && cross(chain[k - 2], chain[k - 1], pts[i]) <= 0) {
            --k;
        }
        chain[k++] = pts[i];
    }
    Polygon out;
    out.n = std::clamp(k - 1, 1, 8);
    std::copy(chain, chain + out.n, out.p);
    return out;
}

double signedDistance(const Polygon& a, const Polygon& b) {
    // Separating axes: the edge normals of both polygons.
    double depth = kInf;
    for (const Polygon* poly : {&a, &b}) {
        for (int i = 0; i < poly->n; ++i) {
            const Point  e0  = poly->p[i];
            const Point  e1  = poly->p[(i + 1) % poly->n];
            const double len = std::hypot(e1.x - e0.x, e1.y - e0.y);
            if (!(len > 0)) {
                continue;
            }
            const Point u{(e0.y - e1.y) / len, (e1.x - e0.x) / len};
            double      a_lo = 0;
            double      a_hi = 0;
            double      b_lo = 0;
            double      b_hi = 0;
            project(a, u, a_lo, a_hi);
            project(b, u, b_lo, b_hi);
            const double apart = std::max(b_lo - a_hi, a_lo - b_hi);
            if (apart > 0) {
                return gap(a, b);
            }
            depth = std::min(depth, -apart);
        }
    }
    return -depth;
}

double signedDistance(const Rect& a, const Rect& b) {
    return signedDistance(polygon(a), polygon(b));
}

} // namespace collision

Clearance footprintClearance(const Pose& robot, const Footprint& footprint, Meters margin,
                             const Field& field) {
    Clearance out;
    out.distance       = -std::numeric_limits<double>::infinity();
    const Footprint& f = footprint;
    const bool       sides_ok =
        collision::finiteNonNegative(f.front) && collision::finiteNonNegative(f.back) &&
        collision::finiteNonNegative(f.left) && collision::finiteNonNegative(f.right);
    if (!finite(robot) || !sides_ok || !collision::finiteNonNegative(margin) ||
        !collision::usable(field)) {
        return out;
    }

    const collision::Rect body = collision::footprintRect(robot, footprint);
    Point                 c[4];
    collision::corners(body, c);
    const Bounds& b    = field.bounds;
    double        best = std::numeric_limits<double>::infinity();
    for (const Point& p : c) {
        best = std::min({best, p.x - b.min_x, b.max_x - p.x, p.y - b.min_y, b.max_y - p.y});
    }
    ObjectId nearest = 0;
    for (const FieldObject& o : field.objects) {
        if (!o.obstacle) {
            continue;
        }
        const double d = collision::signedDistance(body, collision::obstacleRect(o, 0));
        if (d < best) {
            best    = d;
            nearest = o.id;
        }
    }
    // Growing a convex shape by margin on every side (rounded corners)
    // lowers its signed distance by exactly margin.
    out.distance = best - margin;
    out.nearest  = nearest;
    out.clear    = out.distance >= 0;
    return out;
}

} // namespace investigatr
