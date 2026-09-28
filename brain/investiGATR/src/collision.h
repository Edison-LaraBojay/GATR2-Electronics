// collision.h
// Rectangle geometry shared by the planner and footprintClearance. Private to
// the investigatr library.

#pragma once

#include "investigatr/field.h"
#include "investigatr/geometry.h"
#include "investigatr/motion_model.h"

namespace investigatr
{
namespace collision
{

// Rounding allowance: boundary contact is not an overlap.
constexpr Meters kContact = 1e-9;

// Rotated rectangle, field frame: center, unit x axis (c, s), half extents.
struct Rect {
    Point  center;
    double c  = 1;
    double s  = 0;
    Meters hx = 0;
    Meters hy = 0;
};

// Convex polygon, counterclockwise.
struct Polygon {
    Point p[8];
    int   n = 0;
};

// Convex polygon as half-planes, n[i] . p <= d[i] with n unit, and its
// bounding box for quick rejection.
struct Region {
    Point  n[8];
    double d[8];
    int    count = 0;
    Point  low;
    Point  high;
};

// Estimate when valid, else nominal.
Pose obstaclePose(const FieldObject& object);

// Finite bounds with an area, finite obstacle poses and boxes of size >= 0.
bool usable(const Field& field);

// Obstacle box grown by grow on every side.
Rect obstacleRect(const FieldObject& object, Meters grow);

// Footprint rectangle of a robot at pose.
Rect footprintRect(const Pose& robot, const Footprint& footprint);

Point toLocal(const Rect& r, Point p);
Point toField(const Rect& r, Point local);

// Counterclockwise from (+hx, -hy).
void corners(const Rect& r, Point out[4]);

Polygon polygon(const Rect& r);

// Every point within grow of r, bounded by eight tangents: the sides moved
// out by grow and the corners cut at 45 degrees. At most 8.3 percent of grow
// beyond the rounded shape, at its vertices.
Polygon grown(const Rect& r, Meters grow);

// Region r covers while translated by move: the hull of r at both ends.
Polygon sweep(const Rect& r, Point move);

Region region(const Polygon& poly);

// Inside means more than kContact inside every half-plane.

bool inside(const Region& r, Point p);

// Interval (enter, leave) of t where a + t * u is inside r. False when the
// line misses the interior.
bool clip(const Region& r, Point a, Point u, double& enter, double& leave);

// Segment a-b meets the interior of r.
bool crosses(const Region& r, Point a, Point b);

// Gap between a and b, or minus the penetration depth when they overlap.
double signedDistance(const Polygon& a, const Polygon& b);
double signedDistance(const Rect& a, const Rect& b);

} // namespace collision
} // namespace investigatr
