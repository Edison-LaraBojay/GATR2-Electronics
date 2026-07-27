// transforms.h
// Planar poses and rigid transforms, meters and radians. Convention: a value
// named T_a_b is the pose of frame b expressed in frame a, so
// compose(T_a_b, T_b_c) yields T_a_c. Coordinate frames are named by
// FrameId; a wire packet is never a frame in this sense. Fixed-point wire
// units (mm, centidegrees) exist only inside the brain-facing
// implementations that speak them.
//
// x forward, y left, heading counterclockwise from +x. There is no z.

#pragma once
#include <cmath>

#include "core/ids.h"
#include "core/time.h"
#include "math/angles.h"

namespace navigatr
{

struct Pose2D {
    double x_m         = 0.0;
    double y_m         = 0.0;
    double heading_rad = 0.0;
};

// A rigid transform and a pose are the same object in 2D.
using Transform2D = Pose2D;

// T_a_c = T_a_b * T_b_c
inline Transform2D compose(const Transform2D& T_a_b, const Transform2D& T_b_c) {
    const double c = std::cos(T_a_b.heading_rad);
    const double s = std::sin(T_a_b.heading_rad);
    Transform2D  out;
    out.x_m         = T_a_b.x_m + c * T_b_c.x_m - s * T_b_c.y_m;
    out.y_m         = T_a_b.y_m + s * T_b_c.x_m + c * T_b_c.y_m;
    out.heading_rad = wrapAngle(T_a_b.heading_rad + T_b_c.heading_rad);
    return out;
}

// T_b_a from T_a_b
inline Transform2D inverse(const Transform2D& T_a_b) {
    const double c = std::cos(T_a_b.heading_rad);
    const double s = std::sin(T_a_b.heading_rad);
    Transform2D  out;
    out.x_m         = -(c * T_a_b.x_m + s * T_a_b.y_m);
    out.y_m         = -(-s * T_a_b.x_m + c * T_a_b.y_m);
    out.heading_rad = wrapAngle(-T_a_b.heading_rad);
    return out;
}

// A point fixed in frame b, expressed in frame a.
inline void transformPoint(const Transform2D& T_a_b, double x_b, double y_b, double& x_a,
                           double& y_a) {
    const double c = std::cos(T_a_b.heading_rad);
    const double s = std::sin(T_a_b.heading_rad);
    x_a            = T_a_b.x_m + c * x_b - s * y_b;
    y_a            = T_a_b.y_m + s * x_b + c * y_b;
}

// A pose known in frame b, expressed in frame a. Same math as compose,
// named for reading clarity at call sites.
inline Pose2D transformPose(const Transform2D& T_a_b, const Pose2D& pose_b) {
    return compose(T_a_b, pose_b);
}

// A pose plus the coordinate frame it lives in.
struct FramedPose2D {
    FrameId       frame;
    Pose2D        pose;
    MonotonicTime measuredAt;
};

} // namespace navigatr
