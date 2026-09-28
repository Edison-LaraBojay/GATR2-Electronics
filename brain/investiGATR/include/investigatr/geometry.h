// geometry.h
// Units and planar poses. Field frame heading is CCW from +x, robot frame is
// +x forward, +y left.

#pragma once
#include <cstdint>

namespace investigatr
{

using Seconds          = double;
using Meters           = double;
using Radians          = double;
using MetersPerSecond  = double;
using RadiansPerSecond = double;

// Field frame a pose is expressed in. 0 = no frame; unique per source instance.
using FrameGeneration = uint32_t;

constexpr double kPi = 3.14159265358979323846;

struct Point {
    Meters x = 0;
    Meters y = 0;
};

struct Pose {
    Meters  x       = 0;
    Meters  y       = 0;
    Radians heading = 0;
};

// (-pi, pi]
Radians wrapAngle(Radians angle);

// a * b: b's translation rotated by a.heading, headings added and wrapped.
Pose compose(const Pose& a, const Pose& b);

// compose(p, inverse(p)) is the identity.
Pose inverse(const Pose& p);

// b expressed in the frame of a: compose(inverse(a), b).
Pose between(const Pose& a, const Pose& b);

bool finite(const Pose& p);

} // namespace investigatr
