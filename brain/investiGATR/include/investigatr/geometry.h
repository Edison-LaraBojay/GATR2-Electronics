// geometry.h
// Units and planar poses. Field frame heading is CCW from +x, robot frame is
// +x forward, +y left.

#pragma once

namespace investigatr
{

using Seconds = double;
using Meters  = double;
using Radians = double;

constexpr double kPi = 3.14159265358979323846;

struct Pose {
    Meters  x       = 0;
    Meters  y       = 0;
    Radians heading = 0;
};

// (-pi, pi]
Radians wrapAngle(Radians angle);

// a * b: b's translation rotated by a.heading, headings added and wrapped.
Pose compose(const Pose& a, const Pose& b);

} // namespace investigatr
