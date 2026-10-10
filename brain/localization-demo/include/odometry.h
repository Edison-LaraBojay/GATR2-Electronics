// odometry.h
// Two perpendicular tracking wheels plus an IMU heading: wheel counts to
// robot motion to field pose. Portable, no PROS.
//
// Robot frame: +x forward, +y left, origin at the chosen robot center.
// Field frame (same as spectaGATR): +x right, +y up on the field diagram,
// heading counterclockwise from +x, radians.

#pragma once
#include <cstdint>

namespace odometry
{

using Meters  = double;
using Radians = double;

constexpr double kPi = 3.14159265358979323846;

struct Pose {
    Meters  x       = 0;
    Meters  y       = 0;
    Radians heading = 0;
};

struct TrackingWheel {
    int    channel        = 0;    // Pico encoder channel
    Meters diameter       = 0;
    double counts_per_rev = 0;    // decoded counts per wheel turn (x4 quadrature)
    int    direction      = 1;    // +1 or -1, so positive travel is forward / left
    double scale          = 1.0;  // measured travel correction
    Meters offset         = 0;    // forward wheel: left of center. sideways: ahead of center
};

struct WheelSetup {
    TrackingWheel forward;  // measures forward travel
    TrackingWheel sideways; // measures leftward travel
};

// Robot frame motion over one step, along the arc.
struct BodyMotion {
    Meters  forward = 0;
    Meters  left    = 0;
    Radians turn    = 0; // counterclockwise
};

// Wraps to [-pi, pi).
Radians wrapAngle(Radians angle);

Meters wheelTravel(const TrackingWheel& wheel, int32_t delta_counts);

// Removes the travel a wheel sees only because the robot turned about a
// center the wheel is offset from.
BodyMotion bodyMotion(const WheelSetup& wheels, int32_t forward_counts, int32_t sideways_counts,
                      Radians turn);

// Constant curvature step: the chord of the arc, at the mid step heading.
Pose advance(const Pose& pose, const BodyMotion& motion);

// VEX IMU rotation: degrees, clockwise positive, not wrapped. The anchor ties
// one rotation reading to one field heading.
struct ImuAnchor {
    double  rotation_degrees = 0;
    Radians heading          = 0;
};

Radians imuHeading(const ImuAnchor& anchor, double rotation_degrees);

constexpr Radians degreesToRadians(double degrees) { return degrees * kPi / 180.0; }
constexpr double  radiansToDegrees(Radians radians) { return radians * 180.0 / kPi; }

} // namespace odometry
