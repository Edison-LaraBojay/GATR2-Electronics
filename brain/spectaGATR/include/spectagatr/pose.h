// pose.h
// Pose types and unit helpers for the spectaGATR publisher. The API is SI:
// meters and radians. Conventions: spectaGATR/docs/PROTOCOL.md.
//
//   Field  origin at the bottom left corner of the field diagram, +x right,
//          +y up on the diagram, +z up out of the floor
//   Robot  +x forward, +y left, +z up
//   yaw    heading: robot forward from field +x, counterclockwise positive
//   roll   about robot +x, positive lifts the left side
//   pitch  about robot +y, positive tips the nose down
//   Robot to field rotation: Rz(yaw) * Ry(pitch) * Rx(roll)

#pragma once
#include <cstdint>

#include "spectagatr/protocol/units.h"

namespace spectagatr
{

using Meters       = protocol::Meters;
using Radians      = protocol::Radians;
using Inches       = double;
using Degrees      = double;
using Milliseconds = uint32_t; // publisher clock (Brain millis), wraps
using Hertz        = double;

constexpr double kPi = protocol::kPi;

// Orientation only. It never moves the robot: there is no height.
struct Attitude {
    Radians roll  = 0;
    Radians pitch = 0;
    Radians yaw   = 0;
};

constexpr Meters inchesToMeters(Inches inches) { return inches * 0.0254; }

constexpr Inches metersToInches(Meters meters) { return meters / 0.0254; }

constexpr Radians degreesToRadians(Degrees degrees) { return degrees * (kPi / 180.0); }

constexpr Degrees radiansToDegrees(Radians radians) { return radians * (180.0 / kPi); }

} // namespace spectagatr
