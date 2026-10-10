// units.h
// SI values (meters, radians) to and from the wire's fixed point (mm,
// centidegrees). Publishers and receivers convert here and nowhere else.

#pragma once
#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::protocol
{

using Meters  = double;
using Radians = double;

constexpr double kPi = 3.14159265358979323846;

// Rounded to the nearest mm, saturated to +-kMaxCoordinateMm + 1 so an
// out-of-range input stays out of range. Callers reject non-finite input.
Millimeters toMillimeters(Meters meters);

// Rounded to the nearest centidegree and wrapped into (-18000, 18000].
// Callers reject non-finite input.
Centidegrees toCentidegrees(Radians radians);

Meters  toMeters(Millimeters mm);
Radians toRadians(Centidegrees cdeg);

} // namespace spectagatr::protocol
