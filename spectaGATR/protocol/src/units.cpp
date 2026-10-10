// units.cpp

#include "spectagatr/protocol/units.h"

#include <cmath>

namespace spectagatr::protocol
{

Millimeters toMillimeters(Meters meters) {
    const double limit = static_cast<double>(kMaxCoordinateMm) + 1.0;
    double       mm    = std::round(meters * 1000.0);
    if (!(mm <= limit)) {
        mm = limit;
    }
    if (!(mm >= -limit)) {
        mm = -limit;
    }
    return static_cast<Millimeters>(mm);
}

Centidegrees toCentidegrees(Radians radians) {
    // Wrap in floating point first so huge inputs cannot overflow.
    const double turn    = 2.0 * kPi;
    const double wrapped = std::remainder(radians, turn); // [-pi, pi]
    long         cdeg    = std::lround(wrapped * (18000.0 / kPi));
    cdeg %= 36000;
    if (cdeg <= -kHalfTurnCdeg) {
        cdeg += 36000;
    } else if (cdeg > kHalfTurnCdeg) {
        cdeg -= 36000;
    }
    return static_cast<Centidegrees>(cdeg);
}

Meters toMeters(Millimeters mm) { return static_cast<double>(mm) / 1000.0; }

Radians toRadians(Centidegrees cdeg) { return static_cast<double>(cdeg) * (kPi / 18000.0); }

} // namespace spectagatr::protocol
