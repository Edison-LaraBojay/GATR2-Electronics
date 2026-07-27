// angles.h
// Angle helpers, radians internally, wire units at the boundary.

#pragma once
#include <cmath>
#include <cstdint>

namespace navigatr
{

constexpr double kPi = 3.14159265358979323846;

// (-pi, pi]
inline double wrapAngle(double rad) {
    while (rad > kPi) {
        rad -= 2.0 * kPi;
    }
    while (rad <= -kPi) {
        rad += 2.0 * kPi;
    }
    return rad;
}

inline double degToRad(double deg) { return deg * kPi / 180.0; }
inline double radToDeg(double rad) { return rad * 180.0 / kPi; }

inline double cdegToRad(int32_t cdeg) { return degToRad(cdeg / 100.0); }

// wrapped to (-18000, 18000]
inline int32_t radToCdeg(double rad) {
    return static_cast<int32_t>(std::llround(radToDeg(wrapAngle(rad)) * 100.0));
}

} // namespace navigatr
