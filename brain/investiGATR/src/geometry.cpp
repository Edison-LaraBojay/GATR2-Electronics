// geometry.cpp

#include "investigatr/geometry.h"

#include <cmath>

namespace investigatr
{

Radians wrapAngle(Radians angle) {
    double wrapped = std::remainder(angle, 2.0 * kPi);
    if (wrapped <= -kPi) {
        wrapped += 2.0 * kPi;
    }
    return wrapped;
}

Pose compose(const Pose& a, const Pose& b) {
    const double c = std::cos(a.heading);
    const double s = std::sin(a.heading);
    Pose         out;
    out.x       = a.x + c * b.x - s * b.y;
    out.y       = a.y + s * b.x + c * b.y;
    out.heading = wrapAngle(a.heading + b.heading);
    return out;
}

Pose inverse(const Pose& p) {
    const double c = std::cos(p.heading);
    const double s = std::sin(p.heading);
    Pose         out;
    out.x       = -c * p.x - s * p.y;
    out.y       = s * p.x - c * p.y;
    out.heading = wrapAngle(-p.heading);
    return out;
}

Pose between(const Pose& a, const Pose& b) {
    return compose(inverse(a), b);
}

bool finite(const Pose& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.heading);
}

} // namespace investigatr
