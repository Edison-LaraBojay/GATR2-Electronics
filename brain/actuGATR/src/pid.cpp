// pid.cpp

#include "actugatr/pid.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{

Pid::Pid(const PidGains& gains, bool angular) : gains_(gains), angular_(angular) {}

void Pid::reset() {
    integral_      = 0;
    previous_      = 0;
    have_previous_ = false;
}

double Pid::update(double error, Seconds dt) {
    double out = gains_.kP * error;
    if (dt > 0) {
        if (gains_.kI != 0) {
            const double bound = std::fabs(gains_.integral_limit / gains_.kI);
            integral_          = std::clamp(integral_ + error * dt, -bound, bound);
            out += gains_.kI * integral_;
        }
        if (have_previous_) {
            const double change =
                angular_ ? investigatr::wrapAngle(error - previous_) : error - previous_;
            out += gains_.kD * change / dt;
        }
    }
    previous_          = error;
    have_previous_     = true;
    const double limit = std::fabs(gains_.output_limit);
    return std::clamp(out, -limit, limit);
}

bool valid(const PidGains& g) {
    const auto nonNegative = [](double v) { return std::isfinite(v) && v >= 0; };
    return nonNegative(g.kP) && nonNegative(g.kI) && nonNegative(g.kD) &&
           nonNegative(g.integral_limit) && nonNegative(g.output_limit) && g.output_limit > 0;
}

} // namespace actugatr
