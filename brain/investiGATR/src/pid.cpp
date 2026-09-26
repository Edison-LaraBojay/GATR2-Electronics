// pid.cpp

#include "investigatr/pid.h"

#include <algorithm>
#include <cmath>

namespace investigatr
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
            const double change = angular_ ? wrapAngle(error - previous_) : error - previous_;
            out += gains_.kD * change / dt;
        }
    }
    previous_      = error;
    have_previous_ = true;
    const double limit = std::fabs(gains_.output_limit);
    return std::clamp(out, -limit, limit);
}

} // namespace investigatr
