// pid.h
// PID with a bounded integral term and output. Angular controllers take
// the derivative of the wrapped error difference.

#pragma once

#include "investigatr/geometry.h"

namespace investigatr
{

struct PidGains {
    double kP             = 0;
    double kI             = 0;
    double kD             = 0;
    double integral_limit = 0; // max |I term|, 0 = no integral
    double output_limit   = 1; // max |output|
};

class Pid {
public:
    explicit Pid(const PidGains& gains = {}, bool angular = false);

    void reset();

    // dt 0 is P only. The first sample after reset has no D term.
    double update(double error, Seconds dt);

    const PidGains& gains() const { return gains_; }

private:
    PidGains gains_;
    bool     angular_       = false;
    double   integral_      = 0;
    double   previous_      = 0;
    bool     have_previous_ = false;
};

} // namespace investigatr
