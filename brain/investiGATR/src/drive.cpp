// drive.cpp

#include "investigatr/drive.h"

#include <algorithm>
#include <cmath>

namespace investigatr
{

TankOutput mixTank(const DriveCommand& command) {
    TankOutput out;
    out.left           = command.forward - command.turn;
    out.right          = command.forward + command.turn;
    const double scale = std::max({1.0, std::fabs(out.left), std::fabs(out.right)});
    out.left /= scale;
    out.right /= scale;
    return out;
}

double slewLimit(double previous, double target, double rate, Seconds dt) {
    if (rate <= 0) {
        return target;
    }
    if ((previous > 0 && target < 0) || (previous < 0 && target > 0)) {
        previous = 0;
    }
    if (std::fabs(target) <= std::fabs(previous)) {
        return target;
    }
    const double step = rate * std::max(0.0, dt);
    if (target > previous) {
        return std::min(target, previous + step);
    }
    return std::max(target, previous - step);
}

} // namespace investigatr
