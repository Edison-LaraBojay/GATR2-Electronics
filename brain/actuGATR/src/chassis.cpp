// chassis.cpp

#include "actugatr/chassis.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{

ChassisCommand toBody(const ChassisCommand& command, Radians heading) {
    if (command.frame == ChassisFrame::kBody) {
        return command;
    }
    const double   c = std::cos(heading);
    const double   s = std::sin(heading);
    ChassisCommand out;
    out.frame = ChassisFrame::kBody;
    out.vx    = c * command.vx + s * command.vy;
    out.vy    = -s * command.vx + c * command.vy;
    out.omega = command.omega;
    return out;
}

bool finite(const ChassisCommand& command) {
    return std::isfinite(command.vx) && std::isfinite(command.vy) && std::isfinite(command.omega);
}

bool isZero(const ChassisCommand& command, double epsilon) {
    return std::fabs(command.vx) <= epsilon && std::fabs(command.vy) <= epsilon &&
           std::fabs(command.omega) <= epsilon;
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

double rateLimit(double previous, double target, double rate, Seconds dt) {
    if (rate <= 0) {
        return target;
    }
    const double step = rate * std::max(0.0, dt);
    return std::clamp(target, previous - step, previous + step);
}

} // namespace actugatr
