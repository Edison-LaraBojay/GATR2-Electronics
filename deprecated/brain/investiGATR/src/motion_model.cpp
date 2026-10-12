// motion_model.cpp

#include "investigatr/motion_model.h"

#include <algorithm>
#include <cmath>

namespace investigatr
{
namespace
{

bool nonNegative(double v) {
    return std::isfinite(v) && v >= 0;
}

bool positive(double v) {
    return std::isfinite(v) && v > 0;
}

} // namespace

Meters enclosingRadius(const MotionModel& model) {
    const Footprint& f  = model.footprint;
    const double     dx = std::max(f.front, f.back);
    const double     dy = std::max(f.left, f.right);
    return std::hypot(dx, dy) + model.clearance;
}

bool valid(const MotionModel& m, const char** why) {
    struct Check {
        bool        ok;
        const char* what;
    };
    const Footprint& f          = m.footprint;
    const Check      checks[] = {
        {nonNegative(f.front) && nonNegative(f.back) && nonNegative(f.left) &&
             nonNegative(f.right),
         "footprint sides must be finite and >= 0"},
        {f.front + f.back > 0 && f.left + f.right > 0, "footprint must have an area"},
        {nonNegative(m.clearance), "clearance must be >= 0"},
        {nonNegative(m.min_turn_radius), "min_turn_radius must be >= 0"},
        {positive(m.limits.max_speed), "max_speed must be > 0"},
        {positive(m.limits.max_accel), "max_accel must be > 0"},
        {positive(m.limits.max_omega), "max_omega must be > 0"},
        {positive(m.limits.max_alpha), "max_alpha must be > 0"},
    };
    for (const Check& check : checks) {
        if (!check.ok) {
            if (why != nullptr) {
                *why = check.what;
            }
            return false;
        }
    }
    if (why != nullptr) {
        *why = nullptr;
    }
    return true;
}

} // namespace investigatr
