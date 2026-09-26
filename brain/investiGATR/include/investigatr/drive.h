// drive.h
// Platform independent drive demand, tank mixing, and output slew.

#pragma once

#include "investigatr/geometry.h"

namespace investigatr
{

// Fractions of full output in [-1, 1]. +forward drives robot +x, +turn
// rotates counterclockwise.
struct DriveCommand {
    double forward = 0;
    double turn    = 0;
};

struct TankOutput {
    double left  = 0;
    double right = 0;
};

// left = forward - turn, right = forward + turn, scaled so max |side| <= 1.
TankOutput mixTank(const DriveCommand& command);

// Limits increases of |target| to rate per second. Decreases are immediate,
// a sign reversal drops to 0 first. rate 0 = no limit.
double slewLimit(double previous, double target, double rate, Seconds dt);

} // namespace investigatr
