// chassis.h
// Chassis velocity command with explicit units and frame, and rate limits.

#pragma once
#include <cstdint>

#include "investigatr/geometry.h"

namespace actugatr
{

using investigatr::Meters;
using investigatr::MetersPerSecond;
using investigatr::Pose;
using investigatr::Radians;
using investigatr::RadiansPerSecond;
using investigatr::Seconds;

enum class ChassisFrame : uint8_t { kBody, kField };

// vx and vy along the frame's axes (body: forward and left), omega CCW.
struct ChassisCommand {
    ChassisFrame     frame = ChassisFrame::kBody;
    MetersPerSecond  vx    = 0;
    MetersPerSecond  vy    = 0;
    RadiansPerSecond omega = 0;
};

// A field command rotated into the body frame of a robot at heading. Body
// commands pass through.
ChassisCommand toBody(const ChassisCommand& command, Radians heading);

bool finite(const ChassisCommand& command);

// True when every component is within epsilon of zero.
bool isZero(const ChassisCommand& command, double epsilon = 1e-6);

// Limits increases of |target| to rate per second. Decreases are immediate,
// a sign reversal drops to 0 first. rate 0 = no limit.
double slewLimit(double previous, double target, double rate, Seconds dt);

// Moves previous toward target by at most rate * dt. rate 0 = no limit.
double rateLimit(double previous, double target, double rate, Seconds dt);

} // namespace actugatr
