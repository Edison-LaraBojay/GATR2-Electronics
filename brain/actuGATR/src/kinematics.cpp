// kinematics.cpp

#include "actugatr/kinematics.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{
namespace
{

constexpr double kLateralEpsilon = 1e-6;

enum Tank : std::size_t { kLeft, kRight };
enum Mecanum : std::size_t { kFrontLeft, kFrontRight, kRearLeft, kRearRight };

} // namespace

TankKinematics::TankKinematics(Meters track_width) : track_width_(track_width) {}

bool TankKinematics::toWheels(const ChassisCommand& body, WheelSpeeds& out) const {
    if (body.frame != ChassisFrame::kBody || std::fabs(body.vy) > kLateralEpsilon) {
        return false;
    }
    const double half = 0.5 * track_width_;
    out               = WheelSpeeds{};
    out.count         = 2;
    out.speed[kLeft]  = body.vx - body.omega * half;
    out.speed[kRight] = body.vx + body.omega * half;
    return true;
}

ChassisCommand TankKinematics::toChassis(const WheelSpeeds& w) const {
    ChassisCommand out;
    out.vx    = 0.5 * (w.speed[kLeft] + w.speed[kRight]);
    out.omega = (w.speed[kRight] - w.speed[kLeft]) / track_width_;
    return out;
}

MecanumKinematics::MecanumKinematics(Meters track_width, Meters wheelbase)
    : lever_(0.5 * (track_width + wheelbase)) {}

bool MecanumKinematics::toWheels(const ChassisCommand& body, WheelSpeeds& out) const {
    if (body.frame != ChassisFrame::kBody) {
        return false;
    }
    const double turn      = body.omega * lever_;
    out                    = WheelSpeeds{};
    out.count              = 4;
    out.speed[kFrontLeft]  = body.vx - body.vy - turn;
    out.speed[kFrontRight] = body.vx + body.vy + turn;
    out.speed[kRearLeft]   = body.vx + body.vy - turn;
    out.speed[kRearRight]  = body.vx - body.vy + turn;
    return true;
}

ChassisCommand MecanumKinematics::toChassis(const WheelSpeeds& w) const {
    const double   fl = w.speed[kFrontLeft];
    const double   fr = w.speed[kFrontRight];
    const double   rl = w.speed[kRearLeft];
    const double   rr = w.speed[kRearRight];
    ChassisCommand out;
    out.vx    = 0.25 * (fl + fr + rl + rr);
    out.vy    = 0.25 * (-fl + fr + rl - rr);
    out.omega = 0.25 * (-fl + fr - rl + rr) / lever_;
    return out;
}

double desaturate(WheelSpeeds& wheels, MetersPerSecond max_speed) {
    double peak = 0;
    for (std::size_t i = 0; i < wheels.count; ++i) {
        peak = std::max(peak, std::fabs(wheels.speed[i]));
    }
    if (max_speed <= 0 || peak <= max_speed) {
        return 1.0;
    }
    const double factor = max_speed / peak;
    for (std::size_t i = 0; i < wheels.count; ++i) {
        wheels.speed[i] *= factor;
    }
    return factor;
}

} // namespace actugatr
