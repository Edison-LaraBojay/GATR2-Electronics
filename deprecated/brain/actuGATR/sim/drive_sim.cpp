// drive_sim.cpp

#include "sim/drive_sim.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{

DriveSim::DriveSim(const Kinematics& kinematics, const WheelDrive& wheels,
                   const SimMotorOutput& motors, const DriveSimConfig& config)
    : kinematics_(kinematics), wheels_(wheels), motors_(motors), config_(config) {
    speeds_.count = kinematics.groups();
}

ChassisCommand DriveSim::velocity() const {
    return kinematics_.toChassis(speeds_);
}

void DriveSim::step(Seconds dt) {
    Seconds left = dt;
    while (left > 1e-12) {
        const Seconds h     = std::min(left, config_.substep);
        const double  alpha = config_.time_constant > 0 ? 1.0 - std::exp(-h / config_.time_constant)
                                                        : 1.0;
        for (std::size_t i = 0; i < speeds_.count; ++i) {
            const double target = wheelSpeed(wheels_, motors_.rpm(i));
            speeds_.speed[i] += alpha * (target - speeds_.speed[i]);
        }
        const ChassisCommand body = kinematics_.toChassis(speeds_);
        const double         mid  = pose_.heading + 0.5 * body.omega * h;
        const double         c    = std::cos(mid);
        const double         s    = std::sin(mid);
        pose_.x += (c * body.vx - s * body.vy) * h;
        pose_.y += (s * body.vx + c * body.vy) * h;
        pose_.heading = investigatr::wrapAngle(pose_.heading + body.omega * h);
        left -= h;
    }
}

} // namespace actugatr
