// differential_drive_sim.cpp

#include "sim/differential_drive_sim.h"

#include <algorithm>
#include <cmath>

namespace investigatr
{

namespace
{

Pose inverse(const Pose& p) {
    const double c = std::cos(p.heading);
    const double s = std::sin(p.heading);
    return Pose{-c * p.x - s * p.y, s * p.x - c * p.y, wrapAngle(-p.heading)};
}

} // namespace

DifferentialDriveSim::DifferentialDriveSim(const DifferentialDriveConfig& config,
                                           const Pose&                    start)
    : config_(config) {
    setPose(start);
}

void DifferentialDriveSim::step(const TankOutput& output, Seconds dt) {
    if (dt <= 0) {
        return;
    }
    const double left_target  = std::clamp(output.left, -1.0, 1.0) * config_.max_wheel_speed;
    const double right_target = std::clamp(output.right, -1.0, 1.0) * config_.max_wheel_speed;
    const double blend =
        config_.wheel_time_constant > 0 ? 1.0 - std::exp(-dt / config_.wheel_time_constant) : 1.0;
    left_speed_ += (left_target - left_speed_) * blend;
    right_speed_ += (right_target - right_speed_) * blend;

    const Meters  dl   = left_speed_ * dt;
    const Meters  dr   = right_speed_ * dt;
    const Radians turn = (dr - dl) / config_.track_width;
    const Radians mid  = center_.heading + turn / 2.0;
    center_.x += (dl + dr) / 2.0 * std::cos(mid);
    center_.y += (dl + dr) / 2.0 * std::sin(mid);
    center_.heading = wrapAngle(center_.heading + turn);
    left_travel_ += dl;
    right_travel_ += dr;
}

Pose DifferentialDriveSim::pose() const { return compose(center_, config_.origin_offset); }

void DifferentialDriveSim::setPose(const Pose& pose) {
    center_ = compose(pose, inverse(config_.origin_offset));
}

} // namespace investigatr
