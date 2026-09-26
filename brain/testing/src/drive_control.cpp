// drive_control.cpp

#include "drive_control.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <vector>

#include "robot_config.h"

namespace
{

constexpr uint32_t kPeriodMs = 10;

template <std::size_t N> std::vector<std::int8_t> ports(const std::int8_t (&list)[N]) {
    return std::vector<std::int8_t>(list, list + N);
}

int32_t millivolts(double fraction) {
    if (!std::isfinite(fraction)) {
        return 0;
    }
    const double clamped = std::clamp(fraction, -1.0, 1.0);
    return static_cast<int32_t>(std::lround(clamped * robot_config::kMaxVoltageMv));
}

} // namespace

DriveControl::DriveControl(investigatr::InputSource& source, investigatr::Seconds (*now)())
    : now_(now), left_(ports(robot_config::kLeftMotorPorts), robot_config::kGearset),
      right_(ports(robot_config::kRightMotorPorts), robot_config::kGearset),
      navigator_(source, robot_config::navigatorConfig()) {
    left_.set_brake_mode_all(robot_config::kBrakeMode);
    right_.set_brake_mode_all(robot_config::kBrakeMode);
    task_.reset(new pros::Task([this] { run(); }, TASK_PRIORITY_DEFAULT + 1,
                               TASK_STACK_DEPTH_DEFAULT, "drive"));
}

investigatr::CommandId DriveControl::goTo(const investigatr::Pose&          pose,
                                          const investigatr::MotionOptions& options) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    mode_ = DriveMode::kNavigate;
    return navigator_.goTo(pose, options);
}

investigatr::CommandId DriveControl::goToRelative(investigatr::LandmarkId           landmark,
                                                  const investigatr::Pose&          offset,
                                                  const investigatr::MotionOptions& options) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    mode_ = DriveMode::kNavigate;
    return navigator_.goToRelative(landmark, offset, options);
}

investigatr::CommandId DriveControl::follow(const investigatr::Path&          path,
                                            const investigatr::MotionOptions& options) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    mode_ = DriveMode::kNavigate;
    return navigator_.follow(path, options);
}

void DriveControl::manual(const investigatr::DriveCommand& demand) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    navigator_.cancel();
    mode_   = DriveMode::kManual;
    manual_ = demand;
}

void DriveControl::stop() {
    std::lock_guard<pros::Mutex> lock(mutex_);
    navigator_.cancel();
    mode_   = DriveMode::kDisabled;
    manual_ = {};
    apply({});
}

investigatr::MotionStatus DriveControl::status() const {
    std::lock_guard<pros::Mutex> lock(mutex_);
    return navigator_.status();
}

DriveMode DriveControl::mode() const {
    std::lock_guard<pros::Mutex> lock(mutex_);
    return mode_;
}

// The Navigator is updated in every mode so its status and landmark requests
// stay current; its demand only reaches the motors in kNavigate.
void DriveControl::run() {
    uint32_t wake = pros::millis();
    while (true) {
        {
            std::lock_guard<pros::Mutex> lock(mutex_);

            const investigatr::DriveCommand navigation = navigator_.update(now_());
            investigatr::DriveCommand       demand;
            if (mode_ == DriveMode::kNavigate) {
                demand = navigation;
            } else if (mode_ == DriveMode::kManual) {
                demand = manual_;
            }
            apply(investigatr::mixTank(demand));
        }
        pros::Task::delay_until(&wake, kPeriodMs);
    }
}

// Zero on both sides stops per kBrakeMode; anything else is a voltage.
void DriveControl::apply(const investigatr::TankOutput& tank) {
    if (tank.left == 0.0 && tank.right == 0.0) {
        left_.brake();
        right_.brake();
        return;
    }
    left_.move_voltage(millivolts(tank.left));
    right_.move_voltage(millivolts(tank.right));
}
