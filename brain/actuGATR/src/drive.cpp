// drive.cpp

#include "actugatr/drive.h"

#include <cmath>

namespace actugatr
{

const char* toString(DriveFault fault) {
    switch (fault) {
    case DriveFault::kNone: return "none";
    case DriveFault::kWrongFrame: return "wrong frame";
    case DriveFault::kNonFinite: return "non-finite command";
    case DriveFault::kUnsupportedMotion: return "unsupported motion";
    case DriveFault::kStale: return "stale command";
    }
    return "?";
}

Drive::Drive(const Kinematics& kinematics, const WheelDrive& wheels, MotorOutput& output,
             const DriveConfig& config)
    : kinematics_(kinematics), wheels_(wheels), output_(output), config_(config),
      max_speed_(actugatr::maxWheelSpeed(wheels)) {}

DriveFault Drive::apply(const ChassisCommand& command, Seconds issued_at, Seconds now) {
    if (command.frame != ChassisFrame::kBody) {
        return fail(DriveFault::kWrongFrame);
    }
    if (!finite(command) || !std::isfinite(issued_at) || !std::isfinite(now)) {
        return fail(DriveFault::kNonFinite);
    }
    if (now - issued_at > config_.command_timeout) {
        return fail(DriveFault::kStale);
    }
    WheelSpeeds wheels;
    if (!kinematics_.toWheels(command, wheels)) {
        return fail(DriveFault::kUnsupportedMotion);
    }
    const double factor = desaturate(wheels, max_speed_);

    bool moving = false;
    for (std::size_t i = 0; i < wheels.count; ++i) {
        moving = moving || std::fabs(wheels.speed[i]) >= config_.stop_below;
    }
    status_.fault      = DriveFault::kNone;
    status_.wheels     = wheels;
    status_.saturation = factor;
    status_.command    = kinematics_.toChassis(wheels);
    if (!moving) {
        output_.stop(config_.stop_mode);
        status_.stopped = true;
        return DriveFault::kNone;
    }
    for (std::size_t i = 0; i < wheels.count && i < output_.groups(); ++i) {
        output_.setVelocity(i, motorRpm(wheels_, wheels.speed[i]));
    }
    status_.stopped = false;
    return DriveFault::kNone;
}

void Drive::stop(DriveFault fault) {
    output_.stop(config_.stop_mode);
    status_.fault   = fault;
    status_.stopped = true;
    status_.wheels  = WheelSpeeds{};
    status_.command = ChassisCommand{};
}

DriveFault Drive::fail(DriveFault fault) {
    stop(fault);
    return fault;
}

} // namespace actugatr
