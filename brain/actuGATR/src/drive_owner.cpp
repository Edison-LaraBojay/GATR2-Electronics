// drive_owner.cpp

#include "actugatr/drive_owner.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{
namespace
{

double fraction(double v) {
    return std::isfinite(v) ? std::clamp(v, -1.0, 1.0) : 0.0;
}

} // namespace

const char* toString(DriveMode mode) {
    switch (mode) {
    case DriveMode::kDisabled: return "disabled";
    case DriveMode::kManual: return "manual";
    case DriveMode::kNavigate: return "navigate";
    }
    return "?";
}

DriveOwner::DriveOwner(Motion& motion, Drive& drive, const DriveOwnerConfig& config)
    : motion_(motion), drive_(drive), config_(config) {}

void DriveOwner::disable() {
    motion_.cancel();
    mode_   = DriveMode::kDisabled;
    manual_ = ManualDemand{};
}

void DriveOwner::manual(const ManualDemand& demand, Seconds now) {
    motion_.cancel();
    mode_           = DriveMode::kManual;
    manual_.forward = fraction(demand.forward);
    manual_.strafe  = fraction(demand.strafe);
    manual_.turn    = fraction(demand.turn);
    manual_at_      = now;
}

CommandId DriveOwner::goToDirect(const Pose& destination, const Reference& relative_to,
                                 const MoveOptions& options) {
    mode_ = DriveMode::kNavigate;
    return motion_.goToDirect(destination, relative_to, options);
}

CommandId DriveOwner::goToAvoiding(const Pose& destination, const Reference& relative_to,
                                   const MoveOptions& options) {
    mode_ = DriveMode::kNavigate;
    return motion_.goToAvoiding(destination, relative_to, options);
}

void DriveOwner::cancel() {
    disable();
}

void DriveOwner::step(Seconds now) {
    // Motion runs in every mode so its field copy and status stay current.
    const ChassisCommand navigation = motion_.update(now);
    switch (mode_) {
    case DriveMode::kNavigate: drive_.apply(navigation, now, now); break;
    case DriveMode::kManual: {
        if (now - manual_at_ > config_.manual_timeout) {
            drive_.stop(DriveFault::kStale);
            break;
        }
        ChassisCommand command;
        command.vx    = manual_.forward * config_.manual_speed;
        command.vy    = drive_.kinematics().holonomic() ? manual_.strafe * config_.manual_speed : 0;
        command.omega = manual_.turn * config_.manual_omega;
        drive_.apply(command, now, now);
        break;
    }
    case DriveMode::kDisabled: drive_.stop(); break;
    }
}

} // namespace actugatr
