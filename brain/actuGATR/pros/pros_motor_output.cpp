// pros_motor_output.cpp

#include "actugatr/pros_motor_output.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{
namespace
{

pros::v5::MotorGears gears(Cartridge cartridge) {
    switch (cartridge) {
    case Cartridge::kRed: return pros::v5::MotorGears::red;
    case Cartridge::kGreen: return pros::v5::MotorGears::green;
    case Cartridge::kBlue: return pros::v5::MotorGears::blue;
    }
    return pros::v5::MotorGears::blue;
}

pros::v5::MotorBrake brakeMode(StopMode mode) {
    switch (mode) {
    case StopMode::kBrake: return pros::v5::MotorBrake::brake;
    case StopMode::kCoast: return pros::v5::MotorBrake::coast;
    case StopMode::kHold: return pros::v5::MotorBrake::hold;
    }
    return pros::v5::MotorBrake::brake;
}

} // namespace

ProsMotorOutput::ProsMotorOutput(const TankConfig& config) {
    const MotorGroup* list[kMaxWheelGroups] = {};
    const std::size_t count                 = actugatr::groups(config, list);
    build(list, count, config.wheels.cartridge);
}

ProsMotorOutput::ProsMotorOutput(const MecanumConfig& config) {
    const MotorGroup* list[kMaxWheelGroups] = {};
    const std::size_t count                 = actugatr::groups(config, list);
    build(list, count, config.wheels.cartridge);
}

void ProsMotorOutput::build(const MotorGroup* const* list, std::size_t count, Cartridge cartridge) {
    max_rpm_ = static_cast<double>(static_cast<uint16_t>(cartridge));
    groups_.resize(count);
    for (std::size_t g = 0; g < count; ++g) {
        for (std::size_t i = 0; i < list[g]->count && i < kMaxMotorsPerGroup; ++i) {
            const MotorPort& m    = list[g]->motors[i];
            const int8_t     port = static_cast<int8_t>(m.reversed ? -m.port : m.port);
            groups_[g].emplace_back(port, gears(cartridge));
        }
    }
}

void ProsMotorOutput::setVelocity(std::size_t group, double motor_rpm) {
    if (group >= groups_.size()) {
        return;
    }
    const double  clamped = std::isfinite(motor_rpm) ? std::clamp(motor_rpm, -max_rpm_, max_rpm_) : 0;
    const int32_t rpm     = static_cast<int32_t>(std::lround(clamped));
    for (const pros::Motor& motor : groups_[group]) {
        motor.move_velocity(rpm);
    }
}

void ProsMotorOutput::stop(StopMode mode) {
    for (const auto& group : groups_) {
        for (const pros::Motor& motor : group) {
            motor.set_brake_mode(brakeMode(mode));
            motor.brake();
        }
    }
}

} // namespace actugatr
