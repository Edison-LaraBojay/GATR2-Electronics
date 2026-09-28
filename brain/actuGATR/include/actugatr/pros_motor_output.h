// pros_motor_output.h
// MotorOutput over V5 smart motors: one pros::Motor per configured port,
// reversal and cartridge from the drivetrain config, velocity targets through
// the motor's own velocity loop (move_velocity). Only the drive task may use
// it. Source in pros/, PROS builds only.

#pragma once
#include <cstddef>
#include <vector>

#include "actugatr/drivetrain.h"
#include "actugatr/motor_output.h"
#include "pros/motors.hpp"

namespace actugatr
{

class ProsMotorOutput : public MotorOutput {
public:
    explicit ProsMotorOutput(const TankConfig& config);
    explicit ProsMotorOutput(const MecanumConfig& config);

    std::size_t groups() const override { return groups_.size(); }
    void        setVelocity(std::size_t group, double motor_rpm) override;
    void        stop(StopMode mode) override;

private:
    void build(const MotorGroup* const* groups, std::size_t count, Cartridge cartridge);

    std::vector<std::vector<pros::Motor>> groups_;
    double                                max_rpm_ = 0;
};

} // namespace actugatr
