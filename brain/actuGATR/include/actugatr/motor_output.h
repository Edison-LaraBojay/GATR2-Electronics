// motor_output.h
// Motor I/O under Drive: one velocity target per wheel group, in motor rpm
// with positive driving the robot forward, or a stop. Reversal is the
// implementation's. Drive is its only caller.

#pragma once
#include <cstddef>

#include "actugatr/drivetrain.h"

namespace actugatr
{

class MotorOutput {
public:
    virtual ~MotorOutput() = default;

    virtual std::size_t groups() const = 0;

    virtual void setVelocity(std::size_t group, double motor_rpm) = 0;

    // Every group.
    virtual void stop(StopMode mode) = 0;
};

} // namespace actugatr
