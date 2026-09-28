// sim_motor_output.h
// Host MotorOutput: records velocity targets and stops.

#pragma once
#include <cstddef>

#include "actugatr/motor_output.h"

namespace actugatr
{

class SimMotorOutput : public MotorOutput {
public:
    explicit SimMotorOutput(std::size_t groups) : groups_(groups) {}

    std::size_t groups() const override { return groups_; }
    void        setVelocity(std::size_t group, double motor_rpm) override;
    void        stop(StopMode mode) override;

    double   rpm(std::size_t group) const { return group < kMaxWheelGroups ? rpm_[group] : 0; }
    bool     stopped() const { return stopped_; }
    StopMode stopMode() const { return mode_; }
    int      writes() const { return writes_; }
    int      stops() const { return stops_; }

private:
    std::size_t groups_;
    double      rpm_[kMaxWheelGroups] = {};
    bool        stopped_              = true;
    StopMode    mode_                 = StopMode::kBrake;
    int         writes_               = 0;
    int         stops_                = 0;
};

} // namespace actugatr
