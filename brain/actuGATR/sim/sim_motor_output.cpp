// sim_motor_output.cpp

#include "sim/sim_motor_output.h"

namespace actugatr
{

void SimMotorOutput::setVelocity(std::size_t group, double motor_rpm) {
    if (group < groups_ && group < kMaxWheelGroups) {
        rpm_[group] = motor_rpm;
        stopped_    = false;
        ++writes_;
    }
}

void SimMotorOutput::stop(StopMode mode) {
    for (double& r : rpm_) {
        r = 0;
    }
    stopped_ = true;
    mode_    = mode;
    ++stops_;
}

} // namespace actugatr
