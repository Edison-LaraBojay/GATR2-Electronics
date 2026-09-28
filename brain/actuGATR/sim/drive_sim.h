// drive_sim.h
// Host drivetrain: wheel speeds follow the motor targets with a first-order
// lag, and the pose integrates the kinematics. Tank or mecanum by the
// Kinematics passed in.

#pragma once

#include "actugatr/drivetrain.h"
#include "actugatr/kinematics.h"
#include "sim/sim_motor_output.h"

namespace actugatr
{

struct DriveSimConfig {
    Seconds time_constant = 0.05;
    Seconds substep       = 0.001;
};

class DriveSim {
public:
    DriveSim(const Kinematics& kinematics, const WheelDrive& wheels, const SimMotorOutput& motors,
             const DriveSimConfig& config = {});

    void step(Seconds dt);

    const Pose&    pose() const { return pose_; }
    void           setPose(const Pose& pose) { pose_ = pose; }
    ChassisCommand velocity() const; // body frame

private:
    const Kinematics&     kinematics_;
    WheelDrive            wheels_;
    const SimMotorOutput& motors_;
    DriveSimConfig        config_;
    WheelSpeeds           speeds_;
    Pose                  pose_;
};

} // namespace actugatr
