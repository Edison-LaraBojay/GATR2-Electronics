// robot_state.h
// The one continuous fused robot estimate, field frame, meters and radians.
// Persists across cycles; localization prediction replaces it and pose
// correction refines it each cycle.

#pragma once
#include "math/transforms.h"

namespace navigatr
{

struct RobotState {
    Pose2D pose;   // T_field_robot
    double vx_m_s        = 0.0;   // field frame
    double vy_m_s        = 0.0;
    double yaw_rate_rad_s = 0.0;

    double confidence  = 0.0;
    bool   valid       = false;   // an estimator has produced a pose
    bool   initialized = false;   // pose was set from a brain command

    MonotonicTime measuredAt;   // device clock of the newest folded measurement
};

} // namespace navigatr
