// demo_sources.h
// Pose math of the SYNTHETIC and VEX IMU sources. No PROS code.

#pragma once
#include "spectagatr/pose.h"

struct DemoPose {
    spectagatr::Meters   x = 0;
    spectagatr::Meters   y = 0;
    spectagatr::Attitude attitude;
    bool                 full_attitude = false; // else heading only: attitude.yaw
};

// Figure eight at synthetic time t, heading along the path. Full attitude
// with a roll and pitch wobble in every other phase.
DemoPose syntheticPose(spectagatr::Milliseconds t);

// VEX IMU as PROS reports it, degrees: rotation clockwise positive, roll and
// pitch in the IMU frame.
struct VexImuReading {
    spectagatr::Degrees rotation = 0;
    spectagatr::Degrees roll     = 0;
    spectagatr::Degrees pitch    = 0;
};

// Full attitude at the configured fixed x/y.
DemoPose imuPose(const VexImuReading& reading);
