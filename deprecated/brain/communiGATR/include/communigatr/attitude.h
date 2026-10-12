// attitude.h
// Brain VEX IMU roll and pitch in the robot frame, for TELEMETRY. Display
// and recording only; localization never uses it.
//
// Robot frame: +x forward, +y left, +z up. Roll is about +x, positive left
// side up; pitch is about +y, positive nose down (right handed, as in
// translaGATR/frames.h). The VEX angles are taken to follow the same
// convention in the IMU's own frame. UNVERIFIED on hardware: the bench check
// is in docs/brain_setup.md.
//
// mount_yaw_deg: direction of the IMU +x axis in the robot frame, CCW from
// robot +x (0, 90, 180 or 270 for a square mount). The IMU must lie flat.
// The up direction is rotated exactly; no small angle approximation.

#pragma once
#include <cstdint>

#include "translaGATR/frames.h"

namespace communigatr
{

// VEX IMU angles as PROS reports them.
struct VexAttitude {
    bool     valid     = false; // false while calibrating, missing or on an error
    uint32_t stamp_ms  = 0;     // Brain clock at the read
    double   roll_deg  = 0;
    double   pitch_deg = 0;
};

struct RobotAttitude {
    bool   valid     = false; // false for non-finite input
    double roll_rad  = 0;     // (-pi, pi]
    double pitch_rad = 0;     // [-pi/2, pi/2]
};

RobotAttitude robotAttitudeFromVex(double roll_deg, double pitch_deg, double mount_yaw_deg);

// Robot attitude of a VEX reading; invalid when the reading is.
RobotAttitude robotAttitudeFromVex(const VexAttitude& vex, double mount_yaw_deg);

// TELEMETRY attitude group in centidegrees, rounded. Invalid clears the
// group and its flag.
void setAttitude(translagatr::BrainTelemetry& telemetry, const RobotAttitude& attitude);

} // namespace communigatr
