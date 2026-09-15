// attitude.h
// Body attitude as part of the robot estimate: the rotation of the robot
// body frame relative to a named reference, its own timing, validity and
// provenance. Attitude ages separately from the planar pose; repeating an
// old attitude beside a newer planar pose does not make it fresh, and a
// level assumption is labeled as such, never presented as measured.
//
// Convention (R = Rz(yaw) * Ry(pitch) * Rx(roll), right handed, body +x
// forward, +y left, +z up): yaw is about the reference z axis, positive
// counterclockwise from above; pitch is about +y, positive tipping the
// nose down; roll is about +x, positive lifting the left side. Inspection
// reports roll, pitch and yaw in degrees under exactly this convention.

#pragma once
#include <cstdint>
#include <string>

#include "core/time.h"
#include "math/quaternion.h"

namespace navigatr
{

struct Attitude {
    bool valid = false;

    // Rotation of the body in the reference frame. In RobotState the
    // reference is the odometry frame: yaw is the planar heading, roll and
    // pitch the measured tilt.
    Quaternion  q_reference_body;
    std::string reference;   // "odometry", "gravity"

    MonotonicTime measuredAt;         // host clock the tilt was measured at
    MonotonicTime measuredAtSource;   // source clock, for diagnostics
    std::string   source;             // producing sensor or model id
    uint64_t      epoch   = 0;        // source discontinuity generation
    double        quality = 0.0;      // 0..1, producer defined

    // True when no measurement exists and the tilt is an assumption.
    bool assumed_level = false;
};

// Level attitude at a planar heading, explicitly marked as assumed.
inline Attitude assumedLevelAttitude(double yaw_rad) {
    Attitude a;
    a.valid            = false;
    a.q_reference_body = yawQuaternion(yaw_rad);
    a.reference        = "odometry";
    a.assumed_level    = true;
    return a;
}

inline void attitudeEuler(const Attitude& a, double& roll_rad, double& pitch_rad,
                          double& yaw_rad) {
    eulerFromQuaternion(a.q_reference_body, roll_rad, pitch_rad, yaw_rad);
}

} // namespace navigatr
