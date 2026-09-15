// robot_state.h
// The one continuous fused robot estimate, split across two frames:
//
//   O = smooth local odometry frame; T_odom_robot is continuous and updated
//       by localization every accepted observation
//   F = corrected/configured field frame; T_field_odom re-anchors O in F
//       and changes only on explicit events such as a commanded pose init
//
//   T_field_robot = T_field_odom * T_odom_robot
//
// Odometry-anchored data (latched targets, exposure-time lookups) lives in
// O, so a field re-anchor cannot change its physical error relative to the
// robot. odometry_epoch increments whenever O itself becomes discontinuous
// (hard reset, device time regression from a source reboot); anything
// latched in O is valid only while the epoch matches. anchor_revision
// increments on every re-anchor, so consumers can tell which T_field_odom
// a field-frame value was derived under.
//
// This is the lightweight snapshot. History and timestamped lookups live in
// PoseHistory behind the RobotStateFeed; localization is their only writer.
// The planar pose always identifies the configured fixed chassis origin.

#pragma once
#include <cstdint>

#include "math/quaternion.h"
#include "math/se3.h"
#include "math/transforms.h"
#include "state/attitude.h"

namespace navigatr
{

struct RobotState {
    Pose2D odom_pose;        // T_odom_robot
    Pose2D field_from_odom;  // T_field_odom, identity until re-anchored

    uint64_t odometry_epoch  = 0;
    uint64_t anchor_revision = 0;

    double vx_m_s         = 0.0;   // odometry frame
    double vy_m_s         = 0.0;
    double yaw_rate_rad_s = 0.0;

    double confidence  = 0.0;
    bool   valid       = false;   // an estimator has produced a pose
    bool   initialized = false;   // field anchor was set from a request

    // Effective time of odom_pose: the source clock of the newest folded
    // measurement and, when the estimator has a clock mapping, its host
    // time. Unset host time means the pose cannot be placed on the host
    // axis; nothing substitutes the loop time for it.
    MonotonicTime measuredAt;
    MonotonicTime measuredAtHost;

    // Measured tilt (odometry reference, yaw equal to odom_pose.heading)
    // with its own validity and age; assumed level when no source exists.
    Attitude attitude;

    Pose2D fieldPose() const { return compose(field_from_odom, odom_pose); }

    // Full SE(3) body pose in the odometry frame at ground height: yaw from
    // the planar heading and roll/pitch from the attitude when it is
    // measured, level otherwise. Height stays zero: vertical position is an
    // explicit ground-plane assumption, never estimated here.
    Transform3 T_odom_robot3() const {
        Transform3 T;
        if (attitude.valid) {
            double roll = 0.0, pitch = 0.0, yaw = 0.0;
            attitudeEuler(attitude, roll, pitch, yaw);
            T.R = rotationFromEuler(roll, pitch, odom_pose.heading_rad);
        } else {
            T.R = rotationFromEuler(0.0, 0.0, odom_pose.heading_rad);
        }
        T.x_m = odom_pose.x_m;
        T.y_m = odom_pose.y_m;
        T.z_m = 0.0;
        return T;
    }
};

} // namespace navigatr
