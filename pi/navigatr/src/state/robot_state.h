// robot_state.h
// The one continuous fused robot estimate, split across two frames:
//
//   O = smooth local odometry frame; T_odom_robot is continuous and updated
//       by localization prediction every cycle
//   F = corrected/configured field frame; T_field_odom re-anchors O in F
//       and changes only on explicit events such as a commanded pose init
//
//   T_field_robot = T_field_odom * T_odom_robot
//
// Odometry-anchored data (latched targets, exposure-time lookups) lives in
// O, so a field re-anchor cannot change its physical error relative to the
// robot. odometry_epoch increments whenever O itself becomes discontinuous
// (hard reset, device time regression from a Pico reboot); anything latched
// in O is valid only while the epoch matches.
//
// history holds recent T_odom_robot poses on the host clock, appended by the
// framework after each cycle's estimate settles, so perception evidence with
// an exposure timestamp can be evaluated against the pose at exposure.

#pragma once
#include <cstdint>
#include <deque>

#include "math/transforms.h"

namespace navigatr
{

struct TimedOdomPose {
    MonotonicTime at;   // host clock
    Pose2D        T_odom_robot;
};

struct RobotState {
    Pose2D odom_pose;        // T_odom_robot
    Pose2D field_from_odom;  // T_field_odom, identity until re-anchored

    uint64_t odometry_epoch = 0;

    double vx_m_s         = 0.0;   // odometry frame
    double vy_m_s         = 0.0;
    double yaw_rate_rad_s = 0.0;

    double confidence  = 0.0;
    bool   valid       = false;   // an estimator has produced a pose
    bool   initialized = false;   // field anchor was set from a brain command

    MonotonicTime measuredAt;   // device clock of the newest folded measurement

    std::deque<TimedOdomPose> history;   // host clock, oldest first

    Pose2D fieldPose() const { return compose(field_from_odom, odom_pose); }

    // T_odom_robot at a host timestamp, interpolated between history
    // entries and clamped to the ends. False when there is no history or
    // the timestamp is older than everything retained.
    bool odomPoseAt(MonotonicTime t, Pose2D& out) const {
        if (history.empty()) {
            return false;
        }
        if (t <= history.front().at) {
            if ((history.front().at - t) > kHistoryClampMs) {
                return false;
            }
            out = history.front().T_odom_robot;
            return true;
        }
        if (t >= history.back().at) {
            out = history.back().T_odom_robot;
            return true;
        }
        for (std::size_t i = 1; i < history.size(); ++i) {
            if (t <= history[i].at) {
                const TimedOdomPose& a = history[i - 1];
                const TimedOdomPose& b = history[i];
                const double span      = static_cast<double>(b.at - a.at);
                const double f = span <= 0.0 ? 1.0 : static_cast<double>(t - a.at) / span;
                out.x_m = a.T_odom_robot.x_m + (b.T_odom_robot.x_m - a.T_odom_robot.x_m) * f;
                out.y_m = a.T_odom_robot.y_m + (b.T_odom_robot.y_m - a.T_odom_robot.y_m) * f;
                out.heading_rad = wrapAngle(
                    a.T_odom_robot.heading_rad +
                    wrapAngle(b.T_odom_robot.heading_rad - a.T_odom_robot.heading_rad) * f);
                return true;
            }
        }
        out = history.back().T_odom_robot;
        return true;
    }

    static constexpr int64_t     kHistoryClampMs   = 50;
    static constexpr std::size_t kHistoryCapacity  = 512;
};

} // namespace navigatr
