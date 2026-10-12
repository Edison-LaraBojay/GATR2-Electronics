// pose_source.cpp
// REPLACE ME with the robot's odometry, for example:
//
//   OdometryPose currentPose() {
//       const MyPose p = my_odometry.pose();             // your own code
//       return {spectagatr::inchesToMeters(p.x_in),
//               spectagatr::inchesToMeters(p.y_in),
//               spectagatr::degreesToRadians(p.heading_ccw_deg)};
//   }
//   bool currentPoseIsPlaceholder() { return false; }
//
// Check your odometry's axes against PROTOCOL.md first: +x right and +y up on
// the field diagram, heading counterclockwise from +x.

#include "pose_source.h"

OdometryPose currentPose() {
    // Placeholder: a fixed pose 24 in from the left wall, 36 in from the
    // bottom wall, facing +y.
    OdometryPose pose;
    pose.x       = spectagatr::inchesToMeters(24.0);
    pose.y       = spectagatr::inchesToMeters(36.0);
    pose.heading = spectagatr::degreesToRadians(90.0);
    return pose;
}

bool currentPoseIsPlaceholder() { return true; }
