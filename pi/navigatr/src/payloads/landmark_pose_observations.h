// payloads/landmark_pose_observations.h
// Association output for fiducial evidence: a full planar landmark pose in
// the odometry frame, produced at the end of the complete SE(3) chain
//
//   T_odom_landmark = T_odom_robot(exposure) * T_robot_camera
//                     * T_camera_tagSurface * inverse(T_landmark_tagSurface)
//
// stamped with the exposure time and the target generation it was produced
// under, so a late result from a previous target is discardable.

#pragma once
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/time.h"
#include "math/transforms.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kLandmarkPoseObservationSet =
    "association.landmark_pose_observation_set";
} // namespace payload_names

struct LandmarkPoseObservation {
    WorldObjectId landmark;
    std::string   mount_instance;   // the physical mount that won
    Pose2D        T_odom_landmark;
    MonotonicTime exposureAt;   // host clock
    uint64_t      target_generation = 0;
    SensorId      camera;
    double        confidence = 0.0;
};

struct LandmarkPoseObservationSet {
    std::vector<LandmarkPoseObservation> entries;
};

} // namespace navigatr
