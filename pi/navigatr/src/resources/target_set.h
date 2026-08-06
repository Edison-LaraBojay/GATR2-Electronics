// target_set.h
// Configured navigation targets, immutable shared data. A target names what
// the brain can select by wire id and how the desired robot body pose is
// derived; the runtime latching lives in TargetState, never here.
//
// Landmark-relative targets chain, SE(3) end to end with a planar
// projection only at the end:
//
//   T_X_robot_target = T_X_landmark * T_landmark_approach
//                      * T_approach_controlled_desired
//                      * inverse(T_robot_controlled)
//
// Robot-relative targets snapshot the activation pose:
//
//   T_odom_robot_target = T_odom_robot(activation) * delta
//
// Vision policies: none uses the nominal or previously held landmark,
// acquire_once processes evidence only until the selected target latches,
// continuous is reserved for a future implementation.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "core/ids.h"
#include "math/se3.h"
#include "math/transforms.h"

namespace navigatr
{

enum class TargetKind : uint8_t {
    kLandmarkRelative,
    kRobotRelative,
};

enum class VisionPolicy : uint8_t {
    kNone,
    kAcquireOnce,
};

enum class AcquisitionFallback : uint8_t {
    kUseNominalTarget,   // zero visual correction, not a zero pose
    kCancel,
};

struct VisionCorrectionDecl {
    VisionPolicy        policy     = VisionPolicy::kNone;
    AcquisitionFallback on_timeout = AcquisitionFallback::kUseNominalTarget;

    long   minimum_consistent_observations = 0;
    long   maximum_observation_age_ms      = 0;
    double maximum_robot_angular_speed_rad_s = 0.0;
    long   acquisition_timeout_ms          = 0;
    double consistency_translation_m       = 0.0;
    double consistency_heading_rad         = 0.0;

    SensorId                 preferred_camera;   // may be empty
    std::vector<std::string> allowed_mounts;     // empty = every mount
};

struct TargetDecl {
    std::string id;
    uint8_t     wire_id = 0;
    TargetKind  kind    = TargetKind::kRobotRelative;

    // landmark relative
    WorldObjectId        landmark;
    FrameId              approach_frame;
    FrameId              controlled_frame;
    Pose2D               desired_controlled_in_approach;
    Transform3           T_landmark_approach;   // resolved at build
    Transform3           T_robot_controlled;    // resolved at build
    VisionCorrectionDecl vision;

    // robot relative
    Pose2D delta;

    // Desired robot body pose given the landmark pose in any frame X.
    Pose2D resolveFromLandmark(const Pose2D& T_x_landmark) const {
        const Transform3 chain = compose(
            compose(compose(transform3FromPlanar(T_x_landmark), T_landmark_approach),
                    transform3FromPlanar(desired_controlled_in_approach)),
            inverse(T_robot_controlled));
        return planarFromTransform3(chain);
    }
};

struct TargetSet {
    std::vector<TargetDecl> targets;

    const TargetDecl* findByWireId(uint8_t wire_id) const {
        for (const TargetDecl& t : targets) {
            if (t.wire_id == wire_id) {
                return &t;
            }
        }
        return nullptr;
    }

    const TargetDecl* findById(const std::string& id) const {
        for (const TargetDecl& t : targets) {
            if (t.id == id) {
                return &t;
            }
        }
        return nullptr;
    }
};

} // namespace navigatr
