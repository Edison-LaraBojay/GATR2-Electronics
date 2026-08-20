// target_state.h
// The active navigation target, latched in the odometry frame. A target is
// selected by the brain, resolved by the configured target implementation,
// and consumed by publishing; the framework only carries it between cycles.
//
// generation increments on every activation, so evidence stamped with an
// older generation is discarded after a target switch. odometry_epoch
// records the epoch the pose was latched in; when the robot's epoch moves
// on, the latched pose is meaningless and the target is cancelled.

#pragma once
#include <cstdint>
#include <string>

#include "core/ids.h"
#include "core/time.h"
#include "math/transforms.h"

namespace navigatr
{

enum class TargetStatus : uint8_t {
    kNone = 0,             // no active target
    kPendingAcquisition,   // waiting for consistent vision evidence
    kLockedVision,         // latched from vision; gate closed
    kLockedNominal,        // latched from the map (policy none or fallback)
    kLockedRobotRelative,  // latched from the activation snapshot
    kCancelled,            // cancelled (epoch reset or explicit policy)
};

inline const char* targetStatusName(TargetStatus s) {
    switch (s) {
    case TargetStatus::kNone: return "none";
    case TargetStatus::kPendingAcquisition: return "pending_acquisition";
    case TargetStatus::kLockedVision: return "locked_vision";
    case TargetStatus::kLockedNominal: return "locked_nominal";
    case TargetStatus::kLockedRobotRelative: return "locked_robot_relative";
    case TargetStatus::kCancelled: return "cancelled";
    }
    return "unknown";
}

struct TargetState {
    bool         active = false;
    std::string  target_id;   // configured target declaration id
    uint8_t      wire_id = 0;
    uint64_t     generation = 0;
    TargetStatus status     = TargetStatus::kNone;

    // Desired robot body pose, odometry frame. Valid only when latched.
    Pose2D   T_odom_robot_target;
    bool     latched        = false;
    uint64_t odometry_epoch = 0;   // epoch the latch belongs to

    // Vision locks only: the consistent window's mean landmark pose that
    // produced the latch. This is the single piece of camera evidence world
    // estimation may fold (per its commit policy); rejected or unconfirmed
    // evidence never appears here, so it can never reach WorldState.
    bool          has_locked_landmark = false;
    WorldObjectId locked_landmark;
    Pose2D        T_odom_landmark_locked;
    double        locked_confidence = 0.0;

    MonotonicTime activatedAt;   // host clock
};

} // namespace navigatr
