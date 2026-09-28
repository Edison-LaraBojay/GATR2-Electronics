// robot_frames.h
// Named robot interaction frames, resolved to the robot body frame. The
// robot body frame is the exact point RobotState tracks: +x forward, +y
// left, +z up. Frames represent measured rigid geometry (contact points,
// camera optical centers); a mechanism whose position changes during
// operation is not a fixed frame and must not be configured as one.
//
// Immutable shared data after construction, like FieldMap.

#pragma once
#include <unordered_map>

#include "core/ids.h"
#include "math/se3.h"

namespace navigatr
{

struct RobotFrameMap {
    // T_robot_body_frame for every configured frame, plus robot_body itself
    // as identity.
    std::unordered_map<FrameId, Transform3, FrameId::Hash> frames;

    const Transform3* find(const FrameId& id) const {
        const auto it = frames.find(id);
        return it == frames.end() ? nullptr : &it->second;
    }
};

// Root frame id every chain resolves to.
inline FrameId robotBodyFrameId() { return FrameId{"robot_body"}; }

} // namespace navigatr
