// world_state.h
// External object estimates, field frame, keyed by configured object id.
// Deliberately separate from the robot estimate: seeing a displaced goal
// moves the goal estimate, never the robot. Wire protocol ids for these
// objects belong to publisher configuration, never here.

#pragma once
#include <cstdint>
#include <unordered_map>

#include "core/ids.h"
#include "math/transforms.h"

namespace navigatr
{

enum class EstimateSource : uint8_t {
    kNone = 0,
    kFieldMap,   // where the map says it should be
    kObserved,   // updated from an associated observation
};

struct WorldObject {
    FramedPose2D pose;   // frame is field
    double         confidence = 0.0;
    bool           valid      = false;
    bool           observed   = false;   // seen this cycle
    EstimateSource source     = EstimateSource::kNone;
};

struct WorldState {
    std::unordered_map<WorldObjectId, WorldObject, WorldObjectId::Hash> objects;
};

} // namespace navigatr
