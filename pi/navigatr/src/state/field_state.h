// field_state.h
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

struct FieldObjectState {
    // Estimates are expressed in the named canonical field frame but are
    // unbounded: shared global frame error cancels during relative
    // targeting, so a pose outside the nominal field boundary is never
    // clamped or rejected for that reason alone.
    FramedPose2D pose;   // frame is field
    double         confidence = 0.0;
    bool           valid      = false;
    bool           observed   = false;   // updated by evidence this cycle
    MonotonicTime  lastObservedAt;       // host clock of the newest folded evidence
    EstimateSource source     = EstimateSource::kNone;
};

struct FieldState {
    std::unordered_map<FieldObjectId, FieldObjectState, FieldObjectId::Hash> objects;
};

} // namespace navigatr
