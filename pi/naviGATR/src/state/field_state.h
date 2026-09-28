// field_state.h
// External object estimates keyed by configured object id. Deliberately
// separate from the robot estimate: seeing a displaced goal moves the goal
// estimate, never the robot. Wire protocol ids for these objects belong to
// publisher configuration, never here.
//
// An observed estimate is retained in the odometry frame it was measured
// in, tagged with that odometry epoch; its field pose is derived under the
// current anchor whenever the state is published, so a field re-anchor
// re-expresses every estimate consistently while an odometry epoch change
// invalidates it back to the map nominal. A map-seeded entry is field
// geometry and has no odometry-frame value.

#pragma once
#include <cstdint>
#include <string>
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

inline const char* estimateSourceName(EstimateSource s) {
    switch (s) {
    case EstimateSource::kNone: return "none";
    case EstimateSource::kFieldMap: return "field_map";
    case EstimateSource::kObserved: return "observed";
    }
    return "unknown";
}

struct FieldObjectState {
    // Estimates are expressed in the named canonical field frame but are
    // unbounded: shared global frame error cancels during relative
    // targeting, so a pose outside the nominal field boundary is never
    // clamped or rejected for that reason alone.
    FramedPose2D pose;   // frame is field, derived under anchor_revision
    double         confidence = 0.0;
    bool           valid      = false;
    bool           observed   = false;   // updated by evidence this invocation
    MonotonicTime  lastObservedAt;       // host clock of the newest folded evidence
    EstimateSource source     = EstimateSource::kNone;

    // Observed entries only: the measured pose in the odometry frame.
    Pose2D   T_odom_object;
    uint64_t odometry_epoch  = 0;
    uint64_t anchor_revision = 0;   // anchor the field pose was derived under

    std::string last_source;           // producing sensor of the newest evidence
    std::string last_feature;          // configured feature that won
    uint32_t    last_source_sequence = 0;
};

struct FieldState {
    std::unordered_map<FieldObjectId, FieldObjectState, FieldObjectId::Hash> objects;
};

} // namespace navigatr
