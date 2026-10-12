// reference.h
// What a destination is measured from. Origin is the virtual field origin.
// An object reference names a map object by id and the map it came from, so
// an id from another field definition cannot silently pick the wrong object.
// RobotAtStart is the robot pose when the command begins.

#pragma once
#include <cstdint>

#include "investigatr/field.h"
#include "investigatr/geometry.h"

namespace investigatr
{

struct Reference {
    enum class Kind : uint8_t { kOrigin, kObject, kRobotAtStart };

    Kind     kind      = Kind::kOrigin;
    ObjectId object_id = 0;
    MapId    map       = 0;

    static constexpr Reference origin() { return Reference{}; }
    static constexpr Reference robotAtStart() { return Reference{Kind::kRobotAtStart, 0, 0}; }
    static constexpr Reference object(ObjectId id, MapId map) {
        return Reference{Kind::kObject, id, map};
    }
};

// Base of generated field reference tables, which add one Reference per
// named map object:
//   struct Field : investigatr::FieldReferences { static constexpr ... };
struct FieldReferences {
    static constexpr Reference Origin       = Reference::origin();
    static constexpr Reference RobotAtStart = Reference::robotAtStart();
};

enum class ResolveStatus : uint8_t {
    kOk,
    kNoField,         // object reference without a complete field
    kMapMismatch,     // reference made for another map
    kUnknownObject,   // no such object in the map
    kNotReference,    // object not marked as a movement reference
    kNotObserved,     // only a nominal estimate while observed is required
    kNoEstimate,      // object has no valid estimate
    kStale,           // observed estimate older than the policy allows
    kFrameMismatch,   // field estimates are in another robot frame
};

const char* toString(ResolveStatus status);

struct ReferencePolicy {
    bool    require_observed = true; // false also accepts the nominal pose
    Seconds max_age          = 0;    // observed estimate age limit, 0 = none
};

struct Resolved {
    ResolveStatus  status = ResolveStatus::kNoField;
    Pose           reference;   // reference pose, field frame
    Pose           destination; // compose(reference, relative)
    EstimateSource source = EstimateSource::kNone;
};

// Applies the reference once: destination = reference * relative.
// robot_at_start is used only by kRobotAtStart. field may be null for
// kOrigin and kRobotAtStart. robot_frame 0 skips the frame check. now is the
// source clock, for estimate ages.
Resolved resolve(const Reference& reference, const Pose& relative, const Field* field,
                 const Pose& robot_at_start, FrameGeneration robot_frame,
                 const ReferencePolicy& policy, Seconds now);

} // namespace investigatr
