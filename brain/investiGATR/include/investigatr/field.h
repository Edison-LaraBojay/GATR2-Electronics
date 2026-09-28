// field.h
// The field as planning sees it: map identity, boundary, and every configured
// object with its collision box and current estimate. A Field is always
// complete, one map and one estimate snapshot from the same generation.

#pragma once
#include <cstdint>
#include <vector>

#include "investigatr/geometry.h"

namespace investigatr
{

using ObjectId = uint16_t; // stable id from the field definition, 0 = none
using MapId    = uint32_t; // crc32 of the Pi field map document, 0 = none

struct MapIdentity {
    MapId    id       = 0;
    uint16_t revision = 0; // declared in the field definition
};

// Region the robot footprint must stay inside, field frame.
struct Bounds {
    Meters min_x = 0;
    Meters min_y = 0;
    Meters max_x = 0;
    Meters max_y = 0;
};

// Rectangle in its owner's frame. length runs along the box's own x after
// rotating by center.heading, width along its y.
struct Box {
    Pose   center;
    Meters length = 0;
    Meters width  = 0;
};

enum class ObjectKind : uint8_t { kLandmark, kFixed };

enum class EstimateSource : uint8_t { kNone, kNominal, kObserved };

struct FieldObject {
    ObjectId   id        = 0;
    ObjectKind kind      = ObjectKind::kFixed;
    bool       obstacle  = false; // box is a planning obstacle
    bool       estimated = false; // world estimation may correct the pose
    bool       reference = false; // usable as a movement reference
    Pose       nominal;           // field frame
    Box        box;               // object frame, meaningful when obstacle

    EstimateSource source = EstimateSource::kNone;
    bool           valid  = false; // pose is usable
    Pose           pose;           // current estimate, field frame
    bool           age_known = false;
    Seconds        age       = 0; // observation age when the snapshot was taken
};

struct Field {
    uint32_t                 generation = 0; // changes with map or estimates, 0 = none
    MapIdentity              map;
    Bounds                   bounds;
    std::vector<FieldObject> objects; // sorted by id
    FrameGeneration          frame       = 0; // robot field frame of the estimates
    Seconds                  received_at = 0; // source clock, when the snapshot completed

    const FieldObject* find(ObjectId id) const;
};

// Box corners in the field frame for an object at pose, counterclockwise.
void boxCorners(const Box& box, const Pose& owner, Point corners[4]);

} // namespace investigatr
