// field_map.h
// Field landmark data: nominal field poses and the physical tag instances
// mounted on them, meters and radians. This is data, not behavior, and the
// core runtime never reads it; it is loaded as a typed resource and consumed
// only by the implementations that reference it. Brain wire ids are not
// field facts and do not appear here.
//
// A printed tag id is not a unique identity: several physical instances may
// share one observed_id, and deciding which instance produced an observation
// is an association implementation's job.

#pragma once
#include <string>
#include <vector>

#include "config/config_node.h"
#include "core/ids.h"
#include "math/transforms.h"

namespace navigatr
{

struct TagInstanceDecl {
    std::string instance;      // unique physical instance name
    std::string family;        // e.g. tag36h11
    int         observed_id = -1;   // what the camera reports
    Pose2D      mount;         // T_landmark_tag
};

struct LandmarkDecl {
    WorldObjectId id;
    Pose2D        nominal;   // T_field_landmark from the map
    std::vector<TagInstanceDecl> tags;
};

struct FieldMap {
    std::vector<LandmarkDecl> landmarks;

    const LandmarkDecl* find(const WorldObjectId& id) const {
        for (const LandmarkDecl& l : landmarks) {
            if (l.id == id) {
                return &l;
            }
        }
        return nullptr;
    }
};

// Parses the Landmark children of a field map resource node. False and err
// on bad content.
bool parseFieldMap(const ConfigNode& node, FieldMap& out, std::string& err);

} // namespace navigatr
