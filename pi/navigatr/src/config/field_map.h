// field_map.h
// Field landmark data, meters and radians. This is data, not behavior, and
// the core runtime never reads it; it is loaded as a typed resource and
// consumed only by the implementations that reference it. Brain wire ids
// are not field facts and do not appear here.
//
// A landmark has one semantic origin, one nominal field pose, any number of
// physical tag mounts, and any number of named approach frames. A printed
// tag id is not a unique identity: several physical mounts may share one
// observed_id, and deciding which mount produced an observation is an
// association implementation's job, decided by full pose, never forced.
//
// Frames:
//   landmark frame: planar at the landmark origin, yaw = nominal heading
//   tag surface frame S: origin at the center of the detector's four
//     pose-estimation corners, +x outward normal toward a viewer, +z the
//     decoded printed top, +y right-handed completion
//   approach frame: +x outward from the landmark into the approach space,
//     +y left when looking outward, +z up

#pragma once
#include <string>
#include <vector>

#include "config/config_node.h"
#include "core/ids.h"
#include "math/se3.h"
#include "math/transforms.h"

namespace navigatr
{

struct TagMountDecl {
    std::string instance_id;   // globally unique physical mount name
    std::string family;        // e.g. tag36h11
    int         observed_id = -1;   // what the detector reports
    double      detection_size_m = 0.0;   // detector corner edge, not the sticker
    Transform3  T_landmark_tag_surface;
};

struct ApproachFrameDecl {
    FrameId    id;   // globally unique approach frame name
    Transform3 T_landmark_approach;
};

struct LandmarkDecl {
    FieldObjectId id;
    Pose2D        nominal;   // T_field_landmark from the map
    std::vector<ApproachFrameDecl> approaches;
    std::vector<TagMountDecl>      mounts;

    const ApproachFrameDecl* findApproach(const FrameId& frame) const {
        for (const ApproachFrameDecl& a : approaches) {
            if (a.id == frame) {
                return &a;
            }
        }
        return nullptr;
    }

    const TagMountDecl* findMount(const std::string& instance_id) const {
        for (const TagMountDecl& m : mounts) {
            if (m.instance_id == instance_id) {
                return &m;
            }
        }
        return nullptr;
    }
};

struct FieldMap {
    std::vector<LandmarkDecl> landmarks;

    const LandmarkDecl* find(const FieldObjectId& id) const {
        for (const LandmarkDecl& l : landmarks) {
            if (l.id == id) {
                return &l;
            }
        }
        return nullptr;
    }

    // The landmark owning a mount instance, or nullptr.
    const LandmarkDecl* findMountOwner(const std::string& instance_id) const {
        for (const LandmarkDecl& l : landmarks) {
            if (l.findMount(instance_id) != nullptr) {
                return &l;
            }
        }
        return nullptr;
    }
};

// Parses the Landmark children of a field map resource node. Every
// calibration-critical attribute is required and calibration gated; false
// and err on bad content.
bool parseFieldMap(const ConfigNode& node, bool allow_provisional, FieldMap& out,
                   std::string& err);

} // namespace navigatr
