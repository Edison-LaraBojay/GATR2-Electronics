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
// Optional display data, never read by estimation: a Dimensions element
// (perimeter box plus the provenance of the numbers), Feature elements
// (static unobserved geometry drawn as labeled boxes or tape), and one
// Visual per Landmark (prism or box plus the tag carrier plate). See
// docs/field_assets.md.
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

// Display geometry for one landmark. Never read by estimation: the viewer
// draws it, and a wrong value moves a picture, not an estimate.
struct LandmarkVisualDecl {
    bool        declared = false;
    std::string shape;   // octagonal_prism | box
    double      height_m             = 0.0;
    double      base_across_flats_m  = 0.0;   // octagonal_prism
    double      top_across_flats_m   = 0.0;
    double      size_x_m             = 0.0;   // box
    double      size_y_m             = 0.0;
    double      size_z_m             = 0.0;
    double      tag_plate_width_m    = 0.0;   // carrier plate behind every mount, 0 = none
    double      tag_plate_height_m   = 0.0;
    double      tag_plate_thickness_m = 0.0;
    std::string color;   // #rrggbb
    std::string note;
};

struct LandmarkDecl {
    FieldObjectId id;
    Pose2D        nominal;   // T_field_landmark from the map
    std::vector<ApproachFrameDecl> approaches;
    std::vector<TagMountDecl>      mounts;
    LandmarkVisualDecl             visual;

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

// Whole-field display data: the perimeter box, provenance of the numbers,
// and static unobserved features (loaders, toggles, tape) drawn as labeled
// simple geometry. Display only, like LandmarkVisualDecl.
struct FieldDimensions {
    bool        declared = false;
    double      inside_x_m       = 0.0;
    double      inside_y_m       = 0.0;
    double      wall_height_m    = 0.0;
    double      wall_thickness_m = 0.0;
    double      tile_m           = 0.0;   // tile pitch, 0 = undeclared
    std::string source;     // where the numbers came from
    std::string revision;   // document or CAD revision
    std::string units_note;
};

struct FieldFeatureDecl {
    std::string id;
    std::string kind;   // box | tape
    double      x_m = 0.0, y_m = 0.0, z_m = 0.0;   // center, field frame
    double      size_x_m = 0.0, size_y_m = 0.0, size_z_m = 0.0;
    double      yaw_rad = 0.0;
    std::string color;
    std::string note;
};

struct FieldMap {
    std::string                   name;   // display name, may be empty
    FieldDimensions               dimensions;
    std::vector<FieldFeatureDecl> features;
    std::vector<LandmarkDecl>     landmarks;

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
bool parseFieldMap(const ConfigNode& node, FieldMap& out,
                   std::string& err);

} // namespace navigatr
