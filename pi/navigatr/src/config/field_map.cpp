// field_map.cpp

#include "config/field_map.h"

#include "config/pose3_config.h"
#include "math/angles.h"

namespace navigatr
{

namespace
{

bool parseColor(const ConfigNode& node, std::string& out, std::string& err) {
    if (!node.hasAttr("color")) {
        return true;
    }
    out = node.attr("color");
    if (out.size() != 7 || out[0] != '#') {
        err = node.path() + ": color must be #rrggbb";
        return false;
    }
    for (std::size_t i = 1; i < out.size(); ++i) {
        const char c = out[i];
        const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        if (!hex) {
            err = node.path() + ": color must be #rrggbb";
            return false;
        }
    }
    return true;
}

bool parseFieldDimensions(const ConfigNode& node, FieldDimensions& out, std::string& err) {
    if (!node.valid()) {
        return true;
    }
    if (!node.requireDouble("inside_x_m", out.inside_x_m, err) ||
        !node.requireDouble("inside_y_m", out.inside_y_m, err) ||
        !node.requireDouble("wall_height_m", out.wall_height_m, err) ||
        !node.requireDouble("wall_thickness_m", out.wall_thickness_m, err) ||
        !node.getDouble("tile_m", 0.0, out.tile_m, err)) {
        return false;
    }
    if (out.inside_x_m <= 0.0 || out.inside_y_m <= 0.0 || out.wall_height_m <= 0.0 ||
        out.wall_thickness_m <= 0.0 || out.tile_m < 0.0) {
        err = node.path() + ": dimensions must be positive";
        return false;
    }
    if (!node.requireAttr("source", out.source, err) ||
        !node.requireAttr("revision", out.revision, err)) {
        return false;
    }
    out.units_note = node.attr("units_note");
    out.declared   = true;
    return true;
}

bool parseFieldFeature(const ConfigNode& node, FieldFeatureDecl& out, std::string& err) {
    double yaw_deg = 0.0;
    if (!node.requireAttr("id", out.id, err) || !node.requireAttr("kind", out.kind, err) ||
        !node.requireDouble("x_m", out.x_m, err) || !node.requireDouble("y_m", out.y_m, err) ||
        !node.requireDouble("z_m", out.z_m, err) ||
        !node.requireDouble("size_x_m", out.size_x_m, err) ||
        !node.requireDouble("size_y_m", out.size_y_m, err) ||
        !node.requireDouble("size_z_m", out.size_z_m, err) ||
        !node.getDouble("yaw_deg", 0.0, yaw_deg, err) || !parseColor(node, out.color, err)) {
        return false;
    }
    if (out.kind != "box" && out.kind != "tape") {
        err = node.path() + ": kind must be box or tape";
        return false;
    }
    if (out.size_x_m <= 0.0 || out.size_y_m <= 0.0 || out.size_z_m < 0.0) {
        err = node.path() + ": feature sizes must be positive";
        return false;
    }
    out.yaw_rad = degToRad(yaw_deg);
    out.note    = node.attr("note");
    return true;
}

bool parseLandmarkVisual(const ConfigNode& node, LandmarkVisualDecl& out, std::string& err) {
    if (!node.valid()) {
        return true;
    }
    if (!node.requireAttr("shape", out.shape, err)) {
        return false;
    }
    if (out.shape == "octagonal_prism") {
        if (!node.requireDouble("height_m", out.height_m, err) ||
            !node.requireDouble("base_across_flats_m", out.base_across_flats_m, err) ||
            !node.requireDouble("top_across_flats_m", out.top_across_flats_m, err)) {
            return false;
        }
        if (out.height_m <= 0.0 || out.base_across_flats_m <= 0.0 ||
            out.top_across_flats_m <= 0.0) {
            err = node.path() + ": prism dimensions must be positive";
            return false;
        }
    } else if (out.shape == "box") {
        if (!node.requireDouble("size_x_m", out.size_x_m, err) ||
            !node.requireDouble("size_y_m", out.size_y_m, err) ||
            !node.requireDouble("size_z_m", out.size_z_m, err)) {
            return false;
        }
        if (out.size_x_m <= 0.0 || out.size_y_m <= 0.0 || out.size_z_m <= 0.0) {
            err = node.path() + ": box dimensions must be positive";
            return false;
        }
        out.height_m = out.size_z_m;
    } else {
        err = node.path() + ": shape must be octagonal_prism or box";
        return false;
    }
    if (!node.getDouble("tag_plate_width_m", 0.0, out.tag_plate_width_m, err) ||
        !node.getDouble("tag_plate_height_m", 0.0, out.tag_plate_height_m, err) ||
        !node.getDouble("tag_plate_thickness_m", 0.0, out.tag_plate_thickness_m, err) ||
        !parseColor(node, out.color, err)) {
        return false;
    }
    out.note     = node.attr("note");
    out.declared = true;
    return true;
}

} // namespace

bool parseFieldMap(const ConfigNode& node, FieldMap& out,
                   std::string& err) {
    bool ok = true;
    if (node.hasAttr("name")) {
        out.name = node.attr("name");
    }
    if (!parseFieldDimensions(node.child("Dimensions"), out.dimensions, err)) {
        return false;
    }
    node.forEach("Feature", [&](const ConfigNode& f) {
        if (!ok) {
            return;
        }
        FieldFeatureDecl feature;
        if (!parseFieldFeature(f, feature, err)) {
            ok = false;
            return;
        }
        for (const FieldFeatureDecl& seen : out.features) {
            if (seen.id == feature.id) {
                err = f.path() + ": duplicate Feature id " + feature.id;
                ok  = false;
                return;
            }
        }
        out.features.push_back(std::move(feature));
    });
    if (!ok) {
        return false;
    }
    node.forEach("Landmark", [&](const ConfigNode& lm) {
        if (!ok) {
            return;
        }
        LandmarkDecl decl;
        decl.id = FieldObjectId{lm.attr("id")};
        if (decl.id.empty()) {
            err = lm.path() + ": Landmark needs id";
            ok  = false;
            return;
        }
        if (out.find(decl.id) != nullptr) {
            err = lm.path() + ": duplicate Landmark id " + decl.id.value;
            ok  = false;
            return;
        }

        const ConfigNode nominal = lm.child("NominalPose");
        if (!nominal.valid()) {
            err = lm.path() + ": Landmark needs NominalPose";
            ok  = false;
            return;
        }
        if (!parsePlanarPose(nominal, decl.nominal, err)) {
            ok = false;
            return;
        }

        lm.forEach("ApproachFrame", [&](const ConfigNode& af) {
            if (!ok) {
                return;
            }
            ApproachFrameDecl a;
            std::string       id_raw;
            if (!af.requireAttr("id", id_raw, err)) {
                ok = false;
                return;
            }
            a.id = FrameId{id_raw};
            for (const LandmarkDecl& seen : out.landmarks) {
                if (seen.findApproach(a.id) != nullptr) {
                    err = af.path() + ": duplicate ApproachFrame id " + a.id.value;
                    ok  = false;
                    return;
                }
            }
            if (decl.findApproach(a.id) != nullptr) {
                err = af.path() + ": duplicate ApproachFrame id " + a.id.value;
                ok  = false;
                return;
            }
            const ConfigNode pose = af.child("PoseOfApproachFrameInLandmark");
            if (!pose.valid()) {
                err = af.path() + ": ApproachFrame needs PoseOfApproachFrameInLandmark";
                ok  = false;
                return;
            }
            if (!parseTransform3(pose, a.T_landmark_approach, err)) {
                ok = false;
                return;
            }
            decl.approaches.push_back(std::move(a));
        });
        if (!ok) {
            return;
        }

        lm.forEach("TagMount", [&](const ConfigNode& tag) {
            if (!ok) {
                return;
            }
            TagMountDecl t;
            if (!tag.requireAttr("instance_id", t.instance_id, err)) {
                ok = false;
                return;
            }
            for (const LandmarkDecl& seen : out.landmarks) {
                if (seen.findMount(t.instance_id) != nullptr) {
                    err = tag.path() + ": duplicate TagMount instance_id " + t.instance_id;
                    ok  = false;
                    return;
                }
            }
            if (decl.findMount(t.instance_id) != nullptr) {
                err = tag.path() + ": duplicate TagMount instance_id " + t.instance_id;
                ok  = false;
                return;
            }
            long id = -1;
            if (!tag.requireAttr("family", t.family, err) ||
                !tag.requireInt("observed_id", id, err) ||
                !tag.requireDouble("detection_size_m", t.detection_size_m, err)) {
                ok = false;
                return;
            }
            if (id < 0) {
                err = tag.path() + ": observed_id cannot be negative";
                ok  = false;
                return;
            }
            t.observed_id = static_cast<int>(id);
            if (t.detection_size_m <= 0.0) {
                err = tag.path() + ": detection_size_m must be positive";
                ok  = false;
                return;
            }
            const ConfigNode pose = tag.child("PoseOfTagSurfaceInLandmark");
            if (!pose.valid()) {
                err = tag.path() + ": TagMount needs PoseOfTagSurfaceInLandmark";
                ok  = false;
                return;
            }
            if (!parseTransform3(pose, t.T_landmark_tag_surface, err)) {
                ok = false;
                return;
            }
            decl.mounts.push_back(std::move(t));
        });
        if (!ok) {
            return;
        }

        if (!parseLandmarkVisual(lm.child("Visual"), decl.visual, err)) {
            ok = false;
            return;
        }

        out.landmarks.push_back(std::move(decl));
    });
    return ok;
}

} // namespace navigatr
