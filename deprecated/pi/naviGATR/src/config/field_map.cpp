// field_map.cpp

#include "config/field_map.h"

#include <utility>

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

// 1..65535; absent is 0 unless required.
bool parseWireId(const ConfigNode& node, bool required, uint16_t& out, std::string& err) {
    long id = 0;
    if (required ? !node.requireInt("wire_id", id, err) : !node.getInt("wire_id", 0, id, err)) {
        return false;
    }
    if (!node.hasAttr("wire_id")) {
        out = 0;
        return true;
    }
    if (id < 1 || id > 65535) {
        err = node.path() + ": wire_id must be 1..65535";
        return false;
    }
    out = static_cast<uint16_t>(id);
    return true;
}

bool parseCollisionBox(const ConfigNode& owner, CollisionBoxDecl& out, std::string& err) {
    if (!owner.atMostOne("CollisionBox", err)) {
        return false;
    }
    const ConfigNode node = owner.child("CollisionBox");
    if (!node.valid()) {
        return true;
    }
    double yaw_deg = 0.0;
    if (!node.onlyAttributes(
            {"x_m", "y_m", "size_x_m", "size_y_m", "yaw_deg", "note", "calibration_status"}, err) ||
        !node.onlyChildren({}, err) || !node.requireDouble("x_m", out.center.x_m, err) ||
        !node.requireDouble("y_m", out.center.y_m, err) ||
        !node.requireDouble("size_x_m", out.size_x_m, err) ||
        !node.requireDouble("size_y_m", out.size_y_m, err) ||
        !node.getDouble("yaw_deg", 0.0, yaw_deg, err)) {
        return false;
    }
    if (out.size_x_m <= 0.0 || out.size_y_m <= 0.0) {
        err = node.path() + ": CollisionBox sizes must be positive";
        return false;
    }
    out.center.heading_rad = degToRad(yaw_deg);
    out.note               = node.attr("note");
    out.declared           = true;
    return true;
}

bool parseBoundary(const ConfigNode& root, FieldBoundaryDecl& out, std::string& err) {
    if (!root.atMostOne("Boundary", err)) {
        return false;
    }
    const ConfigNode node = root.child("Boundary");
    if (!node.valid()) {
        return true;
    }
    if (!node.onlyAttributes(
            {"min_x_m", "min_y_m", "max_x_m", "max_y_m", "note", "calibration_status"}, err) ||
        !node.onlyChildren({}, err) || !node.requireDouble("min_x_m", out.min_x_m, err) ||
        !node.requireDouble("min_y_m", out.min_y_m, err) ||
        !node.requireDouble("max_x_m", out.max_x_m, err) ||
        !node.requireDouble("max_y_m", out.max_y_m, err)) {
        return false;
    }
    if (!(out.min_x_m < out.max_x_m) || !(out.min_y_m < out.max_y_m)) {
        err = node.path() + ": Boundary min must be below max on both axes";
        return false;
    }
    out.note     = node.attr("note");
    out.declared = true;
    return true;
}

bool parseObstacle(const ConfigNode& node, ObstacleDecl& out, std::string& err) {
    double heading_deg = 0.0;
    if (!node.onlyAttributes(
            {"id", "wire_id", "x_m", "y_m", "heading_deg", "note", "calibration_status"}, err) ||
        !node.onlyChildren({"CollisionBox"}, err) || !node.requireAttr("id", out.id, err) ||
        !parseWireId(node, true, out.wire_id, err) ||
        !node.requireDouble("x_m", out.pose.x_m, err) ||
        !node.requireDouble("y_m", out.pose.y_m, err) ||
        !node.requireDouble("heading_deg", heading_deg, err) ||
        !parseCollisionBox(node, out.box, err)) {
        return false;
    }
    out.pose.heading_rad = degToRad(heading_deg);
    out.note             = node.attr("note");
    return true;
}

// wire ids name one object across landmarks and obstacles
bool uniqueWireIds(const ConfigNode& node, const FieldMap& map, std::string& err) {
    std::vector<std::pair<uint16_t, std::string>> seen;
    const auto add = [&](uint16_t id, const std::string& name) {
        if (id == 0) {
            return true;
        }
        for (const auto& s : seen) {
            if (s.first == id) {
                err = node.path() + ": wire_id " + std::to_string(id) + " used by both " +
                      s.second + " and " + name;
                return false;
            }
        }
        seen.emplace_back(id, name);
        return true;
    };
    for (const ObstacleDecl& o : map.obstacles) {
        if (!add(o.wire_id, o.id)) {
            return false;
        }
    }
    for (const LandmarkDecl& l : map.landmarks) {
        if (!add(l.wire_id, l.id.value)) {
            return false;
        }
    }
    return true;
}

} // namespace

bool parseFieldMap(const ConfigNode& node, FieldMap& out,
                   std::string& err) {
    bool ok = true;
    if (!node.onlyChildren({"Dimensions", "Boundary", "Feature", "Obstacle", "Landmark"}, err) ||
        !node.onlyAttributes({"id", "type", "name", "revision", "calibration_status"}, err)) {
        return false;
    }
    if (node.hasAttr("name")) {
        out.name = node.attr("name");
    }
    long revision = 0;
    if (!node.getInt("revision", 0, revision, err)) {
        return false;
    }
    if (node.hasAttr("revision") && (revision < 1 || revision > 65535)) {
        err = node.path() + ": revision must be 1..65535";
        return false;
    }
    out.revision = static_cast<uint16_t>(revision);
    if (!parseFieldDimensions(node.child("Dimensions"), out.dimensions, err) ||
        !parseBoundary(node, out.boundary, err)) {
        return false;
    }
    node.forEach("Obstacle", [&](const ConfigNode& o) {
        if (!ok) {
            return;
        }
        ObstacleDecl obstacle;
        if (!parseObstacle(o, obstacle, err)) {
            ok = false;
            return;
        }
        if (out.findObstacle(obstacle.id) != nullptr) {
            err = o.path() + ": duplicate Obstacle id " + obstacle.id;
            ok  = false;
            return;
        }
        out.obstacles.push_back(std::move(obstacle));
    });
    if (!ok) {
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
        if (out.findObstacle(decl.id.value) != nullptr) {
            err = lm.path() + ": Landmark id " + decl.id.value + " is also an Obstacle id";
            ok  = false;
            return;
        }
        if (!lm.onlyAttributes({"id", "wire_id", "calibration_status"}, err) ||
            !lm.onlyChildren(
                {"NominalPose", "ApproachFrame", "TagMount", "Visual", "CollisionBox"}, err) ||
            !parseWireId(lm, false, decl.wire_id, err) ||
            !parseCollisionBox(lm, decl.box, err)) {
            ok = false;
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
    return ok && uniqueWireIds(node, out, err);
}

} // namespace navigatr
