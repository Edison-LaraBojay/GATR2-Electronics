// field_map.cpp

#include "config/field_map.h"

#include "math/angles.h"

namespace navigatr
{

namespace
{

bool poseFromAttrs(const ConfigNode& n, Pose2D& out, std::string& err) {
    double heading_deg = 0.0;
    if (!n.getDouble("x_m", 0.0, out.x_m, err) || !n.getDouble("y_m", 0.0, out.y_m, err) ||
        !n.getDouble("heading_deg", 0.0, heading_deg, err)) {
        return false;
    }
    out.heading_rad = degToRad(heading_deg);
    return true;
}

} // namespace

bool parseFieldMap(const ConfigNode& node, FieldMap& out, std::string& err) {
    bool ok = true;
    node.forEach("Landmark", [&](const ConfigNode& lm) {
        if (!ok) {
            return;
        }
        LandmarkDecl decl;
        decl.id = WorldObjectId{lm.attr("id")};
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
        if (!poseFromAttrs(nominal, decl.nominal, err)) {
            ok = false;
            return;
        }

        lm.forEach("Tag", [&](const ConfigNode& tag) {
            if (!ok) {
                return;
            }
            TagInstanceDecl t;
            t.instance = tag.attr("instance");
            if (t.instance.empty()) {
                err = tag.path() + ": Tag needs instance";
                ok  = false;
                return;
            }
            t.family = tag.attr("family");
            long id  = -1;
            if (!tag.getInt("observed_id", -1, id, err)) {
                ok = false;
                return;
            }
            t.observed_id = static_cast<int>(id);
            if (!poseFromAttrs(tag, t.mount, err)) {
                ok = false;
                return;
            }
            decl.tags.push_back(std::move(t));
        });

        out.landmarks.push_back(std::move(decl));
    });
    return ok;
}

} // namespace navigatr
