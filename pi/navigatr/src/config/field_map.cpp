// field_map.cpp

#include "config/field_map.h"

#include "config/calibration.h"
#include "config/pose3_config.h"

namespace navigatr
{

bool parseFieldMap(const ConfigNode& node, bool allow_provisional, FieldMap& out,
                   std::string& err) {
    bool ok = true;
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
        if (!checkCalibration(nominal, allow_provisional, err) ||
            !parsePlanarPose(nominal, decl.nominal, err)) {
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
            if (!checkCalibration(af, allow_provisional, err)) {
                ok = false;
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
            if (!checkCalibration(tag, allow_provisional, err)) {
                ok = false;
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

        out.landmarks.push_back(std::move(decl));
    });
    return ok;
}

} // namespace navigatr
