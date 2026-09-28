// target_set_resource.cpp

#include "impl/resources/target_set_resource.h"

#include <memory>

#include "config/field_map.h"
#include "config/pose3_config.h"
#include "math/angles.h"
#include "resources/robot_frames.h"
#include "resources/target_set.h"

namespace navigatr
{

namespace
{

bool parseVisionCorrection(const ConfigNode& node, const FieldMap* field,
                           const TargetDecl& target, bool have_field,
                           VisionCorrectionDecl& out, std::string& err) {
    std::string type;
    if (!node.requireAttr("type", type, err)) {
        return false;
    }
    if (type == "none") {
        out.policy = VisionPolicy::kNone;
        return true;
    }
    if (type == "continuous") {
        err = node.path() + ": VisionCorrection type continuous is reserved for a "
              "future implementation; use none or acquire_once";
        return false;
    }
    if (type != "acquire_once") {
        err = node.path() + ": VisionCorrection type must be none, acquire_once, or "
              "continuous, not \"" + type + "\"";
        return false;
    }
    out.policy = VisionPolicy::kAcquireOnce;

    std::string fallback;
    if (!node.requireAttr("on_acquisition_timeout", fallback, err)) {
        return false;
    }
    if (fallback == "use_nominal_target") {
        out.on_timeout = AcquisitionFallback::kUseNominalTarget;
    } else if (fallback == "cancel") {
        out.on_timeout = AcquisitionFallback::kCancel;
    } else {
        err = node.path() + ": on_acquisition_timeout must be use_nominal_target or "
              "cancel, not \"" + fallback + "\"";
        return false;
    }

    double max_speed_deg_s = 0.0, consistency_heading_deg = 0.0;
    if (!node.requireInt("minimum_consistent_observations",
                         out.minimum_consistent_observations, err) ||
        !node.requireInt("maximum_observation_age_ms", out.maximum_observation_age_ms,
                         err) ||
        !node.requireDouble("maximum_robot_angular_speed_deg_s", max_speed_deg_s, err) ||
        !node.requireInt("acquisition_timeout_ms", out.acquisition_timeout_ms, err) ||
        !node.requireDouble("consistency_translation_m", out.consistency_translation_m,
                            err) ||
        !node.requireDouble("consistency_heading_deg", consistency_heading_deg, err)) {
        return false;
    }
    if (out.minimum_consistent_observations < 1 || out.maximum_observation_age_ms <= 0 ||
        max_speed_deg_s <= 0.0 || out.acquisition_timeout_ms <= 0 ||
        out.consistency_translation_m <= 0.0 || consistency_heading_deg <= 0.0) {
        err = node.path() + ": acquisition attributes must be positive";
        return false;
    }
    out.maximum_robot_angular_speed_rad_s = degToRad(max_speed_deg_s);
    out.consistency_heading_rad           = degToRad(consistency_heading_deg);

    const ConfigNode preferred = node.child("PreferredCamera");
    if (preferred.valid()) {
        std::string sensor_raw;
        if (!preferred.requireAttr("sensor_id", sensor_raw, err)) {
            return false;
        }
        out.preferred_camera = SensorId{sensor_raw};
    }

    bool ok = true;
    node.forEach("AllowedTagMount", [&](const ConfigNode& mount) {
        if (!ok) {
            return;
        }
        std::string instance;
        if (!mount.requireAttr("instance_id", instance, err)) {
            ok = false;
            return;
        }
        if (have_field) {
            const LandmarkDecl* owner = field->findMountOwner(instance);
            if (owner == nullptr) {
                err = mount.path() + ": AllowedTagMount " + instance +
                      " does not exist in the field map";
                ok = false;
                return;
            }
            if (owner->id != target.landmark) {
                err = mount.path() + ": AllowedTagMount " + instance +
                      " belongs to landmark " + owner->id.value + ", not " +
                      target.landmark.value;
                ok = false;
                return;
            }
        }
        out.allowed_mounts.push_back(std::move(instance));
    });
    return ok;
}

} // namespace

ResourceInstance make_target_set(const ConfigNode&              node,
                                 ResourceInitializationContext& context,
                                 std::string&                   err) {
    std::shared_ptr<const FieldMap>      field;
    std::shared_ptr<const RobotFrameMap> frames;

    const ConfigNode field_ref = node.child("FieldMap");
    if (field_ref.valid()) {
        const ResourceId id{field_ref.attr("resource_id")};
        if (id.empty()) {
            err = field_ref.path() + ": FieldMap needs resource_id";
            return ResourceInstance{};
        }
        field = context.require<const FieldMap>(id, err);
        if (field == nullptr) {
            err = field_ref.path() + ": " + err;
            return ResourceInstance{};
        }
    }
    const ConfigNode frames_ref = node.child("RobotFrames");
    if (frames_ref.valid()) {
        const ResourceId id{frames_ref.attr("resource_id")};
        if (id.empty()) {
            err = frames_ref.path() + ": RobotFrames needs resource_id";
            return ResourceInstance{};
        }
        frames = context.require<const RobotFrameMap>(id, err);
        if (frames == nullptr) {
            err = frames_ref.path() + ": " + err;
            return ResourceInstance{};
        }
    }

    auto set = std::make_shared<TargetSet>();
    bool ok  = true;
    node.forEach("Target", [&](const ConfigNode& t) {
        if (!ok) {
            return;
        }
        TargetDecl decl;
        std::string type;
        long        wire = -1;
        if (!t.requireAttr("id", decl.id, err) || !t.requireAttr("type", type, err) ||
            !t.requireInt("wire_id", wire, err)) {
            ok = false;
            return;
        }
        if (wire < 0 || wire > 255) {
            err = t.path() + ": wire_id must be 0..255";
            ok  = false;
            return;
        }
        decl.wire_id = static_cast<uint8_t>(wire);
        if (set->findById(decl.id) != nullptr) {
            err = t.path() + ": duplicate Target id " + decl.id;
            ok  = false;
            return;
        }
        if (set->findByWireId(decl.wire_id) != nullptr) {
            err = t.path() + ": duplicate Target wire_id " + std::to_string(wire);
            ok  = false;
            return;
        }

        if (type == "landmark_relative") {
            decl.kind = TargetKind::kLandmarkRelative;
            std::string landmark_raw, approach_raw, controlled_raw;
            if (!t.requireAttr("landmark_id", landmark_raw, err) ||
                !t.requireAttr("approach_frame_id", approach_raw, err) ||
                !t.requireAttr("controlled_frame_id", controlled_raw, err)) {
                ok = false;
                return;
            }
            decl.landmark         = FieldObjectId{landmark_raw};
            decl.approach_frame   = FrameId{approach_raw};
            decl.controlled_frame = FrameId{controlled_raw};

            if (field == nullptr) {
                err = t.path() + ": landmark_relative targets need a "
                      "<FieldMap resource_id=.../> reference in the target set";
                ok = false;
                return;
            }
            if (frames == nullptr) {
                err = t.path() + ": landmark_relative targets need a "
                      "<RobotFrames resource_id=.../> reference in the target set";
                ok = false;
                return;
            }
            const LandmarkDecl* lm = field->find(decl.landmark);
            if (lm == nullptr) {
                err = t.path() + ": landmark_id " + landmark_raw +
                      " does not exist in the field map";
                ok = false;
                return;
            }
            const ApproachFrameDecl* approach = lm->findApproach(decl.approach_frame);
            if (approach == nullptr) {
                err = t.path() + ": approach_frame_id " + approach_raw +
                      " does not exist on landmark " + landmark_raw;
                ok = false;
                return;
            }
            decl.T_landmark_approach = approach->T_landmark_approach;
            const Transform3* controlled = frames->find(decl.controlled_frame);
            if (controlled == nullptr) {
                err = t.path() + ": controlled_frame_id " + controlled_raw +
                      " does not exist in the robot frame map";
                ok = false;
                return;
            }
            decl.T_robot_controlled = *controlled;

            const ConfigNode desired = t.child("DesiredControlledFramePose");
            if (!desired.valid()) {
                err = t.path() + ": needs DesiredControlledFramePose";
                ok  = false;
                return;
            }
            if (!parsePlanarPose(desired, decl.desired_controlled_in_approach, err)) {
                ok = false;
                return;
            }
        } else if (type == "robot_relative") {
            decl.kind = TargetKind::kRobotRelative;
            std::string controlled_raw;
            if (!t.requireAttr("controlled_frame_id", controlled_raw, err)) {
                ok = false;
                return;
            }
            if (FrameId{controlled_raw} != robotBodyFrameId()) {
                err = t.path() + ": robot_relative targets control robot_body; other "
                      "controlled frames are not supported yet";
                ok = false;
                return;
            }
            const ConfigNode snapshot = t.child("Snapshot");
            if (!snapshot.valid()) {
                err = t.path() + ": needs <Snapshot frame_id=... delta_axes=.../>";
                ok  = false;
                return;
            }
            std::string snap_frame, delta_axes;
            if (!snapshot.requireAttr("frame_id", snap_frame, err) ||
                !snapshot.requireAttr("delta_axes", delta_axes, err)) {
                ok = false;
                return;
            }
            if (FrameId{snap_frame} != robotBodyFrameId() ||
                delta_axes != "robot_at_activation") {
                err = snapshot.path() + ": only frame_id=\"robot_body\" with "
                      "delta_axes=\"robot_at_activation\" is supported";
                ok = false;
                return;
            }
            const ConfigNode delta = t.child("Delta");
            if (!delta.valid()) {
                err = t.path() + ": needs <Delta x_m=... y_m=... heading_deg=.../>";
                ok  = false;
                return;
            }
            if (!parsePlanarPose(delta, decl.delta, err)) {
                ok = false;
                return;
            }
        } else {
            err = t.path() + ": Target type must be landmark_relative or "
                  "robot_relative, not \"" + type + "\"";
            ok = false;
            return;
        }

        const ConfigNode vision = t.child("VisionCorrection");
        if (!vision.valid()) {
            err = t.path() + ": needs an explicit <VisionCorrection type=.../>, "
                  "including type=\"none\"";
            ok = false;
            return;
        }
        if (!parseVisionCorrection(vision, field.get(), decl, field != nullptr,
                                   decl.vision, err)) {
            ok = false;
            return;
        }
        if (decl.kind == TargetKind::kRobotRelative &&
            decl.vision.policy != VisionPolicy::kNone) {
            err = vision.path() + ": robot_relative targets take no visual "
                  "correction; use type=\"none\"";
            ok = false;
            return;
        }

        set->targets.push_back(std::move(decl));
    });
    if (!ok) {
        return ResourceInstance{};
    }
    if (set->targets.empty()) {
        err = node.path() + ": target_set needs at least one Target";
        return ResourceInstance{};
    }
    return ResourceInstance::asContract<const TargetSet>(std::move(set));
}

} // namespace navigatr
