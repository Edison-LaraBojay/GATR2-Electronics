// tag_mount_association.cpp

#include "impl/world_estimation/tag_mount_association.h"

#include <cmath>
#include <typeindex>

#include "math/angles.h"
#include "payloads/field_object_evidence.h"
#include "payloads/tag_observations.h"
#include "resources/resource_store.h"

namespace navigatr
{

std::unique_ptr<Association> TagMountAssociation::create(const ConfigNode& node,
                                                         SlotInitializationContext& context,
                                                         std::string& err) {
    auto assoc = std::make_unique<TagMountAssociation>();

    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    // Strict schema: navigation intent (Targets, preferred cameras, allowed
    // mounts) does not belong here and is rejected, not ignored.
    for (auto child = node.child(); child.valid(); child = child.next()) {
        const std::string name = child.name();
        if (name != "Observations" && name != "FieldMap" && name != "RobotFrames" &&
            name != "Gates" && name != "Output" && name != "Trace" && name != "Attitude") {
            err = node.path() + " has unknown element " + name;
            return nullptr;
        }
    }

    assoc->observations_ref_ =
        ObservationId{node.child("Observations").attr("observation_id")};
    if (assoc->observations_ref_.empty()) {
        err = node.path() + ": needs <Observations observation_id=.../>";
        return nullptr;
    }
    const std::type_index expected(typeid(TagObservationSet));
    if (!context.requireObservation(assoc->observations_ref_, &expected, node.path(),
                                    err)) {
        return nullptr;
    }

    const auto requireResource = [&](const char* element, auto& out) {
        const ResourceId id{node.child(element).attr("resource_id")};
        if (id.empty()) {
            err = node.path() + ": needs <" + std::string(element) + " resource_id=.../>";
            return false;
        }
        std::string inner;
        out = context.resources
                  ->require<typename std::decay_t<decltype(out)>::element_type>(id, inner);
        if (out == nullptr) {
            err = node.path() + ": " + inner;
            return false;
        }
        return true;
    };
    if (!requireResource("FieldMap", assoc->field_) ||
        !requireResource("RobotFrames", assoc->frames_)) {
        return nullptr;
    }

    const ConfigNode attitude = node.child("Attitude");
    if (attitude.valid()) {
        const std::string policy = attitude.attr("policy");
        if (policy == "assume_level") {
            assoc->attitude_policy_ = AttitudePolicy::kAssumeLevel;
        } else if (policy == "require") {
            assoc->attitude_policy_ = AttitudePolicy::kRequire;
        } else {
            err = attitude.path() + ": policy must be assume_level or require";
            return nullptr;
        }
    }

    const ConfigNode gates = node.child("Gates");
    if (!gates.valid()) {
        err = node.path() + ": needs <Gates .../>";
        return nullptr;
    }
    double heading_deg = 0.0;
    if (!gates.requireDouble("max_translation_error_m", assoc->max_translation_error_m_,
                             err) ||
        !gates.requireDouble("max_heading_error_deg", heading_deg, err) ||
        !gates.requireDouble("ambiguity_margin_m", assoc->ambiguity_margin_m_, err) ||
        !gates.requireDouble("max_range_m", assoc->max_range_m_, err) ||
        !gates.requireDouble("min_decision_margin", assoc->min_decision_margin_, err) ||
        !gates.getInt("max_hamming", 0, assoc->max_hamming_, err) ||
        !gates.getDouble("min_facing_cos", 0.1, assoc->min_facing_cos_, err) ||
        !gates.getDouble("min_projected_size_px", 8.0, assoc->min_projected_size_px_,
                         err) ||
        !gates.getDouble("max_reprojection_error_px", 0.0,
                         assoc->max_reprojection_error_px_, err) ||
        !gates.getDouble("max_alternate_pose_ambiguity", 0.0,
                         assoc->max_alternate_pose_ambiguity_, err)) {
        return nullptr;
    }
    if (assoc->max_translation_error_m_ <= 0.0 || heading_deg <= 0.0 ||
        assoc->ambiguity_margin_m_ <= 0.0 || assoc->max_range_m_ <= 0.0 ||
        assoc->min_decision_margin_ < 0.0 || assoc->max_hamming_ < 0 ||
        assoc->min_facing_cos_ <= 0.0 || assoc->min_facing_cos_ >= 1.0 ||
        assoc->min_projected_size_px_ < 0.0 ||
        assoc->max_reprojection_error_px_ < 0.0 ||
        assoc->max_alternate_pose_ambiguity_ < 0.0) {
        err = gates.path() + ": gate values out of range";
        return nullptr;
    }
    assoc->max_heading_error_rad_ = degToRad(heading_deg);

    assoc->output_ = AssociationId{node.child("Output").attr("association_id")};
    if (assoc->output_.empty()) {
        err = node.path() + ": needs <Output association_id=.../>";
        return nullptr;
    }
    const ConfigNode trace = node.child("Trace");
    if (trace.valid()) {
        assoc->trace_ = AssociationId{trace.attr("association_id")};
        if (assoc->trace_.empty()) {
            err = trace.path() + ": Trace needs association_id";
            return nullptr;
        }
        if (assoc->trace_ == assoc->output_) {
            err = trace.path() + ": Trace and Output need different ids";
            return nullptr;
        }
    }
    return assoc;
}

std::vector<AssociationOutputDecl> TagMountAssociation::produces() const {
    std::vector<AssociationOutputDecl> out;
    out.push_back(AssociationOutputDecl{
        output_, PayloadDescriptor::of<FieldObjectPoseEvidenceSet>(
                     payload_names::kFieldObjectPoseEvidenceSet)});
    if (!trace_.empty()) {
        out.push_back(AssociationOutputDecl{
            trace_, PayloadDescriptor::of<TagAssociationTraceSet>(
                        payload_names::kTagAssociationTraceSet)});
    }
    return out;
}

AssociationOutput TagMountAssociation::run(const AssociationInput& in) {
    AssociationOutput out;

    const auto obs_it = in.observations.find(observations_ref_);
    if (obs_it == in.observations.end()) {
        return out;   // camera between frames; nothing to associate
    }
    const TagObservationSet* set = obs_it->second.payload.get<TagObservationSet>();
    if (set == nullptr) {
        out.status = FunctionStatus::kFault;
        return out;
    }

    TagAssociationTraceSet trace;
    trace.camera         = set->camera;
    trace.frame_sequence = set->frame_sequence;
    trace.frame_epoch    = set->frame_epoch;
    trace.exposureAt     = set->exposureAt;
    trace.exposure_context = in.history.sampleAt(set->exposureAt);
    trace.has_exposure_context = true;
    trace.field_from_odom = in.robot.field_from_odom;
    trace.anchor_revision = in.robot.anchor_revision;

    const auto publishTrace = [&] {
        if (trace_.empty()) {
            return;
        }
        AssociationRecord record;
        record.measuredAt = set->exposureAt;
        record.payload =
            TypedPayload::store(std::move(trace), payload_names::kTagAssociationTraceSet);
        out.associations.emplace(trace_, std::move(record));
    };

    // Whole-frame preconditions, each named in the trace.
    const Transform3* T_robot_camera = frames_->find(set->camera_frame);
    if (T_robot_camera == nullptr) {
        out.status       = FunctionStatus::kFault;   // camera extrinsic frame unmapped
        trace.frame_note = "camera frame " + set->camera_frame.value +
                           " is not in the robot frame map";
        publishTrace();
        return out;
    }
    if (!in.robot.valid) {
        trace.frame_note = "robot estimate not valid";
        publishTrace();
        return out;
    }
    if (!set->exposure_time_reliable) {
        trace.frame_note = "exposure timing is not reliable";
        publishTrace();
        return out;
    }
    if (set->exposureAt > in.now) {
        trace.frame_note = "exposure in the future";   // broken timing, not evidence
        publishTrace();
        return out;
    }
    const PoseLookupResult& at_exposure = trace.exposure_context.pose;
    if (at_exposure.status != LookupStatus::kOk) {
        trace.frame_note =
            std::string("no robot pose at exposure: ") + lookupStatusName(at_exposure.status);
        publishTrace();
        return out;
    }
    if (at_exposure.odometry_epoch != in.robot.odometry_epoch) {
        trace.frame_note = "exposure belongs to a previous odometry epoch";
        publishTrace();
        return out;
    }

    // Attitude at exposure: measured tilt when localization has one,
    // otherwise the configured degraded policy.
    bool                       attitude_assumed = false;
    const AttitudeLookupResult& attitude        = trace.exposure_context.attitude;
    Transform3                 T_odom_robot;
    if (attitude.status == LookupStatus::kOk && attitude.attitude.valid) {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        attitudeEuler(attitude.attitude, roll, pitch, yaw);
        T_odom_robot.R = rotationFromEuler(roll, pitch, at_exposure.odom_pose.heading_rad);
    } else {
        if (attitude_policy_ == AttitudePolicy::kRequire) {
            trace.frame_note = std::string("attitude at exposure unavailable (") +
                               lookupStatusName(attitude.status) + ") and policy requires it";
            publishTrace();
            return out;
        }
        attitude_assumed = true;
        T_odom_robot.R   = rotationFromEuler(0.0, 0.0, at_exposure.odom_pose.heading_rad);
    }
    T_odom_robot.x_m = at_exposure.odom_pose.x_m;
    T_odom_robot.y_m = at_exposure.odom_pose.y_m;
    T_odom_robot.z_m = 0.0;   // ground plane assumption, stated not estimated
    const Transform3 T_odom_camera = compose(T_odom_robot, *T_robot_camera);

    // Heading residual weighted into the ranking score so a
    // translation-close but twisted candidate does not win: one full
    // heading gate costs as much as one full translation gate.
    const double heading_weight = max_translation_error_m_ / max_heading_error_rad_;

    FieldObjectPoseEvidenceSet result;
    for (std::size_t tag_index = 0; tag_index < set->tags.size(); ++tag_index) {
        const TagObservation& tag = set->tags[tag_index];
        TagAssociationTrace   t;
        t.tag_index   = tag_index;
        t.family      = tag.family;
        t.observed_id = tag.observed_id;
        const auto reject = [&](const std::string& why) {
            t.accepted  = false;
            t.rejection = why;
            trace.tags.push_back(t);
        };

        if (!tag.has_pose) {
            reject("no metric pose (camera not calibrated or unknown tag size)");
            continue;
        }
        // Detector quality first: bad decodes are not evidence. An enabled
        // gate whose value the detector did not report rejects; absent is
        // never treated as a passing zero.
        if (tag.hamming > max_hamming_) {
            reject("hamming above gate");
            continue;
        }
        if (tag.decision_margin < min_decision_margin_) {
            reject("decision margin below gate");
            continue;
        }
        if (max_reprojection_error_px_ > 0.0 &&
            (!tag.has_reprojection_error ||
             tag.reprojection_error_px > max_reprojection_error_px_)) {
            reject(tag.has_reprojection_error ? "reprojection error above gate"
                                              : "reprojection error not reported");
            continue;
        }
        if (max_alternate_pose_ambiguity_ > 0.0 &&
            (!tag.has_alternate_pose_ambiguity ||
             tag.alternate_pose_ambiguity > max_alternate_pose_ambiguity_)) {
            reject(tag.has_alternate_pose_ambiguity ? "pose ambiguity above gate"
                                                    : "pose ambiguity not reported");
            continue;
        }

        // Physical plausibility: the tag must sit in front of the camera,
        // within range, with its outward normal facing back at the lens (a
        // mirrored pose solution fails this).
        const double range_m =
            std::sqrt(tag.T_camera_tag.x_m * tag.T_camera_tag.x_m +
                      tag.T_camera_tag.y_m * tag.T_camera_tag.y_m +
                      tag.T_camera_tag.z_m * tag.T_camera_tag.z_m);
        if (tag.T_camera_tag.x_m <= 0.0) {
            reject("solved pose behind the camera");
            continue;
        }
        if (range_m > max_range_m_) {
            reject("beyond max_range_m");
            continue;
        }
        if (tag.T_camera_tag.R.m[0][0] > -min_facing_cos_) {
            reject("surface not facing the camera");
            continue;
        }

        for (const LandmarkDecl& lm : field_->landmarks) {
            // Prior landmark pose in the odometry frame: the current field
            // estimate when one exists, else the nominal map pose.
            Pose2D T_field_landmark = lm.nominal;
            const auto field_it     = in.field.objects.find(lm.id);
            if (field_it != in.field.objects.end() && field_it->second.valid) {
                T_field_landmark = field_it->second.pose.pose;
            }
            const Pose2D prior =
                compose(inverse(in.robot.field_from_odom), T_field_landmark);

            for (const TagMountDecl& mount : lm.mounts) {
                if (mount.observed_id != tag.observed_id || mount.family != tag.family) {
                    continue;
                }
                // Expected mount visibility from the prior: in front of the
                // camera, its outward normal toward the lens (a physically
                // back-facing mount is not a candidate), projecting inside
                // the calibrated image, and large enough to detect.
                const Transform3 expected = compose(
                    inverse(T_odom_camera),
                    compose(transform3FromPlanar(prior), mount.T_landmark_tag_surface));
                if (expected.x_m <= 0.0 || expected.R.m[0][0] > -min_facing_cos_) {
                    continue;
                }
                if (set->intrinsics != nullptr) {
                    const CameraIntrinsics& K = *set->intrinsics;
                    // engineering to optical: image-right = -y, image-down
                    // = -z, depth = +x; nominal (distortion-agnostic)
                    // projection is enough for a visibility prune
                    const double u = K.cx_px + K.fx_px * (-expected.y_m / expected.x_m);
                    const double v = K.cy_px + K.fy_px * (-expected.z_m / expected.x_m);
                    if (u < 0.0 || u >= static_cast<double>(K.calibrated_width_px) ||
                        v < 0.0 || v >= static_cast<double>(K.calibrated_height_px)) {
                        continue;
                    }
                    const double size_px = K.fx_px * mount.detection_size_m / expected.x_m;
                    if (size_px < min_projected_size_px_) {
                        continue;
                    }
                }

                TagAssociationCandidate c;
                c.object  = lm.id;
                c.mount   = mount.instance_id;
                c.implied = planarFromTransform3(
                    compose(compose(T_odom_camera, tag.T_camera_tag),
                            inverse(mount.T_landmark_tag_surface)));
                c.translation_error_m =
                    std::hypot(c.implied.x_m - prior.x_m, c.implied.y_m - prior.y_m);
                c.heading_error_rad =
                    std::fabs(wrapAngle(c.implied.heading_rad - prior.heading_rad));
                c.score = c.translation_error_m + heading_weight * c.heading_error_rad;
                t.candidates.push_back(c);
            }
        }
        if (t.candidates.empty()) {
            reject("no configured mount with this id is expected in view");
            continue;
        }

        const TagAssociationCandidate* best   = nullptr;
        const TagAssociationCandidate* second = nullptr;
        for (const TagAssociationCandidate& c : t.candidates) {
            if (best == nullptr || c.score < best->score) {
                second = best;
                best   = &c;
            } else if (second == nullptr || c.score < second->score) {
                second = &c;
            }
        }

        // Gates, then decisive-margin ambiguity on the combined score: the
        // winner must beat every other candidate clearly, or the evidence
        // stays unassociated.
        if (best->translation_error_m > max_translation_error_m_) {
            reject("best candidate outside the translation gate");
            continue;
        }
        if (best->heading_error_rad > max_heading_error_rad_) {
            reject("best candidate outside the heading gate");
            continue;
        }
        if (second != nullptr && (second->score - best->score) < ambiguity_margin_m_) {
            reject("ambiguous: runner-up " + second->object.value + "/" + second->mount +
                   " within the ambiguity margin");
            continue;
        }

        t.accepted = true;
        t.object   = best->object;
        t.mount    = best->mount;
        trace.tags.push_back(t);

        FieldObjectPoseEvidence entry;
        entry.object           = best->object;
        entry.feature_instance = best->mount;
        entry.frame            = FrameId{"odometry"};
        entry.T_frame_object   = best->implied;
        entry.measuredAt       = set->exposureAt;
        entry.source           = set->camera;
        entry.source_sequence  = set->frame_sequence;
        entry.odometry_epoch   = at_exposure.odometry_epoch;
        entry.attitude_assumed = attitude_assumed;
        // confidence reflects the same combined score the ranking used;
        // its maximum possible value is one translation gate plus one
        // heading gate worth of weighted error
        entry.confidence = 1.0 - best->score / (2.0 * max_translation_error_m_);
        result.entries.push_back(std::move(entry));
    }

    if (!result.entries.empty()) {
        AssociationRecord record;
        record.measuredAt = set->exposureAt;
        record.payload    = TypedPayload::store(
            std::move(result), payload_names::kFieldObjectPoseEvidenceSet);
        out.associations.emplace(output_, std::move(record));
    }
    publishTrace();
    return out;
}

} // namespace navigatr
