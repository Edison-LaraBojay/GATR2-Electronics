// tag_mount_association.cpp

#include "impl/association/tag_mount_association.h"

#include <cmath>
#include <typeindex>

#include "math/angles.h"
#include "payloads/landmark_pose_observations.h"
#include "payloads/tag_observations.h"
#include "resources/resource_map.h"

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
        !requireResource("RobotFrames", assoc->frames_) ||
        !requireResource("Targets", assoc->targets_)) {
        return nullptr;
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
        !gates.requireDouble("ambiguity_margin_m", assoc->ambiguity_margin_m_, err)) {
        return nullptr;
    }
    if (assoc->max_translation_error_m_ <= 0.0 || heading_deg <= 0.0 ||
        assoc->ambiguity_margin_m_ <= 0.0) {
        err = gates.path() + ": gate values must be positive";
        return nullptr;
    }
    assoc->max_heading_error_rad_ = degToRad(heading_deg);

    assoc->output_ = AssociationId{node.child("Output").attr("association_id")};
    if (assoc->output_.empty()) {
        err = node.path() + ": needs <Output association_id=.../>";
        return nullptr;
    }
    return assoc;
}

std::vector<AssociationOutputDecl> TagMountAssociation::produces() const {
    return {AssociationOutputDecl{
        output_, PayloadDescriptor::of<LandmarkPoseObservationSet>(
                     payload_names::kLandmarkPoseObservationSet)}};
}

AssociationOutput TagMountAssociation::run(const AssociationInput& in) {
    AssociationOutput out;

    // Gating: association evidence exists to acquire the selected target.
    // No target pending acquisition means no work and no correction commit.
    if (!in.target.active || in.target.status != TargetStatus::kPendingAcquisition) {
        return out;
    }
    const TargetDecl* decl = targets_->findById(in.target.target_id);
    if (decl == nullptr || decl->kind != TargetKind::kLandmarkRelative) {
        return out;
    }

    const auto obs_it = in.observations.find(observations_ref_);
    if (obs_it == in.observations.end()) {
        return out;   // camera between frames; nothing to associate
    }
    const TagObservationSet* set = obs_it->second.payload.get<TagObservationSet>();
    if (set == nullptr) {
        out.status = FunctionStatus::kFault;
        return out;
    }
    if (!decl->vision.preferred_camera.empty() &&
        set->camera != decl->vision.preferred_camera) {
        return out;
    }

    const Transform3* T_robot_camera = frames_->find(set->camera_frame);
    if (T_robot_camera == nullptr) {
        out.status = FunctionStatus::kFault;   // camera extrinsic frame unmapped
        return out;
    }

    Pose2D robot_at_exposure;
    if (!in.robot.odomPoseAt(set->exposureAt, robot_at_exposure)) {
        return out;   // exposure predates retained history
    }
    const Transform3 T_odom_camera =
        compose(transform3FromPlanar(robot_at_exposure), *T_robot_camera);

    LandmarkPoseObservationSet result;
    for (const TagObservation& tag : set->tags) {
        struct Candidate {
            const LandmarkDecl* landmark = nullptr;
            const TagMountDecl* mount    = nullptr;
            Pose2D              implied;   // T_odom_landmark
            double              translation_error_m = 0.0;
            double              heading_error_rad   = 0.0;
        };
        std::vector<Candidate> candidates;

        for (const LandmarkDecl& lm : field_->landmarks) {
            // Prior landmark pose in the odometry frame: the current world
            // estimate when one exists, else the nominal map pose.
            Pose2D T_field_landmark = lm.nominal;
            const auto world_it     = in.world.objects.find(lm.id);
            if (world_it != in.world.objects.end() && world_it->second.valid) {
                T_field_landmark = world_it->second.pose.pose;
            }
            const Pose2D prior =
                compose(inverse(in.robot.field_from_odom), T_field_landmark);

            for (const TagMountDecl& mount : lm.mounts) {
                if (mount.observed_id != tag.observed_id || mount.family != tag.family) {
                    continue;
                }
                Candidate c;
                c.landmark = &lm;
                c.mount    = &mount;
                c.implied  = planarFromTransform3(
                    compose(compose(T_odom_camera, tag.T_camera_tag),
                            inverse(mount.T_landmark_tag_surface)));
                c.translation_error_m = std::hypot(c.implied.x_m - prior.x_m,
                                                   c.implied.y_m - prior.y_m);
                c.heading_error_rad =
                    std::fabs(wrapAngle(c.implied.heading_rad - prior.heading_rad));
                candidates.push_back(c);
            }
        }
        if (candidates.empty()) {
            continue;   // a tag the map does not know
        }

        const Candidate* best   = nullptr;
        const Candidate* second = nullptr;
        for (const Candidate& c : candidates) {
            if (best == nullptr || c.translation_error_m < best->translation_error_m) {
                second = best;
                best   = &c;
            } else if (second == nullptr ||
                       c.translation_error_m < second->translation_error_m) {
                second = &c;
            }
        }

        // Gates, then decisive-margin ambiguity: the winner must beat every
        // other candidate clearly, or the evidence stays unassociated.
        if (best->translation_error_m > max_translation_error_m_ ||
            best->heading_error_rad > max_heading_error_rad_) {
            continue;
        }
        if (second != nullptr &&
            (second->translation_error_m - best->translation_error_m) <
                ambiguity_margin_m_) {
            continue;
        }

        // Evidence serves the active target; a decisive winner on another
        // landmark is simply not this target's evidence.
        if (best->landmark->id != decl->landmark) {
            continue;
        }

        // A route may allow only specific mounts; a decisive winner outside
        // that list abstains rather than being forced onto an allowed one.
        if (!decl->vision.allowed_mounts.empty()) {
            bool allowed = false;
            for (const std::string& instance : decl->vision.allowed_mounts) {
                if (instance == best->mount->instance_id) {
                    allowed = true;
                    break;
                }
            }
            if (!allowed) {
                continue;
            }
        }

        LandmarkPoseObservation entry;
        entry.landmark          = best->landmark->id;
        entry.mount_instance    = best->mount->instance_id;
        entry.T_odom_landmark   = best->implied;
        entry.exposureAt        = set->exposureAt;
        entry.target_generation = in.target.generation;
        entry.camera            = set->camera;
        entry.confidence =
            1.0 - best->translation_error_m / max_translation_error_m_;
        result.entries.push_back(std::move(entry));
    }

    if (!result.entries.empty()) {
        AssociationRecord record;
        record.measuredAt = set->exposureAt;
        record.payload    = TypedPayload::store(
            std::move(result), payload_names::kLandmarkPoseObservationSet);
        out.associations.emplace(output_, std::move(record));
    }
    return out;
}

} // namespace navigatr
