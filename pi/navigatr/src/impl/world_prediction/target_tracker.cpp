// target_tracker.cpp

#include "impl/world_prediction/target_tracker.h"

#include <cmath>
#include <typeindex>

#include "math/angles.h"
#include "payloads/landmark_pose_observations.h"
#include "resources/resource_map.h"

namespace navigatr
{

std::unique_ptr<WorldPrediction> TargetTracker::create(const ConfigNode& node,
                                                       SlotInitializationContext& context,
                                                       std::string& err) {
    auto tracker = std::make_unique<TargetTracker>();

    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }

    const ResourceId targets_id{node.child("Targets").attr("resource_id")};
    if (targets_id.empty()) {
        err = node.path() + ": needs <Targets resource_id=.../>";
        return nullptr;
    }
    std::string inner;
    tracker->targets_ = context.resources->require<const TargetSet>(targets_id, inner);
    if (tracker->targets_ == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    bool any_landmark = false;
    bool any_acquire  = false;
    for (const TargetDecl& t : tracker->targets_->targets) {
        any_landmark = any_landmark || t.kind == TargetKind::kLandmarkRelative;
        any_acquire  = any_acquire || t.vision.policy == VisionPolicy::kAcquireOnce;
    }

    const ResourceId field_id{node.child("FieldMap").attr("resource_id")};
    if (!field_id.empty()) {
        tracker->field_ = context.resources->require<const FieldMap>(field_id, inner);
        if (tracker->field_ == nullptr) {
            err = node.path() + ": " + inner;
            return nullptr;
        }
    } else if (any_landmark) {
        err = node.path() + ": the target set has landmark_relative targets; needs "
              "<FieldMap resource_id=.../>";
        return nullptr;
    }

    const ConfigNode associations = node.child("Associations");
    if (associations.valid()) {
        tracker->association_ref_ = AssociationId{associations.attr("association_id")};
        const std::type_index expected(typeid(LandmarkPoseObservationSet));
        if (tracker->association_ref_.empty() ||
            !context.requireAssociation(tracker->association_ref_, &expected,
                                        associations.path(), err)) {
            if (err.empty()) {
                err = associations.path() + ": Associations needs association_id";
            }
            return nullptr;
        }
    } else if (any_acquire) {
        err = node.path() + ": the target set has acquire_once targets; needs "
              "<Associations association_id=.../>";
        return nullptr;
    }

    const ConfigNode landmark = node.child("Landmark");
    if (landmark.valid()) {
        if (!landmark.getDouble("blend", 1.0, tracker->blend_, err)) {
            return nullptr;
        }
        if (tracker->blend_ < 0.0 || tracker->blend_ > 1.0) {
            err = landmark.path() + ": blend must be within 0..1";
            return nullptr;
        }
    }
    return tracker;
}

Pose2D TargetTracker::nominalTargetPose(const TargetDecl& decl,
                                        const WorldPredictionInput& in,
                                        const WorldState& world) const {
    Pose2D T_field_landmark;
    const LandmarkDecl* lm = field_->find(decl.landmark);
    if (lm != nullptr) {
        T_field_landmark = lm->nominal;
    }
    const auto it = world.objects.find(decl.landmark);
    if (it != world.objects.end() && it->second.valid) {
        T_field_landmark = it->second.pose.pose;
    }
    const Pose2D T_odom_landmark =
        compose(inverse(in.robot.field_from_odom), T_field_landmark);
    return decl.resolveFromLandmark(T_odom_landmark);
}

WorldPredictionOutput TargetTracker::run(const WorldPredictionInput& in) {
    WorldPredictionOutput out;
    out.world  = in.previousWorld;
    out.target = in.previousTarget;
    TargetState& ts = out.target;

    // Seed nominal poses so every mapped landmark has an estimate.
    if (field_ != nullptr) {
        for (const LandmarkDecl& decl : field_->landmarks) {
            if (out.world.objects.find(decl.id) != out.world.objects.end()) {
                continue;
            }
            WorldObject obj;
            obj.pose.frame      = FrameId{"field"};
            obj.pose.pose       = decl.nominal;
            obj.pose.measuredAt = in.now;
            obj.confidence      = 0.5;
            obj.valid           = true;
            obj.source          = EstimateSource::kFieldMap;
            out.world.objects.emplace(decl.id, obj);
        }
    }
    for (auto& kv : out.world.objects) {
        kv.second.observed = false;
    }

    // Activation edge: one new command sequence, one activation.
    if (in.command.object_sequence != 0 &&
        in.command.object_sequence != last_seen_object_sequence_) {
        last_seen_object_sequence_ = in.command.object_sequence;
        candidates_.clear();
        ts = TargetState{};
        if (in.command.object_requested) {
            const TargetDecl* decl = targets_->findByWireId(in.command.object_wire_id);
            if (decl == nullptr) {
                out.status = FunctionStatus::kFault;   // brain asked for an unmapped target
            } else {
                ts.active         = true;
                ts.target_id      = decl->id;
                ts.wire_id        = decl->wire_id;
                ts.generation     = ++generation_counter_;
                ts.activatedAt    = in.now;
                ts.odometry_epoch = in.robot.odometry_epoch;
                if (decl->kind == TargetKind::kRobotRelative) {
                    ts.T_odom_robot_target = compose(in.robot.odom_pose, decl->delta);
                    ts.latched             = true;
                    ts.status              = TargetStatus::kLockedRobotRelative;
                } else if (decl->vision.policy == VisionPolicy::kNone) {
                    ts.T_odom_robot_target = nominalTargetPose(*decl, in, out.world);
                    ts.latched             = true;
                    ts.status              = TargetStatus::kLockedNominal;
                } else {
                    ts.status = TargetStatus::kPendingAcquisition;
                }
            }
        }
    }

    if (!ts.active) {
        return out;
    }

    // A discontinuous odometry frame invalidates anything anchored in it.
    if (ts.status != TargetStatus::kCancelled &&
        ts.odometry_epoch != in.robot.odometry_epoch) {
        ts.status  = TargetStatus::kCancelled;
        ts.latched = false;
        candidates_.clear();
        return out;
    }

    if (ts.status != TargetStatus::kPendingAcquisition) {
        return out;   // latched targets stay bit for bit unchanged
    }

    const TargetDecl* decl = targets_->findById(ts.target_id);
    if (decl == nullptr) {
        ts.status = TargetStatus::kCancelled;
        return out;
    }

    // Consume this generation's evidence.
    if (!association_ref_.empty()) {
        const auto it = in.associations.find(association_ref_);
        if (it != in.associations.end()) {
            const LandmarkPoseObservationSet* set =
                it->second.payload.get<LandmarkPoseObservationSet>();
            if (set == nullptr) {
                out.status = FunctionStatus::kFault;
                return out;
            }
            for (const LandmarkPoseObservation& entry : set->entries) {
                if (entry.target_generation != ts.generation ||
                    entry.landmark != decl->landmark) {
                    continue;   // stale generation or someone else's evidence
                }
                if ((in.now - entry.exposureAt) > decl->vision.maximum_observation_age_ms) {
                    continue;
                }
                if (std::fabs(in.robot.yaw_rate_rad_s) >
                    decl->vision.maximum_robot_angular_speed_rad_s) {
                    continue;
                }

                // The one landmark this target selects is the only one that
                // may mutate.
                WorldObject& obj = out.world.objects[entry.landmark];
                const Pose2D T_field_landmark =
                    compose(in.robot.field_from_odom, entry.T_odom_landmark);
                if (!obj.valid) {
                    obj.pose.frame = FrameId{"field"};
                    obj.pose.pose  = T_field_landmark;
                } else {
                    obj.pose.pose.x_m +=
                        (T_field_landmark.x_m - obj.pose.pose.x_m) * blend_;
                    obj.pose.pose.y_m +=
                        (T_field_landmark.y_m - obj.pose.pose.y_m) * blend_;
                    obj.pose.pose.heading_rad = wrapAngle(
                        obj.pose.pose.heading_rad +
                        wrapAngle(T_field_landmark.heading_rad -
                                  obj.pose.pose.heading_rad) *
                            blend_);
                }
                obj.pose.measuredAt = in.now;
                obj.valid           = true;
                obj.observed        = true;
                obj.source          = EstimateSource::kObserved;
                obj.confidence      = entry.confidence;

                // Consistency run over successive candidate target poses.
                const Pose2D candidate = decl->resolveFromLandmark(entry.T_odom_landmark);
                if (!candidates_.empty()) {
                    const Pose2D& prev = candidates_.back();
                    const double  dt   = std::hypot(candidate.x_m - prev.x_m,
                                                    candidate.y_m - prev.y_m);
                    const double  dh =
                        std::fabs(wrapAngle(candidate.heading_rad - prev.heading_rad));
                    if (dt > decl->vision.consistency_translation_m ||
                        dh > decl->vision.consistency_heading_rad) {
                        candidates_.clear();   // inconsistent; restart the run
                    }
                }
                candidates_.push_back(candidate);

                if (static_cast<long>(candidates_.size()) >=
                    decl->vision.minimum_consistent_observations) {
                    double sx = 0.0, sy = 0.0, sh_sin = 0.0, sh_cos = 0.0;
                    for (const Pose2D& c : candidates_) {
                        sx += c.x_m;
                        sy += c.y_m;
                        sh_sin += std::sin(c.heading_rad);
                        sh_cos += std::cos(c.heading_rad);
                    }
                    const double n         = static_cast<double>(candidates_.size());
                    ts.T_odom_robot_target = Pose2D{
                        sx / n, sy / n, std::atan2(sh_sin, sh_cos)};
                    ts.latched = true;
                    ts.status  = TargetStatus::kLockedVision;
                    candidates_.clear();
                    return out;   // gate closed for this generation
                }
            }
        }
    }

    // Explicit timeout fallback; silence is not a policy.
    if ((in.now - ts.activatedAt) > decl->vision.acquisition_timeout_ms) {
        candidates_.clear();
        if (decl->vision.on_timeout == AcquisitionFallback::kUseNominalTarget) {
            ts.T_odom_robot_target = nominalTargetPose(*decl, in, out.world);
            ts.latched             = true;
            ts.status              = TargetStatus::kLockedNominal;
        } else {
            ts.latched = false;
            ts.status  = TargetStatus::kCancelled;
        }
    }
    return out;
}

} // namespace navigatr
