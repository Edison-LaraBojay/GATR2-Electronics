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

// Policy none: the current estimate, which is the nominal map pose or a
// previously committed (locked) observation, never in-flight evidence.
Pose2D TargetTracker::estimateTargetPose(const TargetDecl& decl,
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

// Timeout fallback: strictly the immutable field map nominal. "Use nominal
// target" means zero visual correction, so nothing the camera did before
// the timeout may leak in.
Pose2D TargetTracker::mapNominalTargetPose(const TargetDecl& decl,
                                           const WorldPredictionInput& in) const {
    Pose2D T_field_landmark;
    const LandmarkDecl* lm = field_->find(decl.landmark);
    if (lm != nullptr) {
        T_field_landmark = lm->nominal;
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

    // Activation edge: one new command sequence, one activation. A command
    // arriving while localization is not yet valid stays un-consumed and
    // activates on the first valid cycle; nothing ever snapshots zeros.
    if (in.command.object_sequence != 0 &&
        in.command.object_sequence != last_seen_object_sequence_ &&
        (in.robot.valid || !in.command.object_requested)) {
        last_seen_object_sequence_ = in.command.object_sequence;
        acquisition_               = AcquisitionBuffer{};
        ts                         = TargetState{};
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
                    ts.T_odom_robot_target = estimateTargetPose(*decl, in, out.world);
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
        acquisition_ = AcquisitionBuffer{};
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

    // Consume this generation's evidence into the private buffer. At most
    // one candidate per camera frame: three mounts in one image are one
    // observation, not three. Nothing outside the buffer changes until the
    // lock commits.
    if (!association_ref_.empty() && in.robot.valid) {
        const auto it = in.associations.find(association_ref_);
        if (it != in.associations.end()) {
            const LandmarkPoseObservationSet* set =
                it->second.payload.get<LandmarkPoseObservationSet>();
            if (set == nullptr) {
                out.status = FunctionStatus::kFault;
                return out;
            }
            // Best acceptable entry of this frame, by confidence.
            const LandmarkPoseObservation* accepted = nullptr;
            for (const LandmarkPoseObservation& entry : set->entries) {
                if (entry.target_generation != ts.generation ||
                    entry.landmark != decl->landmark) {
                    continue;   // stale generation or someone else's evidence
                }
                if ((in.now - entry.exposureAt) >
                    decl->vision.maximum_observation_age_ms) {
                    continue;
                }
                // Motion is gated at the moment of exposure, not at
                // processing time: with camera latency those differ.
                double rate_at_exposure = std::fabs(in.robot.yaw_rate_rad_s);
                double sampled          = 0.0;
                if (in.robot.yawRateAt(entry.exposureAt, sampled)) {
                    rate_at_exposure = std::fabs(sampled);
                }
                if (rate_at_exposure >
                    decl->vision.maximum_robot_angular_speed_rad_s) {
                    continue;
                }
                if (acquisition_.have_frame &&
                    entry.camera == acquisition_.last_frame_camera &&
                    entry.frame_sequence == acquisition_.last_frame_sequence) {
                    continue;   // this frame already contributed
                }
                if (accepted == nullptr || entry.confidence > accepted->confidence) {
                    accepted = &entry;
                }
            }

            if (accepted != nullptr) {
                acquisition_.have_frame          = true;
                acquisition_.last_frame_camera   = accepted->camera;
                acquisition_.last_frame_sequence = accepted->frame_sequence;

                const Pose2D candidate =
                    decl->resolveFromLandmark(accepted->T_odom_landmark);
                if (!acquisition_.target_candidates.empty()) {
                    const Pose2D& prev = acquisition_.target_candidates.back();
                    const double  dt   = std::hypot(candidate.x_m - prev.x_m,
                                                    candidate.y_m - prev.y_m);
                    const double  dh =
                        std::fabs(wrapAngle(candidate.heading_rad - prev.heading_rad));
                    if (dt > decl->vision.consistency_translation_m ||
                        dh > decl->vision.consistency_heading_rad) {
                        // inconsistent; restart the run, still privately
                        acquisition_.target_candidates.clear();
                        acquisition_.landmark_poses.clear();
                        acquisition_.confidences.clear();
                    }
                }
                acquisition_.target_candidates.push_back(candidate);
                acquisition_.landmark_poses.push_back(accepted->T_odom_landmark);
                acquisition_.confidences.push_back(accepted->confidence);

                if (static_cast<long>(acquisition_.target_candidates.size()) >=
                    decl->vision.minimum_consistent_observations) {
                    // Atomic commit: blend the consistent window's mean
                    // landmark into the world first, then derive the
                    // latched target from that final committed landmark, so
                    // the two can never disagree.
                    double lx = 0.0, ly = 0.0, lh_sin = 0.0, lh_cos = 0.0;
                    double conf = 0.0;
                    const double n =
                        static_cast<double>(acquisition_.landmark_poses.size());
                    for (std::size_t i = 0; i < acquisition_.landmark_poses.size();
                         ++i) {
                        const Pose2D& l = acquisition_.landmark_poses[i];
                        lx += l.x_m;
                        ly += l.y_m;
                        lh_sin += std::sin(l.heading_rad);
                        lh_cos += std::cos(l.heading_rad);
                        conf += acquisition_.confidences[i];
                    }

                    const Pose2D landmark_odom{lx / n, ly / n,
                                               std::atan2(lh_sin, lh_cos)};
                    const Pose2D T_field_landmark =
                        compose(in.robot.field_from_odom, landmark_odom);
                    WorldObject& obj = out.world.objects[decl->landmark];
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
                    obj.confidence      = conf / n;

                    const Pose2D committed_odom =
                        compose(inverse(in.robot.field_from_odom), obj.pose.pose);
                    ts.T_odom_robot_target = decl->resolveFromLandmark(committed_odom);
                    ts.latched             = true;
                    ts.status              = TargetStatus::kLockedVision;
                    acquisition_           = AcquisitionBuffer{};
                    return out;   // gate closed for this generation
                }
            }
        }
    }

    // Explicit timeout fallback; silence is not a policy.
    if ((in.now - ts.activatedAt) > decl->vision.acquisition_timeout_ms) {
        acquisition_ = AcquisitionBuffer{};
        if (decl->vision.on_timeout == AcquisitionFallback::kUseNominalTarget) {
            ts.T_odom_robot_target = mapNominalTargetPose(*decl, in);
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
