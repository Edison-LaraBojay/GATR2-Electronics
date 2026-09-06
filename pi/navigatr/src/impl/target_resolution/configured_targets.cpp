// configured_targets.cpp

#include "impl/target_resolution/configured_targets.h"

#include <cmath>
#include <typeindex>

#include "math/angles.h"
#include "payloads/field_object_evidence.h"
#include "resources/resource_map.h"

namespace navigatr
{

std::unique_ptr<TargetResolution> ConfiguredTargetResolution::create(
    const ConfigNode& node, SlotInitializationContext& context, std::string& err) {
    auto resolver = std::make_unique<ConfiguredTargetResolution>();

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
    resolver->targets_ = context.resources->require<const TargetSet>(targets_id, inner);
    if (resolver->targets_ == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    bool any_landmark = false;
    bool any_acquire  = false;
    for (const auto& t : resolver->targets_->targets) {
        any_landmark = any_landmark || t.kind == TargetKind::kLandmarkRelative;
        any_acquire  = any_acquire || t.vision.policy == VisionPolicy::kAcquireOnce;
    }

    const ResourceId field_id{node.child("FieldMap").attr("resource_id")};
    if (!field_id.empty()) {
        resolver->field_ = context.resources->require<const FieldMap>(field_id, inner);
        if (resolver->field_ == nullptr) {
            err = node.path() + ": " + inner;
            return nullptr;
        }
    } else if (any_landmark) {
        err = node.path() + ": the target set has landmark_relative targets; needs "
              "<FieldMap resource_id=.../>";
        return nullptr;
    }

    const auto evidence = node.child("Evidence");
    if (evidence.valid()) {
        resolver->evidence_ref_ = AssociationId{evidence.attr("association_id")};
        const std::type_index expected(typeid(FieldObjectPoseEvidenceSet));
        if (resolver->evidence_ref_.empty() ||
            !context.requireAssociation(resolver->evidence_ref_, &expected,
                                        evidence.path(), err)) {
            if (err.empty()) {
                err = evidence.path() + ": Evidence needs association_id";
            }
            return nullptr;
        }
    } else if (any_acquire) {
        err = node.path() + ": the target set has acquire_once targets; needs "
              "<Evidence association_id=.../>";
        return nullptr;
    }
    return resolver;
}

// Policy none: the current committed estimate, which is the nominal map
// pose or a previously committed observation, never in-flight evidence.
Pose2D ConfiguredTargetResolution::estimateTargetPose(
    const TargetDecl& decl, const TargetResolutionInput& in) const {
    Pose2D T_field_landmark;
    if (const auto* lm = field_->find(decl.landmark)) {
        T_field_landmark = lm->nominal;
    }
    const auto it = in.field.objects.find(decl.landmark);
    if (it != in.field.objects.end() && it->second.valid) {
        T_field_landmark = it->second.pose.pose;
    }
    const auto T_odom_landmark =
        compose(inverse(in.robot.field_from_odom), T_field_landmark);
    return decl.resolveFromLandmark(T_odom_landmark);
}

// Timeout fallback: strictly the immutable field map nominal. "Use nominal
// target" means zero visual correction, so nothing the camera did before
// the timeout may leak in.
Pose2D ConfiguredTargetResolution::mapNominalTargetPose(
    const TargetDecl& decl, const TargetResolutionInput& in) const {
    Pose2D T_field_landmark;
    if (const auto* lm = field_->find(decl.landmark)) {
        T_field_landmark = lm->nominal;
    }
    const auto T_odom_landmark =
        compose(inverse(in.robot.field_from_odom), T_field_landmark);
    return decl.resolveFromLandmark(T_odom_landmark);
}

TargetResolutionOutput ConfiguredTargetResolution::run(const TargetResolutionInput& in) {
    TargetResolutionOutput out;
    out.target      = in.previous;
    TargetState& ts = out.target;

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
            const auto* decl = targets_->findByWireId(in.command.object_wire_id);
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
                    ts.T_odom_robot_target = estimateTargetPose(*decl, in);
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
        ts.status    = TargetStatus::kCancelled;
        ts.latched   = false;
        acquisition_ = AcquisitionBuffer{};
        return out;
    }

    if (ts.status != TargetStatus::kPendingAcquisition) {
        return out;   // latched targets stay bit for bit unchanged
    }

    const auto* decl = targets_->findById(ts.target_id);
    if (decl == nullptr) {
        ts.status = TargetStatus::kCancelled;
        return out;
    }

    // Consume this generation's evidence into the private buffer. At most
    // one candidate per (camera, frame): three mounts in one image are one
    // observation, not three. Nothing outside the buffer changes until the
    // lock reads it.
    if (!evidence_ref_.empty() && in.robot.valid) {
        const auto it = in.associations.find(evidence_ref_);
        if (it != in.associations.end()) {
            const auto* set = it->second.payload.get<FieldObjectPoseEvidenceSet>();
            if (set == nullptr) {
                out.status = FunctionStatus::kFault;
                return out;
            }
            // Navigation-intent filtering happens here, never in generic
            // association: the requested object, the route's allowed
            // feature mounts and preferred source, freshness, and the
            // activation time all gate which generic evidence may acquire
            // this target. Best acceptable entry of this frame wins by
            // confidence.
            const FieldObjectPoseEvidence* accepted = nullptr;
            for (const auto& entry : set->entries) {
                if (entry.object != decl->landmark ||
                    entry.frame != FrameId{"odometry"}) {
                    continue;   // someone else's evidence or a foreign frame
                }
                if (entry.measuredAt < ts.activatedAt) {
                    continue;   // evidence older than activation never acquires
                }
                if ((in.now - entry.measuredAt) >
                    decl->vision.maximum_observation_age_ms) {
                    continue;
                }
                if (!decl->vision.preferred_camera.empty() &&
                    entry.source != decl->vision.preferred_camera) {
                    continue;
                }
                if (!decl->vision.allowed_mounts.empty()) {
                    bool allowed = false;
                    for (const auto& instance : decl->vision.allowed_mounts) {
                        if (instance == entry.feature_instance) {
                            allowed = true;
                            break;
                        }
                    }
                    if (!allowed) {
                        continue;
                    }
                }
                // Motion is gated at the moment of exposure, not at
                // processing time: with camera latency those differ.
                auto rate_at_exposure = std::fabs(in.robot.yaw_rate_rad_s);
                double sampled        = 0.0;
                if (in.robot.yawRateAt(entry.measuredAt, sampled)) {
                    rate_at_exposure = std::fabs(sampled);
                }
                if (rate_at_exposure >
                    decl->vision.maximum_robot_angular_speed_rad_s) {
                    continue;
                }
                if (acquisition_.have_frame &&
                    entry.source == acquisition_.last_frame_camera &&
                    entry.source_sequence == acquisition_.last_frame_sequence) {
                    continue;   // this frame already contributed
                }
                if (accepted == nullptr || entry.confidence > accepted->confidence) {
                    accepted = &entry;
                }
            }

            if (accepted != nullptr) {
                acquisition_.have_frame          = true;
                acquisition_.last_frame_camera   = accepted->source;
                acquisition_.last_frame_sequence = accepted->source_sequence;

                const auto candidate =
                    decl->resolveFromLandmark(accepted->T_frame_object);
                if (!acquisition_.target_candidates.empty()) {
                    const auto& prev = acquisition_.target_candidates.back();
                    const auto  dt =
                        std::hypot(candidate.x_m - prev.x_m, candidate.y_m - prev.y_m);
                    const auto dh =
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
                acquisition_.landmark_poses.push_back(accepted->T_frame_object);
                acquisition_.confidences.push_back(accepted->confidence);

                if (static_cast<long>(acquisition_.target_candidates.size()) >=
                    decl->vision.minimum_consistent_observations) {
                    // Latch the consistent window's mean landmark pose,
                    // resolved through the configured chain, and close the
                    // gate. The world estimate is a separate product with
                    // its own explicit commit policy.
                    double lx = 0.0, ly = 0.0, lh_sin = 0.0, lh_cos = 0.0;
                    double conf = 0.0;
                    for (std::size_t i = 0; i < acquisition_.landmark_poses.size();
                         ++i) {
                        const auto& l = acquisition_.landmark_poses[i];
                        lx += l.x_m;
                        ly += l.y_m;
                        lh_sin += std::sin(l.heading_rad);
                        lh_cos += std::cos(l.heading_rad);
                        conf += acquisition_.confidences[i];
                    }
                    const auto n =
                        static_cast<double>(acquisition_.landmark_poses.size());
                    const Pose2D landmark_odom{lx / n, ly / n,
                                               std::atan2(lh_sin, lh_cos)};
                    ts.T_odom_robot_target = decl->resolveFromLandmark(landmark_odom);
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
