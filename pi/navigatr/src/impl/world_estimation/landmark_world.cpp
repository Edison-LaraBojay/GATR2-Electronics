// landmark_world.cpp

#include "impl/world_estimation/landmark_world.h"

#include <cmath>
#include <typeindex>

#include "math/angles.h"
#include "payloads/landmark_pose_observations.h"
#include "resources/resource_map.h"

namespace navigatr
{

namespace
{

// Strict nested schema: the composite owns and validates its own children.
bool onlyChildren(const ConfigNode& parent, std::initializer_list<const char*> allowed,
                  std::string& err) {
    for (auto c = parent.child(); c.valid(); c = c.next()) {
        bool known = false;
        for (const char* name : allowed) {
            if (std::string(c.name()) == name) {
                known = true;
                break;
            }
        }
        if (!known) {
            err = parent.path() + " has unknown element " + c.name();
            return false;
        }
    }
    return true;
}

bool exactlyOne(const ConfigNode& parent, const char* name, ConfigNode& out,
                std::string& err) {
    out = parent.child(name);
    if (!out.valid()) {
        err = parent.path() + " needs exactly one " + name;
        return false;
    }
    if (out.next(name).valid()) {
        err = parent.path() + " has more than one " + name;
        return false;
    }
    return true;
}

} // namespace

std::unique_ptr<WorldEstimation> LandmarkWorldEstimation::create(
    const ConfigNode& node, SlotInitializationContext& context, std::string& err) {
    auto composite = std::make_unique<LandmarkWorldEstimation>();

    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    if (!onlyChildren(node, {"FieldMap", "Pipeline"}, err)) {
        return nullptr;
    }

    const ResourceId field_id{node.child("FieldMap").attr("resource_id")};
    if (field_id.empty()) {
        err = node.path() + ": needs <FieldMap resource_id=.../>";
        return nullptr;
    }
    std::string inner;
    composite->field_ = context.resources->require<const FieldMap>(field_id, inner);
    if (composite->field_ == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    ConfigNode pipeline;
    if (!exactlyOne(node, "Pipeline", pipeline, err)) {
        return nullptr;
    }
    if (!onlyChildren(pipeline, {"ObservationExtraction", "Association", "Estimator"},
                      err)) {
        return nullptr;
    }

    // Children build against a private copy of the slot context so their
    // intermediate declarations never leak upstream; the parent republishes
    // what crosses the boundary through produces*().
    SlotInitializationContext child_context = context;

    const auto buildChild = [&](const char* name, auto tag, auto& target) {
        using MakeFunction = typename decltype(tag)::type;
        ConfigNode child;
        if (!exactlyOne(pipeline, name, child, err)) {
            return false;
        }
        const FunctionKey type{child.attr("type")};
        if (type.empty()) {
            err = child.path() + ": needs an explicit type";
            return false;
        }
        const auto* factory = child_context.functions->find<MakeFunction>(type, err);
        if (factory == nullptr) {
            err = child.path() + ": " + err;
            return false;
        }
        target = (*factory)(child, child_context, err);
        return target != nullptr;
    };

    struct PerceptionTag {
        using type = PerceptionMakeFunction;
    };
    struct AssociationTag {
        using type = AssociationMakeFunction;
    };
    if (!buildChild("ObservationExtraction", PerceptionTag{},
                    composite->observation_extraction_)) {
        return nullptr;
    }
    child_context.observations = composite->observation_extraction_->produces();

    if (!buildChild("Association", AssociationTag{}, composite->association_)) {
        return nullptr;
    }
    const auto association_decls = composite->association_->produces();

    ConfigNode estimator;
    if (!exactlyOne(pipeline, "Estimator", estimator, err)) {
        return nullptr;
    }
    std::string estimator_type;
    if (!estimator.requireAttr("type", estimator_type, err)) {
        return nullptr;
    }
    if (estimator_type != "landmark_estimator") {
        err = estimator.path() + ": Estimator type must be landmark_estimator, not \"" +
              estimator_type + "\"";
        return nullptr;
    }
    std::string commit;
    if (!estimator.requireAttr("commit", commit, err)) {
        return nullptr;
    }
    if (commit == "always") {
        composite->commit_ = CommitPolicy::kAlways;
    } else if (commit == "never") {
        composite->commit_ = CommitPolicy::kNever;
    } else if (commit == "on_target_lock") {
        composite->commit_ = CommitPolicy::kOnTargetLock;
    } else {
        err = estimator.path() + ": commit must be always, never, or on_target_lock, "
              "not \"" + commit + "\"";
        return nullptr;
    }
    if (!estimator.getDouble("blend", 1.0, composite->blend_, err)) {
        return nullptr;
    }
    if (composite->blend_ < 0.0 || composite->blend_ > 1.0) {
        err = estimator.path() + ": blend must be within 0..1";
        return nullptr;
    }

    // The estimator folds the association child's published evidence. One
    // declared landmark-pose output is the supported shape; none is legal
    // only when the estimator never commits.
    const std::type_index evidence_type(typeid(LandmarkPoseObservationSet));
    for (const auto& decl : association_decls) {
        if (!decl.payload.matches(evidence_type)) {
            continue;
        }
        if (!composite->evidence_ref_.empty()) {
            err = node.path() + ": more than one landmark evidence output is not "
                  "supported yet";
            return nullptr;
        }
        composite->evidence_ref_ = decl.id;
    }
    if (composite->evidence_ref_.empty() &&
        composite->commit_ != CommitPolicy::kNever) {
        err = estimator.path() + ": the commit policy folds evidence but the "
              "Association child publishes none";
        return nullptr;
    }
    return composite;
}

std::vector<ObservationOutputDecl> LandmarkWorldEstimation::producesObservations() const {
    return observation_extraction_->produces();
}

std::vector<AssociationOutputDecl> LandmarkWorldEstimation::producesAssociations() const {
    return association_->produces();
}

WorldEstimationOutput LandmarkWorldEstimation::run(const WorldEstimationInput& in) {
    WorldEstimationOutput out;
    out.world = in.previousWorld;

    // Children in fixed order. A child fault leaves the previous world
    // untouched and publishes nothing: no partial commit.
    auto per_out = observation_extraction_->run(
        {in.sensorResults, in.artifacts, in.previousTarget, in.now});
    if (per_out.status == FunctionStatus::kFault) {
        out.status = FunctionStatus::kFault;
        return out;
    }

    auto assoc_out = association_->run({per_out.observations, in.robot, in.previousWorld,
                                        in.command, in.previousTarget, in.now});
    if (assoc_out.status == FunctionStatus::kFault) {
        out.status = FunctionStatus::kFault;
        return out;
    }

    // Estimator: seed every mapped landmark, then fold accepted evidence
    // per the explicit commit policy.
    for (const auto& decl : field_->landmarks) {
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
    for (auto& kv : out.world.objects) {
        kv.second.observed = false;
    }

    const auto fold = [&](const LandmarkPoseObservation& entry) {
        auto& obj = out.world.objects[entry.landmark];
        const auto T_field_landmark =
            compose(in.robot.field_from_odom, entry.T_odom_landmark);
        if (!obj.valid) {
            obj.pose.frame = FrameId{"field"};
            obj.pose.pose  = T_field_landmark;
        } else {
            obj.pose.pose.x_m += (T_field_landmark.x_m - obj.pose.pose.x_m) * blend_;
            obj.pose.pose.y_m += (T_field_landmark.y_m - obj.pose.pose.y_m) * blend_;
            obj.pose.pose.heading_rad = wrapAngle(
                obj.pose.pose.heading_rad +
                wrapAngle(T_field_landmark.heading_rad - obj.pose.pose.heading_rad) *
                    blend_);
        }
        obj.pose.measuredAt = in.now;
        obj.valid           = true;
        obj.observed        = true;
        obj.source          = EstimateSource::kObserved;
        obj.confidence      = entry.confidence;
    };

    const LandmarkPoseObservationSet* evidence = nullptr;
    if (!evidence_ref_.empty()) {
        const auto it = assoc_out.associations.find(evidence_ref_);
        if (it != assoc_out.associations.end()) {
            evidence = it->second.payload.get<LandmarkPoseObservationSet>();
            if (evidence == nullptr) {
                out.status = FunctionStatus::kFault;
                out.world  = in.previousWorld;
                return out;
            }
        }
    }

    if (commit_ == CommitPolicy::kAlways && evidence != nullptr) {
        for (const auto& entry : evidence->entries) {
            fold(entry);
        }
    } else if (commit_ == CommitPolicy::kOnTargetLock) {
        // One acceptance path: fold exactly what the resolver locked, once
        // per generation. Nominal fallbacks and cancellations carry no
        // locked landmark, so they leave zero camera trace.
        const auto& t = in.previousTarget;
        if (t.active && t.status == TargetStatus::kLockedVision &&
            t.has_locked_landmark && t.generation != last_committed_generation_) {
            LandmarkPoseObservation locked;
            locked.landmark        = t.locked_landmark;
            locked.T_odom_landmark = t.T_odom_landmark_locked;
            locked.confidence      = t.locked_confidence;
            fold(locked);
            last_committed_generation_ = t.generation;
        }
    }

    out.observations = std::move(per_out.observations);
    out.associations = std::move(assoc_out.associations);
    return out;
}

} // namespace navigatr
