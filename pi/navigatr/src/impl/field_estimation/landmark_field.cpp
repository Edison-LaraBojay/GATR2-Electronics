// landmark_field.cpp

#include "impl/field_estimation/landmark_field.h"

#include <cmath>
#include <map>
#include <typeindex>

#include "math/angles.h"
#include "payloads/field_object_evidence.h"
#include "resources/resource_store.h"

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

std::unique_ptr<FieldEstimation> LandmarkFieldEstimation::create(
    const ConfigNode& node, SlotInitializationContext& context, std::string& err) {
    auto composite = std::make_unique<LandmarkFieldEstimation>();

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
    } else {
        err = estimator.path() + ": commit must be always or never, not \"" + commit +
              "\"";
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
    const std::type_index evidence_type(typeid(FieldObjectPoseEvidenceSet));
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

std::vector<ObservationOutputDecl> LandmarkFieldEstimation::producesObservations() const {
    return observation_extraction_->produces();
}

std::vector<AssociationOutputDecl> LandmarkFieldEstimation::producesAssociations() const {
    return association_->produces();
}

FieldEstimationOutput LandmarkFieldEstimation::run(const FieldEstimationInput& in) {
    FieldEstimationOutput out;
    out.field = in.previousField;

    // Children in fixed order. A child fault leaves the previous field
    // state untouched and publishes nothing: no partial commit.
    auto per_out = observation_extraction_->run({in.sensors, in.now});
    if (per_out.status == FunctionStatus::kFault) {
        out.status     = FunctionStatus::kFault;
        out.diagnostic = per_out.diagnostic;
        return out;
    }

    auto assoc_out = association_->run(
        {per_out.observations, in.robot, in.history, in.previousField, in.now});
    if (assoc_out.status == FunctionStatus::kFault) {
        out.status     = FunctionStatus::kFault;
        out.diagnostic = assoc_out.diagnostic;
        return out;
    }

    // Estimator: seed every mapped landmark, keep observed entries
    // coherent with the current coordinate context, then fold accepted
    // evidence per the explicit commit policy.
    for (const auto& decl : field_->landmarks) {
        if (out.field.objects.find(decl.id) != out.field.objects.end()) {
            continue;
        }
        FieldObjectState obj;
        obj.pose.frame      = FrameId{"field"};
        obj.pose.pose       = decl.nominal;
        obj.pose.measuredAt = in.now;
        obj.confidence      = 0.5;
        obj.valid           = true;
        obj.source          = EstimateSource::kFieldMap;
        out.field.objects.emplace(decl.id, obj);
    }
    for (auto& kv : out.field.objects) {
        FieldObjectState& obj = kv.second;
        obj.observed          = false;
        if (obj.source != EstimateSource::kObserved) {
            continue;
        }
        if (obj.odometry_epoch != in.robot.odometry_epoch) {
            // the frame the estimate was measured in no longer exists: back
            // to the nominal definition, never a silently stale pose
            const LandmarkDecl* decl = field_->find(kv.first);
            obj.source               = EstimateSource::kFieldMap;
            obj.confidence           = 0.5;
            obj.valid                = decl != nullptr;
            if (decl != nullptr) {
                obj.pose.pose = decl->nominal;
            }
            obj.pose.measuredAt = in.now;
            continue;
        }
        if (obj.anchor_revision != in.robot.anchor_revision) {
            // a re-anchor re-expresses the same measurement consistently
            obj.pose.pose       = compose(in.robot.field_from_odom, obj.T_odom_object);
            obj.anchor_revision = in.robot.anchor_revision;
        }
    }

    const FieldObjectPoseEvidenceSet* evidence = nullptr;
    if (!evidence_ref_.empty()) {
        const auto it = assoc_out.associations.find(evidence_ref_);
        if (it != assoc_out.associations.end()) {
            evidence = it->second.payload.get<FieldObjectPoseEvidenceSet>();
            if (evidence == nullptr) {
                out.status = FunctionStatus::kFault;
                out.field  = in.previousField;
                return out;
            }
        }
    }

    if (commit_ == CommitPolicy::kAlways && evidence != nullptr) {
        // Deterministic fusion in the odometry frame: several accepted
        // measurements of one object in one invocation combine by
        // confidence-weighted planar mean and circular heading mean, so
        // container or detector iteration order can never pick the result.
        // Non-finite evidence and evidence from another odometry epoch never
        // reach state.
        struct Accumulated {
            double        wx = 0.0, wy = 0.0, wsin = 0.0, wcos = 0.0;
            double        weight = 0.0, wconf = 0.0;
            MonotonicTime newest;
            std::string   source, feature;
            uint32_t      sequence = 0;
        };
        std::map<FieldObjectId, Accumulated> merged;
        for (const auto& entry : evidence->entries) {
            if (entry.frame != FrameId{"odometry"}) {
                continue;   // this estimator folds odometry-frame evidence
            }
            if (entry.odometry_epoch != in.robot.odometry_epoch) {
                continue;
            }
            const auto& p = entry.T_frame_object;
            if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) ||
                !std::isfinite(p.heading_rad) || !std::isfinite(entry.confidence) ||
                entry.confidence < 0.0) {
                continue;
            }
            const auto w = std::max(entry.confidence, 1e-6);
            auto&      a = merged[entry.object];
            a.wx += w * p.x_m;
            a.wy += w * p.y_m;
            a.wsin += w * std::sin(p.heading_rad);
            a.wcos += w * std::cos(p.heading_rad);
            a.weight += w;
            a.wconf += w * entry.confidence;
            if (!a.newest.isSet() || entry.measuredAt > a.newest) {
                a.newest   = entry.measuredAt;
                a.source   = entry.source.value;
                a.feature  = entry.feature_instance;
                a.sequence = entry.source_sequence;
            }
        }
        for (const auto& kv : merged) {
            const auto&  a = kv.second;
            const Pose2D fused{a.wx / a.weight, a.wy / a.weight,
                               std::atan2(a.wsin, a.wcos)};
            auto& obj = out.field.objects[kv.first];
            if (!obj.valid) {
                obj.T_odom_object = fused;
            } else {
                if (obj.source != EstimateSource::kObserved) {
                    // first evidence blends away from the seeded nominal,
                    // expressed in the same frame as the measurement
                    obj.T_odom_object = compose(inverse(in.robot.field_from_odom), obj.pose.pose);
                }
                obj.T_odom_object.x_m += (fused.x_m - obj.T_odom_object.x_m) * blend_;
                obj.T_odom_object.y_m += (fused.y_m - obj.T_odom_object.y_m) * blend_;
                obj.T_odom_object.heading_rad = wrapAngle(
                    obj.T_odom_object.heading_rad +
                    wrapAngle(fused.heading_rad - obj.T_odom_object.heading_rad) * blend_);
            }
            obj.pose.frame           = FrameId{"field"};
            obj.pose.pose            = compose(in.robot.field_from_odom, obj.T_odom_object);
            obj.pose.measuredAt      = a.newest;
            obj.lastObservedAt       = a.newest;
            obj.valid                = true;
            obj.observed             = true;
            obj.source               = EstimateSource::kObserved;
            obj.confidence           = a.wconf / a.weight;
            obj.odometry_epoch       = in.robot.odometry_epoch;
            obj.anchor_revision      = in.robot.anchor_revision;
            obj.last_source          = a.source;
            obj.last_feature         = a.feature;
            obj.last_source_sequence = a.sequence;
        }
    }

    out.observations = std::move(per_out.observations);
    out.associations = std::move(assoc_out.associations);
    return out;
}

} // namespace navigatr
