// apriltag_world_estimator.cpp

#include "impl/world_estimation/apriltag_world_estimator.h"

#include <typeindex>

#include "impl/world_estimation/apriltag_tag_observation.h"
#include "impl/world_estimation/tag_mount_association.h"
#include "payloads/field_object_evidence.h"
#include "resources/resource_store.h"

namespace navigatr
{

namespace
{

// Strict schema: the implementation owns and validates its own subtree.
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

bool atMostOne(const ConfigNode& parent, const char* name, ConfigNode& out,
               std::string& err) {
    out = parent.child(name);
    if (out.valid() && out.next(name).valid()) {
        err = parent.path() + " has more than one " + name;
        return false;
    }
    return true;
}

bool exactlyOne(const ConfigNode& parent, const char* name, ConfigNode& out,
                std::string& err) {
    if (!atMostOne(parent, name, out, err)) {
        return false;
    }
    if (!out.valid()) {
        err = parent.path() + " needs exactly one " + name;
        return false;
    }
    return true;
}

} // namespace

std::unique_ptr<FieldEstimation> AprilTagWorldEstimator::create(
    const ConfigNode& node, SlotInitializationContext& context, std::string& err) {
    auto estimator = std::make_unique<AprilTagWorldEstimator>();

    if (context.resources == nullptr || context.sensors == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }
    if (!onlyChildren(node, {"FieldMap", "ObservationExtraction", "Association",
                             "LandmarkEstimation"}, err)) {
        return nullptr;
    }

    const ResourceId field_id{node.child("FieldMap").attr("resource_id")};
    if (field_id.empty()) {
        err = node.path() + ": needs <FieldMap resource_id=.../>";
        return nullptr;
    }
    std::string inner;
    std::shared_ptr<const FieldMap> field =
        context.resources->require<const FieldMap>(field_id, inner);
    if (field == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    // The steps build against a private copy of the initialization context
    // so their intermediate declarations never leak upstream; the estimator
    // republishes what crosses the boundary through produces*().
    SlotInitializationContext step_context = context;

    ConfigNode extraction;
    if (!exactlyOne(node, "ObservationExtraction", extraction, err)) {
        return nullptr;
    }
    estimator->observation_extraction_ =
        AprilTagObservationPerception::create(extraction, step_context, err);
    if (estimator->observation_extraction_ == nullptr) {
        return nullptr;
    }
    step_context.observations = estimator->observation_extraction_->produces();

    ConfigNode association;
    if (!atMostOne(node, "Association", association, err)) {
        return nullptr;
    }
    std::vector<AssociationOutputDecl> association_decls;
    if (association.valid()) {
        estimator->association_ = TagMountAssociation::create(association, step_context, err);
        if (estimator->association_ == nullptr) {
            return nullptr;
        }
        association_decls = estimator->association_->produces();
    }

    ConfigNode landmarks;
    if (!exactlyOne(node, "LandmarkEstimation", landmarks, err)) {
        return nullptr;
    }
    estimator->landmarks_ = LandmarkEstimator::create(landmarks, field, err);
    if (!estimator->landmarks_.has_value()) {
        return nullptr;
    }

    // The landmark estimator folds the association's published evidence.
    // One declared landmark-pose output is the supported shape; none is
    // legal only when the estimator never commits.
    const std::type_index evidence_type(typeid(FieldObjectPoseEvidenceSet));
    for (const auto& decl : association_decls) {
        if (!decl.payload.matches(evidence_type)) {
            continue;
        }
        if (!estimator->evidence_ref_.empty()) {
            err = node.path() + ": more than one landmark evidence output is not "
                  "supported yet";
            return nullptr;
        }
        estimator->evidence_ref_ = decl.id;
    }
    if (estimator->evidence_ref_.empty() && estimator->landmarks_->commits()) {
        err = landmarks.path() + ": the commit policy folds evidence but no Association "
              "publishes any; configure an Association or commit=\"never\"";
        return nullptr;
    }
    return estimator;
}

std::vector<ObservationOutputDecl> AprilTagWorldEstimator::producesObservations() const {
    return observation_extraction_->produces();
}

std::vector<AssociationOutputDecl> AprilTagWorldEstimator::producesAssociations() const {
    return association_ == nullptr ? std::vector<AssociationOutputDecl>{}
                                   : association_->produces();
}

void AprilTagWorldEstimator::reset() {
    observation_extraction_->reset();
    if (association_ != nullptr) {
        association_->reset();
    }
}

FieldEstimationOutput AprilTagWorldEstimator::run(const FieldEstimationInput& in) {
    FieldEstimationOutput out;
    out.field = in.previousField;
    const MonotonicTime now = in.context.now;

    // Steps in fixed order. A step fault leaves the previous field state
    // untouched and publishes nothing: no partial commit.
    PerceptionOutput per_out = observation_extraction_->run({in.sensors, now});
    if (per_out.status == FunctionStatus::kFault) {
        out.status     = FunctionStatus::kFault;
        out.diagnostic = per_out.diagnostic;
        return out;
    }

    AssociationOutput assoc_out;
    if (association_ != nullptr) {
        assoc_out = association_->run(
            {per_out.observations, in.robot, in.history, in.previousField, now});
        if (assoc_out.status == FunctionStatus::kFault) {
            out.status     = FunctionStatus::kFault;
            out.diagnostic = assoc_out.diagnostic;
            return out;
        }
    }

    const FieldObjectPoseEvidenceSet* evidence = nullptr;
    if (!evidence_ref_.empty()) {
        const auto it = assoc_out.associations.find(evidence_ref_);
        if (it != assoc_out.associations.end()) {
            evidence = it->second.payload.get<FieldObjectPoseEvidenceSet>();
            if (evidence == nullptr) {
                out.status = FunctionStatus::kFault;
                return out;
            }
        }
    }

    out.field        = landmarks_->run(in.previousField, evidence, in.robot, now);
    out.observations = std::move(per_out.observations);
    out.associations = std::move(assoc_out.associations);
    return out;
}

} // namespace navigatr
