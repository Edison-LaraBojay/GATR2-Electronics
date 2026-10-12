"""e5: apriltag sub-steps built by make_tag_extraction, make_tag_association,
make_landmark_fold and held by the estimator under extract_tags_, associate_tags_,
fold_landmarks_."""
import os
import sys

root = sys.argv[1]   # .../src/pipeline/world_estimation


def edit(rel, pairs):
    p = os.path.join(root, rel)
    s = open(p, newline='').read()
    for a, b in pairs:
        assert s.count(a) == 1, (rel, a)
        s = s.replace(a, b)
    open(p, 'w', newline='').write(s)


# ---- extraction ------------------------------------------------------------
edit('estimators/apriltag/apriltag_tag_observation.h', [
    ('''#pragma once
#include <memory>
#include <string>

#include "pipeline/resources/tag_detector.h"''', '''#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "pipeline/framework/execution_context.h"
#include "pipeline/resources/tag_detector.h"'''),
    ('''    bool     has_processed_           = false;
    uint64_t last_processed_epoch_    = 0;
    uint32_t last_processed_sequence_ = 0;
};
''', '''    bool     has_processed_           = false;
    uint64_t last_processed_epoch_    = 0;
    uint32_t last_processed_sequence_ = 0;
};

using TagExtractionStep = std::function<PerceptionOutput(const PerceptionInput&)>;

struct TagExtractionExecutable {
    TagExtractionStep                  run;
    ResetFunction                      reset;
    std::vector<ObservationOutputDecl> outputs;
};

// extract_tags: node is the <ObservationExtraction> element. Nothing on
// failure.
std::optional<TagExtractionExecutable> make_tag_extraction(const ConfigNode&          node,
                                                           SlotInitializationContext& context,
                                                           std::string&               err);
'''),
])

edit('estimators/apriltag/apriltag_tag_observation.cpp', [
    ('''std::vector<ObservationOutputDecl> AprilTagObservationPerception::produces() const {''',
     '''std::optional<TagExtractionExecutable> make_tag_extraction(const ConfigNode&          node,
                                                           SlotInitializationContext& context,
                                                           std::string&               err) {
    std::shared_ptr<Perception> extraction =
        AprilTagObservationPerception::create(node, context, err);
    if (extraction == nullptr) {
        return std::nullopt;
    }
    TagExtractionExecutable extract_tags;
    extract_tags.run     = [extraction](const PerceptionInput& in) { return extraction->run(in); };
    extract_tags.reset   = [extraction] { extraction->reset(); };
    extract_tags.outputs = extraction->produces();
    return extract_tags;
}

std::vector<ObservationOutputDecl> AprilTagObservationPerception::produces() const {'''),
])

# ---- association -----------------------------------------------------------
edit('estimators/apriltag/tag_mount_association.h', [
    ('''#pragma once
#include <memory>
#include <string>

#include "config/field_map.h"''', '''#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config/field_map.h"
#include "pipeline/framework/execution_context.h"'''),
    ('''    double max_reprojection_error_px_    = 0.0;   // 0 disables
    double max_alternate_pose_ambiguity_ = 0.0;   // 0 disables
};
''', '''    double max_reprojection_error_px_    = 0.0;   // 0 disables
    double max_alternate_pose_ambiguity_ = 0.0;   // 0 disables
};

using TagAssociationStep = std::function<AssociationOutput(const AssociationInput&)>;

struct TagAssociationExecutable {
    TagAssociationStep                 run;
    ResetFunction                      reset;
    std::vector<AssociationOutputDecl> outputs;
};

// associate_tags: node is the <Association> element. Nothing on failure.
std::optional<TagAssociationExecutable> make_tag_association(const ConfigNode&          node,
                                                             SlotInitializationContext& context,
                                                             std::string&               err);
'''),
])

edit('estimators/apriltag/tag_mount_association.cpp', [
    ('''std::vector<AssociationOutputDecl> TagMountAssociation::produces() const {''',
     '''std::optional<TagAssociationExecutable> make_tag_association(const ConfigNode&          node,
                                                             SlotInitializationContext& context,
                                                             std::string&               err) {
    std::shared_ptr<Association> association = TagMountAssociation::create(node, context, err);
    if (association == nullptr) {
        return std::nullopt;
    }
    TagAssociationExecutable associate_tags;
    associate_tags.run   = [association](const AssociationInput& in) {
        return association->run(in);
    };
    associate_tags.reset   = [association] { association->reset(); };
    associate_tags.outputs = association->produces();
    return associate_tags;
}

std::vector<AssociationOutputDecl> TagMountAssociation::produces() const {'''),
])

# ---- landmark fold (generic) -----------------------------------------------
edit('landmark_estimator.h', [
    ('''// landmark_estimator.h
// The landmark estimation step of the AprilTag world estimator: accepted
// evidence in, committed landmark estimates out. It seeds every mapped''',
     '''// landmark_estimator.h
// The generic landmark fold of a world estimator: accepted evidence in,
// committed landmark estimates out. It seeds every mapped'''),
    ('''#pragma once
#include <memory>
#include <optional>
#include <string>
''', '''#pragma once
#include <functional>
#include <memory>
#include <optional>
#include <string>
'''),
    ('''    std::shared_ptr<const FieldMap> field_;
    CommitPolicy                    commit_ = CommitPolicy::kAlways;
    double                          blend_  = 1.0;
};
''', '''    std::shared_ptr<const FieldMap> field_;
    CommitPolicy                    commit_ = CommitPolicy::kAlways;
    double                          blend_  = 1.0;
};

// evidence is null when nothing was accepted this invocation
using LandmarkFoldStep = std::function<FieldState(
    const FieldState& previous, const FieldObjectPoseEvidenceSet* evidence,
    const RobotState& robot, MonotonicTime now)>;

struct LandmarkFoldExecutable {
    LandmarkFoldStep run;
    bool             commits = true;   // accepted evidence moves the estimates
};

// fold_landmarks: node is the <LandmarkEstimation> element. Nothing on
// failure.
std::optional<LandmarkFoldExecutable> make_landmark_fold(const ConfigNode&               node,
                                                         std::shared_ptr<const FieldMap> field,
                                                         std::string&                    err);
'''),
])

edit('landmark_estimator.cpp', [
    ('''FieldState LandmarkEstimator::run(const FieldState&                 previous,''',
     '''std::optional<LandmarkFoldExecutable> make_landmark_fold(const ConfigNode&               node,
                                                         std::shared_ptr<const FieldMap> field,
                                                         std::string&                    err) {
    std::optional<LandmarkEstimator> landmarks =
        LandmarkEstimator::create(node, std::move(field), err);
    if (!landmarks.has_value()) {
        return std::nullopt;
    }
    LandmarkFoldExecutable fold_landmarks;
    fold_landmarks.run = [estimator = *landmarks](const FieldState&                 previous,
                                                  const FieldObjectPoseEvidenceSet* evidence,
                                                  const RobotState& robot, MonotonicTime now) {
        return estimator.run(previous, evidence, robot, now);
    };
    fold_landmarks.commits = landmarks->commits();
    return fold_landmarks;
}

FieldState LandmarkEstimator::run(const FieldState&                 previous,'''),
])

# ---- the estimator ---------------------------------------------------------
edit('estimators/apriltag/apriltag_world_estimator.h', [
    ('''// World estimation from AprilTag landmarks: one selectable implementation
// of the FieldEstimation contract that privately owns a fixed serial
// subpipeline over one camera:
//
//   ObservationExtraction  camera frames -> canonical tag observations
//                          (AprilTagObservationPerception, Perception contract)
//   Association            observations -> decisive landmark pose evidence
//                          (TagMountAssociation, Association contract), optional
//   LandmarkEstimation     evidence -> committed landmark estimates
//                          (LandmarkEstimator)
//''', '''// World estimation from AprilTag landmarks: one selectable implementation
// of the FieldEstimation contract that privately owns a fixed serial
// subpipeline over one camera, each step built by its make_* and called by
// its verb:
//
//   extract_tags    <ObservationExtraction>  camera frames -> canonical tag
//                   observations (make_tag_extraction, Perception contract)
//   associate_tags  <Association>  observations -> decisive landmark pose
//                   evidence (make_tag_association, Association contract),
//                   optional
//   fold_landmarks  <LandmarkEstimation>  evidence -> committed landmark
//                   estimates (make_landmark_fold)
//'''),
    ('''#pragma once
#include <memory>
#include <string>

#include "pipeline/world_estimation/association.h"
#include "pipeline/world_estimation/contract.h"
#include "pipeline/world_estimation/landmark_estimator.h"
#include "pipeline/world_estimation/perception.h"
''', '''#pragma once
#include <memory>
#include <optional>
#include <string>

#include "pipeline/world_estimation/association.h"
#include "pipeline/world_estimation/contract.h"
#include "pipeline/world_estimation/estimators/apriltag/apriltag_tag_observation.h"
#include "pipeline/world_estimation/estimators/apriltag/tag_mount_association.h"
#include "pipeline/world_estimation/landmark_estimator.h"
#include "pipeline/world_estimation/perception.h"
'''),
    ('''private:
    std::unique_ptr<Perception>      observation_extraction_;
    std::unique_ptr<Association>     association_;   // null when not configured
    std::optional<LandmarkEstimator> landmarks_;
''', '''private:
    TagExtractionExecutable                 extract_tags_;
    std::optional<TagAssociationExecutable> associate_tags_;   // empty when not configured
    LandmarkFoldExecutable                  fold_landmarks_;
'''),
])

edit('estimators/apriltag/apriltag_world_estimator.cpp', [
    ('''#include "pipeline/resources/resource_store.h"
#include "pipeline/world_estimation/estimators/apriltag/apriltag_tag_observation.h"
#include "pipeline/world_estimation/estimators/apriltag/tag_mount_association.h"
#include "pipeline/world_estimation/evidence.h"
''', '''#include "pipeline/resources/resource_store.h"
#include "pipeline/world_estimation/evidence.h"
'''),
    ('''    estimator->observation_extraction_ =
        AprilTagObservationPerception::create(extraction, step_context, err);
    if (estimator->observation_extraction_ == nullptr) {
        return nullptr;
    }
    step_context.observations = estimator->observation_extraction_->produces();
''', '''    std::optional<TagExtractionExecutable> extract_tags =
        make_tag_extraction(extraction, step_context, err);
    if (!extract_tags.has_value()) {
        return nullptr;
    }
    estimator->extract_tags_  = std::move(*extract_tags);
    step_context.observations = estimator->extract_tags_.outputs;
'''),
    ('''    if (association.valid()) {
        estimator->association_ = TagMountAssociation::create(association, step_context, err);
        if (estimator->association_ == nullptr) {
            return nullptr;
        }
        association_decls = estimator->association_->produces();
    }
''', '''    if (association.valid()) {
        estimator->associate_tags_ = make_tag_association(association, step_context, err);
        if (!estimator->associate_tags_.has_value()) {
            return nullptr;
        }
        association_decls = estimator->associate_tags_->outputs;
    }
'''),
    ('''    estimator->landmarks_ = LandmarkEstimator::create(landmarks, field, err);
    if (!estimator->landmarks_.has_value()) {
        return nullptr;
    }
''', '''    std::optional<LandmarkFoldExecutable> fold_landmarks =
        make_landmark_fold(landmarks, field, err);
    if (!fold_landmarks.has_value()) {
        return nullptr;
    }
    estimator->fold_landmarks_ = std::move(*fold_landmarks);
'''),
    ('''    if (estimator->evidence_ref_.empty() && estimator->landmarks_->commits()) {''',
     '''    if (estimator->evidence_ref_.empty() && estimator->fold_landmarks_.commits) {'''),
    ('''std::vector<ObservationOutputDecl> AprilTagWorldEstimator::producesObservations() const {
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
''', '''std::vector<ObservationOutputDecl> AprilTagWorldEstimator::producesObservations() const {
    return extract_tags_.outputs;
}

std::vector<AssociationOutputDecl> AprilTagWorldEstimator::producesAssociations() const {
    return associate_tags_.has_value() ? associate_tags_->outputs
                                       : std::vector<AssociationOutputDecl>{};
}

void AprilTagWorldEstimator::reset() {
    extract_tags_.reset();
    if (associate_tags_.has_value()) {
        associate_tags_->reset();
    }
}
'''),
    ('''    PerceptionOutput per_out = observation_extraction_->run({in.sensors, now});''',
     '''    PerceptionOutput per_out = extract_tags_.run({in.sensors, now});'''),
    ('''    if (association_ != nullptr) {
        assoc_out = association_->run(
            {per_out.observations, in.robot, in.history, in.previousField, now});''',
     '''    if (associate_tags_.has_value()) {
        assoc_out = associate_tags_->run(
            {per_out.observations, in.robot, in.history, in.previousField, now});'''),
    ('''    out.field        = landmarks_->run(in.previousField, evidence, in.robot, now);''',
     '''    out.field        = fold_landmarks_.run(in.previousField, evidence, in.robot, now);'''),
])
print('ok')
