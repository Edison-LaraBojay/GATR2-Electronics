// tag_mount_association.h
// Association: decides which physical tag mount produced each canonical tag
// observation, by full pose against every mount sharing the observed id and
// family, never by id alone and never forced by expectation. The winner
// must both pass the geometric gates and beat the runner-up by the
// ambiguity margin; anything less abstains.
//
// Processing is gated by the active target: evidence exists to acquire the
// selected landmark-derived target, so with no target pending acquisition
// no association work runs and no correction can commit.
//
// Acceptance requires, in order: a valid robot pose and non-future exposure
// inside retained history; detector quality (hamming, decision margin, and
// when the detector reports them, reprojection error and alternate-pose
// ambiguity); physical plausibility (tag in front of the camera, facing it,
// within range); the translation and heading gates against the prior; and a
// decisive combined-score margin over the runner-up.
//
//   <Association type="tag_mount_association">
//       <Observations observation_id="tag_observations"/>
//       <FieldMap resource_id="game_field"/>
//       <RobotFrames resource_id="robot_geometry"/>
//       <Targets resource_id="targets"/>
//       <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
//              ambiguity_margin_m="0.15" max_range_m="3.0"
//              min_decision_margin="20" max_hamming="0"
//              min_facing_cos="0.1" min_projected_size_px="8"
//              max_reprojection_error_px="0" max_alternate_pose_ambiguity="0"/>
//       <Output association_id="landmark_pose_observations"/>
//   </Association>
//
// Candidate mounts are pruned by expected visibility from the prior: in
// front of the camera, outward normal toward the lens (min_facing_cos),
// projecting inside the calibrated image, and large enough to detect
// (min_projected_size_px). max_reprojection_error_px and
// max_alternate_pose_ambiguity are 0 to disable; when enabled they reject
// detections that do not report the value, never treating absent as zero.

#pragma once
#include <memory>
#include <string>

#include "config/field_map.h"
#include "contracts/association.h"
#include "resources/robot_frames.h"
#include "resources/target_set.h"

namespace navigatr
{

class TagMountAssociation : public Association
{
public:
    static std::unique_ptr<Association> create(const ConfigNode& node,
                                               SlotInitializationContext& context,
                                               std::string& err);

    AssociationOutput run(const AssociationInput& in) override;

    std::vector<AssociationOutputDecl> produces() const override;

private:
    ObservationId observations_ref_;
    AssociationId output_;

    std::shared_ptr<const FieldMap>      field_;
    std::shared_ptr<const RobotFrameMap> frames_;
    std::shared_ptr<const TargetSet>     targets_;

    double max_translation_error_m_ = 0.0;
    double max_heading_error_rad_   = 0.0;
    double ambiguity_margin_m_      = 0.0;
    double max_range_m_             = 0.0;
    double min_decision_margin_     = 0.0;
    long   max_hamming_             = 0;
    double min_facing_cos_          = 0.1;
    double min_projected_size_px_   = 8.0;
    double max_reprojection_error_px_    = 0.0;   // 0 disables
    double max_alternate_pose_ambiguity_ = 0.0;   // 0 disables
};

} // namespace navigatr
