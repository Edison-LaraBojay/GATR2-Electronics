// tag_mount_association.h
// Association: decides which physical tag mount produced each canonical tag
// observation, by full pose against every mount sharing the observed id and
// family, never by id alone and never forced by expectation. The winner
// must both pass the geometric gates and beat the runner-up by the
// ambiguity margin; anything less abstains.
//
// Association is continuous and target-blind: every decisive observation
// associates against the whole field map, every invocation, whether or not
// any navigation target exists. Filtering by navigation intent (requested
// object, allowed mounts, preferred camera, activation time) belongs to
// target resolution, never here.
//
// Acceptance requires, in order: a valid robot pose at exposure from the
// localization history (exact or interpolated inside the gate) in the
// current odometry epoch; a metric tag pose; detector quality (hamming,
// decision margin, and when the detector reports them, reprojection error
// and alternate-pose ambiguity); physical plausibility (tag in front of
// the camera, facing it, within range); the translation and heading gates
// against the prior; and a decisive combined-score margin over the
// runner-up. Every tag's decision is published in the trace output.
//
//   <Association type="tag_mount_association">
//       <Observations observation_id="tag_observations"/>
//       <FieldMap resource_id="game_field"/>
//       <RobotFrames resource_id="robot_geometry"/>
//       <Attitude policy="assume_level"/>      or require
//       <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
//              ambiguity_margin_m="0.15" max_range_m="3.0"
//              min_decision_margin="20" max_hamming="0"
//              min_facing_cos="0.1" min_projected_size_px="8"
//              max_reprojection_error_px="0" max_alternate_pose_ambiguity="0"/>
//       <Output association_id="landmark_pose_observations"/>
//       <Trace association_id="tag_association_trace"/>   optional
//   </Association>
//
// The capture-time chain is T_odom_tag = T_odom_robot(t) * T_robot_camera
// * T_camera_tag with T_odom_robot(t) in SE(3): yaw from history, roll and
// pitch from the attitude at exposure when localization measured one.
// policy assume_level uses a level body when attitude is missing or stale
// and marks the evidence; require rejects the frame instead.

#pragma once
#include <memory>
#include <string>

#include "config/field_map.h"
#include "contracts/association.h"
#include "resources/robot_frames.h"

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
    enum class AttitudePolicy { kAssumeLevel, kRequire };

    ObservationId observations_ref_;
    AssociationId output_;
    AssociationId trace_;   // empty when not configured

    std::shared_ptr<const FieldMap>      field_;
    std::shared_ptr<const RobotFrameMap> frames_;

    AttitudePolicy attitude_policy_ = AttitudePolicy::kAssumeLevel;

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
