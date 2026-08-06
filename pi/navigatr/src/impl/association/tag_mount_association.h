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
//   <Association type="tag_mount_association">
//       <Observations observation_id="tag_observations"/>
//       <FieldMap resource_id="game_field"/>
//       <RobotFrames resource_id="robot_geometry"/>
//       <Targets resource_id="targets"/>
//       <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
//              ambiguity_margin_m="0.15"/>
//       <Output association_id="landmark_pose_observations"/>
//   </Association>

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
};

} // namespace navigatr
