// apriltag_world_estimator.h
// World estimation from AprilTag landmarks: one selectable implementation
// of the FieldEstimation contract that privately owns a fixed serial
// subpipeline over one camera:
//
//   ObservationExtraction  camera frames -> canonical tag observations
//                          (AprilTagObservationPerception, Perception contract)
//   Association            observations -> decisive landmark pose evidence
//                          (TagMountAssociation, Association contract), optional
//   LandmarkEstimation     evidence -> committed landmark estimates
//                          (LandmarkEstimator)
//
// The steps run in this order against the same internal contracts they
// always had, and are reachable only through this class; no other component
// can address them, and the coordinator never learns they exist. A step
// fault leaves the previous field untouched and publishes nothing: no
// partial commit. Without an Association the extractor's observations still
// publish (camera inspection before the camera has a mount frame) and the
// landmark estimator must not commit, since nothing produces evidence.
//
//   <Estimator id="goals" type="apriltag">
//       <FieldMap resource_id="override_field"/>
//       <ObservationExtraction>
//           <Camera sensor_id="front_camera"/>
//           <Detector resource_id="tag_detector"/>
//           <Output observation_id="tag_observations"/>
//       </ObservationExtraction>
//       <Association>                                          optional
//           <Observations observation_id="tag_observations"/>
//           <FieldMap resource_id="override_field"/>
//           <RobotFrames resource_id="robot_geometry"/>
//           <Attitude policy="assume_level"/>
//           <Gates .../>
//           <Output association_id="landmark_pose_observations"/>
//           <Trace association_id="tag_association_trace"/>     optional
//       </Association>
//       <LandmarkEstimation commit="always" blend="1.0"/>
//   </Estimator>

#pragma once
#include <memory>
#include <string>

#include "contracts/association.h"
#include "contracts/field_estimation.h"
#include "contracts/perception.h"
#include "impl/world_estimation/landmark_estimator.h"

namespace navigatr
{

class AprilTagWorldEstimator : public FieldEstimation
{
public:
    static std::unique_ptr<FieldEstimation> create(const ConfigNode& node,
                                                   SlotInitializationContext& context,
                                                   std::string& err);

    FieldEstimationOutput run(const FieldEstimationInput& in) override;

    std::vector<ObservationOutputDecl> producesObservations() const override;
    std::vector<AssociationOutputDecl> producesAssociations() const override;

    void reset() override;

private:
    std::unique_ptr<Perception>      observation_extraction_;
    std::unique_ptr<Association>     association_;   // null when not configured
    std::optional<LandmarkEstimator> landmarks_;

    AssociationId evidence_ref_;   // the association's landmark evidence output
};

} // namespace navigatr
