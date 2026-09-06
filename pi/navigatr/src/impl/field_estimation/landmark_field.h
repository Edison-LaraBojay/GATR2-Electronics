// landmark_field.h
// Composite world estimation for fiducial landmarks. It privately owns a
// nested pipeline of three children and satisfies the same FieldEstimation
// contract a leaf or a noop would:
//
//   ObservationExtraction (Perception contract)  camera frames -> canonical
//                                                tag observations
//   Association           (Association contract) observations -> decisive
//                                                landmark pose evidence
//   Estimator             (composite-owned)      evidence -> committed
//                                                landmark estimates
//
// The children run in this fixed order, are built from the same registry
// categories as any perception or association implementation (explicit
// type, including noop), and are reachable only through this parent; no
// other component can address them. A child fault leaves the previous world
// untouched: nothing partially commits.
//
// The estimator is deliberately not an open plugin point yet: it seeds
// nominal poses from the field map, and its commit policy is explicit
// configuration. commit="always" (the normal mode) folds every accepted
// evidence record; commit="never" publishes evidence without moving the
// estimates (traces only). Estimation is continuous and target-blind: it
// runs every cycle on whatever association accepted, and navigation state
// can neither start, stop, nor reset it. Several accepted measurements of
// one object in one cycle fuse deterministically by confidence-weighted
// planar mean and circular heading mean, independent of iteration order,
// and non-finite evidence never reaches state.
//
//   <FieldEstimation type="landmark_field">
//       <FieldMap resource_id="game_field"/>
//       <Pipeline>
//           <ObservationExtraction type="apriltag_tag_observation">
//               ...child-owned schema...
//           </ObservationExtraction>
//           <Association type="tag_mount_association">
//               ...child-owned schema...
//           </Association>
//           <Estimator type="landmark_estimator" commit="always" blend="1.0"/>
//       </Pipeline>
//   </FieldEstimation>

#pragma once
#include <memory>
#include <string>

#include "config/field_map.h"
#include "contracts/association.h"
#include "contracts/perception.h"
#include "contracts/field_estimation.h"
#include "payloads/field_object_evidence.h"

namespace navigatr
{

class LandmarkFieldEstimation : public FieldEstimation
{
public:
    static std::unique_ptr<FieldEstimation> create(const ConfigNode& node,
                                                   SlotInitializationContext& context,
                                                   std::string& err);

    FieldEstimationOutput run(const FieldEstimationInput& in) override;

    std::vector<ObservationOutputDecl> producesObservations() const override;
    std::vector<AssociationOutputDecl> producesAssociations() const override;

    void reset() override {
        observation_extraction_->reset();
        association_->reset();
    }

private:
    enum class CommitPolicy { kAlways, kNever };

    std::unique_ptr<Perception>  observation_extraction_;
    std::unique_ptr<Association> association_;

    std::shared_ptr<const FieldMap> field_;
    AssociationId                   evidence_ref_;   // association child's output
    CommitPolicy                    commit_ = CommitPolicy::kAlways;
    double                          blend_  = 1.0;
};

} // namespace navigatr
