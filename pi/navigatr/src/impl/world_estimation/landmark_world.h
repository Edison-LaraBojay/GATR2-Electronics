// landmark_world.h
// Composite world estimation for fiducial landmarks. It privately owns a
// nested pipeline of three children and satisfies the same WorldEstimation
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
// configuration. commit="always" folds every decisive observation
// (diagnostic and estimation profiles); commit="never" publishes evidence
// without moving the world (traces only); commit="on_target_lock" folds
// exactly the locked landmark estimate that target resolution latched
// (TargetState.locked_landmark), once per generation. There is one
// acceptance path: evidence the resolver rejected or never confirmed does
// not exist here, so it can never move a landmark, and a nominal fallback
// or cancellation leaves zero camera trace.
//
//   <WorldEstimation type="landmark_world">
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
//   </WorldEstimation>

#pragma once
#include <memory>
#include <string>

#include "config/field_map.h"
#include "contracts/association.h"
#include "contracts/perception.h"
#include "contracts/world_estimation.h"
#include "payloads/landmark_pose_observations.h"

namespace navigatr
{

class LandmarkWorldEstimation : public WorldEstimation
{
public:
    static std::unique_ptr<WorldEstimation> create(const ConfigNode& node,
                                                   SlotInitializationContext& context,
                                                   std::string& err);

    WorldEstimationOutput run(const WorldEstimationInput& in) override;

    std::vector<ObservationOutputDecl> producesObservations() const override;
    std::vector<AssociationOutputDecl> producesAssociations() const override;

    void reset() override {
        observation_extraction_->reset();
        association_->reset();
        last_committed_generation_ = 0;
    }

private:
    enum class CommitPolicy { kAlways, kNever, kOnTargetLock };

    std::unique_ptr<Perception>  observation_extraction_;
    std::unique_ptr<Association> association_;

    std::shared_ptr<const FieldMap> field_;
    AssociationId                   evidence_ref_;   // association child's output
    CommitPolicy                    commit_ = CommitPolicy::kAlways;
    double                          blend_  = 1.0;

    // on_target_lock only: the last generation whose locked landmark was
    // folded, so each lock commits exactly once.
    uint64_t last_committed_generation_ = 0;
};

} // namespace navigatr
