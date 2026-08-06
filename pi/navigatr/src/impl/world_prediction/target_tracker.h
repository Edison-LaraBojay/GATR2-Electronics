// target_tracker.h
// World prediction that owns the active navigation target. Activation is
// edge triggered by the command object_sequence: a retransmitted command
// never re-activates, a genuinely new command always does (and resnapshots
// a robot-relative target). Targets latch in the odometry frame; a field
// re-anchor cannot move them, and an odometry epoch change cancels them.
//
// Per policy:
//   robot_relative: latch T_odom_robot(activation) * delta once
//   landmark_relative + none: latch from the current landmark estimate
//     (world state when valid, else the nominal map pose); zero visual
//     correction, not a zero pose
//   landmark_relative + acquire_once: consume associated landmark pose
//     evidence stamped with the current generation until the configured
//     number of consistent results arrives, latch their mean, and close the
//     gate; on timeout apply the configured fallback explicitly
//
// Only the selected target's landmark ever mutates in world state, and only
// while acquiring. It also seeds nominal poses for map landmarks so a
// selected-but-unseen landmark still has an estimate.
//
//   <WorldPrediction type="target_tracker">
//       <Targets resource_id="targets"/>
//       <FieldMap resource_id="game_field"/>
//       <Associations association_id="landmark_pose_observations"/>
//       <Landmark blend="1.0"/>   optional, default 1
//   </WorldPrediction>

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "config/field_map.h"
#include "contracts/world_prediction.h"
#include "resources/target_set.h"

namespace navigatr
{

class TargetTracker : public WorldPrediction
{
public:
    static std::unique_ptr<WorldPrediction> create(const ConfigNode& node,
                                                   SlotInitializationContext& context,
                                                   std::string& err);

    WorldPredictionOutput run(const WorldPredictionInput& in) override;

    void reset() override {
        last_seen_object_sequence_ = 0;
        generation_counter_        = 0;
        candidates_.clear();
    }

private:
    Pose2D nominalTargetPose(const TargetDecl& decl, const WorldPredictionInput& in,
                             const WorldState& world) const;

    std::shared_ptr<const TargetSet> targets_;
    std::shared_ptr<const FieldMap>  field_;   // null when no landmark targets
    AssociationId                    association_ref_;   // empty when never acquiring
    double                           blend_ = 1.0;

    uint64_t            last_seen_object_sequence_ = 0;
    uint64_t            generation_counter_        = 0;
    std::vector<Pose2D> candidates_;   // consistent-run window while acquiring
};

} // namespace navigatr
