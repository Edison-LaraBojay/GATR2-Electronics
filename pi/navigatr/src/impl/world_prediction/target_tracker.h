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
//   landmark_relative + acquire_once: buffer accepted evidence privately,
//     at most one candidate per camera frame, until the configured number
//     of consistent results from distinct frames arrives; then commit the
//     landmark update and the latched target pose atomically and close the
//     gate. Nothing mutates before the lock: one bad frame can never move
//     the landmark. On timeout the configured fallback applies explicitly,
//     and use_nominal_target reads the immutable field map nominal, never
//     a possibly-touched world estimate.
//
// Activation, snapshotting, and acquisition all require a valid robot
// estimate; a select command during startup defers until localization is
// real instead of latching zeros.
//
// Only the selected target's landmark ever mutates in world state, and only
// as a lock commit. It also seeds nominal poses for map landmarks so a
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
        acquisition_               = AcquisitionBuffer{};
    }

private:
    // Private evidence buffer for the current generation; nothing here is
    // visible outside until a lock commits it. Frame identity is
    // (camera, sequence): sequences are per device and collide across
    // cameras.
    struct AcquisitionBuffer {
        std::vector<Pose2D> target_candidates;   // consistent run, one per frame
        std::vector<Pose2D> landmark_poses;      // odom frame, same run
        std::vector<double> confidences;
        bool                have_frame          = false;
        SensorId            last_frame_camera;
        uint32_t            last_frame_sequence = 0;
    };

    Pose2D estimateTargetPose(const TargetDecl& decl, const WorldPredictionInput& in,
                              const WorldState& world) const;
    Pose2D mapNominalTargetPose(const TargetDecl& decl,
                                const WorldPredictionInput& in) const;

    std::shared_ptr<const TargetSet> targets_;
    std::shared_ptr<const FieldMap>  field_;   // null when no landmark targets
    AssociationId                    association_ref_;   // empty when never acquiring
    double                           blend_ = 1.0;

    uint64_t          last_seen_object_sequence_ = 0;
    uint64_t          generation_counter_        = 0;
    AcquisitionBuffer acquisition_;
};

} // namespace navigatr
