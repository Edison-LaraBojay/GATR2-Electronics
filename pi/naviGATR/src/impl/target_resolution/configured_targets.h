// configured_targets.h
// Target resolution driven by the configured target set. Focused domain
// logic, deliberately not an open algorithm registry: activation is edge
// triggered by the command object_sequence (a retransmitted command never
// re-activates, a genuinely new one always does), targets latch in the
// odometry frame, a field re-anchor cannot move them, and an odometry epoch
// change cancels them.
//
// Per policy:
//   robot_relative: latch T_odom_robot(activation) * delta once
//   landmark_relative + none: latch from the current committed landmark
//     estimate (world state when valid, else the nominal map pose); zero
//     visual correction, not a zero pose
//   landmark_relative + acquire_once: filter the generic field evidence by
//     navigation intent (requested object, allowed feature mounts,
//     preferred source, freshness, activation time - evidence measured
//     before activation never acquires), buffer accepted evidence privately,
//     at most one candidate per (camera, frame) so several mounts in one
//     image count once, until the configured number of consistent results
//     from distinct frames arrives; then latch the window's mean target
//     pose and close the gate. Nothing outside the buffer changes before
//     the lock. On timeout the configured fallback applies explicitly, and
//     use_nominal_target reads the immutable field map nominal, never a
//     possibly-touched world estimate.
//
// Field estimates are never mutated here: object estimation belongs to the
// Field Estimation slot and its explicit commit policy, which runs
// continuously and independently of any target.
//
// Activation, snapshotting, and acquisition all require a valid robot
// estimate; a select command during startup defers until localization is
// real instead of latching zeros.
//
//   <TargetResolution type="configured_targets">
//       <Targets resource_id="targets"/>
//       <FieldMap resource_id="game_field"/>
//       <Evidence association_id="landmark_pose_observations"/>
//   </TargetResolution>

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "config/field_map.h"
#include "contracts/target_resolution.h"
#include "resources/target_set.h"

namespace navigatr
{

class ConfiguredTargetResolution : public TargetResolution
{
public:
    static std::unique_ptr<TargetResolution> create(const ConfigNode& node,
                                                    SlotInitializationContext& context,
                                                    std::string& err);

    TargetResolutionOutput run(const TargetResolutionInput& in) override;

    void reset() override {
        last_seen_object_sequence_ = 0;
        generation_counter_        = 0;
        acquisition_               = AcquisitionBuffer{};
    }

private:
    // Private evidence buffer for the current generation; nothing here is
    // visible outside until a lock reads it. Frame identity is
    // (camera, sequence): sequences are per device and collide across
    // cameras.
    struct AcquisitionBuffer {
        std::vector<Pose2D> landmark_poses;      // odom frame, consistent run
        std::vector<Pose2D> target_candidates;   // one per accepted frame
        std::vector<double> confidences;
        bool                have_frame          = false;
        SensorId            last_frame_camera;
        uint32_t            last_frame_sequence = 0;
    };

    Pose2D estimateTargetPose(const TargetDecl& decl, const TargetResolutionInput& in) const;
    Pose2D mapNominalTargetPose(const TargetDecl& decl,
                                const TargetResolutionInput& in) const;

    std::shared_ptr<const TargetSet> targets_;
    std::shared_ptr<const FieldMap>  field_;   // null when no landmark targets
    AssociationId                    evidence_ref_;   // empty when never acquiring

    uint64_t          last_seen_object_sequence_ = 0;
    uint64_t          generation_counter_        = 0;
    AcquisitionBuffer acquisition_;
};

} // namespace navigatr
