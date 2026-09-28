// landmark_estimator.h
// The landmark estimation step of the AprilTag world estimator: accepted
// evidence in, committed landmark estimates out. It seeds every mapped
// landmark at its nominal pose, keeps observed entries coherent with the
// current odometry epoch and field anchor, and folds accepted odometry-frame
// pose evidence per an explicit commit policy.
//
//   <LandmarkEstimation commit="always" blend="1.0"/>
//
// commit="always" (the normal mode) folds every accepted evidence record;
// commit="never" keeps the estimates still while evidence and traces still
// publish (camera inspection). Estimation is continuous and target-blind:
// navigation state can neither start, stop, nor reset it. Several accepted
// measurements of one object in one invocation fuse deterministically by
// confidence-weighted planar mean and circular heading mean, independent of
// iteration order; non-finite evidence and evidence from another odometry
// epoch never reach state.

#pragma once
#include <memory>
#include <optional>
#include <string>

#include "config/config_node.h"
#include "config/field_map.h"
#include "core/time.h"
#include "payloads/field_object_evidence.h"
#include "state/field_state.h"
#include "state/robot_state.h"

namespace navigatr
{

class LandmarkEstimator
{
public:
    // node is the <LandmarkEstimation> element. Nothing on failure.
    static std::optional<LandmarkEstimator> create(const ConfigNode&               node,
                                                   std::shared_ptr<const FieldMap> field,
                                                   std::string&                    err);

    // True when accepted evidence moves the estimates.
    bool commits() const { return commit_ == CommitPolicy::kAlways; }

    // The next field state from the previous one. evidence is null when
    // nothing was accepted this invocation.
    FieldState run(const FieldState& previous, const FieldObjectPoseEvidenceSet* evidence,
                   const RobotState& robot, MonotonicTime now) const;

private:
    enum class CommitPolicy { kAlways, kNever };

    std::shared_ptr<const FieldMap> field_;
    CommitPolicy                    commit_ = CommitPolicy::kAlways;
    double                          blend_  = 1.0;
};

} // namespace navigatr
