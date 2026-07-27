// pose_correction.h
// Pose Correction slot: folds associated external evidence, e.g. fixed field
// tags, into the predicted robot pose. Runs after association so the
// associator saw the current cycle's prediction. The no-op passes the
// predicted pose through unchanged, which is a robot trusting dead
// reckoning alone.

#pragma once
#include <functional>
#include <memory>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/function_status.h"
#include "core/records.h"
#include "state/robot_state.h"

namespace navigatr
{

struct PoseCorrectionInput {
    const ArtifactMap&    artifacts;
    const ObservationMap& observations;
    const AssociationMap& associations;
    const RobotState&     predicted;
    MonotonicTime         now;   // host clock
};

struct PoseCorrectionOutput {
    RobotState     robot;
    FunctionStatus status = FunctionStatus::kOk;
};

class PoseCorrection
{
public:
    virtual ~PoseCorrection() = default;

    virtual PoseCorrectionOutput run(const PoseCorrectionInput& in) = 0;

    virtual void reset() {}
};

using PoseCorrectionMakeFunction = std::function<std::unique_ptr<PoseCorrection>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
