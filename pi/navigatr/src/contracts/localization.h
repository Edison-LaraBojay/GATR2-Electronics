// localization.h
// Localization Prediction slot: the motion prediction step of the one
// continuous fused robot estimate, field frame. It runs before perception so
// association and correction work against the current cycle's predicted
// pose; corrections from external evidence happen in the PoseCorrection slot
// afterward.

#pragma once
#include <functional>
#include <memory>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/function_status.h"
#include "core/records.h"
#include "state/command_state.h"
#include "state/robot_state.h"

namespace navigatr
{

struct LocalizationInput {
    const SensorResultsMap& sensorResults;
    const ArtifactMap&      artifacts;
    const CommandState&     command;   // init pose, edge triggered by init_sequence
    const RobotState&       previous;
    MonotonicTime           now;   // host clock
};

struct LocalizationOutput {
    RobotState     robot;
    FunctionStatus status = FunctionStatus::kOk;
};

class Localization
{
public:
    virtual ~Localization() = default;

    virtual LocalizationOutput run(const LocalizationInput& in) = 0;

    virtual void reset() {}
};

using LocalizationMakeFunction = std::function<std::unique_ptr<Localization>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
