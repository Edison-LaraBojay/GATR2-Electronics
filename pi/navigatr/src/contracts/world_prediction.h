// world_prediction.h
// World Prediction slot: external object estimates. The no-op preserves the
// previous world state, so a robot that never looks at the world simply
// carries none. Brain wire ids for world objects are publisher
// configuration, never part of this state.

#pragma once
#include <functional>
#include <memory>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/function_status.h"
#include "core/records.h"
#include "state/command_state.h"
#include "state/robot_state.h"
#include "state/target_state.h"
#include "state/world_state.h"

namespace navigatr
{

struct WorldPredictionInput {
    const ObservationMap& observations;
    const AssociationMap& associations;
    const RobotState&     robot;   // corrected
    const WorldState&     previousWorld;
    const CommandState&   command;
    const TargetState&    previousTarget;
    MonotonicTime         now;   // host clock
};

struct WorldPredictionOutput {
    WorldState     world;
    TargetState    target;   // implementations without targets pass through
    FunctionStatus status = FunctionStatus::kOk;
};

class WorldPrediction
{
public:
    virtual ~WorldPrediction() = default;

    virtual WorldPredictionOutput run(const WorldPredictionInput& in) = 0;

    virtual void reset() {}
};

using WorldPredictionMakeFunction = std::function<std::unique_ptr<WorldPrediction>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
