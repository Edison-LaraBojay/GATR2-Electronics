// publishing.h
// Publishing slot: the boundary encoder. It sees every product of the cycle
// and decides what leaves the Pi. Wire formats, wire object ids, and fixed
// point units are owned here, never by the states being published. The
// no-op publishes nothing, successfully.

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

struct PublishingInput {
    const SensorResultsMap& sensorResults;
    const ArtifactMap&      artifacts;
    const ObservationMap&   observations;
    const AssociationMap&   associations;
    const RobotState&       robot;
    const WorldState&       world;
    const CommandState&     command;
    const TargetState&      target;
    MonotonicTime           now;   // host clock
};

struct PublishingOutput {
    FunctionStatus status = FunctionStatus::kOk;
};

class Publishing
{
public:
    virtual ~Publishing() = default;

    virtual PublishingOutput run(const PublishingInput& in) = 0;

    virtual void reset() {}
};

using PublishingMakeFunction = std::function<std::unique_ptr<Publishing>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
