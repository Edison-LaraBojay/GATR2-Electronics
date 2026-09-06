// target_resolution.h
// Target Resolution slot: combine the configured target definitions, the
// command state, the robot estimate, the world estimate, and the published
// evidence into the active resolved target. This is focused domain logic
// driven by configuration, not an open algorithm ecosystem: the robot is
// not expected to vary here beyond which targets are configured, so the
// registered implementations are the explicit noop and the configured-
// targets resolver.
//
// The noop preserves the previous target state, so a pipeline without
// autonomous intent simply carries none.

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
#include "state/field_state.h"

namespace navigatr
{

struct TargetResolutionInput {
    const CommandState&   command;
    const RobotState&     robot;
    const FieldState&     field;          // committed estimates only
    const ObservationMap& observations;   // world estimation's published evidence
    const AssociationMap& associations;
    const TargetState&    previous;
    MonotonicTime         now;   // host clock
};

struct TargetResolutionOutput {
    TargetState    target;
    FunctionStatus status = FunctionStatus::kOk;
};

class TargetResolution
{
public:
    virtual ~TargetResolution() = default;

    virtual TargetResolutionOutput run(const TargetResolutionInput& in) = 0;

    virtual void reset() {}
};

using TargetResolutionMakeFunction = std::function<std::unique_ptr<TargetResolution>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
