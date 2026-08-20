// world_estimation.h
// World Estimation slot: external-state estimation, nothing else. The
// selected implementation may be a leaf, an explicit noop, or a composite
// that privately owns observation extraction, association, and one or more
// estimators; the runtime interacts with all three forms through this one
// contract and never learns how many internal children exist.
//
// The output carries the world estimate plus the standard evidence maps the
// composite chose to publish. Observations and associations cross this
// boundary because target resolution and publishing genuinely consume them;
// everything else a composite computes stays private to it.
//
// Autonomous intent does not live here: TargetState belongs to the Target
// Resolution slot. The previous target state is an input only so evidence
// gating (which target generation produced this) stays possible.

#pragma once
#include <functional>
#include <memory>
#include <vector>

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

struct WorldEstimationInput {
    const SensorResultsMap& sensorResults;
    const ArtifactMap&      artifacts;
    const RobotState&       robot;   // this cycle's estimate, with history
    const WorldState&       previousWorld;
    const CommandState&     command;
    const TargetState&      previousTarget;
    MonotonicTime           now;   // host clock
};

struct WorldEstimationOutput {
    WorldState     world;
    ObservationMap observations;   // published evidence, may be empty
    AssociationMap associations;   // published evidence, may be empty
    FunctionStatus status = FunctionStatus::kOk;
};

class WorldEstimation
{
public:
    virtual ~WorldEstimation() = default;

    virtual WorldEstimationOutput run(const WorldEstimationInput& in) = 0;

    // Declared published outputs, for build-time reference checks by later
    // slots. A composite reports what its children publish across the
    // boundary, never their private intermediates.
    virtual std::vector<ObservationOutputDecl> producesObservations() const { return {}; }
    virtual std::vector<AssociationOutputDecl> producesAssociations() const { return {}; }

    virtual void reset() {}
};

using WorldEstimationMakeFunction = std::function<std::unique_ptr<WorldEstimation>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
