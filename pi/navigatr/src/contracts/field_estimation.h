// field_estimation.h
// World estimation: external-state estimation, nothing else. This is the
// one input/output contract through which the coordinator drives the
// selected estimator, whatever it is inside. An implementation may be a
// leaf, the explicit noop, or a composite that privately owns a subpipeline
// (observation extraction, association, one or more landmark estimators);
// the runtime interacts with every form through this contract and never
// learns how many internal steps exist.
//
// The input is the entire sensor snapshot read-only, the latest robot state,
// timestamped pose and attitude lookups into localization history, the
// previous field state, and the execution context (host clock, invocation,
// diagnostics). The output carries the field estimate plus the standard
// evidence maps the implementation chose to publish, with status and a
// diagnostic. Observations and associations cross this boundary because
// target resolution, publishing and inspection genuinely consume them;
// everything else an implementation computes stays private to it.
//
// Estimation runs every invocation on whatever evidence the implementation
// accepts, whether or not any navigation target exists; TargetState belongs
// to Target Resolution. It may run on its own worker: nothing here may
// reach back into the coordinator.

#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/execution_context.h"
#include "core/function_status.h"
#include "core/records.h"
#include "state/field_state.h"
#include "state/pose_history.h"
#include "state/robot_state.h"

namespace navigatr
{

struct FieldEstimationInput {
    const SensorMap&        sensors;         // the whole map, read-only
    const RobotState&       robot;           // latest snapshot
    const PoseLookup&       history;         // pose and attitude at measurement time
    const FieldState&       previousField;
    const ExecutionContext& context;         // host clock, invocation, diagnostics
};

struct FieldEstimationOutput {
    FieldState     field;
    ObservationMap observations;   // published evidence, may be empty
    AssociationMap associations;   // published evidence, may be empty
    FunctionStatus status = FunctionStatus::kOk;
    std::string    diagnostic;
};

class FieldEstimation
{
public:
    virtual ~FieldEstimation() = default;

    virtual FieldEstimationOutput run(const FieldEstimationInput& in) = 0;

    // Declared published outputs, for build-time reference checks by later
    // slots and runtime enforcement by the stage. A composite reports what
    // crosses the boundary, never its private intermediates.
    virtual std::vector<ObservationOutputDecl> producesObservations() const { return {}; }
    virtual std::vector<AssociationOutputDecl> producesAssociations() const { return {}; }

    virtual void reset() {}
};

// Signature stored in the FunctionRegistry under the Estimator type; the
// factory resolves its references once and captures everything it keeps.
using FieldEstimationMakeFunction = std::function<std::unique_ptr<FieldEstimation>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
