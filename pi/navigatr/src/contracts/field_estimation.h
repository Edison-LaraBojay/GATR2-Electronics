// field_estimation.h
// Field Estimation: external-state estimation, nothing else. The selected
// implementation may be a leaf, an explicit noop, or a composite that
// privately owns observation extraction, association, and one or more
// estimators; the runtime interacts with all three forms through this one
// contract and never learns how many internal children exist.
//
// The input is a sensor snapshot plus read access to localization: the
// latest robot state and timestamped history lookups. The output carries
// the field estimate plus the standard evidence maps the composite chose
// to publish. Observations and associations cross this boundary because
// target resolution, publishing and inspection genuinely consume them;
// everything else a composite computes stays private to it.
//
// Field estimation runs every invocation on whatever evidence its children
// accept, whether or not any navigation target exists; TargetState belongs
// to Target Resolution. It may run on its own worker: nothing here may
// reach back into the coordinator.

#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/function_status.h"
#include "core/records.h"
#include "state/field_state.h"
#include "state/pose_history.h"
#include "state/robot_state.h"

namespace navigatr
{

struct FieldEstimationInput {
    const SensorMap&  sensors;
    const RobotState& robot;     // latest snapshot
    const PoseLookup& history;   // pose at measurement time
    const FieldState& previousField;
    MonotonicTime     now;   // host clock
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
    // slots. A composite reports what its children publish across the
    // boundary, never their private intermediates.
    virtual std::vector<ObservationOutputDecl> producesObservations() const { return {}; }
    virtual std::vector<AssociationOutputDecl> producesAssociations() const { return {}; }

    virtual void reset() {}
};

using FieldEstimationMakeFunction = std::function<std::unique_ptr<FieldEstimation>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
