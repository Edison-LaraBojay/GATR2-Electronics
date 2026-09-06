// association.h
// Association slot: decides what each observation most likely is, using the
// current cycle's predicted robot pose and the previous world estimate.
// Ambiguity the evidence cannot resolve stays ambiguous; pretending an id
// solved it is an implementation bug, not a framework guarantee.

#pragma once
#include <functional>
#include <memory>
#include <vector>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/function_status.h"
#include "core/records.h"
#include "state/robot_state.h"
#include "state/field_state.h"

namespace navigatr
{

struct AssociationInput {
    const ObservationMap& observations;
    const RobotState&     robot;   // this cycle's predicted pose, with history
    const FieldState&     field;   // prior estimates, association priors
    MonotonicTime         now;   // host clock
};

struct AssociationOutput {
    AssociationMap associations;
    FunctionStatus status = FunctionStatus::kOk;
};

class Association
{
public:
    virtual ~Association() = default;

    virtual AssociationOutput run(const AssociationInput& in) = 0;

    // Association ids this configuration can produce, with payload contracts.
    virtual std::vector<AssociationOutputDecl> produces() const { return {}; }

    virtual void reset() {}
};

using AssociationMakeFunction = std::function<std::unique_ptr<Association>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
