// perception.h
// Perception slot: sensor results and artifacts in, observations of external
// things out. Detectors produce generic observation records; nothing
// downstream depends on how an observation was made. Cameras and other
// heavyweight sources are resources captured at initialization.

#pragma once
#include <functional>
#include <memory>
#include <vector>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/function_status.h"
#include "core/records.h"
#include "state/target_state.h"

namespace navigatr
{

struct PerceptionInput {
    const SensorResultsMap& sensorResults;
    const ArtifactMap&      artifacts;
    const TargetState&      target;   // previous cycle; gates expensive work
    MonotonicTime           now;   // host clock
};

struct PerceptionOutput {
    ObservationMap observations;
    FunctionStatus status = FunctionStatus::kOk;
};

class Perception
{
public:
    virtual ~Perception() = default;

    virtual PerceptionOutput run(const PerceptionInput& in) = 0;

    // Observation ids this configuration can produce, with payload contracts.
    virtual std::vector<ObservationOutputDecl> produces() const { return {}; }

    virtual void reset() {}
};

using PerceptionMakeFunction = std::function<std::unique_ptr<Perception>(
    const ConfigNode&, SlotInitializationContext&, std::string& err)>;

} // namespace navigatr
