// perception.h
// Perception: sensor results in, observations of external things out. An
// internal step contract of a world estimator's subpipeline, never a
// coordinator slot: the estimator that owns a perception step decides when
// it runs and what it publishes across the boundary. Detectors produce
// generic observation records; nothing downstream depends on how an
// observation was made. Cameras and other heavyweight sources are
// resources captured at initialization. A detector failure is a fault
// carried through diagnostics, never an empty result.

#pragma once
#include <string>
#include <vector>

#include "config/config_node.h"
#include "contracts/slot_init.h"
#include "core/function_status.h"
#include "core/records.h"

namespace navigatr
{

struct PerceptionInput {
    const SensorMap& sensors;
    MonotonicTime    now;   // host clock
};

struct PerceptionOutput {
    ObservationMap observations;
    FunctionStatus status = FunctionStatus::kOk;
    std::string    diagnostic;
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

} // namespace navigatr
