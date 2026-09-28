// sensor.h
// A configured measurement processor, delivered by its factory as a captured
// executable: the payload contract it publishes plus the callables that poll
// and reset it. At initialization the factory binds its inputs to declared
// resource outputs by (resource id, output id) with the payload type it
// expects, and captures parsed configuration and owning state. At runtime it
// reads the whole ResourceMap through those bindings and reports state plus
// an optional publication; the sensor stage does the record bookkeeping
// (receivedAt, sequence, epoch, retention) itself.

#pragma once
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "core/execution_context.h"
#include "core/function_registry.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

class ResourceStore;
class ResourceCatalog;

struct SensorExecutable {
    PayloadDescriptor outputPayload;

    std::function<PollResult(const ResourceMap&, const ExecutionContext&)> execute;

    // Back to power-on state. May be empty when there is nothing to reset.
    std::function<void()> reset;
};

struct SensorInitializationContext {
    const ResourceStore*      resources = nullptr;   // direct binding of static data
    const ResourceCatalog*    outputs   = nullptr;   // declared executable outputs
    const FunctionRegistry*   functions = nullptr;
    std::vector<std::string>* warnings  = nullptr;
};

// signature stored in the FunctionRegistry
using SensorMakeFunction = std::function<std::optional<SensorExecutable>(
    const ConfigNode&, SensorInitializationContext&, std::string& err)>;

} // namespace navigatr
