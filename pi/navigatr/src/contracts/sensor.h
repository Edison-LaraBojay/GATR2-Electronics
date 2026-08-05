// sensor.h
// A configured measurement producer, delivered by its factory as a captured
// executable: the payload contract it publishes plus the callables that poll
// and reset it. Resources are captured inside the closure at construction;
// polling input stays small and implementations cannot reach into arbitrary
// resources at runtime. The framework does the record bookkeeping
// (receivedAt, sequence, retention) itself.

#pragma once
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "core/function_registry.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

struct Diagnostics;
class ResourceMap;

struct SensorExecutionInput {
    MonotonicTime now;   // host clock
    uint64_t      cycle       = 0;
    Diagnostics*  diagnostics = nullptr;
};

struct SensorExecutable {
    PayloadDescriptor outputPayload;

    std::function<SensorPollResult(const SensorExecutionInput&)> execute;

    // Back to power-on state. May be empty when there is nothing to reset.
    std::function<void()> reset;
};

struct SensorInitializationContext {
    const ResourceMap*        resources = nullptr;
    const FunctionRegistry*   functions = nullptr;
    std::vector<std::string>* warnings  = nullptr;
};

// signature stored in the FunctionRegistry
using SensorMakeFunction = std::function<std::optional<SensorExecutable>(
    const ConfigNode&, SensorInitializationContext&, std::string& err)>;

} // namespace navigatr
