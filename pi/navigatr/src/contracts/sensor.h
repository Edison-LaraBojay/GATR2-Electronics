// sensor.h
// A configured measurement producer. Each one owns its device knowledge,
// its channel mapping, and its private config schema; the framework calls
// poll and does the record bookkeeping (receivedAt, sequence, retention)
// itself. Resources are captured at construction; polling input stays small
// and implementations cannot reach into arbitrary resources at runtime.

#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "config/config_node.h"
#include "core/function_registry.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

struct Diagnostics;
class ResourceStore;

struct SensorExecutionInput {
    MonotonicTime now;   // host clock
    uint64_t      cycle       = 0;
    Diagnostics*  diagnostics = nullptr;
};

class Sensor
{
public:
    virtual ~Sensor() = default;

    virtual SensorPollResult poll(const SensorExecutionInput& input) = 0;

    // Payload contract of every publication this sensor produces.
    virtual const PayloadDescriptor& outputPayload() const = 0;

    virtual void reset() {}
};

struct SensorInitializationContext {
    const ResourceStore*      resources = nullptr;
    const FunctionRegistry*   functions = nullptr;
    std::vector<std::string>* warnings  = nullptr;
};

// signature stored in the FunctionRegistry under sensor/* keys
using SensorMakeFunction = std::function<std::unique_ptr<Sensor>(
    const ConfigNode&, SensorInitializationContext&, std::string& err)>;

} // namespace navigatr
