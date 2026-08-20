// preprocessing.h
// Preprocessing slot: sensor results in, artifacts out. This is the boundary
// where measurements are calibrated, normalized, and combined. An algorithm
// here knows the payload contracts it consumes, never the hardware behind
// them; input is the same whether it came from a device, replay, or
// simulation.
//
// Every executable declares its artifact output ids and payload contracts at
// initialization, so duplicate outputs and impossible wiring die before
// runtime. Downstream code references artifact ids, never preprocessor
// executables; execution authority stays with the fixed pipeline.

#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "core/function_registry.h"
#include "core/function_status.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

struct Diagnostics;
class SensorCatalog;
class ResourceMap;

struct PreprocessingInput {
    const SensorResultsMap& sensorResults;
    MonotonicTime           now;   // host clock
    uint64_t                cycle       = 0;
    Diagnostics*            diagnostics = nullptr;
};

struct PreprocessingOutput {
    ArtifactMap    artifacts;
    FunctionStatus status = FunctionStatus::kOk;
};

struct ArtifactOutputDecl {
    ArtifactId        id;
    PayloadDescriptor payload;
};

class Preprocessing
{
public:
    virtual ~Preprocessing() = default;

    virtual PreprocessingOutput run(const PreprocessingInput& in) = 0;

    // Artifact ids this configuration can produce, with payload contracts.
    virtual std::vector<ArtifactOutputDecl> produces() const { return {}; }

    virtual void reset() {}
};

// One configured executable inside a composite preprocessing implementation.
class PreprocessorExecutable
{
public:
    virtual ~PreprocessorExecutable() = default;

    virtual FunctionStatus run(const PreprocessingInput& in, ArtifactMap& out) = 0;

    virtual const PreprocessorId& id() const = 0;

    virtual std::vector<ArtifactOutputDecl> outputs() const = 0;

    virtual void reset() {}
};

struct PreprocessorInitializationContext {
    const SensorCatalog*      sensors   = nullptr;
    const ResourceMap*        resources = nullptr;
    const FunctionRegistry*   functions = nullptr;
    std::vector<std::string>* warnings  = nullptr;
};

// signatures stored in the FunctionRegistry
using PreprocessingMakeFunction = std::function<std::unique_ptr<Preprocessing>(
    const ConfigNode&, PreprocessorInitializationContext&, std::string& err)>;
using PreprocessorMakeFunction = std::function<std::unique_ptr<PreprocessorExecutable>(
    const ConfigNode&, PreprocessorInitializationContext&, std::string& err)>;

} // namespace navigatr
