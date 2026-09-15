// sensor_stage.h
// make_sensors and the aggregate executor it returns.
//
//   execute_sensors = make_sensors(<Sensors>, registry, store, resource outputs)
//   sensor_map = execute_sensors(resource_map, context)
//
// The generic builder owns id, type, duplicates and factory lookup; the
// selected factory parses its own subtree, binds its inputs to declared
// resource outputs with the payload it expects, and returns a captured
// executable. The executor owns iteration, declared-output enforcement,
// sequence/epoch/receipt bookkeeping, retention, diagnostics and map
// assembly; sensors only report state and publications.

#pragma once
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "config/config_node.h"
#include "contracts/sensor.h"
#include "core/execution_context.h"
#include "core/function_registry.h"
#include "core/ids.h"
#include "core/records.h"
#include "resources/resource_store.h"
#include "runtime/resource_catalog.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

// Configured sensor executables, declaration order, id indexed.
class SensorFunctions
{
public:
    struct Entry {
        SensorId         id;
        SensorExecutable sensor;
        std::string      label;   // diagnostics
    };

    // False on a duplicate id.
    bool add(SensorId id, SensorExecutable sensor, std::string label) {
        if (index_by_id_.find(id) != index_by_id_.end()) {
            return false;
        }
        index_by_id_.emplace(id, execution_order_.size());
        execution_order_.push_back(Entry{std::move(id), std::move(sensor), std::move(label)});
        return true;
    }

    const Entry* find(const SensorId& id) const {
        const auto it = index_by_id_.find(id);
        return it == index_by_id_.end() ? nullptr : &execution_order_[it->second];
    }

    std::vector<Entry>&       executionOrder() { return execution_order_; }
    const std::vector<Entry>& executionOrder() const { return execution_order_; }

    std::size_t size() const { return execution_order_.size(); }

private:
    std::vector<Entry>                                        execution_order_;
    std::unordered_map<SensorId, std::size_t, SensorId::Hash> index_by_id_;
};

// execute_sensors: the captured collection plus the retained result map.
class SensorExecutor
{
public:
    SensorExecutor() = default;
    SensorExecutor(SensorFunctions functions, SensorCatalog catalog);

    // Polls every sensor once against the resource results and returns the
    // retained map. The reference stays valid until the next call or reset.
    const SensorMap& operator()(const ResourceMap& resources, const ExecutionContext& context);

    void reset();

    const SensorCatalog& outputs() const { return catalog_; }
    const SensorMap&     retained() const { return retained_; }

private:
    SensorFunctions functions_;
    SensorCatalog   catalog_;
    SensorMap       retained_;
};

// node may be invalid (no Sensors section). Nothing on failure.
std::optional<SensorExecutor> make_sensors(const ConfigNode&         node,
                                           const FunctionRegistry&   functions,
                                           const ResourceStore&      store,
                                           const ResourceCatalog&    resource_outputs,
                                           std::vector<std::string>* warnings,
                                           std::string&              err);

} // namespace navigatr
