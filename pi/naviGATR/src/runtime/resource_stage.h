// resource_stage.h
// make_resources and the aggregate executor it returns.
//
//   built = make_resources(<Resources>, registry, options)
//   resource_map = built.execute(context)
//
// The generic builder owns id, type, duplicates, factory lookup, replay
// overrides and dependency order; the selected factory owns everything else
// in its subtree. Every factory runs once. Its initialized object goes into
// the ResourceStore for direct binding, and its executable, when it has one,
// is captured by the executor. The executor owns iteration, output
// validation, sequence/epoch/receipt bookkeeping, retention, diagnostics
// and map assembly; the coordinator only calls it.

#pragma once
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "config/config_node.h"
#include "contracts/resource.h"
#include "core/execution_context.h"
#include "core/function_registry.h"
#include "core/ids.h"
#include "core/records.h"
#include "resources/resource_store.h"
#include "runtime/build_options.h"
#include "runtime/resource_catalog.h"

namespace navigatr
{

// Configured resource executables, build order, id indexed.
class ResourceFunctions
{
public:
    struct Entry {
        ResourceId         id;
        ResourceExecutable executable;
        std::string        label;   // diagnostics
    };

    // False on a duplicate id.
    bool add(ResourceId id, ResourceExecutable executable, std::string label) {
        if (index_by_id_.find(id) != index_by_id_.end()) {
            return false;
        }
        index_by_id_.emplace(id, execution_order_.size());
        execution_order_.push_back(
            Entry{std::move(id), std::move(executable), std::move(label)});
        return true;
    }

    const Entry* find(const ResourceId& id) const {
        const auto it = index_by_id_.find(id);
        return it == index_by_id_.end() ? nullptr : &execution_order_[it->second];
    }

    std::vector<Entry>&       executionOrder() { return execution_order_; }
    const std::vector<Entry>& executionOrder() const { return execution_order_; }

    std::size_t size() const { return execution_order_.size(); }

private:
    std::vector<Entry>                                            execution_order_;
    std::unordered_map<ResourceId, std::size_t, ResourceId::Hash> index_by_id_;
};

// execute_resources: the captured collection plus the retained result map.
class ResourceExecutor
{
public:
    ResourceExecutor() = default;
    ResourceExecutor(ResourceFunctions functions, ResourceCatalog catalog);

    // Polls every executable once and returns the retained map. The
    // reference stays valid until the next call or reset.
    const ResourceMap& operator()(const ExecutionContext& context);

    // Back to power-on: every resource reset once, records cleared into
    // the next epoch.
    void reset();

    const ResourceCatalog& outputs() const { return catalog_; }
    const ResourceMap&     retained() const { return retained_; }

private:
    ResourceFunctions functions_;
    ResourceCatalog   catalog_;
    ResourceMap       retained_;
};

struct ResourceBuild {
    ResourceStore    store;     // initialized objects, bound directly by consumers
    ResourceExecutor execute;   // captured executables, one call per cycle
};

// node may be invalid (no Resources section). Nothing on failure.
std::optional<ResourceBuild> make_resources(const ConfigNode&         node,
                                            const FunctionRegistry&   functions,
                                            const BuildOptions&       options,
                                            std::vector<std::string>* warnings,
                                            std::string&              err);

} // namespace navigatr
