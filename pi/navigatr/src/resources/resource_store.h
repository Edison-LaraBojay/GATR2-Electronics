// resource_store.h
// Owns the initialized live resources: buses, links, shared decoders, shared
// data. It is not the factory registry (that stores construction functions)
// and it is not passive data (the objects inside stay stateful). The id to
// object mapping becomes read-only after startup; resources outlive every
// consumer that captured them because consumers hold shared handles.
//
// Recurring behavior belongs on the resource object itself; the store never
// holds getter functions that do work on retrieval.
//
// Construction resolves dependencies lazily by id with cycle detection, so
// XML declaration order never matters:
//
//   unbuilt -> building -> built | failed
//
// Requesting a resource that is already building reports the whole cycle.

#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "core/function_registry.h"
#include "core/ids.h"
#include "resources/resource_value.h"

namespace navigatr
{

class ResourceStoreBuilder;

// What a resource factory receives. resources resolves dependencies on other
// declared resources, building them on demand.
struct ResourceInitializationContext {
    ResourceStoreBuilder*     resolver  = nullptr;
    const FunctionRegistry*   functions = nullptr;
    std::vector<std::string>* warnings  = nullptr;

    template <typename Contract>
    std::shared_ptr<Contract> require(const ResourceId& id, std::string& err);
};

// signature stored in the FunctionRegistry under resource/* keys
using ResourceMakeFunction = std::function<ResourceValue(
    const ConfigNode&, ResourceInitializationContext&, std::string& err)>;

class ResourceStore
{
public:
    template <typename Contract>
    std::shared_ptr<Contract> require(const ResourceId& id, std::string& err) const {
        const ResourceValue* value = findValue(id);
        if (value == nullptr) {
            err = "unknown resource id " + id.value;
            return nullptr;
        }
        std::string inner;
        auto        handle = value->require<Contract>(inner);
        if (handle == nullptr) {
            err = "resource " + id.value + ": " + inner;
        }
        return handle;
    }

    const ResourceValue* findValue(const ResourceId& id) const;

    struct Record {
        ResourceId    id;
        FunctionKey   implementationType;
        ResourceValue value;
    };

    const std::vector<Record>& records() const { return records_; }

private:
    friend class ResourceStoreBuilder;
    std::vector<Record> records_;   // build order; destroyed in reverse
};

// Two-pass construction: index every definition first, then build each one,
// resolving references through the builder so order never matters.
class ResourceStoreBuilder
{
public:
    ResourceStoreBuilder(const FunctionRegistry& functions,
                         std::vector<std::string>* warnings)
        : functions_(functions), warnings_(warnings) {}

    // False on duplicate id or missing id/type. Node views must stay valid
    // until buildAll returns.
    bool index(const ConfigNode& node, std::string& err);

    // Replaces a declared definition's factory, e.g. replay swapping a live
    // link for a capture file. False when the id was never declared.
    bool overrideFactory(const ResourceId& id, ResourceMakeFunction factory,
                         std::string& err);

    bool declared(const ResourceId& id) const;

    // Dependency-resolving lookup used by factories mid-build.
    const ResourceValue* resolve(const ResourceId& id, std::string& err);

    // Force construction of every indexed definition.
    bool buildAll(std::string& err);

    // Valid after buildAll succeeded.
    ResourceStore take() { return std::move(store_); }

private:
    enum class State { kUnbuilt, kBuilding, kBuilt, kFailed };

    struct Definition {
        ResourceId          id;
        FunctionKey         type;
        ConfigNode          node;
        ResourceMakeFunction override_factory;   // empty unless overridden
        State               state = State::kUnbuilt;
    };

    Definition* findDefinition(const ResourceId& id);

    const FunctionRegistry&   functions_;
    std::vector<std::string>* warnings_;
    std::vector<Definition>   definitions_;
    std::vector<std::string>  build_stack_;   // for cycle reporting
    ResourceStore             store_;
};

template <typename Contract>
std::shared_ptr<Contract> ResourceInitializationContext::require(const ResourceId& id,
                                                                 std::string&      err) {
    if (resolver == nullptr) {
        err = "no resource resolver available";
        return nullptr;
    }
    const ResourceValue* value = resolver->resolve(id, err);
    if (value == nullptr) {
        return nullptr;
    }
    std::string inner;
    auto        handle = value->require<Contract>(inner);
    if (handle == nullptr) {
        err = "resource " + id.value + ": " + inner;
    }
    return handle;
}

} // namespace navigatr
