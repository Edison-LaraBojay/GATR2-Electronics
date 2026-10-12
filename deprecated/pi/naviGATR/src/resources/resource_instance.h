// resource_instance.h
// Type-erased ownership of one initialized resource with typed retrieval,
// plus its optional runtime executable. A resource is stored under the
// contract it satisfies (SpiBus, SerialLink, PicoTelemetry, ...); asking for
// the wrong contract or an unknown id fails during initialization, and no
// caller ever sees a raw void pointer.
//
// This contract erasure is a separate concept from the immutable payload
// erasure in TypedPayload.

#pragma once
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <typeindex>
#include <utility>

#include "contracts/resource.h"

namespace navigatr
{

class ResourceInstance
{
public:
    ResourceInstance() = default;

    template <typename Contract, typename Implementation>
    static ResourceInstance asContract(std::shared_ptr<Implementation> implementation) {
        // constness is restored on retrieval; the value is only reachable
        // through the recorded contract type
        using Mutable = std::remove_const_t<Contract>;
        ResourceInstance v;
        v.contract_ = std::type_index(typeid(Contract));
        v.object_   = std::static_pointer_cast<void>(std::const_pointer_cast<Mutable>(
            std::static_pointer_cast<Contract>(std::move(implementation))));
        v.contract_name_ = typeid(Contract).name();
        return v;
    }

    bool empty() const { return object_ == nullptr; }

    template <typename Contract>
    std::shared_ptr<Contract> require(std::string& err) const {
        if (empty()) {
            err = "resource holds no object";
            return nullptr;
        }
        if (contract_ != std::type_index(typeid(Contract))) {
            err = std::string("resource provides contract ") + contract_name_ +
                  " but " + typeid(Contract).name() + " was requested";
            return nullptr;
        }
        return std::static_pointer_cast<Contract>(object_);
    }

    // Runtime half: declared outputs polled every cycle and the once-only
    // reset. Absent for configuration-only resources.
    void setExecutable(ResourceExecutable executable) {
        executable_ = std::move(executable);
    }
    const std::optional<ResourceExecutable>& executable() const { return executable_; }

private:
    std::type_index                   contract_ = std::type_index(typeid(void));
    std::shared_ptr<void>             object_;
    const char*                       contract_name_ = "none";
    std::optional<ResourceExecutable> executable_;
};

} // namespace navigatr
