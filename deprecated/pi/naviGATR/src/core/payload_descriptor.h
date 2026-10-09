// payload_descriptor.h
// What a producer publishes: the in-process C++ type for validation plus a
// stable name for error messages, documentation, and any future replay or
// wire format. Compiler typeid names are never serialized.

#pragma once
#include <string>
#include <typeindex>

namespace navigatr
{

struct PayloadDescriptor {
    std::type_index type = std::type_index(typeid(void));
    std::string     stable_name;

    template <typename T>
    static PayloadDescriptor of(std::string name) {
        PayloadDescriptor d;
        d.type        = std::type_index(typeid(T));
        d.stable_name = std::move(name);
        return d;
    }

    bool matches(const std::type_index& other) const { return type == other; }
};

} // namespace navigatr
