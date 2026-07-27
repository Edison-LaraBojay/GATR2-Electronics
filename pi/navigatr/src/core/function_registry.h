// function_registry.h
// The one registry of registered construction functions, across every
// category: resources, sensors, preprocessors, pipeline slots. Keys are
// namespaced (sensor/pico_encoder_channel, perception/noop) and each entry
// remembers its exact function signature, so retrieving a key with the wrong
// signature fails loudly, as do unknown keys and duplicate registrations.
//
// Registration happens through explicit register_* calls at startup; nothing
// relies on static initializer ordering. Holding the registry grants no
// execution authority: the pipeline coordinator decides which category it is
// retrieving and when the result may run.

#pragma once
#include <any>
#include <map>
#include <string>
#include <typeindex>
#include <vector>

#include "core/ids.h"

namespace navigatr
{

class FunctionRegistry
{
public:
    // False on an empty key or a duplicate.
    template <typename Fn>
    bool add(const FunctionKey& key, Fn fn) {
        if (key.empty()) {
            return false;
        }
        Entry entry{std::type_index(typeid(Fn)), std::any(std::move(fn))};
        return entries_.emplace(key.value, std::move(entry)).second;
    }

    // Null and err on an unknown key or a signature mismatch.
    template <typename Fn>
    const Fn* find(const FunctionKey& key, std::string& err) const {
        const auto it = entries_.find(key.value);
        if (it == entries_.end()) {
            err = "unknown function key " + key.value;
            return nullptr;
        }
        if (it->second.signature != std::type_index(typeid(Fn))) {
            err = "function " + key.value + " exists but has a different signature "
                  "than this slot requires";
            return nullptr;
        }
        return std::any_cast<Fn>(&it->second.fn);
    }

    bool has(const FunctionKey& key) const {
        return entries_.find(key.value) != entries_.end();
    }

    std::vector<std::string> keys() const {
        std::vector<std::string> out;
        out.reserve(entries_.size());
        for (const auto& kv : entries_) {
            out.push_back(kv.first);
        }
        return out;
    }

private:
    struct Entry {
        std::type_index signature;
        std::any        fn;
    };

    std::map<std::string, Entry> entries_;
};

} // namespace navigatr
