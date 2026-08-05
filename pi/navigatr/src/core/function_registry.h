// function_registry.h
// The one registry of registered construction functions, across every
// category: resources, sensors, preprocessors, pipeline slots. Keys are
// opaque registered names (pico_encoder_channel, linux_serial_link, noop)
// scoped by the exact function signature, so every category can register its
// own noop while a duplicate within a category still fails, retrieval with
// the wrong signature fails, and unknown keys fail.
//
// Registration happens through explicit register_* calls at startup; nothing
// relies on static initializer ordering, and the register_* aggregation
// aborts loudly on a collision. Holding the registry grants no execution
// authority: the pipeline coordinator decides which category of factory it
// retrieves and when the result may run.

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
    // False on an empty key or a duplicate name within the same signature.
    template <typename Fn>
    bool add(const FunctionKey& key, Fn fn) {
        if (key.empty()) {
            return false;
        }
        const std::type_index signature(typeid(Fn));
        std::vector<Entry>&   entries = entries_[key.value];
        for (const Entry& e : entries) {
            if (e.signature == signature) {
                return false;
            }
        }
        entries.push_back(Entry{signature, std::any(std::move(fn))});
        return true;
    }

    // Null and err on an unknown key or a signature mismatch.
    template <typename Fn>
    const Fn* find(const FunctionKey& key, std::string& err) const {
        const auto it = entries_.find(key.value);
        if (it == entries_.end()) {
            err = "unknown function key " + key.value;
            return nullptr;
        }
        const std::type_index signature(typeid(Fn));
        for (const Entry& e : it->second) {
            if (e.signature == signature) {
                return std::any_cast<Fn>(&e.fn);
            }
        }
        err = "function " + key.value + " exists but has a different signature "
              "than this slot requires";
        return nullptr;
    }

    template <typename Fn>
    bool has(const FunctionKey& key) const {
        const auto it = entries_.find(key.value);
        if (it == entries_.end()) {
            return false;
        }
        const std::type_index signature(typeid(Fn));
        for (const Entry& e : it->second) {
            if (e.signature == signature) {
                return true;
            }
        }
        return false;
    }

    bool hasAnySignature(const FunctionKey& key) const {
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

    std::map<std::string, std::vector<Entry>> entries_;
};

} // namespace navigatr
