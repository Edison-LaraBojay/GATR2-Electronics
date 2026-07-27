// sensor_catalog.h
// Initialization-time view of the configured sensors: which ids exist and
// what payload each publishes. Consumers bind here once, so a reference to
// a missing sensor or an incompatible payload dies at build, and runtime
// lookups are just a map find with a typed cast. There is no way to search
// sensors by implementation type; relationships are explicit references.

#pragma once
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

// Resolved typed reference to one sensor's results.
template <typename T>
struct TypedSensorBinding {
    SensorId id;

    const SensorRecord* record(const SensorResultsMap& results) const {
        const auto it = results.find(id);
        return it == results.end() ? nullptr : &it->second;
    }

    const StoredSensorSample* stored(const SensorResultsMap& results) const {
        const SensorRecord* rec = record(results);
        if (rec == nullptr || !rec->latest.has_value()) {
            return nullptr;
        }
        return &*rec->latest;
    }

    const T* sample(const SensorResultsMap& results) const {
        const StoredSensorSample* s = stored(results);
        return s == nullptr ? nullptr : s->payload.get<T>();
    }
};

class SensorCatalog
{
public:
    void add(SensorId id, PayloadDescriptor payload) {
        entries_.push_back(Entry{std::move(id), std::move(payload)});
    }

    const PayloadDescriptor* payloadOf(const SensorId& id) const {
        for (const Entry& e : entries_) {
            if (e.id == id) {
                return &e.payload;
            }
        }
        return nullptr;
    }

    // Fails when the id is unknown or publishes a different payload than T.
    template <typename T>
    bool bind(const SensorId& id, const std::string& who, TypedSensorBinding<T>& out,
              std::string& err) const {
        const PayloadDescriptor* payload = payloadOf(id);
        if (payload == nullptr) {
            err = who + " references sensor " + id.value + " which is not configured";
            return false;
        }
        if (!payload->matches(std::type_index(typeid(T)))) {
            err = who + " requires a different payload than sensor " + id.value +
                  " produces (" + payload->stable_name + ")";
            return false;
        }
        out.id = id;
        return true;
    }

    std::size_t size() const { return entries_.size(); }

private:
    struct Entry {
        SensorId          id;
        PayloadDescriptor payload;
    };
    std::vector<Entry> entries_;
};

} // namespace navigatr
