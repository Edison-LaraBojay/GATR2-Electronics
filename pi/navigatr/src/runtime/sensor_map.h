// sensor_map.h
// SensorId to configured executable sensor. Holds the executables, not the
// results; SensorResultsMap holds the latest standardized records. Execution
// order is deterministic declaration order backed by an id index; nothing
// depends on hash iteration order, and nothing searches sensors by
// implementation type.

#pragma once
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "contracts/sensor.h"
#include "core/ids.h"

namespace navigatr
{

class SensorMap
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

} // namespace navigatr
