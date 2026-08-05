// preprocessing_map.h
// PreprocessorId to configured executable preprocessor, owned by the
// configured collection. Deterministic declaration order backed by an id
// index; artifact outputs are what downstream references, these ids exist
// for diagnostics, configuration identity, and reset targeting.

#pragma once
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "contracts/preprocessing.h"
#include "core/ids.h"

namespace navigatr
{

class PreprocessingMap
{
public:
    struct Entry {
        PreprocessorId                          id;
        std::unique_ptr<PreprocessorExecutable> executable;
    };

    // False on a duplicate id.
    bool add(PreprocessorId id, std::unique_ptr<PreprocessorExecutable> executable) {
        if (index_by_id_.find(id) != index_by_id_.end()) {
            return false;
        }
        index_by_id_.emplace(id, execution_order_.size());
        execution_order_.push_back(Entry{std::move(id), std::move(executable)});
        return true;
    }

    const Entry* find(const PreprocessorId& id) const {
        const auto it = index_by_id_.find(id);
        return it == index_by_id_.end() ? nullptr : &execution_order_[it->second];
    }

    std::vector<Entry>&       executionOrder() { return execution_order_; }
    const std::vector<Entry>& executionOrder() const { return execution_order_; }

    std::size_t size() const { return execution_order_.size(); }

private:
    std::vector<Entry>                                              execution_order_;
    std::unordered_map<PreprocessorId, std::size_t, PreprocessorId::Hash> index_by_id_;
};

} // namespace navigatr
