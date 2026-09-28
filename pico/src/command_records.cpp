// command_records.cpp

#include "command_records.h"

namespace pilink
{

CommandRecord* CommandRecords::find(uint16_t request_id, uint8_t op, uint8_t body) {
    if (request_id == 0) {
        return nullptr;
    }
    for (CommandRecord& r : slots_) {
        if (r.request_id == request_id && r.op == op && r.body == body) {
            return &r;
        }
    }
    return nullptr;
}

CommandRecord& CommandRecords::add(const CommandRecord& record) {
    uint8_t slot = 0;
    bool    same = false;
    for (uint8_t i = 0; i < kCapacity; ++i) {
        if (slots_[i].request_id == record.request_id) {
            slot = i;
            same = true;
            break;
        }
    }
    if (!same) {
        for (uint8_t i = 1; i < kCapacity; ++i) {
            if (order_[i] < order_[slot]) {
                slot = i;
            }
        }
    }
    slots_[slot] = record;
    order_[slot] = next_order_++;
    return slots_[slot];
}

void CommandRecords::clear() {
    for (uint8_t i = 0; i < kCapacity; ++i) {
        slots_[i] = CommandRecord{};
        order_[i] = 0;
    }
    next_order_ = 1;
}

} // namespace pilink
