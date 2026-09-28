// command_records.h
// The last Pi commands the Pico ran, keyed by (request_id, op, body), so a
// resent command is answered from its record and never run again.
// No hardware dependencies.

#pragma once
#include <stdint.h>

namespace pilink
{

struct CommandRecord {
    uint16_t request_id = 0; // 0 = empty slot
    uint8_t  op         = 0;
    uint8_t  body       = 0; // the op's body byte, 0 when it has none
    uint8_t  status     = 0; // gatr2::PicoCommandStatus
    uint8_t  detail     = 0; // gatr2::PicoCommandDetail
};

class CommandRecords {
  public:
    static constexpr uint8_t kCapacity = 4;

    // The record of this exact command, or nullptr.
    CommandRecord* find(uint16_t request_id, uint8_t op, uint8_t body);

    // Stores a record as the newest. A record with the same request id is
    // replaced, else an empty slot, else the oldest.
    CommandRecord& add(const CommandRecord& record);

    // Slot i, 0 <= i < kCapacity; empty slots have request_id 0.
    CommandRecord& at(uint8_t i) { return slots_[i]; }

    void clear();

  private:
    CommandRecord slots_[kCapacity] = {};
    uint32_t      order_[kCapacity] = {}; // insertion order, larger is newer
    uint32_t      next_order_       = 1;
};

} // namespace pilink
