// latest_record.h
// Newest received record, one writer task and any number of readers, no lock.
// A seqlock over atomic words: a reader that overlaps a write retries, so a
// task deleted mid read or mid write never leaves anything locked. Portable.

#pragma once
#include <atomic>
#include <cstdint>

#include "encoder_record.h"

namespace localization
{

struct ReceivedRecord {
    encoder_record::Record record;
    uint32_t               arrived_ms = 0; // Brain time when decoded
    uint32_t               number     = 0; // 1, 2, ... per decoded record; 0 none yet
};

class LatestRecord {
  public:
    // Writer task only.
    void store(const ReceivedRecord& r) {
        const uint32_t v = version_.load(std::memory_order_relaxed);
        version_.store(v + 1, std::memory_order_relaxed); // odd: writing
        std::atomic_thread_fence(std::memory_order_release);
        const uint32_t ids = static_cast<uint32_t>(r.record.run_id) |
                             (static_cast<uint32_t>(r.record.seq) << 16);
        ids_.store(ids, std::memory_order_relaxed);
        channel0_.store(static_cast<uint32_t>(r.record.channel0), std::memory_order_relaxed);
        channel1_.store(static_cast<uint32_t>(r.record.channel1), std::memory_order_relaxed);
        arrived_ms_.store(r.arrived_ms, std::memory_order_relaxed);
        number_.store(r.number, std::memory_order_relaxed);
        version_.store(v + 2, std::memory_order_release);
    }

    // False when nothing was stored yet or every try overlapped a write.
    bool load(ReceivedRecord& out) const {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const uint32_t before = version_.load(std::memory_order_acquire);
            if (before & 1u) {
                continue;
            }
            ReceivedRecord r;
            const uint32_t ids = ids_.load(std::memory_order_relaxed);
            r.record.run_id    = static_cast<uint16_t>(ids);
            r.record.seq       = static_cast<uint16_t>(ids >> 16);
            r.record.channel0  = static_cast<int32_t>(channel0_.load(std::memory_order_relaxed));
            r.record.channel1  = static_cast<int32_t>(channel1_.load(std::memory_order_relaxed));
            r.arrived_ms       = arrived_ms_.load(std::memory_order_relaxed);
            r.number           = number_.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (version_.load(std::memory_order_relaxed) == before) {
                out = r;
                return r.number != 0;
            }
        }
        return false;
    }

  private:
    std::atomic<uint32_t> version_{0};
    std::atomic<uint32_t> ids_{0};
    std::atomic<uint32_t> channel0_{0};
    std::atomic<uint32_t> channel1_{0};
    std::atomic<uint32_t> arrived_ms_{0};
    std::atomic<uint32_t> number_{0};
};

} // namespace localization
