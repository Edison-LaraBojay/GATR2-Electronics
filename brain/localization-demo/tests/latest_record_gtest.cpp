// latest_record_gtest.cpp

#include "latest_record.h"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

namespace localization
{
namespace
{

// Every field derived from one number, so a torn copy is detectable.
ReceivedRecord make(uint32_t n) {
    ReceivedRecord r;
    r.number          = n;
    r.arrived_ms      = n * 3;
    r.record.run_id   = static_cast<uint16_t>(n * 7);
    r.record.seq      = static_cast<uint16_t>(n);
    r.record.channel0 = static_cast<int32_t>(n * 11);
    r.record.channel1 = -static_cast<int32_t>(n * 13);
    return r;
}

bool consistent(const ReceivedRecord& r) {
    const uint32_t n = r.number;
    return r.arrived_ms == n * 3 && r.record.run_id == static_cast<uint16_t>(n * 7) &&
           r.record.seq == static_cast<uint16_t>(n) &&
           r.record.channel0 == static_cast<int32_t>(n * 11) &&
           r.record.channel1 == -static_cast<int32_t>(n * 13);
}

TEST(LatestRecord, EmptyThenNewest) {
    LatestRecord   slot;
    ReceivedRecord out;
    EXPECT_FALSE(slot.load(out));
    slot.store(make(1));
    slot.store(make(2));
    ASSERT_TRUE(slot.load(out));
    EXPECT_EQ(out.number, 2u);
    EXPECT_TRUE(consistent(out));
}

TEST(LatestRecord, ReaderNeverSeesATornRecord) {
    LatestRecord      slot;
    std::atomic<bool> done{false};
    std::thread       writer([&] {
        for (uint32_t n = 1; n <= 200000; ++n) {
            slot.store(make(n));
        }
        done = true;
    });
    uint32_t last = 0, reads = 0;
    bool     ok   = true;
    while (!done) {
        ReceivedRecord out;
        if (slot.load(out)) {
            ok = ok && consistent(out) && out.number >= last;
            last = out.number;
            ++reads;
        }
    }
    writer.join();
    EXPECT_TRUE(ok);
    EXPECT_GT(reads, 0u);
}

} // namespace
} // namespace localization
