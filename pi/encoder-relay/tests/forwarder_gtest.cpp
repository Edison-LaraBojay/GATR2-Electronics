// forwarder_gtest.cpp

#include "forwarder.h"

#include <gtest/gtest.h>

#include <algorithm>

#include <string>
#include <vector>

namespace relay
{
namespace
{

using encoder_record::Record;

std::vector<uint8_t> raw(uint16_t seq, int32_t ch0 = 0, int32_t ch1 = 0) {
    Record r;
    r.run_id   = 0x1234;
    r.seq      = seq;
    r.channel0 = ch0;
    r.channel1 = ch1;
    const encoder_record::RecordBytes b = encoder_record::encode(r);
    return {b.begin(), b.end()};
}

std::string text(uint16_t seq, int32_t ch0 = 0, int32_t ch1 = 0) {
    const std::vector<uint8_t>       bytes = raw(seq, ch0, ch1);
    encoder_record::RecordBytes      b{};
    std::copy(bytes.begin(), bytes.end(), b.begin());
    const encoder_record::Line line = encoder_record::encodeLine(b);
    return {line.begin(), line.end()};
}

void feed(Forwarder& f, const std::vector<uint8_t>& bytes) {
    f.receive(bytes.data(), bytes.size());
}

std::string writeAll(Forwarder& f) {
    std::string out;
    for (Pending p = f.pending(); p.size > 0; p = f.pending()) {
        out.append(p.data, p.size);
        f.wrote(p.size);
    }
    return out;
}

TEST(Forwarder, ForwardsTheSameRecordAsALine) {
    Forwarder f;
    feed(f, raw(1, -5, 77));
    EXPECT_EQ(writeAll(f), text(1, -5, 77));
    EXPECT_EQ(f.stats().lines, 1u);
    EXPECT_EQ(f.pending().size, 0u);
}

TEST(Forwarder, KeepsOnlyTheNewestUnsentRecord) {
    Forwarder f;
    std::vector<uint8_t> burst;
    for (uint16_t s = 0; s < 50; ++s) {
        const auto r = raw(s, s, s);
        burst.insert(burst.end(), r.begin(), r.end());
    }
    feed(f, burst);
    EXPECT_EQ(writeAll(f), text(49, 49, 49));
    EXPECT_EQ(f.stats().records, 50u);
    EXPECT_EQ(f.stats().superseded, 49u);
}

TEST(Forwarder, FinishesAPartlyWrittenLineBeforeTheNext) {
    Forwarder f;
    feed(f, raw(1));
    Pending p = f.pending();
    ASSERT_EQ(p.size, encoder_record::kLineSize);
    std::string out(p.data, 10);
    f.wrote(10);
    feed(f, raw(2)); // arrives mid line
    feed(f, raw(3));
    out += writeAll(f);
    EXPECT_EQ(out, text(1) + text(3));
    EXPECT_EQ(f.stats().superseded, 1u);
}

TEST(Forwarder, DropPendingAfterReconnect) {
    Forwarder f;
    feed(f, raw(1));
    f.pending();
    f.wrote(5);
    feed(f, raw(2));
    f.dropPending();
    EXPECT_EQ(f.pending().size, 0u);
    EXPECT_EQ(f.stats().discarded, 2u);
    feed(f, raw(3));
    EXPECT_EQ(writeAll(f), text(3));
}

TEST(Forwarder, ResyncsAfterGarbageAndUartReopen) {
    Forwarder f;
    std::vector<uint8_t> bytes = {0x00, 0xE7, 0x13};
    const auto partial         = raw(1);
    bytes.insert(bytes.end(), partial.begin(), partial.begin() + 7);
    feed(f, bytes);
    f.resetInput(); // UART reopened mid record
    feed(f, raw(2));
    EXPECT_EQ(writeAll(f), text(2));
    EXPECT_EQ(f.stats().records, 1u);
}

} // namespace
} // namespace relay
