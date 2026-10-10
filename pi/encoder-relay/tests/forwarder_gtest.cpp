// forwarder_gtest.cpp

#include "forwarder.h"
#include "input_drain.h"

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

TEST(Forwarder, ReplacesAPendingLineWhenNoBytesWereWritten) {
    Forwarder f;
    feed(f, raw(1));
    ASSERT_EQ(f.pending().size, encoder_record::kLineSize); // poll/write preparation
    f.wrote(0); // a nonblocking write made no progress
    feed(f, raw(2));
    feed(f, raw(3));
    EXPECT_EQ(writeAll(f), text(3));
    EXPECT_EQ(f.stats().superseded, 2u);
}

// A nonblocking source that returns no more than a real UART read's capacity.
struct BurstInput {
    std::vector<uint8_t> bytes;
    std::size_t at = 0;
    std::size_t calls = 0;

    void append(uint16_t seq) {
        const auto record = raw(seq, seq, -seq);
        bytes.insert(bytes.end(), record.begin(), record.end());
    }

    std::ptrdiff_t read(uint8_t* buffer, std::size_t capacity) {
        ++calls;
        const std::size_t count = std::min(capacity, bytes.size() - at);
        std::copy_n(bytes.data() + at, count, buffer);
        at += count;
        return static_cast<std::ptrdiff_t>(count);
    }
};

TEST(InputDrain, SendsNewestFromABurstLargerThanOneRead) {
    Forwarder f;
    feed(f, raw(0));
    ASSERT_GT(f.pending().size, 0u); // an old line was selected before polling
    BurstInput input;
    for (uint16_t seq = 1; seq <= 100; ++seq) {
        input.append(seq);
    }
    const DrainResult result = drainInput(f, [&](uint8_t* data, std::size_t capacity) {
        return input.read(data, capacity);
    });
    EXPECT_GT(input.calls, 2u); // several data reads, then the empty read
    EXPECT_EQ(input.at, input.bytes.size());
    EXPECT_FALSE(result.limited);
    EXPECT_EQ(result.records, 100u);
    EXPECT_EQ(writeAll(f), text(100, 100, -100));
    EXPECT_EQ(f.stats().superseded, 100u);
}

TEST(InputDrain, RetainsPartialTailUntilTheNextPass) {
    Forwarder f;
    BurstInput input;
    for (uint16_t seq = 1; seq <= 40; ++seq) {
        input.append(seq);
    }
    const auto tail = raw(41, -12, 80);
    input.bytes.insert(input.bytes.end(), tail.begin(), tail.begin() + 9);
    const auto read = [&](uint8_t* data, std::size_t capacity) {
        return input.read(data, capacity);
    };
    EXPECT_EQ(drainInput(f, read).records, 40u);
    EXPECT_EQ(writeAll(f), text(40, 40, -40));
    input.bytes.insert(input.bytes.end(), tail.begin() + 9, tail.end());
    EXPECT_EQ(drainInput(f, read).records, 1u);
    EXPECT_EQ(writeAll(f), text(41, -12, 80));
}

TEST(InputDrain, FinishesStartedLineThenSendsNewestFromTheBurst) {
    Forwarder f;
    feed(f, raw(0));
    const Pending p = f.pending();
    std::string output(p.data, 10);
    f.wrote(10);
    BurstInput input;
    for (uint16_t seq = 1; seq <= 100; ++seq) {
        input.append(seq);
    }
    drainInput(f, [&](uint8_t* data, std::size_t capacity) {
        return input.read(data, capacity);
    });
    output += writeAll(f);
    EXPECT_EQ(output, text(0) + text(100, 100, -100));
}

TEST(InputDrain, ContinuousInputYieldsAtTheReadLimit) {
    Forwarder f;
    std::size_t calls = 0;
    const DrainResult result = drainInput(f, [&](uint8_t* data, std::size_t capacity) {
        const auto record = raw(static_cast<uint16_t>(++calls));
        EXPECT_GE(capacity, record.size());
        std::copy(record.begin(), record.end(), data);
        return static_cast<std::ptrdiff_t>(record.size());
    });
    EXPECT_TRUE(result.limited);
    EXPECT_EQ(calls, kInputReadLimit);
    EXPECT_EQ(result.records, kInputReadLimit);
    EXPECT_EQ(writeAll(f), text(static_cast<uint16_t>(kInputReadLimit)));
}

TEST(InputDrain, RepeatedInterruptionsAreAlsoBounded) {
    Forwarder f;
    std::size_t calls = 0;
    const DrainResult result = drainInput(f, [&](uint8_t*, std::size_t) {
        ++calls;
        return std::ptrdiff_t{-1};
    });
    EXPECT_TRUE(result.limited);
    EXPECT_EQ(calls, kInputReadLimit);
    EXPECT_EQ(result.records, 0u);
    EXPECT_EQ(f.pending().size, 0u);
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
