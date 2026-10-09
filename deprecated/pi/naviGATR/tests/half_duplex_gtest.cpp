// half_duplex_gtest.cpp
// RS-485 transmit sequencing over a fake port on a fake clock: driver enable
// order, partial writes, transmitter empty before release, release and
// discard on every failure, transmit windows and input pending. Also the
// transmit window behavior of MemoryLink and the SerialLink defaults.

#include <gtest/gtest.h>

#include <algorithm>
#include <climits>
#include <cstdint>
#include <string>
#include <vector>

#include "impl/resources/serial_links.h"
#include "transport/half_duplex.h"

using namespace navigatr;

namespace
{

constexpr int64_t kStartUs = 1'000'000;

// The transmitter shifts out one character per character time after
// writeSome accepts it. setDriver, inputPending, writeSome, waitWritable and
// discardOutput are logged in order.
class FakePort : public HalfDuplexPort
{
public:
    explicit FakePort(int baud = 115200) : char_us_(characterTimeUs(baud)) {}

    // Knobs.
    std::vector<int> accept;                       // per writeSome call: 0 full, -1 error
    int              accept_after     = INT_MAX;   // after the scripted calls
    bool             input_pending    = false;
    bool             enable_fails     = false;     // setDriver(true) fails
    bool             never_empty      = false;
    bool             status_fails     = false;
    int64_t          release_delay_us = 0;         // preemption before the release lands

    // Observations.
    int64_t                  now_us = kStartUs;
    bool                     driver = false;
    std::vector<std::string> events;
    std::vector<uint8_t>     wire;
    std::size_t              bytes_without_driver  = 0;
    bool                     released_before_empty = false;
    int64_t                  driver_on_us          = -1;
    int64_t                  driver_off_us         = -1;

    int64_t txDoneUs() const { return tx_done_us_; }

    int64_t nowUs() override { return now_us; }

    void sleepUs(int64_t us) override {
        if (us > 0) {
            now_us += us;
        }
    }

    bool setDriver(bool on) override {
        events.push_back(on ? "on" : "off");
        if (on) {
            if (enable_fails) {
                return false;
            }
            driver       = true;
            driver_on_us = now_us;
            return true;
        }
        now_us += release_delay_us;
        if (driver && now_us < tx_done_us_) {
            released_before_empty = true;
        }
        driver        = false;
        driver_off_us = now_us;
        return true;
    }

    bool inputPending() override {
        events.push_back("input?");
        return input_pending;
    }

    int writeSome(const uint8_t* data, std::size_t size) override {
        int n = call_ < accept.size() ? accept[call_] : accept_after;
        ++call_;
        if (n > 0 && static_cast<std::size_t>(n) > size) {
            n = static_cast<int>(size);
        }
        events.push_back("write " + std::to_string(n));
        if (n <= 0) {
            return n;
        }
        if (!driver) {
            bytes_without_driver += static_cast<std::size_t>(n);
        }
        wire.insert(wire.end(), data, data + n);
        tx_done_us_ = std::max(tx_done_us_, now_us) + n * char_us_;
        return n;
    }

    bool waitWritable(int64_t timeout_us) override {
        events.push_back("wait");
        now_us += std::max<int64_t>(0, std::min(timeout_us, char_us_));
        return true;
    }

    int transmitterEmpty() override {
        if (status_fails) {
            return -1;
        }
        if (never_empty) {
            return 0;
        }
        return now_us >= tx_done_us_ ? 1 : 0;
    }

    void discardOutput() override {
        events.push_back("discard");
        tx_done_us_ = now_us;
    }

private:
    int64_t     char_us_;
    int64_t     tx_done_us_ = 0;
    std::size_t call_       = 0;
};

std::vector<uint8_t> frame(std::size_t size) {
    std::vector<uint8_t> bytes(size);
    for (std::size_t i = 0; i < size; ++i) {
        bytes[i] = static_cast<uint8_t>(0xA0 + i);
    }
    return bytes;
}

ByteSpan span(const std::vector<uint8_t>& bytes) { return ByteSpan{bytes.data(), bytes.size()}; }

using Events = std::vector<std::string>;

} // namespace

TEST(HalfDuplex, FrameBudgetAndCharacterTime) {
    EXPECT_EQ(characterTimeUs(115200), 87);
    EXPECT_EQ(characterTimeUs(0), 0);
    // 59 * 10 / 115200 s = 5121.5 us, rounded up, plus 2 ms
    EXPECT_EQ(transmitBudgetUs(59, 115200, 2000), 7122);
    EXPECT_EQ(transmitBudgetUs(0, 115200, 2000), 2000);
    // the largest v4 frame, 128 bytes: 11111.1 us rounded up, plus 2 ms
    EXPECT_EQ(transmitBudgetUs(128, 115200, 2000), 13112);
}

TEST(HalfDuplex, LargestFrameHoldsTheDriverUntilItsLastStopBit) {
    FakePort               port;
    const HalfDuplexTiming timing;
    const auto             bytes = frame(128);

    const SerialWriteResult r = transmitHalfDuplex(port, span(bytes), TransmitWindow{}, timing);

    EXPECT_TRUE(r.ok);
    EXPECT_FALSE(r.late_release);
    EXPECT_EQ(port.wire, bytes);
    EXPECT_FALSE(port.released_before_empty);
    EXPECT_GE(port.driver_off_us, port.txDoneUs() + timing.post_guard_us);
    EXPECT_LE(port.driver_off_us - port.driver_on_us,
              transmitBudgetUs(bytes.size(), timing.baud, timing.margin_us) +
                  timing.post_guard_us);
}

TEST(HalfDuplex, DriverOnBeforeFirstByteAndOffAfterTransmitterEmpty) {
    FakePort               port;
    const HalfDuplexTiming timing;
    const auto             bytes = frame(10);

    const SerialWriteResult r = transmitHalfDuplex(port, span(bytes), TransmitWindow{}, timing);

    EXPECT_TRUE(r.ok);
    EXPECT_EQ(r.error, nullptr);
    EXPECT_FALSE(r.expired);
    EXPECT_FALSE(r.input_pending);
    EXPECT_FALSE(r.late_release);
    EXPECT_EQ(port.events, (Events{"input?", "on", "write 10", "off"}));
    EXPECT_EQ(port.wire, bytes);
    EXPECT_EQ(port.bytes_without_driver, 0u);
    EXPECT_FALSE(port.driver);
    EXPECT_FALSE(port.released_before_empty);
    // held for the post guard after the last stop bit
    EXPECT_GE(port.driver_off_us, port.txDoneUs() + timing.post_guard_us);
}

TEST(HalfDuplex, PartialWritesWaitForSpaceAndSendEveryByte) {
    FakePort port;
    port.accept      = {3, 0, 4, 0};
    const auto bytes = frame(12);

    const SerialWriteResult r =
        transmitHalfDuplex(port, span(bytes), TransmitWindow{}, HalfDuplexTiming{});

    EXPECT_TRUE(r.ok);
    EXPECT_EQ(port.events, (Events{"input?", "on", "write 3", "write 0", "wait", "write 4",
                                   "write 0", "wait", "write 5", "off"}));
    EXPECT_EQ(port.wire, bytes);
    EXPECT_EQ(port.bytes_without_driver, 0u);
    EXPECT_FALSE(port.released_before_empty);
}

TEST(HalfDuplex, WriteErrorDiscardsAndReleases) {
    FakePort port;
    port.accept      = {3, -1};
    const auto bytes = frame(10);

    const SerialWriteResult r =
        transmitHalfDuplex(port, span(bytes), TransmitWindow{}, HalfDuplexTiming{});

    EXPECT_FALSE(r.ok);
    ASSERT_NE(r.error, nullptr);
    EXPECT_FALSE(r.expired);
    EXPECT_EQ(port.events, (Events{"input?", "on", "write 3", "write -1", "discard", "off"}));
    EXPECT_FALSE(port.driver);
}

TEST(HalfDuplex, WriteTimeoutDiscardsAndReleases) {
    FakePort               port;
    const HalfDuplexTiming timing;
    const auto             bytes = frame(10);

    port.accept_after = 0;   // output buffer never frees up

    const SerialWriteResult r = transmitHalfDuplex(port, span(bytes), TransmitWindow{}, timing);

    EXPECT_FALSE(r.ok);
    ASSERT_NE(r.error, nullptr);
    EXPECT_EQ(std::string(r.error), "write timed out");
    ASSERT_GE(port.events.size(), 2u);
    EXPECT_EQ(port.events[port.events.size() - 2], "discard");
    EXPECT_EQ(port.events.back(), "off");
    EXPECT_FALSE(port.driver);
    const int64_t deadline =
        port.driver_on_us + transmitBudgetUs(bytes.size(), timing.baud, timing.margin_us);
    EXPECT_GE(port.driver_off_us, deadline);
    EXPECT_LE(port.driver_off_us, deadline + characterTimeUs(timing.baud));
}

TEST(HalfDuplex, NeverEmptyTransmitterReleasesAtTheDeadline) {
    FakePort               port;
    const HalfDuplexTiming timing;
    const auto             bytes = frame(10);

    port.never_empty = true;

    const SerialWriteResult r = transmitHalfDuplex(port, span(bytes), TransmitWindow{}, timing);

    EXPECT_FALSE(r.ok);
    ASSERT_NE(r.error, nullptr);
    EXPECT_EQ(std::string(r.error), "transmitter never empty");
    EXPECT_EQ(port.events, (Events{"input?", "on", "write 10", "discard", "off"}));
    EXPECT_FALSE(port.driver);
    const int64_t deadline =
        port.driver_on_us + transmitBudgetUs(bytes.size(), timing.baud, timing.margin_us);
    EXPECT_EQ(port.driver_off_us, deadline);
    EXPECT_FALSE(r.late_release);
}

TEST(HalfDuplex, TransmitterStatusErrorDiscardsAndReleases) {
    FakePort port;
    port.status_fails = true;
    const auto bytes  = frame(4);

    const SerialWriteResult r =
        transmitHalfDuplex(port, span(bytes), TransmitWindow{}, HalfDuplexTiming{});

    EXPECT_FALSE(r.ok);
    EXPECT_EQ(port.events, (Events{"input?", "on", "write 4", "discard", "off"}));
    EXPECT_FALSE(port.driver);
}

TEST(HalfDuplex, DriverEnableFailureSendsNothingAndStillReleases) {
    FakePort port;
    port.enable_fails = true;
    const auto bytes  = frame(4);

    const SerialWriteResult r =
        transmitHalfDuplex(port, span(bytes), TransmitWindow{}, HalfDuplexTiming{});

    EXPECT_FALSE(r.ok);
    ASSERT_NE(r.error, nullptr);
    EXPECT_EQ(port.events, (Events{"input?", "on", "discard", "off"}));
    EXPECT_TRUE(port.wire.empty());
}

TEST(HalfDuplex, ExpiredWindowNeverDrivesTheBus) {
    FakePort   port;
    const auto bytes = frame(4);

    TransmitWindow window;
    window.not_before_us = kStartUs - 5000;
    window.deadline_us   = kStartUs - 1;
    const SerialWriteResult r = transmitHalfDuplex(port, span(bytes), window, HalfDuplexTiming{});

    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(r.expired);
    EXPECT_TRUE(port.events.empty());
    EXPECT_TRUE(port.wire.empty());
}

TEST(HalfDuplex, WindowOpeningAfterItsDeadlineIsExpiredWithoutWaiting) {
    FakePort   port;
    const auto bytes = frame(4);

    TransmitWindow window;
    window.not_before_us = kStartUs + 2000;
    window.deadline_us   = kStartUs + 1000;
    const SerialWriteResult r = transmitHalfDuplex(port, span(bytes), window, HalfDuplexTiming{});

    EXPECT_TRUE(r.expired);
    EXPECT_TRUE(port.events.empty());
    EXPECT_EQ(port.now_us, kStartUs);
}

TEST(HalfDuplex, WaitsForNotBeforeThenSends) {
    FakePort   port;
    const auto bytes = frame(4);

    TransmitWindow window;
    window.not_before_us = kStartUs + 500;
    window.deadline_us   = kStartUs + 40'000;
    const SerialWriteResult r = transmitHalfDuplex(port, span(bytes), window, HalfDuplexTiming{});

    EXPECT_TRUE(r.ok);
    EXPECT_EQ(port.driver_on_us, kStartUs + 500);
    EXPECT_EQ(port.wire, bytes);
}

TEST(HalfDuplex, InputPendingNeverDrivesTheBus) {
    FakePort port;
    port.input_pending = true;
    const auto bytes   = frame(4);

    const SerialWriteResult r =
        transmitHalfDuplex(port, span(bytes), TransmitWindow{}, HalfDuplexTiming{});

    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(r.input_pending);
    EXPECT_FALSE(r.expired);
    EXPECT_EQ(port.events, (Events{"input?"}));
    EXPECT_TRUE(port.wire.empty());
}

TEST(HalfDuplex, NextWriteAfterAnErrorIsSequencedNormally) {
    FakePort               port;
    const HalfDuplexTiming timing;
    const auto             first  = frame(10);
    const auto             second = frame(6);

    port.accept = {3, -1};
    EXPECT_FALSE(transmitHalfDuplex(port, span(first), TransmitWindow{}, timing).ok);
    EXPECT_FALSE(port.driver);

    port.events.clear();
    port.wire.clear();
    const SerialWriteResult r = transmitHalfDuplex(port, span(second), TransmitWindow{}, timing);

    EXPECT_TRUE(r.ok);
    EXPECT_EQ(port.events, (Events{"input?", "on", "write 6", "off"}));
    EXPECT_EQ(port.wire, second);
    EXPECT_EQ(port.bytes_without_driver, 0u);
    EXPECT_FALSE(port.released_before_empty);
    EXPECT_GE(port.driver_off_us, port.txDoneUs() + timing.post_guard_us);
}

TEST(HalfDuplex, LateReleaseIsReported) {
    FakePort port;
    port.release_delay_us = 5000;
    const auto bytes      = frame(10);

    const SerialWriteResult r =
        transmitHalfDuplex(port, span(bytes), TransmitWindow{}, HalfDuplexTiming{});

    EXPECT_TRUE(r.ok);
    EXPECT_TRUE(r.late_release);
}

TEST(HalfDuplex, HalfDuplexLinkWithoutItsGpioFailsEveryWrite) {
    LinuxSerialLink link;
    std::string     err;
    EXPECT_FALSE(link.enableHalfDuplex(-1, HalfDuplexTiming{}, err));
    EXPECT_FALSE(err.empty());
    EXPECT_TRUE(link.halfDuplex());

    const auto        bytes = frame(4);
    SerialWriteResult r     = link.write(span(bytes));
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.error, nullptr);
    r = link.write(span(bytes), TransmitWindow{});
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.expired);
}

TEST(MemoryLinkWindow, InjectedClockAndSteadyDefault) {
    MemoryLink link;
    int64_t    now = 42;
    link.setClock([&now] { return now; });
    EXPECT_EQ(link.nowUs(), 42);
    now = 1000;
    EXPECT_EQ(link.nowUs(), 1000);

    link.setClock({});
    const int64_t before = steadyNowUs();
    const int64_t t      = link.nowUs();
    EXPECT_GE(t, before);
    EXPECT_LE(t, steadyNowUs());
}

TEST(MemoryLinkWindow, WritesInsideTheWindowOnly) {
    MemoryLink link;
    int64_t    now = 3000;
    link.setClock([&now] { return now; });
    const auto bytes = frame(3);

    TransmitWindow window;
    window.not_before_us = 2000;
    window.deadline_us   = 4000;
    EXPECT_TRUE(link.write(span(bytes), window).ok);
    EXPECT_EQ(link.output().takeAll(), bytes);

    // never waits: a start before not_before_us is sent at once
    now = 1000;
    EXPECT_TRUE(link.write(span(bytes), window).ok);
    EXPECT_EQ(link.output().takeAll(), bytes);

    now                       = 4001;
    const SerialWriteResult r = link.write(span(bytes), window);
    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(r.expired);
    EXPECT_EQ(link.output().size(), 0u);

    // the plain write ignores windows
    EXPECT_TRUE(link.write(span(bytes)).ok);
    EXPECT_EQ(link.output().takeAll(), bytes);
}

TEST(MemoryLinkWindow, InputPendingRefusesWindowedWrite) {
    MemoryLink link;
    link.setClock([] { return int64_t{0}; });
    link.input().feed({0x01});
    EXPECT_TRUE(link.inputPending());
    const auto bytes = frame(3);

    const SerialWriteResult r = link.write(span(bytes), TransmitWindow{});
    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(r.input_pending);
    EXPECT_EQ(link.output().size(), 0u);

    uint8_t buf[4];
    EXPECT_EQ(link.readAvailable(MutableByteSpan{buf, sizeof(buf)}).bytes, 1u);
    EXPECT_FALSE(link.inputPending());
    EXPECT_TRUE(link.write(span(bytes), TransmitWindow{}).ok);
    EXPECT_EQ(link.output().takeAll(), bytes);
}

TEST(SerialLinkDefaults, WindowedWriteForwardsToPlainWrite) {
    // implements only the pure virtuals, like links written before windows
    class PlainLink : public SerialLink
    {
    public:
        SerialReadResult  readAvailable(MutableByteSpan) override { return SerialReadResult{}; }
        SerialWriteResult write(ByteSpan source) override {
            written += source.size;
            return SerialWriteResult{true};
        }
        std::size_t written = 0;
    };

    PlainLink     plain;
    SerialLink&   link   = plain;
    const auto    bytes  = frame(5);
    const int64_t before = steadyNowUs();

    TransmitWindow missed;
    missed.deadline_us = link.nowUs() - 1'000'000;
    EXPECT_TRUE(link.write(span(bytes), missed).ok);
    EXPECT_EQ(plain.written, 5u);
    EXPECT_FALSE(link.inputPending());
    EXPECT_GE(link.nowUs(), before);
}
