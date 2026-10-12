// pi_link_gtest.cpp

#include <gtest/gtest.h>

#include <set>

#include "frame_codec.h"
#include "frames.h"
#include "pi_link.h"

using namespace pilink;

namespace
{

constexpr uint32_t kTick   = 20000;
constexpr uint32_t kPeriod = 200000;
constexpr uint32_t kGap    = kTick;
constexpr uint32_t kGuard  = 3257;

} // namespace

TEST(PiLink, BootIdIsNeverZero) {
    for (uint32_t e = 0; e < 200000; ++e) {
        EXPECT_NE(makeBootId(e, 0), 0) << e;
        EXPECT_NE(makeBootId(0, e), 0) << e;
    }
}

TEST(PiLink, BootIdSpreadsOverTheRange) {
    std::set<uint16_t> ids;
    for (uint32_t e = 0; e < 1000; ++e) {
        ids.insert(makeBootId(e * 7919u, 1500000u + e));
    }
    EXPECT_GT(ids.size(), 980u);
}

TEST(PiLink, BootIdDependsOnEntropyAndTime) {
    EXPECT_NE(makeBootId(1, 1000), makeBootId(2, 1000));
    EXPECT_NE(makeBootId(1, 1000), makeBootId(1, 1001));
    EXPECT_NE(makeBootId(1, 1000), makeBootId(1, 1000 + (1ull << 32)));
}

TEST(PiLink, AirtimeAtTheLinkBaud) {
    EXPECT_EQ(airtimeUs(26, 115200), 2257u);
    EXPECT_EQ(airtimeUs(31, 115200), 2691u);
    EXPECT_EQ(airtimeUs(0, 115200), 0u);
}

TEST(PiLink, FramesFitTheTxFifo) {
    const uint16_t sensor = translagatr::sensorV2FrameLen(translagatr::kSensorEnc0 | translagatr::kSensorEnc1 |
                                                    translagatr::kSensorEnc2 | translagatr::kSensorGyroZ);
    EXPECT_EQ(sensor, 31);
    EXPECT_LE(sensor, 32);
    EXPECT_LE(translagatr::kPicoStatusLen + translagatr::kLinkEnvelopeLen, 32);
}

TEST(PiLink, FirstStatusGoesAtTheFirstWindow) {
    StatusSchedule s(kPeriod, kGap, kGuard);
    EXPECT_TRUE(s.ready(1000, kTick, true));
}

TEST(PiLink, StatusWaitsForAnEmptyFifo) {
    StatusSchedule s(kPeriod, kGap, kGuard);
    EXPECT_FALSE(s.ready(1000, kTick, false));
}

TEST(PiLink, StatusNeverCrowdsTheNextSensorTick) {
    StatusSchedule s(kPeriod, kGap, kGuard);
    EXPECT_TRUE(s.ready(kTick - kGuard, kTick, true));
    EXPECT_FALSE(s.ready(kTick - kGuard + 1, kTick, true));
    EXPECT_FALSE(s.ready(kTick, kTick, true));
    EXPECT_FALSE(s.ready(kTick + 500, kTick, true)); // tick overdue
}

TEST(PiLink, StatusGuardHoldsAcrossTheClockWrap) {
    StatusSchedule s(kPeriod, kGap, kGuard);
    const uint32_t next = 1000; // after the wrap
    EXPECT_TRUE(s.ready(next - kGuard, next, true));
    EXPECT_FALSE(s.ready(next - kGuard + 1, next, true));
}

TEST(PiLink, PeriodicStatusEvery200ms) {
    StatusSchedule s(kPeriod, kGap, kGuard);
    s.sent(0);
    EXPECT_FALSE(s.ready(kPeriod - 1, kPeriod + kTick, true));
    EXPECT_TRUE(s.ready(kPeriod, kPeriod + kTick, true));
}

TEST(PiLink, RequestedStatusGoesWithinOneTick) {
    StatusSchedule s(kPeriod, kGap, kGuard);
    s.sent(0);
    s.request();
    EXPECT_FALSE(s.ready(kGap - 1, 10 * kTick, true));
    EXPECT_TRUE(s.ready(kGap, 10 * kTick, true));
    s.sent(kGap);
    EXPECT_FALSE(s.ready(2 * kGap, 10 * kTick, true));
}

TEST(PiLink, AStatusEveryPassWouldNeverDelayASensorFrame) {
    // Simulated 50 Hz ticks with the status schedule filling the idle time:
    // at every tick the FIFO must be empty.
    const uint32_t byte_us   = 87;
    const uint32_t sensor_us = 31 * byte_us;
    const uint32_t status_us = 26 * byte_us;
    StatusSchedule s(kPeriod, 0, airtimeUs(26, 115200) + 1000);

    uint32_t fifo_empty_at = 0;
    uint32_t next_tick     = kTick;
    int      statuses      = 0;
    for (uint32_t now = 0; now < 2000000; now += 50) {
        if (static_cast<int32_t>(now - next_tick) >= 0) {
            ASSERT_GE(now, fifo_empty_at) << "sensor frame delayed at " << now;
            fifo_empty_at = now + sensor_us;
            next_tick += kTick;
            continue;
        }
        s.request();
        if (s.ready(now, next_tick, now >= fifo_empty_at)) {
            fifo_empty_at = now + status_us;
            s.sent(now);
            ++statuses;
        }
    }
    EXPECT_GT(statuses, 100);
}

// ---------------------------------------------------------------------------
// Diagnostic frames
// ---------------------------------------------------------------------------

namespace
{

constexpr uint32_t kDiagGuard = 2778 + 1000; // airtime of 32 bytes plus the margin

} // namespace

TEST(DiagLink, DiagFrameFitsTheTxFifo) {
    EXPECT_EQ(airtimeUs(translagatr::kPicoDiagLen + translagatr::kLinkEnvelopeLen, 115200),
              2778u);
    EXPECT_LE(translagatr::kPicoDiagLen + translagatr::kLinkEnvelopeLen, 32);
}

TEST(DiagLink, OffUntilARateIsSet) {
    DiagSchedule d(kDiagGuard);
    EXPECT_EQ(d.rate(), 0);
    for (uint32_t now = 0; now < 3000000; now += 1000) {
        ASSERT_FALSE(d.ready(now, now + kTick, true)) << now;
    }
}

TEST(DiagLink, FirstFrameAtTheNextWindowThenAtTheRate) {
    DiagSchedule d(kDiagGuard);
    d.setRate(2);
    EXPECT_TRUE(d.ready(1000, kTick, true));
    d.sent(1000);
    EXPECT_FALSE(d.ready(1000 + 499999, 10000000, true));
    EXPECT_TRUE(d.ready(1000 + 500000, 10000000, true));
}

TEST(DiagLink, WaitsForAnEmptyFifoAndTheGuard) {
    DiagSchedule d(kDiagGuard);
    d.setRate(1);
    EXPECT_FALSE(d.ready(0, kTick, false));
    EXPECT_FALSE(d.ready(kTick - kDiagGuard + 1, kTick, true));
    EXPECT_TRUE(d.ready(kTick - kDiagGuard, kTick, true));
}

TEST(DiagLink, StoppingAndRestartingTheRate) {
    DiagSchedule d(kDiagGuard);
    d.setRate(5);
    d.sent(0);
    d.setRate(0);
    EXPECT_FALSE(d.ready(5000000, 10000000, true));
    d.setRate(5);
    EXPECT_TRUE(d.ready(5000100, 10000000, true));   // a new rate goes at once
    d.sent(5000100);
    d.setRate(5);                                    // unchanged: no extra frame
    EXPECT_FALSE(d.ready(5000200, 10000000, true));
}

TEST(DiagLink, RateHoldsAcrossTheClockWrap) {
    DiagSchedule d(kDiagGuard);
    d.setRate(1);
    const uint32_t start = 0xFFFFFFFFu - 100000u;
    d.sent(start);
    EXPECT_FALSE(d.ready(start + 999999u, start + 999999u + kTick, true));
    EXPECT_TRUE(d.ready(start + 1000000u, start + 1000000u + kTick, true));
}

TEST(DiagLink, StatusAndDiagnosticsNeverDelayASensorFrame) {
    // 50 Hz ticks, status every 200 ms and on every pass requested, diagnostics
    // at the highest rate, both sharing the idle time as main.cpp does: at
    // every tick the FIFO must already be empty.
    const uint32_t byte_us   = 87;
    const uint32_t sensor_us = 31 * byte_us;
    const uint32_t status_us = 26 * byte_us;
    const uint32_t diag_us   = 32 * byte_us;
    StatusSchedule s(kPeriod, kGap, airtimeUs(26, 115200) + 1000);
    DiagSchedule   d(airtimeUs(32, 115200) + 1000);
    d.setRate(5);

    uint32_t fifo_empty_at = 0;
    uint32_t next_tick     = kTick;
    int      statuses = 0, diags = 0;
    for (uint32_t now = 0; now < 4000000; now += 37) {
        if (static_cast<int32_t>(now - next_tick) >= 0) {
            ASSERT_GE(now, fifo_empty_at) << "sensor frame delayed at " << now;
            fifo_empty_at = now + sensor_us;
            next_tick += kTick;
            continue;
        }
        if (now % 3000 < 37) {
            s.request();
        }
        const bool empty = now >= fifo_empty_at;
        if (s.ready(now, next_tick, empty)) {
            fifo_empty_at = now + status_us;
            s.sent(now);
            ++statuses;
        } else if (d.ready(now, next_tick, empty)) {
            fifo_empty_at = now + diag_us;
            d.sent(now);
            ++diags;
        }
    }
    EXPECT_GT(statuses, 100);
    // 5 Hz over 4 s, all delivered despite the busy link
    EXPECT_GE(diags, 19);
    EXPECT_LE(diags, 21);
}
