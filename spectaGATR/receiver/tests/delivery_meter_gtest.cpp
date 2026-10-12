// delivery_meter_gtest.cpp

#include "receiver/feed/delivery_meter.h"

#include <gtest/gtest.h>

namespace spectagatr::receiver::feed
{
namespace
{

TrackedPose pose(uint32_t seq, uint32_t t_ms, double rx_ms, uint32_t session = 0xABCD) {
    TrackedPose p;
    p.record.session = session;
    p.record.epoch   = 1;
    p.record.seq     = seq;
    p.record.t_ms    = t_ms;
    p.rx_ms          = rx_ms;
    return p;
}

TEST(DeliveryMeter, SteadyTenHertzHasNoLossOrLag) {
    DeliveryMeter meter(750);
    (void)meter.take(1000);
    for (uint32_t i = 0; i < 10; ++i) {
        meter.onPose(pose(i, 5000 + 100 * i, 1000 + 100 * i + 30));
    }
    meter.onBytes(400);
    meter.onBytes(200);
    const DeliveryWindow w = meter.take(2000);
    EXPECT_EQ(w.chunks, 2u);
    EXPECT_EQ(w.poses, 10u);
    EXPECT_NEAR(w.rate_hz, 10.0, 1e-9);
    EXPECT_NEAR(w.max_gap_ms, 100.0, 1e-9);
    EXPECT_EQ(w.seq_lost, 0u);
    EXPECT_EQ(w.stale_gaps, 0u);
    EXPECT_NEAR(w.max_lag_ms, 0.0, 1e-9);
    EXPECT_NEAR(w.bytes_per_s, 600.0, 1e-9);
}

TEST(DeliveryMeter, BacklogShowsAsGrowingLagGapsAndLoss) {
    DeliveryMeter meter(750);
    (void)meter.take(0);
    // Every 500 ms, a pose 100 ms older than real time than the last one, every third line lost.
    meter.onPose(pose(0, 0, 100));
    meter.onPose(pose(3, 400, 600));
    meter.onPose(pose(6, 800, 1100));
    meter.onPose(pose(9, 1200, 2000)); // a 900 ms hole: stale
    const DeliveryWindow w = meter.take(2000);
    EXPECT_EQ(w.poses, 4u);
    EXPECT_EQ(w.seq_lost, 6u);
    EXPECT_EQ(w.stale_gaps, 1u);
    EXPECT_NEAR(w.max_gap_ms, 900.0, 1e-9);
    EXPECT_NEAR(w.max_lag_ms, 700.0, 1e-9);
}

TEST(DeliveryMeter, OpenGapCountsAndWindowsReset) {
    DeliveryMeter meter(750);
    (void)meter.take(0);
    meter.onPose(pose(0, 0, 100));
    const DeliveryWindow first = meter.take(1000);
    EXPECT_NEAR(first.max_gap_ms, 900.0, 1e-9); // nothing since 100 ms
    const DeliveryWindow second = meter.take(2000);
    EXPECT_EQ(second.poses, 0u);
    EXPECT_NEAR(second.max_gap_ms, 1900.0, 1e-9);
    EXPECT_EQ(second.seq_lost, 0u);
}

TEST(DeliveryMeter, NewSessionRestartsSeqAndLag) {
    DeliveryMeter meter(750);
    (void)meter.take(0);
    meter.onPose(pose(100, 50000, 100));
    meter.onPose(pose(0, 10, 200, 0x1234)); // publisher restarted: new clock, seq from 0
    meter.onPose(pose(1, 110, 300, 0x1234));
    const DeliveryWindow w = meter.take(400);
    EXPECT_EQ(w.seq_lost, 0u);
    EXPECT_NEAR(w.max_lag_ms, 0.0, 1e-9);
}

} // namespace
} // namespace spectagatr::receiver::feed
