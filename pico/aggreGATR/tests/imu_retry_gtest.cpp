// imu_retry_gtest.cpp

#include <gtest/gtest.h>

#include "imu_status.h"
#include "imu_retry.h"

using namespace imu;

namespace
{

// Fails one attempt with a boot timeout; returns the wait.
uint32_t failAttempt(RetryPolicy& r) {
    r.attempt();
    return r.failed(imu::kReasonBoot);
}

DeviceView enabled() {
    DeviceView d;
    d.enabled = true;
    return d;
}

} // namespace

TEST(ImuRetry, FiveQuickAttemptsThenSlowRetries) {
    RetryPolicy r;
    EXPECT_EQ(failAttempt(r), 500000u);
    EXPECT_EQ(failAttempt(r), 1000000u);
    EXPECT_EQ(failAttempt(r), 2000000u);
    EXPECT_EQ(failAttempt(r), 4000000u);
    EXPECT_FALSE(r.exhausted());
    EXPECT_EQ(failAttempt(r), kSlowRetryUs);
    EXPECT_TRUE(r.exhausted());
    for (int i = 0; i < 20; ++i) {
        EXPECT_EQ(failAttempt(r), kSlowRetryUs);
    }
    EXPECT_EQ(r.attempts(), 25);
    EXPECT_TRUE(r.exhausted());
}

TEST(ImuRetry, BackoffNeverExceedsTheCap) {
    for (uint16_t n = 1; n < kQuickAttempts; ++n) {
        RetryPolicy r;
        uint32_t    wait = 0;
        for (uint16_t i = 0; i < n; ++i) {
            wait = failAttempt(r);
        }
        EXPECT_GE(wait, kBackoffMinUs);
        EXPECT_LE(wait, kBackoffMaxUs);
    }
}

TEST(ImuRetry, StableEndsTheEpisode) {
    RetryPolicy r;
    failAttempt(r);
    failAttempt(r);
    r.attempt();
    r.stable();
    EXPECT_EQ(r.attempts(), 3);
    EXPECT_EQ(r.reason(), imu::kReasonNone);
    EXPECT_FALSE(r.exhausted());

    // Losing the stable device opens a new episode with the shortest wait.
    EXPECT_EQ(r.failed(imu::kReasonStream), kBackoffMinUs);
    EXPECT_EQ(r.attempts(), 0);
    EXPECT_EQ(r.reason(), imu::kReasonStream);
    EXPECT_EQ(failAttempt(r), 500000u);
    EXPECT_EQ(failAttempt(r), 1000000u);
    EXPECT_EQ(r.attempts(), 2);
}

TEST(ImuRetry, SlowAttemptThatComesUpClearsFailed) {
    RetryPolicy r;
    for (int i = 0; i < kQuickAttempts; ++i) {
        failAttempt(r);
    }
    ASSERT_TRUE(r.exhausted());
    r.attempt();
    EXPECT_TRUE(r.exhausted()); // still failed until stable
    r.stable();
    EXPECT_FALSE(r.exhausted());
}

TEST(ImuRetry, RestartGivesFreshQuickAttempts) {
    RetryPolicy r;
    for (int i = 0; i < kQuickAttempts + 3; ++i) {
        failAttempt(r);
    }
    ASSERT_TRUE(r.exhausted());
    r.restart();
    EXPECT_FALSE(r.exhausted());
    EXPECT_EQ(r.attempts(), 0);
    EXPECT_EQ(r.reason(), imu::kReasonNone);
    EXPECT_EQ(failAttempt(r), 500000u);
}

TEST(ImuRetry, AttemptsSaturate) {
    RetryPolicy r;
    for (uint32_t i = 0; i < 70000; ++i) {
        r.attempt();
    }
    EXPECT_EQ(r.attempts(), UINT16_MAX);
}

TEST(ImuRetry, StatusStateFollowsTheDevice) {
    RetryPolicy r;
    DeviceView  d;
    EXPECT_EQ(statusState(d, r), imu::kStateDisabled);

    d = enabled();
    r.attempt();
    EXPECT_EQ(statusState(d, r), imu::kStateInitializing);

    d.up = true;
    EXPECT_EQ(statusState(d, r), imu::kStateAligning);
    d.ready = true;
    EXPECT_EQ(statusState(d, r), imu::kStateReady);
    EXPECT_EQ(statusReason(d, r), imu::kReasonNone);

    d = enabled();
    r.failed(imu::kReasonStream);
    d.waiting = true;
    EXPECT_EQ(statusState(d, r), imu::kStateRetrying);
    EXPECT_EQ(statusReason(d, r), imu::kReasonStream);
    d.waiting = false;
    EXPECT_EQ(statusState(d, r), imu::kStateInitializing);
}

TEST(ImuRetry, StatusStateFailedWhileExhausted) {
    RetryPolicy r;
    for (int i = 0; i < kQuickAttempts; ++i) {
        failAttempt(r);
    }
    DeviceView d = enabled();
    d.waiting    = true;
    EXPECT_EQ(statusState(d, r), imu::kStateFailed);
    EXPECT_EQ(statusReason(d, r), imu::kReasonBoot);

    // A slow attempt in progress still reports failed until the device is up.
    d.waiting = false;
    r.attempt();
    EXPECT_EQ(statusState(d, r), imu::kStateFailed);
    d.up = d.ready = true;
    EXPECT_EQ(statusState(d, r), imu::kStateReady);
}

TEST(ImuRetry, RestartWaitIsNotRetrying) {
    RetryPolicy r;
    failAttempt(r);
    r.restart();
    DeviceView d = enabled();
    d.waiting    = true;
    EXPECT_EQ(statusState(d, r), imu::kStateInitializing);
}

TEST(ImuRetry, DisabledReportsNoReason) {
    RetryPolicy r;
    failAttempt(r);
    DeviceView d;
    EXPECT_EQ(statusState(d, r), imu::kStateDisabled);
    EXPECT_EQ(statusReason(d, r), imu::kReasonNone);
}
