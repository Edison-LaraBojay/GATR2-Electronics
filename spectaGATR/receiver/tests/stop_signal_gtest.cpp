// stop_signal_gtest.cpp
// Waits end on request, not on their timeout.

#include "receiver/platform/stop_signal.h"

#include <gtest/gtest.h>
#include <thread>

namespace spectagatr::receiver::platform
{
namespace
{

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

TEST(StopSignal, ZeroWaitReturnsAtOnceWithoutRequest) {
    StopSignal stop;
    EXPECT_FALSE(stop.requested());
    EXPECT_FALSE(stop.waitFor(0ms));
}

TEST(StopSignal, RequestedBeforeWaiting) {
    StopSignal stop;
    stop.request();
    EXPECT_TRUE(stop.requested());
    const SteadyClock::time_point start = SteadyClock::now();
    EXPECT_TRUE(stop.waitFor(60s));
    EXPECT_LT(SteadyClock::now() - start, 1s);
}

TEST(StopSignal, RequestWakesAWaitingThread) {
    StopSignal            stop;
    bool                  woke    = false;
    SteadyClock::duration elapsed = SteadyClock::duration::zero();
    std::thread           waiter([&stop, &woke, &elapsed] {
        const SteadyClock::time_point start = SteadyClock::now();
        woke                                = stop.waitFor(60s);
        elapsed                             = SteadyClock::now() - start;
    });
    stop.request();
    waiter.join();
    EXPECT_TRUE(woke);
    EXPECT_LT(elapsed, 5s);
}

} // namespace
} // namespace spectagatr::receiver::platform
