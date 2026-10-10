// allocation_gtest.cpp
// Counts global operator new calls around the publisher's steady state.

#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <limits>
#include <new>

#include "spectagatr/publisher.h"

namespace
{

std::atomic<long> g_allocations{0};

} // namespace

void* operator new(std::size_t size) {
    g_allocations.fetch_add(1);
    if (void* p = std::malloc(size == 0 ? 1 : size)) {
        return p;
    }
    throw std::bad_alloc();
}

void operator delete(void* p) noexcept { std::free(p); }

void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace
{

using namespace spectagatr;

Milliseconds g_now = 0;

Milliseconds fakeClock() { return g_now; }

TEST(SpectaAllocation, NoneAfterConstruction) {
    Publisher publisher(&fakeClock);
    publisher.begin(0x12345678u);
    char buffer[kLineCapacity];

    const long   before = g_allocations.load();
    const double nan    = std::numeric_limits<double>::quiet_NaN();
    std::size_t  lines  = 0;
    for (Milliseconds t = 0; t < 10000; ++t) {
        g_now = t;
        if (t % 3 == 0) {
            publisher.updatePose(1.0, 2.0, Attitude{0.1, 0.2, 0.3});
        } else {
            publisher.updatePose(1.0, 2.0, 0.3);
        }
        if (t % 1000 == 0) {
            publisher.resetTrail();
            publisher.updatePose(nan, 0.0, 0.0);
            publisher.setRate(20);
        }
        if (publisher.nextLine(t, buffer, sizeof(buffer)) > 0) {
            ++lines;
            publisher.recordWrite(true);
        }
        (void)publisher.stats();
    }
    const long after = g_allocations.load();

    EXPECT_EQ(after - before, 0);
    EXPECT_EQ(lines, 200u);
}

} // namespace
