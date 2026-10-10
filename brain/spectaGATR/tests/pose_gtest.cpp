// pose_gtest.cpp

#include <gtest/gtest.h>

#include "spectagatr/pose.h"
#include "spectagatr/pros_publisher.h" // must stay free of PROS headers
#include "spectagatr/publisher.h"

namespace
{

using namespace spectagatr;

static_assert(inchesToMeters(1.0) == 0.0254, "constexpr inches");
static_assert(degreesToRadians(180.0) - kPi < 1e-12 && kPi - degreesToRadians(180.0) < 1e-12,
              "constexpr degrees");

TEST(SpectaUnits, InchesToMeters) {
    EXPECT_DOUBLE_EQ(inchesToMeters(0.0), 0.0);
    EXPECT_DOUBLE_EQ(inchesToMeters(12.0), 0.3048);
    EXPECT_DOUBLE_EQ(inchesToMeters(-24.0), -0.6096);
    EXPECT_DOUBLE_EQ(metersToInches(inchesToMeters(70.25)), 70.25);
}

TEST(SpectaUnits, DegreesToRadians) {
    EXPECT_DOUBLE_EQ(degreesToRadians(90.0), kPi / 2);
    EXPECT_DOUBLE_EQ(degreesToRadians(-45.0), -kPi / 4);
    EXPECT_DOUBLE_EQ(radiansToDegrees(kPi), 180.0);
    EXPECT_DOUBLE_EQ(radiansToDegrees(degreesToRadians(33.5)), 33.5);
}

TEST(SpectaAttitude, BraceOrderIsRollPitchYaw) {
    const Attitude a{0.1, 0.2, 0.3};
    EXPECT_EQ(a.roll, 0.1);
    EXPECT_EQ(a.pitch, 0.2);
    EXPECT_EQ(a.yaw, 0.3);
    const Attitude zero;
    EXPECT_EQ(zero.roll, 0.0);
    EXPECT_EQ(zero.pitch, 0.0);
    EXPECT_EQ(zero.yaw, 0.0);
}

TEST(SpectaSessionId, NonzeroAndInputDependent) {
    const uint64_t a[] = {1, 2, 3};
    const uint64_t b[] = {1, 2, 4};
    EXPECT_NE(makeSessionId(a, 3), 0u);
    EXPECT_EQ(makeSessionId(a, 3), makeSessionId(a, 3));
    EXPECT_NE(makeSessionId(a, 3), makeSessionId(b, 3));
    EXPECT_NE(makeSessionId(nullptr, 0), 0u);
    // Neighboring microsecond counts give unrelated ids.
    for (uint64_t us = 0; us < 1000; ++us) {
        const uint64_t one[] = {us};
        const uint64_t two[] = {us + 1};
        EXPECT_NE(makeSessionId(one, 1), makeSessionId(two, 1));
        EXPECT_NE(makeSessionId(one, 1), 0u);
    }
}

} // namespace
