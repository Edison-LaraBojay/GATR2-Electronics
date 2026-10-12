// chassis_gtest.cpp

#include "actugatr/chassis.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace actugatr;
using investigatr::kPi;

TEST(Chassis, BodyCommandsPassThrough) {
    ChassisCommand c{ChassisFrame::kBody, 0.5, -0.2, 1.0};
    const ChassisCommand out = toBody(c, 1.2);
    EXPECT_EQ(out.frame, ChassisFrame::kBody);
    EXPECT_DOUBLE_EQ(out.vx, 0.5);
    EXPECT_DOUBLE_EQ(out.vy, -0.2);
    EXPECT_DOUBLE_EQ(out.omega, 1.0);
}

TEST(Chassis, FieldCommandRotatesIntoTheBody) {
    // Facing +y, a field +x velocity is to the robot's right (-y body).
    ChassisCommand c{ChassisFrame::kField, 1.0, 0.0, 0.3};
    const ChassisCommand out = toBody(c, kPi / 2);
    EXPECT_EQ(out.frame, ChassisFrame::kBody);
    EXPECT_NEAR(out.vx, 0.0, 1e-12);
    EXPECT_NEAR(out.vy, -1.0, 1e-12);
    EXPECT_DOUBLE_EQ(out.omega, 0.3);

    // Facing +y, a field +y velocity is straight ahead.
    const ChassisCommand ahead = toBody({ChassisFrame::kField, 0.0, 0.7, 0.0}, kPi / 2);
    EXPECT_NEAR(ahead.vx, 0.7, 1e-12);
    EXPECT_NEAR(ahead.vy, 0.0, 1e-12);
}

TEST(Chassis, FiniteAndZero) {
    EXPECT_TRUE(finite(ChassisCommand{}));
    EXPECT_TRUE(isZero(ChassisCommand{}));
    ChassisCommand c;
    c.omega = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(finite(c));
    EXPECT_FALSE(isZero({ChassisFrame::kBody, 0.01, 0, 0}));
}

TEST(Chassis, SlewLimitsIncreasesOnly) {
    EXPECT_DOUBLE_EQ(slewLimit(0.0, 1.0, 2.0, 0.1), 0.2);
    EXPECT_DOUBLE_EQ(slewLimit(0.8, 0.1, 2.0, 0.1), 0.1);
    // A reversal drops to zero first.
    EXPECT_DOUBLE_EQ(slewLimit(0.5, -1.0, 2.0, 0.1), -0.2);
    EXPECT_DOUBLE_EQ(slewLimit(0.0, 1.0, 0.0, 0.1), 1.0);
}

TEST(Chassis, RateLimitIsSymmetric) {
    EXPECT_DOUBLE_EQ(rateLimit(0.0, 1.0, 2.0, 0.1), 0.2);
    EXPECT_DOUBLE_EQ(rateLimit(1.0, 0.0, 2.0, 0.1), 0.8);
    EXPECT_DOUBLE_EQ(rateLimit(1.0, 0.0, 0.0, 0.1), 0.0);
}
