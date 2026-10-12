// pid_gtest.cpp

#include "actugatr/pid.h"

#include <gtest/gtest.h>

using namespace actugatr;
using investigatr::kPi;

TEST(Pid, ProportionalOnlyAtZeroDt) {
    Pid pid(PidGains{2.0, 5.0, 3.0, 1.0, 10.0});
    EXPECT_DOUBLE_EQ(pid.update(0.5, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(pid.update(0.7, 0.0), 1.4);
}

TEST(Pid, NoDerivativeOnFirstSample) {
    Pid pid(PidGains{1.0, 0.0, 0.5, 0.0, 10.0});
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.1), 1.0);
    // Second sample: 1 * 0.8 + 0.5 * (0.8 - 1.0) / 0.1
    EXPECT_NEAR(pid.update(0.8, 0.1), -0.2, 1e-12);
}

TEST(Pid, ResetDropsHistory) {
    Pid pid(PidGains{1.0, 1.0, 1.0, 5.0, 10.0});
    pid.update(1.0, 0.1);
    pid.update(2.0, 0.1);
    pid.reset();
    // No integral carried, no derivative on the first sample after reset.
    EXPECT_NEAR(pid.update(1.0, 0.1), 1.0 + 1.0 * 0.1, 1e-12);
}

TEST(Pid, ZeroIntegralGainHasNoIntegral) {
    Pid pid(PidGains{1.0, 0.0, 0.0, 1.0, 10.0});
    for (int i = 0; i < 100; ++i) {
        EXPECT_DOUBLE_EQ(pid.update(0.5, 0.1), 0.5);
    }
}

TEST(Pid, IntegralTermClamped) {
    Pid    pid(PidGains{0.0, 2.0, 0.0, 0.3, 10.0});
    double out = 0;
    for (int i = 0; i < 100; ++i) {
        out = pid.update(1.0, 0.1);
    }
    EXPECT_NEAR(out, 0.3, 1e-12);
    // Unwinds from the clamp, not from the unclamped sum.
    out = pid.update(-1.0, 0.1);
    EXPECT_NEAR(out, 2.0 * (0.15 - 0.1), 1e-12);
}

TEST(Pid, IntegralLimitZeroDisablesIntegral) {
    Pid pid(PidGains{0.0, 2.0, 0.0, 0.0, 10.0});
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.1), 0.0);
}

TEST(Pid, OutputClamped) {
    Pid pid(PidGains{10.0, 0.0, 0.0, 0.0, 0.7});
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.1), 0.7);
    EXPECT_DOUBLE_EQ(pid.update(-1.0, 0.1), -0.7);
}

TEST(Pid, AngularDerivativeWraps) {
    Pid angular(PidGains{0.0, 0.0, 1.0, 0.0, 100.0}, true);
    Pid linear(PidGains{0.0, 0.0, 1.0, 0.0, 100.0});
    angular.update(3.1, 0.1);
    linear.update(3.1, 0.1);
    // 3.1 to -3.1 is a 0.083 rad step across pi, not -6.2.
    EXPECT_NEAR(angular.update(-3.1, 0.1), (2.0 * kPi - 6.2) / 0.1, 1e-9);
    EXPECT_NEAR(linear.update(-3.1, 0.1), -6.2 / 0.1, 1e-9);
}

TEST(Pid, GainValidity) {
    EXPECT_TRUE(valid(PidGains{1.0, 0.0, 0.1, 0.0, 1.0}));
    EXPECT_FALSE(valid(PidGains{-1.0, 0.0, 0.0, 0.0, 1.0}));
    EXPECT_FALSE(valid(PidGains{1.0, 0.0, 0.0, 0.0, 0.0}));
}
