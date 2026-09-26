// drive_gtest.cpp

#include "investigatr/drive.h"

#include <gtest/gtest.h>

using namespace investigatr;

TEST(MixTank, ForwardAndTurn) {
    const TankOutput out = mixTank(DriveCommand{0.5, 0.2});
    EXPECT_NEAR(out.left, 0.3, 1e-12);
    EXPECT_NEAR(out.right, 0.7, 1e-12);
}

TEST(MixTank, PositiveTurnIsCounterclockwise) {
    const TankOutput out = mixTank(DriveCommand{0.0, 0.4});
    EXPECT_DOUBLE_EQ(out.left, -0.4);
    EXPECT_DOUBLE_EQ(out.right, 0.4);
}

TEST(MixTank, ScalesToKeepRatio) {
    const TankOutput out = mixTank(DriveCommand{0.8, 0.6});
    EXPECT_DOUBLE_EQ(out.right, 1.0);
    EXPECT_NEAR(out.left, 0.2 / 1.4, 1e-12);
}

TEST(Slew, LimitsIncrease) {
    EXPECT_DOUBLE_EQ(slewLimit(0.0, 1.0, 2.0, 0.1), 0.2);
    EXPECT_DOUBLE_EQ(slewLimit(0.2, 1.0, 2.0, 0.1), 0.4);
    EXPECT_DOUBLE_EQ(slewLimit(-0.2, -1.0, 2.0, 0.1), -0.4);
    EXPECT_DOUBLE_EQ(slewLimit(0.3, 0.35, 2.0, 0.1), 0.35);
}

TEST(Slew, DecreaseIsImmediate) {
    EXPECT_DOUBLE_EQ(slewLimit(0.8, 0.1, 2.0, 0.01), 0.1);
    EXPECT_DOUBLE_EQ(slewLimit(-0.8, 0.0, 2.0, 0.01), 0.0);
}

TEST(Slew, ReversalDropsToZeroFirst) {
    EXPECT_DOUBLE_EQ(slewLimit(0.8, -0.8, 2.0, 0.1), -0.2);
    EXPECT_DOUBLE_EQ(slewLimit(-0.5, 0.5, 2.0, 0.0), 0.0);
}

TEST(Slew, ZeroRateIsUnlimited) { EXPECT_DOUBLE_EQ(slewLimit(0.0, 0.9, 0.0, 0.01), 0.9); }
