#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "gravity_alignment.h"

namespace {

using bno08x::GravityAlignment;
using bno08x::Vector3;
constexpr double kGravity = 9.80665;
constexpr Vector3 kUp{0, 0, 1};

Vector3 scaled(Vector3 v, double s) { return {v.x * s, v.y * s, v.z * s}; }

void stationarySample(GravityAlignment& a, uint32_t stamp, Vector3 up = kUp) {
    double yaw = 123.0;
    a.projectGyro(stamp, {}, yaw);
    a.addAcceleration(stamp, scaled(up, kGravity));
}

uint32_t calibrate(GravityAlignment& a, Vector3 up = kUp, uint32_t first = 0) {
    for (uint32_t i = 0; i <= 400; ++i) stationarySample(a, first + i * 5000, up);
    EXPECT_TRUE(a.ready());
    return first + 2000000;
}

TEST(GravityAlignment, RequiresBothDurationAndEnoughDistinctReports) {
    GravityAlignment dense;
    for (uint32_t i = 0; i < 400; ++i) {
        double yaw = 123.0;
        EXPECT_FALSE(dense.projectGyro(i * 5000, {}, yaw));
        EXPECT_EQ(yaw, 123.0);
        dense.addAcceleration(i * 5000, scaled(kUp, kGravity));
        EXPECT_FALSE(dense.ready());
    }
    stationarySample(dense, 2000000);
    EXPECT_TRUE(dense.ready());
    EXPECT_EQ(dense.samples(), 401u);

    GravityAlignment sparse;
    for (uint32_t i = 0; i < 199; ++i) stationarySample(sparse, i * 50000);
    EXPECT_FALSE(sparse.ready());
    EXPECT_EQ(sparse.samples(), 199u);
    stationarySample(sparse, 199 * 50000);
    EXPECT_TRUE(sparse.ready());
}

TEST(GravityAlignment, ProjectsYawForUprightInvertedSidewaysAndObliqueMounts) {
    const Vector3 mounts[] = {{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {0, -1, 0},
                             {2.0 / 3.0, -1.0 / 3.0, 2.0 / 3.0}};
    for (const auto up : mounts) {
        SCOPED_TRACE(::testing::Message() << up.x << "," << up.y << "," << up.z);
        GravityAlignment a;
        const uint32_t end = calibrate(a, up);
        double yaw = 0;
        ASSERT_TRUE(a.projectGyro(end + 5000, scaled(up, 1.25), yaw));
        EXPECT_NEAR(yaw, 1.25, 1e-12);
        ASSERT_TRUE(a.projectGyro(end + 10000, scaled(up, -0.75), yaw));
        EXPECT_NEAR(yaw, -0.75, 1e-12);
    }
}

TEST(GravityAlignment, RejectsRotationPerpendicularToStartupUp) {
    GravityAlignment a;
    const uint32_t end = calibrate(a, {2.0 / 3.0, -1.0 / 3.0, 2.0 / 3.0});
    double yaw = 0;
    ASSERT_TRUE(a.projectGyro(end + 5000, {1, 2, 0}, yaw));
    EXPECT_NEAR(yaw, 0, 1e-12);
}

TEST(GravityAlignment, AccelAndGyroMayArriveInEitherOrder) {
    GravityAlignment a;
    double yaw = 0;
    for (uint32_t i = 0; i <= 401; ++i) {
        a.addAcceleration(i * 5000, {0, 0, kGravity});
        a.projectGyro(i * 5000, {}, yaw);
    }
    EXPECT_TRUE(a.ready());
    ASSERT_TRUE(a.projectGyro(2010000, {0, 0, 0.5}, yaw));
    EXPECT_NEAR(yaw, 0.5, 1e-12);
}

TEST(GravityAlignment, GyroMotionResetsTheWholeStationaryWindow) {
    GravityAlignment a;
    for (uint32_t i = 0; i < 300; ++i) stationarySample(a, i * 5000);
    double yaw = 0;
    EXPECT_FALSE(a.projectGyro(1500000, {0.11, 0, 0}, yaw));
    a.addAcceleration(1500000, {0, 0, kGravity});
    EXPECT_EQ(a.samples(), 0u);
    EXPECT_FALSE(a.ready());
    for (uint32_t i = 301; i <= 500; ++i) stationarySample(a, i * 5000);
    EXPECT_FALSE(a.ready());
    calibrate(a, kUp, 2505000);
}

TEST(GravityAlignment, FreeFallAndExcessAccelerationNeverCalibrate) {
    for (Vector3 acceleration : {Vector3{}, Vector3{0, 0, 11.0}}) {
        GravityAlignment a;
        double yaw = 0;
        for (uint32_t i = 0; i < 1000; ++i) {
            a.projectGyro(i * 5000, {}, yaw);
            a.addAcceleration(i * 5000, acceleration);
        }
        EXPECT_FALSE(a.ready());
        EXPECT_EQ(a.samples(), 0u);
    }
}

TEST(GravityAlignment, DirectionChangesResetEvenWithOneGAndSmallGyro) {
    GravityAlignment a;
    double yaw = 0;
    for (uint32_t i = 0; i < 1000; ++i) {
        const double tilt = static_cast<double>(i) * 0.001;
        a.projectGyro(i * 5000, {0.05, 0, 0}, yaw);
        a.addAcceleration(i * 5000, {kGravity * std::sin(tilt), 0,
                                     kGravity * std::cos(tilt)});
    }
    EXPECT_FALSE(a.ready());
    EXPECT_LT(a.samples(), 30u);
}

TEST(GravityAlignment, DuplicateReportsDoNotAdvanceOrRestartCalibration) {
    GravityAlignment a;
    stationarySample(a, 0);
    double yaw = 0;
    for (int i = 0; i < 1000; ++i) {
        EXPECT_FALSE(a.projectGyro(0, {}, yaw));
        a.addAcceleration(0, {0, 0, kGravity});
    }
    EXPECT_EQ(a.samples(), 1u);
    EXPECT_FALSE(a.ready());
    for (uint32_t i = 1; i <= 400; ++i) stationarySample(a, i * 5000);
    EXPECT_TRUE(a.ready());
    EXPECT_FALSE(a.projectGyro(2000000, {0, 0, 1}, yaw));
}

TEST(GravityAlignment, MissingOrStaleGyroPreventsCalibration) {
    GravityAlignment a;
    for (uint32_t i = 0; i < 500; ++i) a.addAcceleration(i * 5000, {0, 0, kGravity});
    EXPECT_FALSE(a.ready());
    EXPECT_EQ(a.samples(), 0u);

    a.reset();
    stationarySample(a, 0);
    a.addAcceleration(50000, {0, 0, kGravity});
    EXPECT_EQ(a.samples(), 2u);
    a.addAcceleration(50001, {0, 0, kGravity});
    EXPECT_EQ(a.samples(), 0u);
}

TEST(GravityAlignment, MissingAccelerationAndReportGapsResetPartialWindow) {
    GravityAlignment a;
    for (uint32_t i = 0; i < 300; ++i) stationarySample(a, i * 5000);
    double yaw = 0;
    for (uint32_t i = 300; i <= 311; ++i) a.projectGyro(i * 5000, {}, yaw);
    EXPECT_EQ(a.samples(), 0u);
    EXPECT_FALSE(a.ready());
    stationarySample(a, 1560000);
    EXPECT_EQ(a.samples(), 1u);
    stationarySample(a, 1620000);
    EXPECT_EQ(a.samples(), 1u);
    EXPECT_FALSE(a.ready());
}

TEST(GravityAlignment, OutOfOrderReportsResetPartialWithoutMovingClockBackward) {
    GravityAlignment a;
    for (uint32_t i = 0; i < 300; ++i) stationarySample(a, i * 5000);
    a.addAcceleration(1490000, {0, 0, kGravity});
    EXPECT_EQ(a.samples(), 0u);
    stationarySample(a, 1500000);
    EXPECT_EQ(a.samples(), 1u);
    double yaw = 0;
    EXPECT_FALSE(a.projectGyro(1490000, {}, yaw));
    EXPECT_EQ(a.samples(), 0u);
    a.addAcceleration(1505000, {0, 0, kGravity});
    EXPECT_EQ(a.samples(), 1u);
    EXPECT_FALSE(a.ready());
}

TEST(GravityAlignment, CounterWrapMaintainsWindowAndProjection) {
    GravityAlignment a;
    const uint32_t end = calibrate(a, kUp, 0xFFF00000u);
    double yaw = 0;
    ASSERT_TRUE(a.projectGyro(end + 5000, {0, 0, -1}, yaw));
    EXPECT_DOUBLE_EQ(yaw, -1);
}

TEST(GravityAlignment, NonfiniteReportsResetPartialAndCannotCreateOutput) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    GravityAlignment a;
    stationarySample(a, 0);
    a.addAcceleration(5000, {nan, 0, kGravity});
    EXPECT_EQ(a.samples(), 0u);
    stationarySample(a, 10000);
    double yaw = 123;
    EXPECT_FALSE(a.projectGyro(15000, {0, inf, 0}, yaw));
    EXPECT_EQ(yaw, 123);
    EXPECT_EQ(a.samples(), 0u);
    a.addAcceleration(15000, {0, 0, kGravity});
    EXPECT_EQ(a.samples(), 0u);
    a.reset();
    const uint32_t end = calibrate(a);
    EXPECT_FALSE(a.projectGyro(end + 5000, {0, 0, nan}, yaw));
    EXPECT_EQ(yaw, 123);
    EXPECT_TRUE(a.ready());
    EXPECT_TRUE(a.projectGyro(end + 10000, {0, 0, 1}, yaw));
    EXPECT_DOUBLE_EQ(yaw, 1);
}

TEST(GravityAlignment, CalibrationStaysFrozenDuringMotionAndReportGaps) {
    GravityAlignment a;
    const uint32_t end = calibrate(a);
    const uint32_t samples = a.samples();
    double yaw = 0;
    for (uint32_t i = 1; i < 100; ++i) {
        a.addAcceleration(end + i * 5000, {kGravity, 0, 0});
        ASSERT_TRUE(a.projectGyro(end + i * 5000, {2, 3, 4}, yaw));
        EXPECT_NEAR(yaw, 4, 1e-12);
    }
    EXPECT_EQ(a.samples(), samples);
    EXPECT_TRUE(a.ready());
    ASSERT_TRUE(a.projectGyro(end + 1000000, {0, 0, 0.7}, yaw));
    EXPECT_NEAR(yaw, 0.7, 1e-12);
    EXPECT_FALSE(a.projectGyro(end + 999999, {0, 0, 3}, yaw));
    EXPECT_TRUE(a.ready());
}

TEST(GravityAlignment, ResetRequiresNewCalibrationAndCanChangeMount) {
    GravityAlignment a;
    calibrate(a);
    a.reset();
    EXPECT_FALSE(a.ready());
    EXPECT_EQ(a.samples(), 0u);
    double yaw = 123;
    EXPECT_FALSE(a.projectGyro(0, {0, 0, 1}, yaw));
    EXPECT_EQ(yaw, 123);
    const uint32_t end = calibrate(a, {0, 0, -1}, 5000);
    ASSERT_TRUE(a.projectGyro(end + 5000, {0, 0, -2}, yaw));
    EXPECT_NEAR(yaw, 2, 1e-12);
}

} // namespace
