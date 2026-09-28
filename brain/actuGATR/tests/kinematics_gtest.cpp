// kinematics_gtest.cpp
// Tank and mecanum kinematics, desaturation, and the drivetrain unit
// conversions and motion models.

#include "actugatr/drivetrain.h"
#include "actugatr/kinematics.h"

#include <cmath>
#include <gtest/gtest.h>

using namespace actugatr;
using investigatr::kPi;

namespace
{

ChassisCommand body(double vx, double vy, double omega) {
    return ChassisCommand{ChassisFrame::kBody, vx, vy, omega};
}

} // namespace

TEST(TankKinematics, ForwardAndTurnSigns) {
    const TankKinematics tank(0.30);
    WheelSpeeds          w;
    ASSERT_TRUE(tank.toWheels(body(1.0, 0, 0), w));
    EXPECT_EQ(w.count, 2u);
    EXPECT_DOUBLE_EQ(w.speed[0], 1.0);
    EXPECT_DOUBLE_EQ(w.speed[1], 1.0);

    // CCW turn in place: left backward, right forward, rim speed omega * r.
    ASSERT_TRUE(tank.toWheels(body(0, 0, 2.0), w));
    EXPECT_DOUBLE_EQ(w.speed[0], -0.3);
    EXPECT_DOUBLE_EQ(w.speed[1], 0.3);
}

TEST(TankKinematics, RefusesSidewaysMotionAndFieldFrame) {
    const TankKinematics tank(0.30);
    WheelSpeeds          w;
    w.count    = 7;
    EXPECT_FALSE(tank.toWheels(body(0.5, 0.1, 0), w));
    EXPECT_EQ(w.count, 7u);
    EXPECT_FALSE(tank.toWheels(ChassisCommand{ChassisFrame::kField, 0.5, 0, 0}, w));
    EXPECT_TRUE(tank.toWheels(body(0.5, 1e-9, 0), w));
}

TEST(TankKinematics, ChassisRoundTrip) {
    const TankKinematics tank(0.42);
    WheelSpeeds          w;
    ASSERT_TRUE(tank.toWheels(body(0.7, 0, -1.3), w));
    const ChassisCommand back = tank.toChassis(w);
    EXPECT_NEAR(back.vx, 0.7, 1e-12);
    EXPECT_NEAR(back.vy, 0.0, 1e-12);
    EXPECT_NEAR(back.omega, -1.3, 1e-12);
}

TEST(MecanumKinematics, WheelPatterns) {
    const MecanumKinematics m(0.30, 0.25);
    WheelSpeeds             w;
    ASSERT_TRUE(m.toWheels(body(1.0, 0, 0), w));
    EXPECT_EQ(w.count, 4u);
    for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(w.speed[i], 1.0);
    }
    // Strafe left: front left and rear right back, the others forward.
    ASSERT_TRUE(m.toWheels(body(0, 1.0, 0), w));
    EXPECT_DOUBLE_EQ(w.speed[0], -1.0);
    EXPECT_DOUBLE_EQ(w.speed[1], 1.0);
    EXPECT_DOUBLE_EQ(w.speed[2], 1.0);
    EXPECT_DOUBLE_EQ(w.speed[3], -1.0);
    // CCW: left side back, right side forward, by omega * (half track + half base).
    ASSERT_TRUE(m.toWheels(body(0, 0, 1.0), w));
    EXPECT_DOUBLE_EQ(w.speed[0], -0.275);
    EXPECT_DOUBLE_EQ(w.speed[1], 0.275);
    EXPECT_DOUBLE_EQ(w.speed[2], -0.275);
    EXPECT_DOUBLE_EQ(w.speed[3], 0.275);
}

TEST(MecanumKinematics, IndependentAxesRoundTrip) {
    const MecanumKinematics m(0.36, 0.30);
    WheelSpeeds             w;
    ASSERT_TRUE(m.toWheels(body(0.4, -0.3, 0.9), w));
    const ChassisCommand back = m.toChassis(w);
    EXPECT_NEAR(back.vx, 0.4, 1e-12);
    EXPECT_NEAR(back.vy, -0.3, 1e-12);
    EXPECT_NEAR(back.omega, 0.9, 1e-12);
    EXPECT_FALSE(m.toWheels(ChassisCommand{ChassisFrame::kField, 1, 0, 0}, w));
}

TEST(Kinematics, DesaturationKeepsDirectionAndCurvature) {
    const MecanumKinematics m(0.30, 0.30);
    WheelSpeeds             w;
    ASSERT_TRUE(m.toWheels(body(2.0, 1.0, 3.0), w));
    const ChassisCommand before = m.toChassis(w);
    const double         factor = desaturate(w, 1.0);
    EXPECT_LT(factor, 1.0);
    double peak = 0;
    for (std::size_t i = 0; i < w.count; ++i) {
        peak = std::max(peak, std::fabs(w.speed[i]));
    }
    EXPECT_NEAR(peak, 1.0, 1e-12);
    const ChassisCommand after = m.toChassis(w);
    EXPECT_NEAR(after.vx / before.vx, factor, 1e-12);
    EXPECT_NEAR(after.vy / before.vy, factor, 1e-12);
    EXPECT_NEAR(after.omega / before.omega, factor, 1e-12);

    // Within limits nothing changes.
    WheelSpeeds slow;
    ASSERT_TRUE(m.toWheels(body(0.1, 0, 0), slow));
    EXPECT_DOUBLE_EQ(desaturate(slow, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(slow.speed[0], 0.1);
}

TEST(Drivetrain, UnitConversionsThroughGearing) {
    WheelDrive w;
    w.wheel_diameter  = 0.1016; // 4 in
    w.gear_ratio      = 0.6;    // 36:60
    w.cartridge       = Cartridge::kBlue;
    w.usable_fraction = 1.0;
    // 600 rpm * 0.6 = 360 wheel rpm = 6 rev/s.
    EXPECT_NEAR(maxWheelSpeed(w), 6.0 * kPi * 0.1016, 1e-12);
    EXPECT_NEAR(motorRpm(w, 6.0 * kPi * 0.1016), 600.0, 1e-9);
    EXPECT_NEAR(wheelSpeed(w, 300.0), 3.0 * kPi * 0.1016, 1e-12);
    EXPECT_NEAR(motorRpm(w, -1.0), -1.0 * 60.0 / (kPi * 0.1016) / 0.6, 1e-9);

    w.usable_fraction = 0.9;
    EXPECT_NEAR(maxWheelSpeed(w), 0.9 * 6.0 * kPi * 0.1016, 1e-12);
}

TEST(Drivetrain, ConfigValidation) {
    TankConfig t;
    t.left.count            = 1;
    t.left.motors[0]        = {1, true};
    t.right.count           = 1;
    t.right.motors[0]       = {2, false};
    t.track_width           = 0.3;
    t.wheels.wheel_diameter = 0.08;
    const char* why         = nullptr;
    EXPECT_TRUE(valid(t, &why)) << why;

    TankConfig bad = t;
    bad.right.motors[0].port = 22;
    EXPECT_FALSE(valid(bad, &why));
    bad             = t;
    bad.left.count  = 0;
    EXPECT_FALSE(valid(bad, &why));
    bad                       = t;
    bad.wheels.gear_ratio     = 0;
    EXPECT_FALSE(valid(bad, &why));
    bad             = t;
    bad.track_width = -1;
    EXPECT_FALSE(valid(bad, &why));

    MecanumConfig m;
    for (MotorGroup* g : {&m.front_left, &m.front_right, &m.rear_left, &m.rear_right}) {
        g->count = 1;
    }
    m.front_left.motors[0]  = {1, false};
    m.front_right.motors[0] = {2, true};
    m.rear_left.motors[0]   = {3, false};
    m.rear_right.motors[0]  = {4, true};
    m.track_width           = 0.3;
    m.wheelbase             = 0.3;
    m.wheels.wheel_diameter = 0.1;
    EXPECT_TRUE(valid(m, &why)) << why;
    m.wheelbase = 0;
    EXPECT_FALSE(valid(m, &why));
}

TEST(Drivetrain, GroupOrderMatchesKinematics) {
    MecanumConfig    m;
    const MotorGroup* g[kMaxWheelGroups] = {};
    ASSERT_EQ(groups(m, g), 4u);
    EXPECT_EQ(g[0], &m.front_left);
    EXPECT_EQ(g[1], &m.front_right);
    EXPECT_EQ(g[2], &m.rear_left);
    EXPECT_EQ(g[3], &m.rear_right);
    TankConfig t;
    ASSERT_EQ(groups(t, g), 2u);
    EXPECT_EQ(g[0], &t.left);
    EXPECT_EQ(g[1], &t.right);
}

TEST(Drivetrain, MotionModelsCarryCapabilityAndReachableLimits) {
    TankConfig t;
    t.track_width           = 0.3;
    t.wheels.wheel_diameter = 0.08;
    t.wheels.gear_ratio     = 1.0;
    t.wheels.cartridge      = Cartridge::kGreen;
    investigatr::MotionLimits limits;
    limits.max_speed = 10.0;
    limits.max_omega = 50.0;
    const investigatr::MotionModel tm = motionModel(t, {0.2, 0.2, 0.2, 0.2}, 0.05, limits);
    EXPECT_FALSE(tm.holonomic);
    EXPECT_TRUE(tm.turn_in_place);
    EXPECT_DOUBLE_EQ(tm.limits.max_speed, maxWheelSpeed(t.wheels));
    EXPECT_DOUBLE_EQ(tm.limits.max_omega, 2.0 * maxWheelSpeed(t.wheels) / 0.3);
    EXPECT_DOUBLE_EQ(tm.clearance, 0.05);

    MecanumConfig m;
    m.track_width           = 0.3;
    m.wheelbase             = 0.2;
    m.wheels.wheel_diameter = 0.1;
    const investigatr::MotionModel mm = motionModel(m, {0.2, 0.2, 0.2, 0.2}, 0.05, limits);
    EXPECT_TRUE(mm.holonomic);
    EXPECT_DOUBLE_EQ(mm.limits.max_omega, maxWheelSpeed(m.wheels) / 0.25);

    limits.max_speed = 0.3;
    EXPECT_DOUBLE_EQ(motionModel(t, {0.2, 0.2, 0.2, 0.2}, 0.05, limits).limits.max_speed, 0.3);
}
