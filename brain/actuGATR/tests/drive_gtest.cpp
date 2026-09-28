// drive_gtest.cpp
// Drive: unit conversion to motor targets, desaturation, and immediate stops
// for commands it cannot execute.

#include "actugatr/drive.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "sim/sim_motor_output.h"

using namespace actugatr;
using investigatr::kPi;

namespace
{

WheelDrive wheels() {
    WheelDrive w;
    w.wheel_diameter  = 0.1;
    w.gear_ratio      = 1.0;
    w.cartridge       = Cartridge::kBlue;
    w.usable_fraction = 1.0;
    return w;
}

ChassisCommand body(double vx, double vy, double omega) {
    return ChassisCommand{ChassisFrame::kBody, vx, vy, omega};
}

} // namespace

TEST(Drive, ConvertsWheelSpeedToMotorRpm) {
    const TankKinematics tank(0.3);
    SimMotorOutput       out(2);
    Drive                drive(tank, wheels(), out);
    // 0.5 m/s on a 0.1 m wheel is 0.5 / (pi * 0.1) rev/s.
    ASSERT_EQ(drive.apply(body(0.5, 0, 0), 1.0, 1.0), DriveFault::kNone);
    const double rpm = 0.5 / (kPi * 0.1) * 60.0;
    EXPECT_NEAR(out.rpm(0), rpm, 1e-9);
    EXPECT_NEAR(out.rpm(1), rpm, 1e-9);
    EXPECT_FALSE(out.stopped());
    EXPECT_FALSE(drive.status().stopped);
}

TEST(Drive, GearingScalesMotorRpm) {
    const TankKinematics tank(0.3);
    SimMotorOutput       out(2);
    WheelDrive           geared = wheels();
    geared.gear_ratio           = 0.5; // wheel turns half as fast as the motor
    Drive drive(tank, geared, out);
    ASSERT_EQ(drive.apply(body(0.5, 0, 0), 0, 0), DriveFault::kNone);
    EXPECT_NEAR(out.rpm(0), 2.0 * 0.5 / (kPi * 0.1) * 60.0, 1e-9);
}

TEST(Drive, DesaturatesTogether) {
    const TankKinematics tank(0.3);
    SimMotorOutput       out(2);
    Drive                drive(tank, wheels(), out);
    const double         max = drive.maxWheelSpeed();
    ASSERT_EQ(drive.apply(body(max, 0, 2.0 * max / 0.3), 0, 0), DriveFault::kNone);
    EXPECT_LT(drive.status().saturation, 1.0);
    // Left was 0, right twice the limit: scaled by one half, left stays 0.
    EXPECT_NEAR(out.rpm(0), 0.0, 1e-9);
    EXPECT_NEAR(out.rpm(1), 600.0, 1e-9);
    EXPECT_NEAR(drive.status().command.vx, 0.5 * max, 1e-9);
}

TEST(Drive, StopsForSidewaysMotionOnTank) {
    const TankKinematics tank(0.3);
    SimMotorOutput       out(2);
    Drive                drive(tank, wheels(), out);
    ASSERT_EQ(drive.apply(body(0.5, 0, 0), 0, 0), DriveFault::kNone);
    EXPECT_EQ(drive.apply(body(0.5, 0.2, 0), 0, 0), DriveFault::kUnsupportedMotion);
    EXPECT_TRUE(out.stopped());
    EXPECT_TRUE(drive.status().stopped);
    EXPECT_EQ(drive.status().fault, DriveFault::kUnsupportedMotion);
}

TEST(Drive, MecanumDrivesSideways) {
    const MecanumKinematics m(0.3, 0.3);
    SimMotorOutput          out(4);
    Drive                   drive(m, wheels(), out);
    ASSERT_EQ(drive.apply(body(0, 0.3, 0), 0, 0), DriveFault::kNone);
    EXPECT_LT(out.rpm(0), 0);
    EXPECT_GT(out.rpm(1), 0);
    EXPECT_GT(out.rpm(2), 0);
    EXPECT_LT(out.rpm(3), 0);
}

TEST(Drive, StopsForNonFiniteStaleAndFieldFrame) {
    const TankKinematics tank(0.3);
    SimMotorOutput       out(2);
    DriveConfig          config;
    config.command_timeout = 0.1;
    config.stop_mode       = StopMode::kHold;
    Drive drive(tank, wheels(), out, config);

    ASSERT_EQ(drive.apply(body(0.5, 0, 0), 0, 0), DriveFault::kNone);
    EXPECT_EQ(drive.apply(body(std::numeric_limits<double>::quiet_NaN(), 0, 0), 0, 0),
              DriveFault::kNonFinite);
    EXPECT_TRUE(out.stopped());
    EXPECT_EQ(out.stopMode(), StopMode::kHold);

    ASSERT_EQ(drive.apply(body(0.5, 0, 0), 1.0, 1.05), DriveFault::kNone);
    EXPECT_EQ(drive.apply(body(0.5, 0, 0), 1.0, 1.2), DriveFault::kStale);
    EXPECT_TRUE(out.stopped());

    EXPECT_EQ(drive.apply(ChassisCommand{ChassisFrame::kField, 0.5, 0, 0}, 2.0, 2.0),
              DriveFault::kWrongFrame);
    EXPECT_TRUE(out.stopped());
}

TEST(Drive, ZeroCommandStopsWithTheConfiguredMode) {
    const TankKinematics tank(0.3);
    SimMotorOutput       out(2);
    DriveConfig          config;
    config.stop_mode = StopMode::kCoast;
    Drive drive(tank, wheels(), out, config);
    ASSERT_EQ(drive.apply(body(0.5, 0, 0), 0, 0), DriveFault::kNone);
    const int stops = out.stops();
    ASSERT_EQ(drive.apply(body(0, 0, 0), 0, 0), DriveFault::kNone);
    EXPECT_TRUE(out.stopped());
    EXPECT_EQ(out.stops(), stops + 1);
    EXPECT_EQ(out.stopMode(), StopMode::kCoast);
    EXPECT_EQ(drive.status().fault, DriveFault::kNone);
}
