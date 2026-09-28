// attitude_gtest.cpp
// VEX IMU roll and pitch to the robot frame: square mounts, combined tilts,
// the exact gravity rotation, and the TELEMETRY attitude group.

#include "communigatr/attitude.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace communigatr;

namespace
{

constexpr double kPi  = 3.14159265358979323846;
constexpr double kRad = kPi / 180.0;
constexpr double kTol = 1e-9;

struct Vec {
    double x, y, z;
};

// Up direction in a frame tilted by roll and pitch (the translaGATR convention).
Vec up(double roll, double pitch) {
    return {-std::sin(pitch), std::sin(roll) * std::cos(pitch), std::cos(roll) * std::cos(pitch)};
}

Vec rotateZ(const Vec& v, double yaw) {
    return {std::cos(yaw) * v.x - std::sin(yaw) * v.y, std::sin(yaw) * v.x + std::cos(yaw) * v.y,
            v.z};
}

// What an IMU mounted at mount_deg reads for a robot tilted by roll and pitch.
void imuReading(double robot_roll_deg, double robot_pitch_deg, double mount_deg,
                double& roll_deg, double& pitch_deg) {
    const Vec u = rotateZ(up(robot_roll_deg * kRad, robot_pitch_deg * kRad), -mount_deg * kRad);
    roll_deg    = std::atan2(u.y, u.z) / kRad;
    pitch_deg   = std::atan2(-u.x, std::hypot(u.y, u.z)) / kRad;
}

void expectAttitude(const RobotAttitude& a, double roll_deg, double pitch_deg) {
    ASSERT_TRUE(a.valid);
    EXPECT_NEAR(a.roll_rad, roll_deg * kRad, kTol);
    EXPECT_NEAR(a.pitch_rad, pitch_deg * kRad, kTol);
}

} // namespace

TEST(Attitude, LevelIsLevelAtEveryMount) {
    for (double mount : {0.0, 90.0, 180.0, 270.0, -90.0, 37.0}) {
        SCOPED_TRACE(mount);
        expectAttitude(robotAttitudeFromVex(0.0, 0.0, mount), 0.0, 0.0);
    }
}

TEST(Attitude, MountZeroPassesThrough) {
    expectAttitude(robotAttitudeFromVex(10.0, 0.0, 0.0), 10.0, 0.0);
    expectAttitude(robotAttitudeFromVex(0.0, -12.5, 0.0), 0.0, -12.5);
    expectAttitude(robotAttitudeFromVex(25.0, 40.0, 0.0), 25.0, 40.0);
    expectAttitude(robotAttitudeFromVex(170.0, -5.0, 0.0), 170.0, -5.0);
}

// IMU +x pointing left: IMU roll is robot pitch, IMU pitch is minus robot roll.
TEST(Attitude, Mount90SwapsAxes) {
    expectAttitude(robotAttitudeFromVex(10.0, 0.0, 90.0), 0.0, 10.0);
    expectAttitude(robotAttitudeFromVex(0.0, 10.0, 90.0), -10.0, 0.0);
    expectAttitude(robotAttitudeFromVex(-7.0, 0.0, -270.0), 0.0, -7.0);
}

TEST(Attitude, Mount180Negates) {
    expectAttitude(robotAttitudeFromVex(10.0, 0.0, 180.0), -10.0, 0.0);
    expectAttitude(robotAttitudeFromVex(0.0, 10.0, 180.0), 0.0, -10.0);
    expectAttitude(robotAttitudeFromVex(20.0, -30.0, 180.0), -20.0, 30.0);
}

TEST(Attitude, Mount270SwapsTheOtherWay) {
    expectAttitude(robotAttitudeFromVex(10.0, 0.0, 270.0), 0.0, -10.0);
    expectAttitude(robotAttitudeFromVex(0.0, 10.0, 270.0), 10.0, 0.0);
    expectAttitude(robotAttitudeFromVex(0.0, 10.0, -90.0), 10.0, 0.0);
}

// Every combined tilt at every mount comes back to the robot tilt that
// produced the IMU reading.
TEST(Attitude, CombinedTiltsRoundTripAtEveryMount) {
    for (double mount : {0.0, 90.0, 180.0, 270.0, 30.0, -135.0}) {
        for (double roll : {-150.0, -45.0, -3.0, 0.0, 12.0, 35.0, 89.0, 179.0}) {
            for (double pitch : {-80.0, -25.0, 0.0, 7.5, 30.0, 60.0}) {
                SCOPED_TRACE(testing::Message() << mount << " " << roll << " " << pitch);
                double imu_roll  = 0;
                double imu_pitch = 0;
                imuReading(roll, pitch, mount, imu_roll, imu_pitch);
                expectAttitude(robotAttitudeFromVex(imu_roll, imu_pitch, mount), roll, pitch);
            }
        }
    }
}

// Gravity in the robot frame equals gravity in the IMU frame rotated by the
// mount, exactly, where adding or swapping the angles would be off.
TEST(Attitude, ExactRotationNotSmallAngle) {
    const double       roll = 40.0, pitch = 35.0, mount = 90.0;
    const RobotAttitude a   = robotAttitudeFromVex(roll, pitch, mount);
    ASSERT_TRUE(a.valid);
    const Vec expected = rotateZ(up(roll * kRad, pitch * kRad), mount * kRad);
    const Vec got      = up(a.roll_rad, a.pitch_rad);
    EXPECT_NEAR(got.x, expected.x, kTol);
    EXPECT_NEAR(got.y, expected.y, kTol);
    EXPECT_NEAR(got.z, expected.z, kTol);
    // The small angle swap (roll -pitch, pitch roll) misses by degrees here.
    EXPECT_GT(std::fabs(a.roll_rad - (-pitch * kRad)) / kRad, 1.0);
    EXPECT_GT(std::fabs(a.pitch_rad - roll * kRad) / kRad, 1.0);

    const RobotAttitude b = robotAttitudeFromVex(30.0, 30.0, 45.0);
    ASSERT_TRUE(b.valid);
    const Vec e2 = rotateZ(up(30.0 * kRad, 30.0 * kRad), 45.0 * kRad);
    const Vec g2 = up(b.roll_rad, b.pitch_rad);
    EXPECT_NEAR(g2.x, e2.x, kTol);
    EXPECT_NEAR(g2.y, e2.y, kTol);
    EXPECT_NEAR(g2.z, e2.z, kTol);
}

TEST(Attitude, NonFiniteOrInvalidReadingIsInvalid) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(robotAttitudeFromVex(nan, 0.0, 0.0).valid);
    EXPECT_FALSE(robotAttitudeFromVex(0.0, inf, 0.0).valid); // PROS_ERR_F
    EXPECT_FALSE(robotAttitudeFromVex(0.0, 0.0, nan).valid);

    VexAttitude vex;
    vex.roll_deg = 5.0;
    EXPECT_FALSE(robotAttitudeFromVex(vex, 0.0).valid);
    vex.valid = true;
    expectAttitude(robotAttitudeFromVex(vex, 0.0), 5.0, 0.0);
}

TEST(Attitude, TelemetryGroupInCentidegrees) {
    translagatr::BrainTelemetry t;
    t.flags = translagatr::kTelemetryMotion;
    setAttitude(t, robotAttitudeFromVex(12.346, -6.789, 0.0));
    EXPECT_EQ(t.flags, translagatr::kTelemetryMotion | translagatr::kTelemetryAttitude);
    EXPECT_EQ(t.roll_cdeg, 1235);
    EXPECT_EQ(t.pitch_cdeg, -679);

    // Upside down stays in (-18000, 18000].
    setAttitude(t, robotAttitudeFromVex(180.0, 0.0, 0.0));
    EXPECT_EQ(t.roll_cdeg, 18000);
    RobotAttitude flip;
    flip.valid    = true;
    flip.roll_rad = -kPi;
    setAttitude(t, flip);
    EXPECT_EQ(t.roll_cdeg, 18000);

    setAttitude(t, RobotAttitude{});
    EXPECT_EQ(t.flags, translagatr::kTelemetryMotion);
    EXPECT_EQ(t.roll_cdeg, 0);
    EXPECT_EQ(t.pitch_cdeg, 0);
}
