#include "drive_config.h"
#include "mecanum.h"

#include <gtest/gtest.h>

#include <limits>

namespace drive
{
namespace
{

void expectWheels(const WheelCommands& actual, int front_left, int front_right, int back_left,
                  int back_right) {
    EXPECT_EQ(actual.front_left, front_left);
    EXPECT_EQ(actual.front_right, front_right);
    EXPECT_EQ(actual.back_left, back_left);
    EXPECT_EQ(actual.back_right, back_right);
}

TEST(Mecanum, CenteredSticksStopEveryWheel) {
    expectWheels(mix(0, 0, 0), 0, 0, 0, 0);
}

TEST(Mecanum, ForwardAndReverseMoveAllWheelsTogether) {
    expectWheels(mix(127, 0, 0), 12000, 12000, 12000, 12000);
    expectWheels(mix(-127, 0, 0), -12000, -12000, -12000, -12000);
}

TEST(Mecanum, StrafingUsesOppositeDiagonalPairs) {
    expectWheels(mix(0, 127, 0), 12000, -12000, -12000, 12000);
    expectWheels(mix(0, -127, 0), -12000, 12000, 12000, -12000);
}

TEST(Mecanum, RotationUsesOppositeSides) {
    expectWheels(mix(0, 0, 127), 12000, -12000, 12000, -12000);
    expectWheels(mix(0, 0, -127), -12000, 12000, -12000, 12000);
}

TEST(Mecanum, SmallInputsKeepProportionalVoltage) {
    expectWheels(mix(64, 0, 0), 6047, 6047, 6047, 6047);
    expectWheels(mix(-64, 0, 0), -6047, -6047, -6047, -6047);
}

TEST(Mecanum, DeadbandIncludesBoundaryAndAppliesBeforeMixing) {
    expectWheels(mix(8, -8, 8), 0, 0, 0, 0);
    expectWheels(mix(-8, 8, -8), 0, 0, 0, 0);
    expectWheels(mix(9, 8, -8), 850, 850, 850, 850);
    expectWheels(mix(-8, -9, 8), -850, 850, 850, -850);
    expectWheels(mix(8, -8, -9), -850, 850, -850, 850);
    expectWheels(mix(15, -15, 15, 15), 0, 0, 0, 0);
}

TEST(Mecanum, DiagonalTranslationStopsTheOtherDiagonal) {
    expectWheels(mix(127, 127, 0), 12000, 0, 0, 12000);
    expectWheels(mix(127, -127, 0), 0, 12000, 12000, 0);
}

TEST(Mecanum, SaturatedMixturesPreserveWheelRatios) {
    expectWheels(mix(100, 40, 20), 12000, 3000, 6000, 9000);
    expectWheels(mix(-100, -40, -20), -12000, -3000, -6000, -9000);
    expectWheels(mix(127, 127, 127), 12000, -4000, 4000, 4000);
}

TEST(Mecanum, OutOfRangeAxesAreClampedBeforeMixing) {
    const int highest = std::numeric_limits<int>::max();
    const int lowest  = std::numeric_limits<int>::min();
    expectWheels(mix(highest, 0, 0), 12000, 12000, 12000, 12000);
    expectWheels(mix(lowest, 0, 0), -12000, -12000, -12000, -12000);
    expectWheels(mix(highest, lowest, highest), 4000, 4000, 12000, -4000);
}

TEST(Mecanum, VoltageLimitScalesAllWheelsAndStaysWithinMotorRange) {
    expectWheels(mix(100, 40, 20, 8, 6000), 6000, 1500, 3000, 4500);
    expectWheels(mix(127, 127, 127, 8, 0), 0, 0, 0, 0);
    expectWheels(mix(127, 127, 127, 8, -1), 0, 0, 0, 0);
    expectWheels(mix(127, 127, 127, 8, std::numeric_limits<int>::max()),
                 12000, -4000, 4000, 4000);
}

TEST(Mecanum, InvalidDeadbandIsClampedToControllerRange) {
    expectWheels(mix(1, 0, 0, -1), 94, 94, 94, 94);
    expectWheels(mix(127, -127, 127, std::numeric_limits<int>::max()), 0, 0, 0, 0);
}

} // namespace
} // namespace drive

namespace drive_config
{
namespace
{

MotorPorts validMotorPorts() {
    return {MotorPair{1, -2}, MotorPair{3, -4}, MotorPair{5, -6}, MotorPair{20, -21}};
}

TEST(DriveConfig, EightUniquePortsAcceptReversalsAndSmartPortBoundaries) {
    EXPECT_TRUE(validPorts(validMotorPorts(), 9));
}

TEST(DriveConfig, EveryMotorPortMustBeSet) {
    for (std::size_t wheel = 0; wheel < 4; ++wheel) {
        for (std::size_t motor = 0; motor < 2; ++motor) {
            SCOPED_TRACE(wheel * 2 + motor);
            MotorPorts ports = validMotorPorts();
            ports[wheel][motor] = 0;
            EXPECT_FALSE(validPorts(ports, 9));
        }
    }
}

TEST(DriveConfig, PositiveAndReversedPortsMustFitSmartPortRange) {
    for (const int invalid : {22, -22, 127, -128}) {
        SCOPED_TRACE(invalid);
        MotorPorts ports = validMotorPorts();
        ports[3][1] = static_cast<std::int8_t>(invalid);
        EXPECT_FALSE(validPorts(ports, 9));
    }
}

TEST(DriveConfig, DuplicatePhysicalPortsAreRejectedWithinAndAcrossWheels) {
    MotorPorts ports = validMotorPorts();
    ports[0][1] = ports[0][0];
    EXPECT_FALSE(validPorts(ports, 9));

    ports = validMotorPorts();
    ports[3][1] = ports[0][0];
    EXPECT_FALSE(validPorts(ports, 9));

    ports = validMotorPorts();
    ports[3][1] = -ports[0][0];
    EXPECT_FALSE(validPorts(ports, 9));

    ports = validMotorPorts();
    ports[3][1] = -ports[0][1];
    EXPECT_FALSE(validPorts(ports, 9));
}

TEST(DriveConfig, EveryMotorMustUseADifferentPhysicalPortFromTheImu) {
    const MotorPorts ports = validMotorPorts();
    for (const MotorPair& pair : ports) {
        for (const int signed_port : pair) {
            const int imu_port = signed_port < 0 ? -signed_port : signed_port;
            SCOPED_TRACE(imu_port);
            EXPECT_FALSE(validPorts(ports, imu_port));
        }
    }
}

} // namespace
} // namespace drive_config
