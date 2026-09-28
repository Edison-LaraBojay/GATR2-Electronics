// ports_gtest.cpp

#include "actugatr/ports.h"

#include <gtest/gtest.h>
#include <string>

using namespace actugatr;

TEST(PortMap, NamesBothDevicesOfAConflict) {
    PortMap map;
    EXPECT_TRUE(map.add(1, "VEX IMU"));
    MotorGroup left;
    left.count     = 2;
    left.motors[0] = {1, true};
    left.motors[1] = {2, true};
    EXPECT_FALSE(map.add(left, "left drive"));
    EXPECT_FALSE(map.ok());
    EXPECT_EQ(std::string(map.error()), "port 1: VEX IMU and left drive");
    EXPECT_STREQ(map.owner(2), "left drive");
    EXPECT_EQ(map.owner(3), nullptr);
}

TEST(PortMap, RejectsPortsOutsideTheBrain) {
    PortMap map;
    EXPECT_FALSE(map.add(0, "link"));
    EXPECT_EQ(std::string(map.error()), "link: port 0 outside 1..21");
    PortMap other;
    EXPECT_FALSE(other.add(22, "imu"));
    EXPECT_TRUE(PortMap().ok());
}

TEST(PortMap, KeepsTheFirstProblem) {
    PortMap map;
    map.add(5, "a");
    map.add(5, "b");
    map.add(5, "c");
    EXPECT_EQ(std::string(map.error()), "port 5: a and b");
}

TEST(PortMap, WholeDrivetrains) {
    TankConfig t;
    t.left.count      = 1;
    t.left.motors[0]  = {11, false};
    t.right.count     = 1;
    t.right.motors[0] = {11, true};
    PortMap map;
    EXPECT_FALSE(map.add(t));
    EXPECT_EQ(std::string(map.error()), "port 11: left drive and right drive");
}

TEST(PortMap, ConstantListsForStaticAssert) {
    constexpr uint8_t good[] = {1, 11, 12, 21};
    constexpr uint8_t twice[] = {1, 2, 1};
    constexpr uint8_t range[] = {0, 2};
    static_assert(distinctPorts(good), "distinct");
    static_assert(!distinctPorts(twice), "duplicate");
    static_assert(!distinctPorts(range), "range");
    SUCCEED();
}
