// transforms_gtest.cpp
// T_A_B convention math, SI units.

#include <gtest/gtest.h>

#include "math/transforms.h"

using namespace navigatr;

TEST(Transforms, ComposeChainsFrames) {
    // robot 1 m forward of field origin facing +y; point 0.1 m ahead of robot
    Transform2D T_field_robot;
    T_field_robot.x_m         = 1.0;
    T_field_robot.heading_rad = kPi / 2.0;

    Transform2D T_robot_point;
    T_robot_point.x_m = 0.1;

    const Transform2D T_field_point = compose(T_field_robot, T_robot_point);
    EXPECT_NEAR(T_field_point.x_m, 1.0, 1e-12);
    EXPECT_NEAR(T_field_point.y_m, 0.1, 1e-12);
    EXPECT_NEAR(T_field_point.heading_rad, kPi / 2.0, 1e-12);
}

TEST(Transforms, InverseRoundTripsToIdentity) {
    Transform2D T;
    T.x_m         = 0.123;
    T.y_m         = -0.456;
    T.heading_rad = 0.7;

    const Transform2D I = compose(T, inverse(T));
    EXPECT_NEAR(I.x_m, 0.0, 1e-12);
    EXPECT_NEAR(I.y_m, 0.0, 1e-12);
    EXPECT_NEAR(I.heading_rad, 0.0, 1e-12);
}

TEST(Transforms, LandmarkRelativeTargetResolves) {
    // T_field_goal = T_field_landmark * T_landmark_goal
    Transform2D T_field_landmark;
    T_field_landmark.x_m         = 1.8;
    T_field_landmark.y_m         = 0.9;
    T_field_landmark.heading_rad = kPi;   // landmark faces -x

    Transform2D T_landmark_goal;
    T_landmark_goal.x_m = 0.3;   // 0.3 m in front of the landmark face

    const Transform2D T_field_goal = compose(T_field_landmark, T_landmark_goal);
    EXPECT_NEAR(T_field_goal.x_m, 1.5, 1e-12);
    EXPECT_NEAR(T_field_goal.y_m, 0.9, 1e-12);
}

TEST(Transforms, WrapAngleAndWireConversion) {
    EXPECT_NEAR(wrapAngle(3.0 * kPi), kPi, 1e-12);
    EXPECT_NEAR(wrapAngle(-3.0 * kPi), kPi, 1e-12);
    EXPECT_EQ(radToCdeg(kPi), 18000);
    EXPECT_EQ(radToCdeg(-kPi / 2.0), -9000);
}
