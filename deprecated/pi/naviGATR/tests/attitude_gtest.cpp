// attitude_gtest.cpp
// Quaternion conventions, yaw/tilt separation, and the SE(3) body pose the
// robot state derives from a measured or assumed attitude.

#include <gtest/gtest.h>

#include "math/angles.h"
#include "math/quaternion.h"
#include "state/robot_state.h"

using namespace navigatr;

TEST(Quaternion, EulerRoundTripsUnderTheProjectConvention) {
    const double cases[][3] = {{0.1, -0.2, 0.3}, {-1.0, 0.4, 2.5}, {0.0, 0.0, -3.0},
                               {0.7, 1.2, 0.0}};
    for (const auto& c : cases) {
        const Quaternion q = quaternionFromEuler(c[0], c[1], c[2]);
        EXPECT_TRUE(isUnit(q));
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        eulerFromQuaternion(q, roll, pitch, yaw);
        EXPECT_NEAR(roll, c[0], 1e-9);
        EXPECT_NEAR(pitch, c[1], 1e-9);
        EXPECT_NEAR(yaw, c[2], 1e-9);
        // and the matrix form agrees with se3.h
        const Rotation3 R = rotationFromEuler(c[0], c[1], c[2]);
        const Rotation3 Q = rotationFromQuaternion(q);
        for (int r = 0; r < 3; ++r) {
            for (int k = 0; k < 3; ++k) {
                EXPECT_NEAR(R.m[r][k], Q.m[r][k], 1e-9);
            }
        }
    }
}

TEST(Quaternion, MultiplyConjugateAndAngle) {
    const Quaternion a = quaternionFromEuler(0.2, 0.1, 1.0);
    const Quaternion b = quaternionFromEuler(-0.1, 0.3, -0.4);
    const Quaternion ab = multiply(a, b);
    const Rotation3  R  = multiply(rotationFromQuaternion(a), rotationFromQuaternion(b));
    const Rotation3  Q  = rotationFromQuaternion(ab);
    for (int r = 0; r < 3; ++r) {
        for (int k = 0; k < 3; ++k) {
            EXPECT_NEAR(R.m[r][k], Q.m[r][k], 1e-9);
        }
    }
    const Quaternion identity = multiply(a, conjugate(a));
    EXPECT_NEAR(identity.w, 1.0, 1e-9);
    EXPECT_NEAR(angleBetween(a, a), 0.0, 1e-9);
    EXPECT_NEAR(angleBetween(yawQuaternion(0.0), yawQuaternion(0.5)), 0.5, 1e-9);
}

TEST(Quaternion, SlerpHalfwayAndShortestArc) {
    const Quaternion a   = quaternionFromEuler(0.0, 0.0, 0.0);
    const Quaternion b   = quaternionFromEuler(degToRad(40.0), 0.0, 0.0);
    const Quaternion mid = slerp(a, b, 0.5);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    eulerFromQuaternion(mid, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), 20.0, 1e-9);

    // the same rotation with the opposite sign still goes the short way
    const Quaternion b_neg{-b.w, -b.x, -b.y, -b.z};
    eulerFromQuaternion(slerp(a, b_neg, 0.5), roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), 20.0, 1e-9);
    EXPECT_NEAR(angleBetween(slerp(a, b, 0.0), a), 0.0, 1e-9);
    EXPECT_NEAR(angleBetween(slerp(a, b, 1.0), b), 0.0, 1e-9);
}

TEST(Quaternion, SplitYawAndTiltRecombinesWithoutDoubleYaw) {
    const Quaternion q   = quaternionFromEuler(0.15, -0.25, 2.0);
    double           yaw = 0.0;
    Quaternion       tilt;
    splitYawAndTilt(q, yaw, tilt);
    EXPECT_NEAR(yaw, 2.0, 1e-9);
    double r = 0.0, p = 0.0, y = 0.0;
    eulerFromQuaternion(tilt, r, p, y);
    EXPECT_NEAR(r, 0.15, 1e-9);
    EXPECT_NEAR(p, -0.25, 1e-9);
    EXPECT_NEAR(y, 0.0, 1e-9);
    // a different localization heading replaces the source yaw exactly once
    const Quaternion combined = multiply(yawQuaternion(0.5), tilt);
    eulerFromQuaternion(combined, r, p, y);
    EXPECT_NEAR(r, 0.15, 1e-9);
    EXPECT_NEAR(p, -0.25, 1e-9);
    EXPECT_NEAR(y, 0.5, 1e-9);
}

TEST(RobotState, BodyTransformUsesMeasuredTiltOrLevel) {
    RobotState r;
    r.odom_pose = Pose2D{1.0, 2.0, kPi / 2.0};
    r.attitude  = assumedLevelAttitude(kPi / 2.0);
    EXPECT_TRUE(r.attitude.assumed_level);
    EXPECT_FALSE(r.attitude.valid);
    Transform3 level = r.T_odom_robot3();
    EXPECT_NEAR(level.x_m, 1.0, 1e-12);
    EXPECT_NEAR(level.z_m, 0.0, 1e-12);
    EXPECT_NEAR(level.R.m[2][2], 1.0, 1e-12);

    r.attitude.valid            = true;
    r.attitude.assumed_level    = false;
    r.attitude.q_reference_body = multiply(yawQuaternion(kPi / 2.0),
                                           quaternionFromEuler(0.0, degToRad(10.0), 0.0));
    const Transform3 tilted = r.T_odom_robot3();
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    eulerFromRotation(tilted.R, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(pitch), 10.0, 1e-9);
    EXPECT_NEAR(yaw, kPi / 2.0, 1e-9);
    // a nose-down pitch tips body +x below the ground plane
    double bx = 0.0, by = 0.0, bz = 0.0;
    transformPoint3(tilted, 1.0, 0.0, 0.0, bx, by, bz);
    EXPECT_LT(bz, 0.0);
}
