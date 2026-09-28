// geometry_gtest.cpp

#include "investigatr/geometry.h"

#include <cmath>
#include <gtest/gtest.h>

using namespace investigatr;

TEST(Geometry, WrapAngleRange) {
    EXPECT_DOUBLE_EQ(wrapAngle(0.0), 0.0);
    EXPECT_DOUBLE_EQ(wrapAngle(kPi), kPi);
    EXPECT_DOUBLE_EQ(wrapAngle(-kPi), kPi);
    EXPECT_NEAR(wrapAngle(3.0 * kPi), kPi, 1e-12);
    EXPECT_NEAR(wrapAngle(-3.0 * kPi), kPi, 1e-12);
    EXPECT_NEAR(wrapAngle(2.0 * kPi + 0.1), 0.1, 1e-12);
    EXPECT_NEAR(wrapAngle(-2.0 * kPi - 0.1), -0.1, 1e-12);
}

TEST(Geometry, WrapAngleAcrossPi) {
    EXPECT_NEAR(wrapAngle(3.1 - (-3.1)), 6.2 - 2.0 * kPi, 1e-12);
    EXPECT_NEAR(wrapAngle(-3.1 - 3.1), 2.0 * kPi - 6.2, 1e-12);
}

TEST(Geometry, ComposeIdentity) {
    const Pose a{1.0, -2.0, 0.7};
    const Pose left  = compose(Pose{}, a);
    const Pose right = compose(a, Pose{});
    EXPECT_DOUBLE_EQ(left.x, a.x);
    EXPECT_DOUBLE_EQ(left.y, a.y);
    EXPECT_DOUBLE_EQ(left.heading, a.heading);
    EXPECT_DOUBLE_EQ(right.x, a.x);
    EXPECT_DOUBLE_EQ(right.y, a.y);
    EXPECT_DOUBLE_EQ(right.heading, a.heading);
}

TEST(Geometry, ComposeRotatesOffset) {
    // Landmark facing +y, offset 0.5 m in front of it and 0.1 m to its left.
    const Pose landmark{2.0, 1.0, kPi / 2.0};
    const Pose out = compose(landmark, Pose{0.5, 0.1, kPi});
    EXPECT_NEAR(out.x, 1.9, 1e-12);
    EXPECT_NEAR(out.y, 1.5, 1e-12);
    EXPECT_NEAR(out.heading, -kPi / 2.0, 1e-12);
}

TEST(Geometry, ComposeHeadingWraps) {
    const Pose out = compose(Pose{0.0, 0.0, 3.0}, Pose{1.0, 0.0, 0.5});
    EXPECT_NEAR(out.x, std::cos(3.0), 1e-12);
    EXPECT_NEAR(out.y, std::sin(3.0), 1e-12);
    EXPECT_NEAR(out.heading, 3.5 - 2.0 * kPi, 1e-12);
}

TEST(Geometry, ComposeIsAssociative) {
    const Pose a{0.3, -0.2, 2.5};
    const Pose b{-1.0, 0.4, 1.9};
    const Pose c{0.25, 0.75, -2.8};
    const Pose left  = compose(compose(a, b), c);
    const Pose right = compose(a, compose(b, c));
    EXPECT_NEAR(left.x, right.x, 1e-12);
    EXPECT_NEAR(left.y, right.y, 1e-12);
    EXPECT_NEAR(wrapAngle(left.heading - right.heading), 0.0, 1e-12);
}

TEST(Geometry, InverseUndoesCompose) {
    const Pose a{1.5, -0.4, 2.2};
    const Pose b{-0.3, 0.9, -1.1};
    const Pose back = compose(inverse(a), compose(a, b));
    EXPECT_NEAR(back.x, b.x, 1e-12);
    EXPECT_NEAR(back.y, b.y, 1e-12);
    EXPECT_NEAR(wrapAngle(back.heading - b.heading), 0.0, 1e-12);

    const Pose identity = compose(a, inverse(a));
    EXPECT_NEAR(identity.x, 0.0, 1e-12);
    EXPECT_NEAR(identity.y, 0.0, 1e-12);
    EXPECT_NEAR(identity.heading, 0.0, 1e-12);
}

TEST(Geometry, BetweenExpressesInFirstFrame) {
    // Robot at (1, 1) facing +y; a point 0.5 m ahead of it.
    const Pose robot{1.0, 1.0, kPi / 2.0};
    const Pose ahead{1.0, 1.5, kPi};
    const Pose rel = between(robot, ahead);
    EXPECT_NEAR(rel.x, 0.5, 1e-12);
    EXPECT_NEAR(rel.y, 0.0, 1e-12);
    EXPECT_NEAR(rel.heading, kPi / 2.0, 1e-12);

    const Pose again = compose(robot, rel);
    EXPECT_NEAR(again.x, ahead.x, 1e-12);
    EXPECT_NEAR(again.y, ahead.y, 1e-12);
    EXPECT_NEAR(wrapAngle(again.heading - ahead.heading), 0.0, 1e-12);
}

TEST(Geometry, FiniteRejectsNanAndInf) {
    EXPECT_TRUE(finite(Pose{1.0, 2.0, 3.0}));
    EXPECT_FALSE(finite(Pose{std::nan(""), 0.0, 0.0}));
    EXPECT_FALSE(finite(Pose{0.0, INFINITY, 0.0}));
    EXPECT_FALSE(finite(Pose{0.0, 0.0, -INFINITY}));
}
