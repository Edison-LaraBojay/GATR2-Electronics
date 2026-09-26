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
