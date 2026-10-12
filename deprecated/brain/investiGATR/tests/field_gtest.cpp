// field_gtest.cpp

#include "investigatr/field.h"

#include <cmath>
#include <gtest/gtest.h>

using namespace investigatr;

namespace
{

FieldObject object(ObjectId id) {
    FieldObject o;
    o.id = id;
    return o;
}

} // namespace

TEST(Field, FindBySortedId) {
    Field field;
    field.objects = {object(2), object(5), object(9)};
    ASSERT_NE(field.find(5), nullptr);
    EXPECT_EQ(field.find(5)->id, 5);
    EXPECT_EQ(field.find(2)->id, 2);
    EXPECT_EQ(field.find(9)->id, 9);
    EXPECT_EQ(field.find(1), nullptr);
    EXPECT_EQ(field.find(6), nullptr);
    EXPECT_EQ(field.find(10), nullptr);
    EXPECT_EQ(Field{}.find(1), nullptr);
}

TEST(Field, BoxCornersAxisAligned) {
    const Box box{Pose{}, 0.4, 0.2};
    Point     c[4];
    boxCorners(box, Pose{1.0, 2.0, 0.0}, c);
    const Point want[4] = {{1.2, 1.9}, {1.2, 2.1}, {0.8, 2.1}, {0.8, 1.9}};
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(c[i].x, want[i].x, 1e-12) << i;
        EXPECT_NEAR(c[i].y, want[i].y, 1e-12) << i;
    }
}

TEST(Field, BoxCornersFollowOwnerAndOffset) {
    // Box centered 0.3 m ahead of its owner and turned 90 degrees in the
    // owner frame; owner at (1, 1) facing +y.
    const Box box{Pose{0.3, 0.0, kPi / 2.0}, 0.4, 0.2};
    Point     c[4];
    boxCorners(box, Pose{1.0, 1.0, kPi / 2.0}, c);
    // Center (1, 1.3), box x axis along field -x.
    const Point want[4] = {{0.8, 1.4}, {0.8, 1.2}, {1.2, 1.2}, {1.2, 1.4}};
    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(c[i].x, want[i].x, 1e-12) << i;
        EXPECT_NEAR(c[i].y, want[i].y, 1e-12) << i;
    }
}

TEST(Field, BoxCornersCounterclockwise) {
    const Box box{Pose{0.1, -0.2, 0.7}, 0.5, 0.3};
    Point     c[4];
    boxCorners(box, Pose{2.0, -1.0, -2.4}, c);
    double area2 = 0;
    for (int i = 0; i < 4; ++i) {
        const Point& a = c[i];
        const Point& b = c[(i + 1) % 4];
        area2 += a.x * b.y - b.x * a.y;
    }
    EXPECT_NEAR(0.5 * area2, 0.5 * 0.3, 1e-12);
}
