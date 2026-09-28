// motion_model_gtest.cpp

#include "investigatr/motion_model.h"

#include <cmath>
#include <gtest/gtest.h>
#include <string>

using namespace investigatr;

namespace
{

MotionModel model() {
    MotionModel m;
    m.footprint = Footprint{0.2, 0.2, 0.15, 0.15};
    return m;
}

} // namespace

TEST(MotionModel, EnclosingRadiusSymmetric) {
    MotionModel m = model();
    m.clearance   = 0.05;
    EXPECT_NEAR(enclosingRadius(m), std::hypot(0.2, 0.15) + 0.05, 1e-12);
}

TEST(MotionModel, EnclosingRadiusUsesFarthestCorner) {
    // Origin off center: the far corner is front right.
    MotionModel m;
    m.footprint = Footprint{0.3, 0.1, 0.2, 0.25};
    m.clearance = 0.02;
    EXPECT_NEAR(enclosingRadius(m), std::hypot(0.3, 0.25) + 0.02, 1e-12);

    m.footprint = Footprint{0.05, 0.4, 0.3, 0.0};
    m.clearance = 0.0;
    EXPECT_NEAR(enclosingRadius(m), std::hypot(0.4, 0.3), 1e-12);
}

TEST(MotionModel, DefaultsNeedAFootprint) {
    const char* why = nullptr;
    EXPECT_FALSE(valid(MotionModel{}, &why));
    ASSERT_NE(why, nullptr);
    EXPECT_TRUE(valid(model(), &why));
    EXPECT_EQ(why, nullptr);
}

TEST(MotionModel, ValidRejectsEachBadField) {
    struct Case {
        void (*edit)(MotionModel&);
        const char* what;
    };
    const Case cases[] = {
        {[](MotionModel& m) { m.footprint.front = -0.1; }, "footprint sides"},
        {[](MotionModel& m) { m.footprint.left = NAN; }, "footprint sides"},
        {[](MotionModel& m) { m.footprint.left = m.footprint.right = 0; }, "area"},
        {[](MotionModel& m) { m.clearance = -0.01; }, "clearance"},
        {[](MotionModel& m) { m.min_turn_radius = -1; }, "min_turn_radius"},
        {[](MotionModel& m) { m.limits.max_speed = 0; }, "max_speed"},
        {[](MotionModel& m) { m.limits.max_accel = INFINITY; }, "max_accel"},
        {[](MotionModel& m) { m.limits.max_omega = -2; }, "max_omega"},
        {[](MotionModel& m) { m.limits.max_alpha = 0; }, "max_alpha"},
    };
    for (const Case& c : cases) {
        MotionModel m = model();
        c.edit(m);
        const char* why = nullptr;
        EXPECT_FALSE(valid(m, &why)) << c.what;
        ASSERT_NE(why, nullptr) << c.what;
        EXPECT_NE(std::string(why).find(c.what), std::string::npos) << why;
    }
}

TEST(MotionModel, ValidWithoutReason) {
    EXPECT_TRUE(valid(model()));
    EXPECT_FALSE(valid(MotionModel{}));
}
