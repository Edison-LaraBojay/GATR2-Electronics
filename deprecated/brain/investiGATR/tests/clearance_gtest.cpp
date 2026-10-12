// clearance_gtest.cpp

#include "investigatr/planner.h"

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

using namespace investigatr;

namespace
{

FieldObject obstacle(ObjectId id, const Pose& pose, Meters length, Meters width,
                     const Pose& offset = {}) {
    FieldObject o;
    o.id       = id;
    o.kind     = ObjectKind::kFixed;
    o.obstacle = true;
    o.nominal  = pose;
    o.pose     = pose;
    o.valid    = true;
    o.source   = EstimateSource::kNominal;
    o.box      = Box{offset, length, width};
    return o;
}

Field field(std::vector<FieldObject> objects, const Bounds& bounds = Bounds{-5, -5, 5, 5}) {
    Field f;
    f.generation = 1;
    f.map.id     = 0xF1E1D;
    f.bounds     = bounds;
    f.objects    = std::move(objects);
    return f;
}

Footprint square(Meters half) { return Footprint{half, half, half, half}; }

} // namespace

TEST(Clearance, SeparatedAlongAnAxis) {
    const Field     f = field({obstacle(3, Pose{1.0, 0.0, 0.0}, 0.2, 0.2)});
    const Clearance c = footprintClearance(Pose{}, square(0.1), 0.05, f);
    EXPECT_TRUE(c.clear);
    EXPECT_NEAR(c.distance, 0.8 - 0.05, 1e-12);
    EXPECT_EQ(c.nearest, 3);
}

TEST(Clearance, CornerToCornerIsEuclidean) {
    const Field     f = field({obstacle(3, Pose{1.0, 1.0, 0.0}, 0.2, 0.2)});
    const Clearance c = footprintClearance(Pose{}, square(0.1), 0.0, f);
    EXPECT_TRUE(c.clear);
    EXPECT_NEAR(c.distance, std::hypot(0.8, 0.8), 1e-12);
}

TEST(Clearance, RotatedObstacle) {
    // Diamond whose left vertex is at x = 0.8.
    const Meters    side = 0.2 * std::sqrt(2.0);
    const Field     f    = field({obstacle(3, Pose{1.0, 0.0, kPi / 4.0}, side, side)});
    const Clearance c    = footprintClearance(Pose{}, square(0.1), 0.0, f);
    EXPECT_NEAR(c.distance, 0.7, 1e-12);

    // Moved up, the diamond's lower edge faces the robot's corner.
    const Field     g = field({obstacle(3, Pose{0.5, 0.5, kPi / 4.0}, side, side)});
    const Clearance d = footprintClearance(Pose{}, square(0.1), 0.0, g);
    // Edge x + y = 0.8 against corner (0.1, 0.1).
    EXPECT_NEAR(d.distance, (0.8 - 0.2) / std::sqrt(2.0), 1e-12);
}

TEST(Clearance, RotatedRobot) {
    const Field     f = field({obstacle(3, Pose{1.0, 0.0, 0.0}, 0.2, 0.2)});
    const Clearance c = footprintClearance(Pose{0.0, 0.0, kPi / 4.0}, square(0.1), 0.0, f);
    EXPECT_NEAR(c.distance, 0.9 - 0.1 * std::sqrt(2.0), 1e-12);
}

TEST(Clearance, OverlapReportsPenetrationDepth) {
    const Field     f = field({obstacle(4, Pose{0.15, 0.0, 0.0}, 0.2, 0.2)});
    const Clearance c = footprintClearance(Pose{}, square(0.1), 0.0, f);
    EXPECT_FALSE(c.clear);
    EXPECT_NEAR(c.distance, -0.05, 1e-12);
    EXPECT_EQ(c.nearest, 4);

    const Clearance m = footprintClearance(Pose{}, square(0.1), 0.02, f);
    EXPECT_NEAR(m.distance, -0.07, 1e-12);
}

TEST(Clearance, ContainedObstacle) {
    const Field     f = field({obstacle(4, Pose{0.05, 0.0, 0.0}, 0.02, 0.02)});
    const Clearance c = footprintClearance(Pose{}, square(0.1), 0.0, f);
    EXPECT_FALSE(c.clear);
    // Shortest way out is along +x: 0.1 + 0.01 - 0.05.
    EXPECT_NEAR(c.distance, -0.06, 1e-12);
}

TEST(Clearance, TouchingIsClearWithoutMargin) {
    const Field f = field({obstacle(4, Pose{0.2, 0.0, 0.0}, 0.2, 0.2)});
    EXPECT_TRUE(footprintClearance(Pose{}, square(0.1), 0.0, f).clear);
    const Clearance c = footprintClearance(Pose{}, square(0.1), 0.01, f);
    EXPECT_FALSE(c.clear);
    EXPECT_NEAR(c.distance, -0.01, 1e-12);
}

TEST(Clearance, Bounds) {
    const Field     f = field({}, Bounds{0, 0, 2, 2});
    const Footprint fp{0.1, 0.2, 0.1, 0.1};

    Clearance c = footprintClearance(Pose{0.3, 1.0, 0.0}, fp, 0.0, f);
    EXPECT_TRUE(c.clear);
    EXPECT_NEAR(c.distance, 0.1, 1e-12);
    EXPECT_EQ(c.nearest, 0);

    c = footprintClearance(Pose{0.3, 1.0, kPi}, fp, 0.0, f);
    EXPECT_NEAR(c.distance, 0.2, 1e-12);

    c = footprintClearance(Pose{0.3, 1.9, kPi / 2.0}, fp, 0.0, f);
    EXPECT_NEAR(c.distance, 0.0, 1e-12);

    c = footprintClearance(Pose{0.05, 1.0, kPi}, fp, 0.0, f);
    EXPECT_FALSE(c.clear);
    EXPECT_NEAR(c.distance, -0.05, 1e-12);
    EXPECT_EQ(c.nearest, 0);
}

TEST(Clearance, FootprintAboutTheReportedOrigin) {
    // Origin near the back: 0.3 m ahead, 0.05 m behind.
    const Footprint fp{0.3, 0.05, 0.1, 0.1};
    const Field     f = field({obstacle(8, Pose{0.5, 1.0, 0.0}, 0.2, 0.2)});
    EXPECT_NEAR(footprintClearance(Pose{1.0, 1.0, kPi}, fp, 0.0, f).distance, 0.1, 1e-12);
    EXPECT_NEAR(footprintClearance(Pose{1.0, 1.0, 0.0}, fp, 0.0, f).distance, 0.35, 1e-12);
}

TEST(Clearance, NearestObject) {
    const Field f = field(
        {obstacle(2, Pose{0.0, 0.6, 0.0}, 0.2, 0.2), obstacle(5, Pose{0.7, 0.0, 0.0}, 0.2, 0.2)},
        Bounds{-1, -1, 1, 1});
    Clearance c = footprintClearance(Pose{}, square(0.1), 0.0, f);
    EXPECT_NEAR(c.distance, 0.4, 1e-12);
    EXPECT_EQ(c.nearest, 2);

    c = footprintClearance(Pose{0.0, 0.0, 0.0}, Footprint{0.1, 0.9, 0.1, 0.1}, 0.0, f);
    EXPECT_NEAR(c.distance, 0.1, 1e-12);
    EXPECT_EQ(c.nearest, 0);
}

TEST(Clearance, EstimateWhenValidElseNominal) {
    FieldObject o = obstacle(6, Pose{0.5, 0.0, 0.0}, 0.2, 0.2);
    o.source      = EstimateSource::kObserved;
    o.pose        = Pose{2.0, 0.0, 0.0};
    Field f       = field({o});
    EXPECT_NEAR(footprintClearance(Pose{}, square(0.1), 0.0, f).distance, 1.8, 1e-12);

    f.objects[0].valid  = false;
    f.objects[0].source = EstimateSource::kNone;
    EXPECT_NEAR(footprintClearance(Pose{}, square(0.1), 0.0, f).distance, 0.3, 1e-12);
}

TEST(Clearance, BoxOffsetInObjectFrame) {
    // Landmark origin at (1, 0) facing +y; its box sits 0.3 m ahead of it.
    const Field     f = field({obstacle(7, Pose{1.0, 0.0, kPi / 2.0}, 0.2, 0.2, Pose{0.3, 0, 0})});
    const Clearance c = footprintClearance(Pose{1.0, 0.0, 0.0}, square(0.1), 0.0, f);
    EXPECT_TRUE(c.clear);
    EXPECT_NEAR(c.distance, 0.1, 1e-12);
    EXPECT_FALSE(footprintClearance(Pose{1.0, 0.3, 0.0}, square(0.1), 0.0, f).clear);
}

TEST(Clearance, NonObstaclesIgnored) {
    FieldObject o = obstacle(9, Pose{}, 0.5, 0.5);
    o.obstacle    = false;
    const Field f = field({o});
    EXPECT_TRUE(footprintClearance(Pose{}, square(0.1), 0.0, f).clear);
}

TEST(Clearance, InvalidInputIsNotClear) {
    const Field f = field({obstacle(3, Pose{2.0, 0.0, 0.0}, 0.2, 0.2)});
    EXPECT_TRUE(footprintClearance(Pose{}, square(0.1), 0.0, f).clear);
    EXPECT_FALSE(footprintClearance(Pose{NAN, 0, 0}, square(0.1), 0.0, f).clear);
    EXPECT_FALSE(footprintClearance(Pose{}, Footprint{-0.1, 0.1, 0.1, 0.1}, 0.0, f).clear);
    EXPECT_FALSE(footprintClearance(Pose{}, square(0.1), -0.01, f).clear);
    EXPECT_FALSE(footprintClearance(Pose{}, square(0.1), NAN, f).clear);

    Field bad        = f;
    bad.bounds.max_x = bad.bounds.min_x;
    EXPECT_FALSE(footprintClearance(Pose{}, square(0.1), 0.0, bad).clear);
    bad                   = f;
    bad.objects[0].pose.x = INFINITY;
    EXPECT_FALSE(footprintClearance(Pose{}, square(0.1), 0.0, bad).clear);
    bad                      = f;
    bad.objects[0].box.width = -1;
    EXPECT_FALSE(footprintClearance(Pose{}, square(0.1), 0.0, bad).clear);
}
