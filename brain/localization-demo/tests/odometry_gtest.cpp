// odometry_gtest.cpp

#include "odometry.h"

#include <gtest/gtest.h>

#include <cmath>

namespace odometry
{
namespace
{

constexpr double kTol = 1e-9;

// 1000 counts per meter on both wheels, forward wheel 0.1 m right of center,
// sideways wheel 0.05 m ahead.
WheelSetup testWheels() {
    WheelSetup w;
    w.forward.channel        = 1;
    w.forward.diameter       = 1.0 / kPi;
    w.forward.counts_per_rev = 1000;
    w.forward.offset         = -0.1;
    w.sideways.channel        = 0;
    w.sideways.diameter       = 1.0 / kPi;
    w.sideways.counts_per_rev = 1000;
    w.sideways.offset         = 0.05;
    return w;
}

TEST(Odometry, WrapAngle) {
    EXPECT_NEAR(wrapAngle(0.0), 0.0, kTol);
    EXPECT_NEAR(wrapAngle(3 * kPi / 2), -kPi / 2, kTol);
    EXPECT_NEAR(wrapAngle(-3 * kPi / 2), kPi / 2, kTol);
    EXPECT_NEAR(wrapAngle(kPi), -kPi, kTol);
    EXPECT_NEAR(wrapAngle(10 * kPi + 0.25), 0.25, 1e-9);
}

TEST(Odometry, WheelTravelUsesDirectionAndScale) {
    TrackingWheel w = testWheels().forward;
    EXPECT_NEAR(wheelTravel(w, 1000), 1.0, kTol);
    w.direction = -1;
    w.scale     = 2.0;
    EXPECT_NEAR(wheelTravel(w, 1000), -2.0, kTol);
}

TEST(Odometry, PureForward) {
    const BodyMotion m = bodyMotion(testWheels(), 500, 0, 0.0);
    EXPECT_NEAR(m.forward, 0.5, kTol);
    EXPECT_NEAR(m.left, 0.0, kTol);

    const Pose facing_x = advance(Pose{1.0, 2.0, 0.0}, m);
    EXPECT_NEAR(facing_x.x, 1.5, kTol);
    EXPECT_NEAR(facing_x.y, 2.0, kTol);

    const Pose facing_y = advance(Pose{1.0, 2.0, kPi / 2}, m);
    EXPECT_NEAR(facing_y.x, 1.0, kTol);
    EXPECT_NEAR(facing_y.y, 2.5, kTol);
}

TEST(Odometry, PureSidewaysIsLeft) {
    const BodyMotion m = bodyMotion(testWheels(), 0, 300, 0.0);
    EXPECT_NEAR(m.left, 0.3, kTol);

    // Facing +x, left is +y.
    const Pose p = advance(Pose{0, 0, 0}, m);
    EXPECT_NEAR(p.x, 0.0, kTol);
    EXPECT_NEAR(p.y, 0.3, kTol);

    // Facing +y, left is -x.
    const Pose q = advance(Pose{0, 0, kPi / 2}, m);
    EXPECT_NEAR(q.x, -0.3, kTol);
    EXPECT_NEAR(q.y, 0.0, kTol);
}

TEST(Odometry, PureRotationCancelsWheelOffsets) {
    // Turning in place counterclockwise by 90 deg: the forward wheel, right
    // of center, rolls forward; the sideways wheel, ahead of center, rolls left.
    const WheelSetup w    = testWheels();
    const Radians    turn = kPi / 2;
    const int32_t    fwd  = static_cast<int32_t>(std::lround(0.1 * turn * 1000));
    const int32_t    side = static_cast<int32_t>(std::lround(0.05 * turn * 1000));
    const BodyMotion m    = bodyMotion(w, fwd, side, turn);
    EXPECT_NEAR(m.forward, 0.0, 1e-3); // count rounding
    EXPECT_NEAR(m.left, 0.0, 1e-3);

    const Pose p = advance(Pose{1.0, 1.0, 0.3}, m);
    EXPECT_NEAR(p.x, 1.0, 1e-3);
    EXPECT_NEAR(p.y, 1.0, 1e-3);
    EXPECT_NEAR(p.heading, 0.3 + turn, kTol);
}

TEST(Odometry, RotationWithoutCompensationWouldDrift) {
    // Same turn with zero offsets moves the pose: the offsets matter.
    WheelSetup w      = testWheels();
    w.forward.offset  = 0;
    w.sideways.offset = 0;
    const Pose p      = advance(Pose{}, bodyMotion(w, 157, 79, kPi / 2));
    EXPECT_GT(std::hypot(p.x, p.y), 0.1);
}

TEST(Odometry, QuarterArcEndsOnTheCircle) {
    // Forward 0.5*pi/2 m while turning 90 deg CCW: radius 0.5 arc from the origin.
    BodyMotion m;
    m.forward  = 0.5 * kPi / 2;
    m.turn     = kPi / 2;
    const Pose p = advance(Pose{}, m);
    EXPECT_NEAR(p.x, 0.5, kTol);
    EXPECT_NEAR(p.y, 0.5, kTol);
    EXPECT_NEAR(p.heading, kPi / 2, kTol);
}

TEST(Odometry, SmallStepsMatchOneArc) {
    BodyMotion step;
    step.forward = 0.01;
    step.left    = 0.002;
    step.turn    = 0.003;
    Pose many;
    for (int i = 0; i < 100; ++i) {
        many = advance(many, step);
    }
    BodyMotion all;
    all.forward    = 1.0;
    all.left       = 0.2;
    all.turn       = 0.3;
    const Pose one = advance(Pose{}, all);
    EXPECT_NEAR(many.x, one.x, 1e-9);
    EXPECT_NEAR(many.y, one.y, 1e-9);
    EXPECT_NEAR(many.heading, one.heading, 1e-9);
}

TEST(Odometry, TinyTurnIsStraightAndFinite) {
    for (double turn : {0.0, 1e-12, -1e-9, 1e-7, 2e-6}) {
        BodyMotion m;
        m.forward    = 1.0;
        m.turn       = turn;
        const Pose p = advance(Pose{}, m);
        EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y));
        EXPECT_NEAR(p.x, 1.0, 1e-9);
        EXPECT_NEAR(p.y, turn / 2, 1e-9);
    }
}

TEST(Odometry, HeadingWrapsAcrossPi) {
    BodyMotion m;
    m.turn       = 0.2;
    const Pose p = advance(Pose{0, 0, kPi - 0.1}, m);
    EXPECT_NEAR(p.heading, -kPi + 0.1, kTol);
}

TEST(Odometry, VexRotationIsClockwiseDegrees) {
    const ImuAnchor anchor{10.0, 0.0};
    // VEX +90 deg (clockwise) from the anchor reading: heading -90 deg.
    EXPECT_NEAR(imuHeading(anchor, 100.0), -kPi / 2, kTol);
    EXPECT_NEAR(imuHeading(anchor, -80.0), kPi / 2, kTol);
    // Unwrapped rotation after several turns still wraps.
    EXPECT_NEAR(imuHeading(anchor, 10.0 - 720.0 - 45.0), kPi / 4, 1e-9);
    // Anchor heading is the offset.
    EXPECT_NEAR(imuHeading(ImuAnchor{0.0, kPi / 2}, 0.0), kPi / 2, kTol);
}

} // namespace
} // namespace odometry
