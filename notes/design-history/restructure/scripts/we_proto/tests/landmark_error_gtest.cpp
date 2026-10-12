// landmark_error_gtest.cpp
// LandmarkError against the full-pose composition it replaces.

#include <gtest/gtest.h>

#include <random>

#include "math/angles.h"
#include "math/se3.h"
#include "math/transforms.h"
#include "resources/target_set.h"
#include "state/landmark_error.h"

using namespace navigatr;

namespace
{

constexpr double kTol = 1e-12;

// the chain of vision_targets_gtest
TargetDecl visionTarget() {
    TargetDecl decl;
    decl.kind                           = TargetKind::kLandmarkRelative;
    decl.T_landmark_approach            = makeTransform3(-0.14, 0, 0, 0, 0, kPi);
    decl.T_robot_controlled             = makeTransform3(-0.40, 0, 0.05, 0, 0, kPi);
    decl.desired_controlled_in_approach = Pose2D{0.05, 0.0, kPi};
    return decl;
}

// Random poses; every fourth heading within 1e-9 of +-pi.
struct Cases {
    std::mt19937_64                        rng{20260928};
    std::uniform_real_distribution<double> pos{-5.0, 5.0}, err{-0.5, 0.5}, ang{-kPi, kPi},
        near{-1e-9, 1e-9}, dth{-0.6, 0.6}, off{-0.6, 0.6}, tilt{-0.2, 0.2};
    int i = 0;

    double heading() {
        ++i;
        return i % 4 == 0 ? wrapAngle((i % 8 == 0 ? kPi : -kPi) + near(rng)) : ang(rng);
    }
    Pose2D pose() {
        const double x = pos(rng), y = pos(rng);
        return Pose2D{x, y, heading()};
    }
    // displaced from nominal, heading error anywhere
    Pose2D estimate(const Pose2D& nominal) {
        const double x = nominal.x_m + err(rng), y = nominal.y_m + err(rng);
        return Pose2D{x, y, i % 3 == 0 ? heading() : wrapAngle(nominal.heading_rad + dth(rng))};
    }
    TargetDecl tilted() {
        TargetDecl decl;
        decl.kind = TargetKind::kLandmarkRelative;
        decl.T_landmark_approach =
            makeTransform3(off(rng), off(rng), 0.1, tilt(rng), tilt(rng), ang(rng));
        decl.T_robot_controlled =
            makeTransform3(off(rng), off(rng), 0.05, tilt(rng), tilt(rng), ang(rng));
        decl.desired_controlled_in_approach = Pose2D{off(rng), off(rng), ang(rng)};
        return decl;
    }
};

void expectSamePose(const Pose2D& a, const Pose2D& b, double tol) {
    EXPECT_NEAR(a.x_m, b.x_m, tol);
    EXPECT_NEAR(a.y_m, b.y_m, tol);
    EXPECT_NEAR(wrapAngle(a.heading_rad - b.heading_rad), 0.0, tol);
}

} // namespace

TEST(LandmarkError, ZeroIsTheIdentity) {
    Cases cases;
    for (int n = 0; n < 10000; ++n) {
        const Pose2D landmark = cases.pose();
        const Pose2D target   = cases.pose();
        const Pose2D moved    = applyLandmarkError(LandmarkError{}, landmark, target);
        EXPECT_EQ(moved.x_m, target.x_m);
        EXPECT_EQ(moved.y_m, target.y_m);
        EXPECT_EQ(moved.heading_rad, target.heading_rad);

        const LandmarkError none = landmarkError(landmark, landmark);
        EXPECT_EQ(none.dx, 0.0);
        EXPECT_EQ(none.dy, 0.0);
        EXPECT_EQ(none.dtheta, 0.0);
    }
}

TEST(LandmarkError, RotatesTheOffsetAboutTheNominalLandmarkInFieldAxes) {
    const Pose2D landmark{1.0, 2.0, kPi / 2.0};
    const Pose2D target{1.5, 2.0, kPi / 2.0};   // 0.5 m along field +x
    const Pose2D moved =
        applyLandmarkError(LandmarkError{0.1, -0.2, kPi / 2.0}, landmark, target);
    EXPECT_NEAR(moved.x_m, 1.1, kTol);
    EXPECT_NEAR(moved.y_m, 2.3, kTol);
    EXPECT_NEAR(moved.heading_rad, kPi, kTol);
}

TEST(LandmarkError, HelperEqualsTheFullPoseComposition) {
    Cases            cases;
    const TargetDecl vision = visionTarget();
    for (int n = 0; n < 100000; ++n) {
        const Pose2D nominal         = cases.pose();
        const Pose2D estimated       = cases.estimate(nominal);
        const Pose2D field_from_odom = cases.pose();
        const TargetDecl& decl       = n % 2 == 0 ? vision : cases.tilted();

        const Pose2D old = decl.resolveFromLandmark(compose(inverse(field_from_odom), estimated));
        const Pose2D now = compose(inverse(field_from_odom),
                                   applyLandmarkError(landmarkError(nominal, estimated), nominal,
                                                      decl.resolveFromLandmark(nominal)));
        expectSamePose(now, old, kTol);
    }
}

TEST(LandmarkError, RoundTripsWithTheFieldPose) {
    Cases cases;
    for (int n = 0; n < 100000; ++n) {
        const Pose2D        nominal   = cases.pose();
        const Pose2D        estimated = cases.estimate(nominal);
        const LandmarkError error     = landmarkError(nominal, estimated);

        // nominal plus error
        expectSamePose(applyLandmarkError(error, nominal, nominal), estimated, kTol);

        const LandmarkError again =
            landmarkError(nominal, applyLandmarkError(error, nominal, nominal));
        EXPECT_NEAR(again.dx, error.dx, kTol);
        EXPECT_NEAR(again.dy, error.dy, kTol);
        EXPECT_NEAR(wrapAngle(again.dtheta - error.dtheta), 0.0, kTol);
    }
}
