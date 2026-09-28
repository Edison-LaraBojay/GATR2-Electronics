// se3_gtest.cpp
// SE(3) math, the project frame conventions, the fixed detector
// normalization, and the exact backing transform example. These tests pin
// the conventions so "top", "left", and "which way is 180" cannot later be
// reinterpreted.

#include <gtest/gtest.h>

#include <cmath>

#include "math/angles.h"
#include "math/se3.h"
#include "resources/target_set.h"

using namespace navigatr;

namespace
{

void expectRotationNear(const Rotation3& a, const Rotation3& b, double tol = 1e-12) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            EXPECT_NEAR(a.m[r][c], b.m[r][c], tol) << "element " << r << "," << c;
        }
    }
}

} // namespace

TEST(Se3, EulerConventionIsZyxRightHanded) {
    // yaw alone turns +x toward +y (counterclockwise from above)
    const Rotation3 yaw90 = rotationFromEuler(0, 0, kPi / 2.0);
    EXPECT_NEAR(yaw90.m[1][0], 1.0, 1e-12);   // body x maps to +y
    EXPECT_NEAR(yaw90.m[0][0], 0.0, 1e-12);

    // positive pitch tilts +x downward (toward -z)
    const Rotation3 pitch10 = rotationFromEuler(0, degToRad(10), 0);
    EXPECT_LT(pitch10.m[2][0], 0.0);

    // positive roll turns +y toward +z
    const Rotation3 roll10 = rotationFromEuler(degToRad(10), 0, 0);
    EXPECT_GT(roll10.m[2][1], 0.0);

    // round trip through the extraction
    double roll = 0, pitch = 0, yaw = 0;
    eulerFromRotation(rotationFromEuler(0.3, -0.2, 2.5), roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.3, 1e-12);
    EXPECT_NEAR(pitch, -0.2, 1e-12);
    EXPECT_NEAR(yaw, 2.5, 1e-12);
}

TEST(Se3, ComposeInverseRoundTrip) {
    const Transform3 T = makeTransform3(0.4, -1.2, 0.7, 0.3, -0.5, 1.9);
    const Transform3 I = compose(T, inverse(T));
    expectRotationNear(I.R, Rotation3{});
    EXPECT_NEAR(I.x_m, 0.0, 1e-12);
    EXPECT_NEAR(I.y_m, 0.0, 1e-12);
    EXPECT_NEAR(I.z_m, 0.0, 1e-12);

    // T_a_c = T_a_b * T_b_c moves a b-fixed point correctly
    const Transform3 T_a_b = makeTransform3(1, 0, 0, 0, 0, kPi / 2.0);
    const Transform3 T_b_c = makeTransform3(1, 0, 0, 0, 0, 0);
    const Transform3 T_a_c = compose(T_a_b, T_b_c);
    EXPECT_NEAR(T_a_c.x_m, 1.0, 1e-12);
    EXPECT_NEAR(T_a_c.y_m, 1.0, 1e-12);
}

TEST(Se3, PlanarProjectionAfterFullChain) {
    // a chain with pitch and heights still projects to the correct planar pose
    const Transform3 T_f_r = transform3FromPlanar(Pose2D{1.0, 2.0, kPi / 4.0});
    const Transform3 T_r_c = makeTransform3(0.2, 0.0, 0.3, 0.0, degToRad(15), 0.0);
    const Pose2D     planar = planarFromTransform3(compose(T_f_r, T_r_c));
    EXPECT_NEAR(planar.x_m, 1.0 + 0.2 * std::cos(kPi / 4.0), 1e-12);
    EXPECT_NEAR(planar.y_m, 2.0 + 0.2 * std::sin(kPi / 4.0), 1e-12);
    EXPECT_NEAR(planar.heading_rad, kPi / 4.0, 1e-12);   // pitch does not yaw
}

TEST(FrameConventions, RobotForwardFollowsFieldHeading) {
    // robot yaw 0: forward motion moves field +x
    Pose2D robot{0, 0, 0};
    Pose2D step{0.1, 0, 0};
    Pose2D moved = compose(robot, step);
    EXPECT_NEAR(moved.x_m, 0.1, 1e-12);
    EXPECT_NEAR(moved.y_m, 0.0, 1e-12);

    // robot yaw +90: forward motion moves field +y, robot-left is field -x
    robot.heading_rad = kPi / 2.0;
    moved             = compose(robot, step);
    EXPECT_NEAR(moved.x_m, 0.0, 1e-12);
    EXPECT_NEAR(moved.y_m, 0.1, 1e-12);
    Pose2D left = compose(robot, Pose2D{0, 0.1, 0});
    EXPECT_NEAR(left.x_m, -0.1, 1e-12);
}

TEST(FrameConventions, EngineeringCameraFromDetectorOptical) {
    const Rotation3 R = rotationEngineeringFromOptical();
    // optical forward (an object straight ahead) is engineering +x
    EXPECT_NEAR(R.m[0][2], 1.0, 1e-12);
    // image-right is engineering -y
    EXPECT_NEAR(R.m[1][0], -1.0, 1e-12);
    EXPECT_NEAR(R.m[0][0], 0.0, 1e-12);
    // image-down is engineering -z
    EXPECT_NEAR(R.m[2][1], -1.0, 1e-12);
}

TEST(AprilTagNormalization, HeadOnNativeMapsToCanonical) {
    // native head-on: translation (0, 0, distance), identity rotation
    const double     distance = 1.5;
    Transform3       native;
    native.z_m = distance;

    Transform3 T_ce_cd;
    T_ce_cd.R = rotationEngineeringFromOptical();
    Transform3 T_sd_s;
    T_sd_s.R = rotationCanonicalTagFromNative();

    const Transform3 T_ce_s = compose(compose(T_ce_cd, native), T_sd_s);

    // positive forward range on the engineering x axis
    EXPECT_NEAR(T_ce_s.x_m, distance, 1e-12);
    EXPECT_NEAR(T_ce_s.y_m, 0.0, 1e-12);
    EXPECT_NEAR(T_ce_s.z_m, 0.0, 1e-12);

    // the tag outward normal (+x of the surface frame) points back toward
    // the camera, so the canonical tag yaw is 180 degrees
    double roll = 0, pitch = 0, yaw = 0;
    eulerFromRotation(T_ce_s.R, roll, pitch, yaw);
    EXPECT_NEAR(std::fabs(yaw), kPi, 1e-12);
    EXPECT_NEAR(roll, 0.0, 1e-12);
    EXPECT_NEAR(pitch, 0.0, 1e-12);
    // outward normal in camera coordinates
    EXPECT_NEAR(T_ce_s.R.m[0][0], -1.0, 1e-12);
    // printed top is up
    EXPECT_NEAR(T_ce_s.R.m[2][2], 1.0, 1e-12);
}

TEST(AprilTagNormalization, PrintedTopRotationIsPreserved) {
    // rotate the head-on tag in the image plane (about optical forward)
    const double a = degToRad(25);
    Transform3   native;
    native.z_m = 1.0;
    native.R   = rotationFromEuler(0, 0, 0);
    // in-plane rotation in the native frame is a rotation about native z
    Rotation3 rz;
    rz.m[0][0] = std::cos(a);
    rz.m[0][1] = -std::sin(a);
    rz.m[1][0] = std::sin(a);
    rz.m[1][1] = std::cos(a);
    native.R   = rz;

    Transform3 T_ce_cd;
    T_ce_cd.R = rotationEngineeringFromOptical();
    Transform3 T_sd_s;
    T_sd_s.R = rotationCanonicalTagFromNative();
    const Transform3 T_ce_s = compose(compose(T_ce_cd, native), T_sd_s);

    // the printed top (+z of the surface) tilts by exactly the printed
    // rotation, never lost and never doubled
    const double tilt = std::acos(std::min(1.0, std::max(-1.0, T_ce_s.R.m[2][2])));
    EXPECT_NEAR(tilt, a, 1e-12);
}

TEST(BackingTransforms, ExactRearContactExample) {
    // the two legitimate 180s cancel; no third 180 for driving backwards
    TargetDecl decl;
    decl.kind                          = TargetKind::kLandmarkRelative;
    decl.T_landmark_approach           = Transform3{};   // identity: approach = landmark
    decl.T_robot_controlled            = makeTransform3(-0.40, 0, 0, 0, 0, kPi);
    decl.desired_controlled_in_approach = Pose2D{0.05, 0.0, kPi};

    const Pose2D body = decl.resolveFromLandmark(Pose2D{0, 0, 0});
    EXPECT_NEAR(body.x_m, 0.45, 1e-12);
    EXPECT_NEAR(body.y_m, 0.0, 1e-12);
    EXPECT_NEAR(body.heading_rad, 0.0, 1e-12);
}

TEST(BackingTransforms, FullChainThroughApproachFrame) {
    // center goal at the official coordinates, west approach face, rear
    // contact backing target
    TargetDecl decl;
    decl.kind                = TargetKind::kLandmarkRelative;
    decl.T_landmark_approach = makeTransform3(-0.14, 0, 0, 0, 0, kPi);
    decl.T_robot_controlled  = makeTransform3(-0.40, 0, 0.05, 0, 0, kPi);
    decl.desired_controlled_in_approach = Pose2D{0.05, 0.0, kPi};

    const Pose2D body = decl.resolveFromLandmark(Pose2D{1.7832, 1.7832, 0.0});
    EXPECT_NEAR(body.x_m, 1.7832 - 0.14 - 0.45, 1e-12);
    EXPECT_NEAR(body.y_m, 1.7832, 1e-12);
    // the robot faces away from the goal; its rear contact faces into it
    EXPECT_NEAR(std::fabs(body.heading_rad), kPi, 1e-12);
}
