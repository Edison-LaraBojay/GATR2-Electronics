// se3.h
// Full 3D rigid transforms for camera and tag chains. The same naming rule
// as the planar math applies: T_a_b is the pose of frame b expressed in
// frame a, and compose(T_a_b, T_b_c) yields T_a_c.
//
// Euler angles in configuration are applied as R = Rz(yaw) * Ry(pitch) *
// Rx(roll), right handed, radians in memory. Chains that involve a camera or
// a tag surface stay in SE(3) end to end and project to the planar pose only
// after the chain is complete.
//
// Axis conventions (documented once, enforced by tests):
//   robot body:  +x forward, +y left, +z up
//   engineering camera: +x looking direction, +y camera-left, +z camera-up
//   canonical tag surface: +x outward normal toward a viewer, +z printed
//   top, +y completes the right-handed frame

#pragma once
#include <cmath>

#include "math/angles.h"
#include "math/transforms.h"

namespace navigatr
{

struct Rotation3 {
    // row major, identity by default
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

inline Rotation3 multiply(const Rotation3& a, const Rotation3& b) {
    Rotation3 out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] =
                a.m[r][0] * b.m[0][c] + a.m[r][1] * b.m[1][c] + a.m[r][2] * b.m[2][c];
        }
    }
    return out;
}

inline Rotation3 transpose(const Rotation3& a) {
    Rotation3 out;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] = a.m[c][r];
        }
    }
    return out;
}

// R = Rz(yaw) * Ry(pitch) * Rx(roll)
inline Rotation3 rotationFromEuler(double roll_rad, double pitch_rad, double yaw_rad) {
    const double cr = std::cos(roll_rad), sr = std::sin(roll_rad);
    const double cp = std::cos(pitch_rad), sp = std::sin(pitch_rad);
    const double cy = std::cos(yaw_rad), sy = std::sin(yaw_rad);
    Rotation3 out;
    out.m[0][0] = cy * cp;
    out.m[0][1] = cy * sp * sr - sy * cr;
    out.m[0][2] = cy * sp * cr + sy * sr;
    out.m[1][0] = sy * cp;
    out.m[1][1] = sy * sp * sr + cy * cr;
    out.m[1][2] = sy * sp * cr - cy * sr;
    out.m[2][0] = -sp;
    out.m[2][1] = cp * sr;
    out.m[2][2] = cp * cr;
    return out;
}

// Inverse of rotationFromEuler for the same ZYX convention.
inline void eulerFromRotation(const Rotation3& R, double& roll_rad, double& pitch_rad,
                              double& yaw_rad) {
    pitch_rad = std::asin(-R.m[2][0] < -1.0   ? -1.0
                          : -R.m[2][0] > 1.0 ? 1.0
                                             : -R.m[2][0]);
    if (std::fabs(R.m[2][0]) < 1.0 - 1e-9) {
        roll_rad = std::atan2(R.m[2][1], R.m[2][2]);
        yaw_rad  = std::atan2(R.m[1][0], R.m[0][0]);
    } else {
        // gimbal lock: fold everything into yaw
        roll_rad = 0.0;
        yaw_rad  = std::atan2(-R.m[0][1], R.m[1][1]);
    }
}

struct Transform3 {
    Rotation3 R;
    double    x_m = 0.0;
    double    y_m = 0.0;
    double    z_m = 0.0;
};

inline Transform3 makeTransform3(double x_m, double y_m, double z_m, double roll_rad,
                                 double pitch_rad, double yaw_rad) {
    Transform3 out;
    out.R   = rotationFromEuler(roll_rad, pitch_rad, yaw_rad);
    out.x_m = x_m;
    out.y_m = y_m;
    out.z_m = z_m;
    return out;
}

// T_a_c = T_a_b * T_b_c
inline Transform3 compose(const Transform3& T_a_b, const Transform3& T_b_c) {
    Transform3 out;
    out.R = multiply(T_a_b.R, T_b_c.R);
    out.x_m = T_a_b.x_m + T_a_b.R.m[0][0] * T_b_c.x_m + T_a_b.R.m[0][1] * T_b_c.y_m +
              T_a_b.R.m[0][2] * T_b_c.z_m;
    out.y_m = T_a_b.y_m + T_a_b.R.m[1][0] * T_b_c.x_m + T_a_b.R.m[1][1] * T_b_c.y_m +
              T_a_b.R.m[1][2] * T_b_c.z_m;
    out.z_m = T_a_b.z_m + T_a_b.R.m[2][0] * T_b_c.x_m + T_a_b.R.m[2][1] * T_b_c.y_m +
              T_a_b.R.m[2][2] * T_b_c.z_m;
    return out;
}

// T_b_a from T_a_b
inline Transform3 inverse(const Transform3& T_a_b) {
    Transform3 out;
    out.R = transpose(T_a_b.R);
    out.x_m =
        -(out.R.m[0][0] * T_a_b.x_m + out.R.m[0][1] * T_a_b.y_m + out.R.m[0][2] * T_a_b.z_m);
    out.y_m =
        -(out.R.m[1][0] * T_a_b.x_m + out.R.m[1][1] * T_a_b.y_m + out.R.m[1][2] * T_a_b.z_m);
    out.z_m =
        -(out.R.m[2][0] * T_a_b.x_m + out.R.m[2][1] * T_a_b.y_m + out.R.m[2][2] * T_a_b.z_m);
    return out;
}

// A point fixed in frame b, expressed in frame a.
inline void transformPoint3(const Transform3& T_a_b, double x_b, double y_b, double z_b,
                            double& x_a, double& y_a, double& z_a) {
    x_a = T_a_b.x_m + T_a_b.R.m[0][0] * x_b + T_a_b.R.m[0][1] * y_b + T_a_b.R.m[0][2] * z_b;
    y_a = T_a_b.y_m + T_a_b.R.m[1][0] * x_b + T_a_b.R.m[1][1] * y_b + T_a_b.R.m[1][2] * z_b;
    z_a = T_a_b.z_m + T_a_b.R.m[2][0] * x_b + T_a_b.R.m[2][1] * y_b + T_a_b.R.m[2][2] * z_b;
}

// Planar projection, taken only after an SE(3) chain is complete: x, y, and
// the yaw of the transformed body x axis.
inline Pose2D planarFromTransform3(const Transform3& T) {
    Pose2D out;
    out.x_m         = T.x_m;
    out.y_m         = T.y_m;
    out.heading_rad = wrapAngle(std::atan2(T.R.m[1][0], T.R.m[0][0]));
    return out;
}

// Lifts a planar pose into SE(3) with the given height and a pure yaw.
inline Transform3 transform3FromPlanar(const Pose2D& pose, double z_m = 0.0) {
    return makeTransform3(pose.x_m, pose.y_m, z_m, 0.0, 0.0, pose.heading_rad);
}

// Fixed conversion from the detector-native optical camera frame Cd
// (+x image-right, +y image-down, +z optical-forward) into the engineering
// camera frame Ce (+x looking direction, +y camera-left, +z camera-up).
// This constant lives in perception code only; individual camera
// configurations never encode it as extra rotations.
inline Rotation3 rotationEngineeringFromOptical() {
    Rotation3 out;
    out.m[0][0] = 0;  out.m[0][1] = 0;  out.m[0][2] = 1;
    out.m[1][0] = -1; out.m[1][1] = 0;  out.m[1][2] = 0;
    out.m[2][0] = 0;  out.m[2][1] = -1; out.m[2][2] = 0;
    return out;
}

// Fixed conversion from the detector-native tag frame Sd (+x image-right on
// the tag, +y image-down, +z optical-forward, identity rotation when viewed
// head on) into the canonical tag surface frame S (+x outward normal toward
// the viewer, +z printed top, +y right-handed completion).
inline Rotation3 rotationCanonicalTagFromNative() {
    Rotation3 out;
    out.m[0][0] = 0;  out.m[0][1] = 1; out.m[0][2] = 0;
    out.m[1][0] = 0;  out.m[1][1] = 0; out.m[1][2] = -1;
    out.m[2][0] = -1; out.m[2][1] = 0; out.m[2][2] = 0;
    return out;
}

} // namespace navigatr
