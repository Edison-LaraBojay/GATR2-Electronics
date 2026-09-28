// quaternion.h
// Unit quaternions for body attitude. q_a_b rotates vectors expressed in
// frame b into frame a, the same reading as T_a_b, and multiply(q_a_b,
// q_b_c) yields q_a_c. Euler conversions use the project convention
// R = Rz(yaw) * Ry(pitch) * Rx(roll), right handed, radians.

#pragma once
#include <cmath>

#include "math/angles.h"
#include "math/se3.h"

namespace navigatr
{

struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline double norm(const Quaternion& q) {
    return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

inline Quaternion normalized(const Quaternion& q) {
    const double n = norm(q);
    if (n < 1e-12) {
        return Quaternion{};
    }
    return Quaternion{q.w / n, q.x / n, q.y / n, q.z / n};
}

inline bool isUnit(const Quaternion& q, double tolerance = 1e-6) {
    return std::fabs(norm(q) - 1.0) <= tolerance;
}

inline Quaternion conjugate(const Quaternion& q) { return Quaternion{q.w, -q.x, -q.y, -q.z}; }

// q_a_c = q_a_b * q_b_c
inline Quaternion multiply(const Quaternion& a, const Quaternion& b) {
    return Quaternion{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Rotation3 rotationFromQuaternion(const Quaternion& q_in) {
    const Quaternion q = normalized(q_in);
    Rotation3        R;
    R.m[0][0] = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    R.m[0][1] = 2.0 * (q.x * q.y - q.z * q.w);
    R.m[0][2] = 2.0 * (q.x * q.z + q.y * q.w);
    R.m[1][0] = 2.0 * (q.x * q.y + q.z * q.w);
    R.m[1][1] = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    R.m[1][2] = 2.0 * (q.y * q.z - q.x * q.w);
    R.m[2][0] = 2.0 * (q.x * q.z - q.y * q.w);
    R.m[2][1] = 2.0 * (q.y * q.z + q.x * q.w);
    R.m[2][2] = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    return R;
}

inline Quaternion quaternionFromRotation(const Rotation3& R) {
    Quaternion   q;
    const double trace = R.m[0][0] + R.m[1][1] + R.m[2][2];
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w            = 0.25 * s;
        q.x            = (R.m[2][1] - R.m[1][2]) / s;
        q.y            = (R.m[0][2] - R.m[2][0]) / s;
        q.z            = (R.m[1][0] - R.m[0][1]) / s;
    } else if (R.m[0][0] > R.m[1][1] && R.m[0][0] > R.m[2][2]) {
        const double s = std::sqrt(1.0 + R.m[0][0] - R.m[1][1] - R.m[2][2]) * 2.0;
        q.w            = (R.m[2][1] - R.m[1][2]) / s;
        q.x            = 0.25 * s;
        q.y            = (R.m[0][1] + R.m[1][0]) / s;
        q.z            = (R.m[0][2] + R.m[2][0]) / s;
    } else if (R.m[1][1] > R.m[2][2]) {
        const double s = std::sqrt(1.0 + R.m[1][1] - R.m[0][0] - R.m[2][2]) * 2.0;
        q.w            = (R.m[0][2] - R.m[2][0]) / s;
        q.x            = (R.m[0][1] + R.m[1][0]) / s;
        q.y            = 0.25 * s;
        q.z            = (R.m[1][2] + R.m[2][1]) / s;
    } else {
        const double s = std::sqrt(1.0 + R.m[2][2] - R.m[0][0] - R.m[1][1]) * 2.0;
        q.w            = (R.m[1][0] - R.m[0][1]) / s;
        q.x            = (R.m[0][2] + R.m[2][0]) / s;
        q.y            = (R.m[1][2] + R.m[2][1]) / s;
        q.z            = 0.25 * s;
    }
    return normalized(q);
}

inline Quaternion quaternionFromEuler(double roll_rad, double pitch_rad, double yaw_rad) {
    return quaternionFromRotation(rotationFromEuler(roll_rad, pitch_rad, yaw_rad));
}

inline void eulerFromQuaternion(const Quaternion& q, double& roll_rad, double& pitch_rad,
                                double& yaw_rad) {
    eulerFromRotation(rotationFromQuaternion(q), roll_rad, pitch_rad, yaw_rad);
}

// Yaw component only, as a rotation about the world z axis.
inline Quaternion yawQuaternion(double yaw_rad) {
    return Quaternion{std::cos(yaw_rad / 2.0), 0.0, 0.0, std::sin(yaw_rad / 2.0)};
}

// Rotation angle between two attitudes, radians in [0, pi].
inline double angleBetween(const Quaternion& a, const Quaternion& b) {
    const Quaternion d = multiply(conjugate(normalized(a)), normalized(b));
    const double     w = std::fabs(d.w) > 1.0 ? 1.0 : std::fabs(d.w);
    return 2.0 * std::acos(w);
}

// Spherical interpolation along the shortest arc, f in [0, 1].
inline Quaternion slerp(const Quaternion& a_in, const Quaternion& b_in, double f) {
    const Quaternion a = normalized(a_in);
    Quaternion       b = normalized(b_in);
    double           dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (dot < 0.0) {
        b   = Quaternion{-b.w, -b.x, -b.y, -b.z};
        dot = -dot;
    }
    if (dot > 0.9995) {
        // nearly identical: linear blend, renormalized
        return normalized(Quaternion{a.w + (b.w - a.w) * f, a.x + (b.x - a.x) * f,
                                     a.y + (b.y - a.y) * f, a.z + (b.z - a.z) * f});
    }
    const double theta0 = std::acos(dot);
    const double theta  = theta0 * f;
    const double s0     = std::cos(theta) - dot * std::sin(theta) / std::sin(theta0);
    const double s1     = std::sin(theta) / std::sin(theta0);
    return normalized(Quaternion{s0 * a.w + s1 * b.w, s0 * a.x + s1 * b.x, s0 * a.y + s1 * b.y,
                                 s0 * a.z + s1 * b.z});
}

// Splits an attitude into its yaw about world z and the remaining tilt
// (roll and pitch about the tilted body axes), so a planar heading can be
// combined with a measured tilt without applying either twice:
//   q = yawQuaternion(yaw) * tilt
inline void splitYawAndTilt(const Quaternion& q, double& yaw_rad, Quaternion& tilt) {
    double roll = 0.0, pitch = 0.0;
    eulerFromQuaternion(q, roll, pitch, yaw_rad);
    tilt = quaternionFromEuler(roll, pitch, 0.0);
}

} // namespace navigatr
