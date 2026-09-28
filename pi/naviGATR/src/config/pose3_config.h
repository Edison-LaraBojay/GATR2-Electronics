// pose3_config.h
// Strict pose parsing helpers. XML uses meters and degrees; C++ converts to
// radians here. Every attribute is required: an unmeasured value must fail
// the build, never silently become zero.
//
// SE(3) elements carry x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg applied
// as R = Rz(yaw) * Ry(pitch) * Rx(roll). Planar elements carry x_m, y_m,
// heading_deg.

#pragma once
#include <string>

#include "config/config_node.h"
#include "math/angles.h"
#include "math/se3.h"

namespace navigatr
{

inline bool parseTransform3(const ConfigNode& node, Transform3& out, std::string& err) {
    double x = 0.0, y = 0.0, z = 0.0, roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    if (!node.requireDouble("x_m", x, err) || !node.requireDouble("y_m", y, err) ||
        !node.requireDouble("z_m", z, err) ||
        !node.requireDouble("roll_deg", roll_deg, err) ||
        !node.requireDouble("pitch_deg", pitch_deg, err) ||
        !node.requireDouble("yaw_deg", yaw_deg, err)) {
        return false;
    }
    out = makeTransform3(x, y, z, degToRad(roll_deg), degToRad(pitch_deg),
                         degToRad(yaw_deg));
    return true;
}

inline bool parsePlanarPose(const ConfigNode& node, Pose2D& out, std::string& err) {
    double heading_deg = 0.0;
    if (!node.requireDouble("x_m", out.x_m, err) ||
        !node.requireDouble("y_m", out.y_m, err) ||
        !node.requireDouble("heading_deg", heading_deg, err)) {
        return false;
    }
    out.heading_rad = degToRad(heading_deg);
    return true;
}

} // namespace navigatr
