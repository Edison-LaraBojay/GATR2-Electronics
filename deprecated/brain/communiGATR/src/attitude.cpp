// attitude.cpp

#include "communigatr/attitude.h"

#include <cmath>

namespace communigatr
{

namespace
{

constexpr double kPi  = 3.14159265358979323846;
constexpr double kRad = kPi / 180.0;

// Centidegrees of an angle in (-pi, pi], wrapped to (-18000, 18000].
int16_t centidegrees(double rad) {
    long cdeg = std::lround(rad / kRad * 100.0);
    while (cdeg > 18000) {
        cdeg -= 36000;
    }
    while (cdeg <= -18000) {
        cdeg += 36000;
    }
    return static_cast<int16_t>(cdeg);
}

} // namespace

RobotAttitude robotAttitudeFromVex(double roll_deg, double pitch_deg, double mount_yaw_deg) {
    RobotAttitude a;
    if (!std::isfinite(roll_deg) || !std::isfinite(pitch_deg) || !std::isfinite(mount_yaw_deg)) {
        return a;
    }
    const double roll  = roll_deg * kRad;
    const double pitch = pitch_deg * kRad;
    const double yaw   = mount_yaw_deg * kRad;

    // Up direction in the IMU frame.
    const double ux = -std::sin(pitch);
    const double uy = std::sin(roll) * std::cos(pitch);
    const double uz = std::cos(roll) * std::cos(pitch);

    // IMU frame to robot frame: a rotation by the mount yaw about +z.
    const double c  = std::cos(yaw);
    const double s  = std::sin(yaw);
    const double rx = c * ux - s * uy;
    const double ry = s * ux + c * uy;

    a.roll_rad  = std::atan2(ry, uz);
    a.pitch_rad = std::atan2(-rx, std::hypot(ry, uz));
    a.valid     = true;
    return a;
}

RobotAttitude robotAttitudeFromVex(const VexAttitude& vex, double mount_yaw_deg) {
    if (!vex.valid) {
        return RobotAttitude{};
    }
    return robotAttitudeFromVex(vex.roll_deg, vex.pitch_deg, mount_yaw_deg);
}

void setAttitude(translagatr::BrainTelemetry& telemetry, const RobotAttitude& attitude) {
    if (!attitude.valid || !std::isfinite(attitude.roll_rad) ||
        !std::isfinite(attitude.pitch_rad)) {
        telemetry.flags &= static_cast<uint8_t>(~translagatr::kTelemetryAttitude);
        telemetry.roll_cdeg  = 0;
        telemetry.pitch_cdeg = 0;
        return;
    }
    telemetry.flags |= translagatr::kTelemetryAttitude;
    telemetry.roll_cdeg  = centidegrees(attitude.roll_rad);
    telemetry.pitch_cdeg = centidegrees(attitude.pitch_rad);
}

} // namespace communigatr
