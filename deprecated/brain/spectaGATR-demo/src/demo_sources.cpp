// demo_sources.cpp

#include "demo_sources.h"

#include <cmath>

#include "demo_config.h"

namespace
{

using spectagatr::degreesToRadians;
using spectagatr::inchesToMeters;
using spectagatr::kPi;
using spectagatr::Milliseconds;
using spectagatr::Radians;

// Phase of a periodic motion, radians.
double phase(Milliseconds t, Milliseconds period) {
    return 2.0 * kPi * static_cast<double>(t % period) / static_cast<double>(period);
}

// Robot frame roll and pitch from VEX IMU roll and pitch. Retained from
// the former link client's robotAttitudeFromVex helper: the IMU up
// direction turned by the mount yaw into the robot frame. Assumes VEX reports
// roll and pitch in the robot convention in its own frame; UNVERIFIED on
// hardware.
void robotTilt(double roll_deg, double pitch_deg, double mount_yaw_deg, Radians& roll,
               Radians& pitch) {
    const double r   = degreesToRadians(roll_deg);
    const double p   = degreesToRadians(pitch_deg);
    const double yaw = degreesToRadians(mount_yaw_deg);

    // Up direction in the IMU frame.
    const double ux = -std::sin(p);
    const double uy = std::sin(r) * std::cos(p);
    const double uz = std::cos(r) * std::cos(p);

    // IMU frame to robot frame: a rotation by the mount yaw about +z.
    const double c  = std::cos(yaw);
    const double s  = std::sin(yaw);
    const double rx = c * ux - s * uy;
    const double ry = s * ux + c * uy;

    roll  = std::atan2(ry, uz);
    pitch = std::atan2(-rx, std::hypot(ry, uz));
}

} // namespace

DemoPose syntheticPose(Milliseconds t) {
    using namespace demo_config;
    const double a = inchesToMeters(kEightHalfWidthInches);
    const double b = inchesToMeters(kEightHalfHeightInches);
    const double w = phase(t, kEightPeriodMs);

    DemoPose pose;
    pose.x = inchesToMeters(kEightCenterXInches) + a * std::sin(w);
    pose.y = inchesToMeters(kEightCenterYInches) + b * std::sin(2.0 * w);
    // Along the path: the direction of (dx/dt, dy/dt), never both zero.
    pose.attitude.yaw = std::atan2(2.0 * b * std::cos(2.0 * w), a * std::cos(w));

    pose.full_attitude = (t / kAttitudePhaseMs) % 2 == 1;
    if (pose.full_attitude) {
        pose.attitude.roll =
            degreesToRadians(kWobbleRollDegrees) * std::sin(phase(t, kWobbleRollPeriodMs));
        pose.attitude.pitch =
            degreesToRadians(kWobblePitchDegrees) * std::sin(phase(t, kWobblePitchPeriodMs));
    }
    return pose;
}

DemoPose imuPose(const VexImuReading& reading) {
    using namespace demo_config;
    DemoPose pose;
    pose.x             = inchesToMeters(kImuFixedXInches);
    pose.y             = inchesToMeters(kImuFixedYInches);
    pose.full_attitude = true;
    // PROS rotation is clockwise positive; heading is CCW positive.
    pose.attitude.yaw = degreesToRadians(kImuStartHeadingDegrees - reading.rotation);
    robotTilt(kImuRollSign * reading.roll, kImuPitchSign * reading.pitch, kImuMountYawDegrees,
              pose.attitude.roll, pose.attitude.pitch);
    return pose;
}
