// odometry.cpp

#include "odometry.h"

#include <cmath>

namespace odometry
{

Radians wrapAngle(Radians angle) {
    const Radians wrapped = std::fmod(angle + kPi, 2.0 * kPi);
    return (wrapped < 0 ? wrapped + 2.0 * kPi : wrapped) - kPi;
}

Meters wheelTravel(const TrackingWheel& wheel, int32_t delta_counts) {
    const double turns = static_cast<double>(delta_counts) / wheel.counts_per_rev;
    return wheel.direction * wheel.scale * turns * kPi * wheel.diameter;
}

BodyMotion bodyMotion(const WheelSetup& wheels, int32_t forward_counts, int32_t sideways_counts,
                      Radians turn) {
    // A wheel left of center rolls backward on a counterclockwise turn; one
    // ahead of center rolls left.
    BodyMotion motion;
    motion.forward = wheelTravel(wheels.forward, forward_counts) + wheels.forward.offset * turn;
    motion.left    = wheelTravel(wheels.sideways, sideways_counts) - wheels.sideways.offset * turn;
    motion.turn    = turn;
    return motion;
}

Pose advance(const Pose& pose, const BodyMotion& motion) {
    const Radians half = motion.turn / 2.0;
    // sin(half) / half, series near zero.
    const double chord =
        std::fabs(half) < 1e-6 ? 1.0 - half * half / 6.0 : std::sin(half) / half;
    const Radians mid     = pose.heading + half;
    const Meters  forward = motion.forward * chord;
    const Meters  left    = motion.left * chord;

    Pose next;
    next.x       = pose.x + forward * std::cos(mid) - left * std::sin(mid);
    next.y       = pose.y + forward * std::sin(mid) + left * std::cos(mid);
    next.heading = wrapAngle(pose.heading + motion.turn);
    return next;
}

Radians imuHeading(const ImuAnchor& anchor, double rotation_degrees) {
    return wrapAngle(anchor.heading - degreesToRadians(rotation_degrees - anchor.rotation_degrees));
}

} // namespace odometry
