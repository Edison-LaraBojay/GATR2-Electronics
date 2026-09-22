// motion_step.cpp

#include "impl/localization/motion_step.h"

#include <cmath>
#include <typeindex>

namespace navigatr
{

bool PlacementEdge::apply(RobotState& r, const PlacementRequest& placement) {
    if (!placement.requested ||
        (placement.origin == origin && placement.sequence == sequence)) {
        return false;
    }
    origin            = placement.origin;
    sequence          = placement.sequence;
    r.field_from_odom = compose(placement.pose, inverse(r.odom_pose));
    r.anchor_revision += 1;
    r.valid       = true;
    r.initialized = true;
    return true;
}

void chordOfArc(double dx, double dy, double dtheta, double& lx, double& ly) {
    lx = dx;
    ly = dy;
    if (std::fabs(dtheta) > 1e-9) {
        const double s = std::sin(dtheta) / dtheta;
        const double c = (1.0 - std::cos(dtheta)) / dtheta;
        lx             = dx * s - dy * c;
        ly             = dx * c + dy * s;
    }
}

bool AttitudeFold::configure(const ConfigNode& node,
                             const StateEstimatorInitializationContext& context,
                             std::string& err) {
    const ConfigNode attitude = node.child("Attitude");
    if (!attitude.valid()) {
        return true;
    }
    ref_ = ObservationId{attitude.attr("observation_id")};
    if (ref_.empty()) {
        err = attitude.path() + ": Attitude needs observation_id";
        return false;
    }
    const std::type_index attitude_type(typeid(AttitudeObservation));
    if (!context.requireObservation(ref_, &attitude_type, attitude.path(), err)) {
        return false;
    }
    if (!attitude.getInt("max_age_ms", 200, max_age_ms_, err)) {
        return false;
    }
    if (max_age_ms_ <= 0) {
        err = attitude.path() + ": max_age_ms must be positive";
        return false;
    }
    return true;
}

void AttitudeFold::apply(StateEstimatorOutput& out, const StateEstimatorInput& in,
                         const DeviceToHostClock& clock, const std::string& motion_clock,
                         bool clock_valid) {
    RobotState& r = out.robot;
    if (!ref_.empty()) {
        const auto it = in.observations.find(ref_);
        if (it != in.observations.end()) {
            const AttitudeObservation* obs = it->second.payload.get<AttitudeObservation>();
            if (obs != nullptr && isUnit(obs->q_reference_body, 1e-3) &&
                std::isfinite(obs->quality) && obs->measuredAt.isSet()) {
                out.accepted.push_back(ref_);
                retained_.valid       = true;
                retained_.observation = *obs;
                retained_.hostAt      = MonotonicTime{};
                if (obs->measuredAt.domain == ClockDomain::kHost) {
                    retained_.hostAt = obs->measuredAt;
                } else if (obs->measuredAt.domain == ClockDomain::kDevice && clock_valid &&
                           !motion_clock.empty() && obs->source.clock == motion_clock) {
                    retained_.hostAt = clock.toHost(obs->measuredAt);
                }
            } else {
                retained_ = Retained{};
                out.rejected.push_back(ref_);
                out.diagnostic = "invalid attitude observation";
            }
        } else if (retained_.valid && !retained_.hostAt.isSet() &&
                   retained_.observation.measuredAt.domain == ClockDomain::kDevice &&
                   !motion_clock.empty() &&
                   retained_.observation.source.clock == motion_clock && clock_valid) {
            retained_.hostAt = clock.toHost(retained_.observation.measuredAt);
        }
    }

    const double heading = r.odom_pose.heading_rad;
    if (retained_.valid && retained_.hostAt.isSet() && in.context.now.isSet() &&
        sameDomain(in.context.now, retained_.hostAt) &&
        (in.context.now - retained_.hostAt) >= 0 &&
        (in.context.now - retained_.hostAt) <= max_age_ms_) {
        // measured tilt under the planar heading
        double     yaw = 0.0;
        Quaternion tilt;
        splitYawAndTilt(retained_.observation.q_reference_body, yaw, tilt);
        Attitude a;
        a.valid            = true;
        a.q_reference_body = multiply(yawQuaternion(heading), tilt);
        a.reference        = "odometry";
        a.measuredAt       = retained_.hostAt;
        a.measuredAtSource = retained_.observation.measuredAt;
        a.source           = retained_.observation.source.source;
        a.epoch            = retained_.observation.source.epoch;
        a.quality          = retained_.observation.quality;
        a.assumed_level    = false;
        r.attitude         = a;
    } else {
        Attitude level = assumedLevelAttitude(heading);
        if (retained_.valid) {
            level.source           = retained_.observation.source.source;
            level.measuredAt       = retained_.hostAt;
            level.measuredAtSource = retained_.observation.measuredAt;
        }
        r.attitude = level;
    }
}

} // namespace navigatr
