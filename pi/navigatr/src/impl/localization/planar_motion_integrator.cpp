// planar_motion_integrator.cpp

#include "impl/localization/planar_motion_integrator.h"

#include <cmath>
#include <cstdlib>
#include <typeindex>

#include "math/angles.h"

namespace navigatr
{

std::unique_ptr<StateEstimator>
PlanarMotionIntegrator::create(const ConfigNode& node, StateEstimatorInitializationContext& context,
                               std::string& err) {
    auto estimator = std::make_unique<PlanarMotionIntegrator>();

    estimator->motion_ref_ = ObservationId{node.child("Motion").attr("observation_id")};
    if (estimator->motion_ref_.empty()) {
        err = node.path() + ": needs <Motion observation_id=.../>";
        return nullptr;
    }
    const std::type_index motion_type(typeid(BodyMotionIncrement));
    if (!context.requireObservation(estimator->motion_ref_, &motion_type, node.path(), err)) {
        return nullptr;
    }

    const ConfigNode heading = node.child("Heading");
    if (heading.valid()) {
        estimator->heading_ref_ = ObservationId{heading.attr("observation_id")};
        if (estimator->heading_ref_.empty()) {
            err = heading.path() + ": Heading needs observation_id";
            return nullptr;
        }
        const std::type_index heading_type(typeid(HeadingIncrement));
        if (!context.requireObservation(estimator->heading_ref_, &heading_type, heading.path(),
                                        err)) {
            return nullptr;
        }
        if (!heading.getInt("interval_tolerance_ms", 20, estimator->heading_tolerance_ms_,
                            err)) {
            return nullptr;
        }
        if (estimator->heading_tolerance_ms_ < 0) {
            err = heading.path() + ": interval_tolerance_ms cannot be negative";
            return nullptr;
        }
    }

    const ConfigNode attitude = node.child("Attitude");
    if (attitude.valid()) {
        estimator->attitude_ref_ = ObservationId{attitude.attr("observation_id")};
        if (estimator->attitude_ref_.empty()) {
            err = attitude.path() + ": Attitude needs observation_id";
            return nullptr;
        }
        const std::type_index attitude_type(typeid(AttitudeObservation));
        if (!context.requireObservation(estimator->attitude_ref_, &attitude_type,
                                        attitude.path(), err)) {
            return nullptr;
        }
        if (!attitude.getInt("max_age_ms", 200, estimator->attitude_max_age_ms_, err)) {
            return nullptr;
        }
        if (estimator->attitude_max_age_ms_ <= 0) {
            err = attitude.path() + ": max_age_ms must be positive";
            return nullptr;
        }
    }
    return estimator;
}

void PlanarMotionIntegrator::reset() {
    last_placement_origin_.clear();
    last_placement_sequence_ = 0;
    attitude_                = RetainedAttitude{};
    clock_.reset();
    motion_clock_.clear();
}

void PlanarMotionIntegrator::applyAttitude(StateEstimatorOutput& out, const StateEstimatorInput& in,
                                           bool clock_valid) {
    RobotState& r = out.robot;
    if (!attitude_ref_.empty()) {
        const auto it = in.observations.find(attitude_ref_);
        if (it != in.observations.end()) {
            const AttitudeObservation* obs = it->second.payload.get<AttitudeObservation>();
            if (obs != nullptr && isUnit(obs->q_reference_body, 1e-3) &&
                std::isfinite(obs->quality) && obs->measuredAt.isSet()) {
                out.accepted.push_back(attitude_ref_);
                attitude_.valid       = true;
                attitude_.observation = *obs;
                attitude_.hostAt      = MonotonicTime{};
                if (obs->measuredAt.domain == ClockDomain::kHost) {
                    attitude_.hostAt = obs->measuredAt;
                } else if (obs->measuredAt.domain == ClockDomain::kDevice && clock_valid &&
                           !motion_clock_.empty() && obs->source.clock == motion_clock_) {
                    attitude_.hostAt = clock_.toHost(obs->measuredAt);
                }
            } else {
                attitude_ = RetainedAttitude{};
                out.rejected.push_back(attitude_ref_);
                out.diagnostic = "invalid attitude observation";
            }
        } else if (attitude_.valid && !attitude_.hostAt.isSet() &&
                   attitude_.observation.measuredAt.domain == ClockDomain::kDevice &&
                   !motion_clock_.empty() &&
                   attitude_.observation.source.clock == motion_clock_ &&
                   clock_valid) {
            attitude_.hostAt = clock_.toHost(attitude_.observation.measuredAt);
        }
    }

    const double heading = r.odom_pose.heading_rad;
    if (attitude_.valid && attitude_.hostAt.isSet() && in.context.now.isSet() &&
        sameDomain(in.context.now, attitude_.hostAt) &&
        (in.context.now - attitude_.hostAt) >= 0 &&
        (in.context.now - attitude_.hostAt) <= attitude_max_age_ms_) {
        // measured tilt under the planar heading: the attitude source's own
        // yaw is never applied on top of the localization heading
        double     yaw = 0.0;
        Quaternion tilt;
        splitYawAndTilt(attitude_.observation.q_reference_body, yaw, tilt);
        Attitude a;
        a.valid            = true;
        a.q_reference_body = multiply(yawQuaternion(heading), tilt);
        a.reference        = "odometry";
        a.measuredAt       = attitude_.hostAt;
        a.measuredAtSource = attitude_.observation.measuredAt;
        a.source           = attitude_.observation.source.source;
        a.epoch            = attitude_.observation.source.epoch;
        a.quality          = attitude_.observation.quality;
        a.assumed_level    = false;
        r.attitude         = a;
    } else {
        Attitude level = assumedLevelAttitude(heading);
        if (attitude_.valid) {
            level.source           = attitude_.observation.source.source;
            level.measuredAt       = attitude_.hostAt;
            level.measuredAtSource = attitude_.observation.measuredAt;
        }
        r.attitude = level;
    }
}

StateEstimatorOutput PlanarMotionIntegrator::run(const StateEstimatorInput& in) {
    StateEstimatorOutput out;
    out.robot     = in.previous;
    RobotState& r = out.robot;
    for (const auto& observation : in.observations) {
        if (observation.first != motion_ref_ && observation.first != heading_ref_ &&
            observation.first != attitude_ref_) {
            out.rejected.push_back(observation.first);
            out.diagnostic = "observation not consumed by this estimator";
        }
    }

    // A placement request re-anchors the odometry frame in the field frame.
    // The continuous odometry pose is untouched, so anything latched in the
    // odometry frame keeps its physical meaning.
    const PlacementRequest& placement = in.requests.placement;
    if (placement.requested &&
        (placement.origin != last_placement_origin_ ||
         placement.sequence != last_placement_sequence_)) {
        last_placement_origin_   = placement.origin;
        last_placement_sequence_ = placement.sequence;
        r.field_from_odom        = compose(placement.pose, inverse(r.odom_pose));
        r.anchor_revision += 1;
        r.valid       = true;
        r.initialized = true;
    }

    const auto motion_it = in.observations.find(motion_ref_);
    if (motion_it == in.observations.end()) {
        out.status       = FunctionStatus::kNoData;   // pose and effective time hold
        out.clock_mapped = clock_.valid();
        applyAttitude(out, in, clock_.valid());
        return out;
    }
    const BodyMotionIncrement* motion = motion_it->second.payload.get<BodyMotionIncrement>();
    if (motion == nullptr) {
        out.rejected.push_back(motion_ref_);
        out.status = FunctionStatus::kFault;
        applyAttitude(out, in, clock_.valid());
        return out;
    }

    double              dx     = motion->dx_m;
    double              dy     = motion->dy_m;
    double              dtheta = motion->dtheta_rad;
    bool                have_rotation = motion->has_rotation;
    const double        dt     = motion->dt_s;
    const MonotonicTime stamp  = motion->endAt.isSet() ? motion->endAt : motion_it->second.measuredAt;

    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dtheta) ||
        !std::isfinite(dt) || dt <= 0.0 || !stamp.isSet()) {
        out.rejected.push_back(motion_ref_);
        out.status     = FunctionStatus::kFault;
        out.diagnostic = "motion increment without a positive interval";
        applyAttitude(out, in, clock_.valid());
        return out;
    }

    // A device clock domain alone does not identify a clock. An attitude
    // from another device must never borrow the wheel device's offset.
    std::string source_clock;
    for (const auto& source : motion->sources) {
        if (stamp.domain != ClockDomain::kDevice) break;
        if (source.clock.empty()) continue;
        if (!source_clock.empty() && source_clock != source.clock) {
            out.rejected.push_back(motion_ref_);
            out.status = FunctionStatus::kFault;
            out.diagnostic = "motion observation combines different source clocks";
            applyAttitude(out, in, clock_.valid());
            return out;
        }
        source_clock = source.clock;
    }
    if (stamp.domain == ClockDomain::kDevice && source_clock != motion_clock_) {
        clock_.reset();
        attitude_ = RetainedAttitude{};
        motion_clock_ = source_clock;
    }

    bool heading_used = false;
    if (!heading_ref_.empty()) {
        const auto heading_it = in.observations.find(heading_ref_);
        if (heading_it != in.observations.end()) {
            const HeadingIncrement* heading = heading_it->second.payload.get<HeadingIncrement>();
            if (heading == nullptr || !std::isfinite(heading->dtheta_rad)) {
                out.rejected.push_back(heading_ref_);
                out.status = FunctionStatus::kFault;
                applyAttitude(out, in, clock_.valid());
                return out;
            }
            bool independent = true;
            for (const Provenance& hs : heading->sources) {
                for (const Provenance& ms : motion->sources) {
                    if (hs.source == ms.source) {
                        independent = false;
                    }
                }
            }
            const bool aligned =
                heading->startAt.isSet() && motion->startAt.isSet() &&
                sameDomain(heading->startAt, motion->startAt) &&
                sameDomain(heading->endAt, motion->endAt) &&
                std::llabs(heading->startAt - motion->startAt) <= heading_tolerance_ms_ &&
                std::llabs(heading->endAt - motion->endAt) <= heading_tolerance_ms_;
            if (!independent) {
                out.rejected.push_back(heading_ref_);
                out.diagnostic = "heading shares a source with the motion observation; ignored";
            } else if (!aligned) {
                out.diagnostic = "heading interval does not match the motion interval; ignored";
                if (!sameDomain(heading->endAt, motion->endAt) ||
                    heading->endAt.ms <= motion->endAt.ms) {
                    out.rejected.push_back(heading_ref_);
                } else if (!have_rotation) {
                    out.rejected.push_back(motion_ref_);
                }
            } else {
                dtheta        = heading->dtheta_rad;
                have_rotation = true;
                heading_used  = true;
            }
        }
    }

    if (!have_rotation) {
        out.status     = FunctionStatus::kNoData;
        out.diagnostic = "motion increment without observed rotation and no aligned heading";
        applyAttitude(out, in, clock_.valid());
        return out;
    }

    // Device time running backwards means the source rebooted: the odometry
    // frame is discontinuous. Bump the epoch so odometry-anchored latches
    // are invalidated, and do not integrate the garbage step.
    if (r.measuredAt.isSet() && sameDomain(r.measuredAt, stamp) && stamp < r.measuredAt) {
        out.rejected.push_back(motion_ref_);
        if (heading_used) out.rejected.push_back(heading_ref_);
        r.odometry_epoch += 1;
        r.vx_m_s         = 0.0;
        r.vy_m_s         = 0.0;
        r.yaw_rate_rad_s = 0.0;
        r.measuredAt     = stamp;
        r.measuredAtHost = MonotonicTime{};
        clock_.reset();   // the device clock restarted; old offsets are void
        attitude_ = RetainedAttitude{};
        out.status     = FunctionStatus::kFault;
        out.diagnostic = "source time regression; odometry epoch advanced";
        applyAttitude(out, in, false);
        return out;
    }

    if (r.measuredAt.isSet() && sameDomain(r.measuredAt, stamp) &&
        stamp.ms == r.measuredAt.ms) {
        out.rejected.push_back(motion_ref_);
        if (heading_used) out.rejected.push_back(heading_ref_);
        out.status = FunctionStatus::kNoData;
        out.diagnostic = "motion effective time already consumed";
        applyAttitude(out, in, clock_.valid());
        return out;
    }

    // Pair the source stamp with the actual host receipt of the newest
    // consumed sample; pairing with the loop time would count processing
    // delay as clock offset.
    if (stamp.domain == ClockDomain::kDevice) {
        if (motion_it->second.receivedAt.domain == ClockDomain::kHost) {
            clock_.observe(stamp, motion_it->second.receivedAt);
        }
    }

    // chord of the constant-curvature arc across this step
    double lx = dx;
    double ly = dy;
    if (std::fabs(dtheta) > 1e-9) {
        const double s = std::sin(dtheta) / dtheta;
        const double c = (1.0 - std::cos(dtheta)) / dtheta;
        lx             = dx * s - dy * c;
        ly             = dx * c + dy * s;
    }

    const double h  = r.odom_pose.heading_rad;
    const double gx = lx * std::cos(h) - ly * std::sin(h);
    const double gy = lx * std::sin(h) + ly * std::cos(h);

    r.odom_pose.x_m += gx;
    r.odom_pose.y_m += gy;
    r.odom_pose.heading_rad = wrapAngle(h + dtheta);

    r.vx_m_s         = gx / dt;
    r.vy_m_s         = gy / dt;
    r.yaw_rate_rad_s = dtheta / dt;

    r.valid      = true;
    r.confidence = 1.0;
    r.measuredAt = stamp;
    if (stamp.domain == ClockDomain::kHost) {
        r.measuredAtHost = stamp;
    } else if (clock_.valid()) {
        r.measuredAtHost = clock_.toHost(stamp);
    } else {
        r.measuredAtHost = MonotonicTime{};
    }
    out.advanced     = true;
    out.accepted.push_back(motion_ref_);
    if (heading_used) out.accepted.push_back(heading_ref_);
    out.clock_mapped = stamp.domain == ClockDomain::kHost || clock_.valid();
    applyAttitude(out, in, clock_.valid());
    return out;
}

} // namespace navigatr
