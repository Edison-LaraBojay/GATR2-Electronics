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

    if (!estimator->attitude_.configure(node, context, err)) {
        return nullptr;
    }
    return estimator;
}

void PlanarMotionIntegrator::reset() {
    placement_.reset();
    attitude_.reset();
    clock_.reset();
    motion_clock_.clear();
}

StateEstimatorOutput PlanarMotionIntegrator::run(const StateEstimatorInput& in) {
    StateEstimatorOutput out;
    out.robot     = in.previous;
    RobotState& r = out.robot;
    for (const auto& observation : in.observations) {
        if (observation.first != motion_ref_ && observation.first != heading_ref_ &&
            observation.first != attitude_.ref()) {
            out.rejected.push_back(observation.first);
            out.diagnostic = "observation not consumed by this estimator";
        }
    }

    // A placement request re-anchors the odometry frame in the field frame.
    // The continuous odometry pose is untouched, so anything latched in the
    // odometry frame keeps its physical meaning.
    placement_.apply(r, in.requests.placement);

    const auto applyAttitude = [&](bool clock_valid) {
        attitude_.apply(out, in, clock_, motion_clock_, clock_valid);
    };

    const auto motion_it = in.observations.find(motion_ref_);
    if (motion_it == in.observations.end()) {
        out.status       = FunctionStatus::kNoData;   // pose and effective time hold
        out.clock_mapped = clock_.valid();
        applyAttitude(clock_.valid());
        return out;
    }
    const BodyMotionIncrement* motion = motion_it->second.payload.get<BodyMotionIncrement>();
    if (motion == nullptr) {
        out.rejected.push_back(motion_ref_);
        out.status = FunctionStatus::kFault;
        applyAttitude(clock_.valid());
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
        applyAttitude(clock_.valid());
        return out;
    }

    // A device clock domain alone does not identify a clock. An attitude
    // from another device must never borrow the wheel device offset.
    std::string source_clock;
    for (const auto& source : motion->sources) {
        if (stamp.domain != ClockDomain::kDevice) break;
        if (source.clock.empty()) continue;
        if (!source_clock.empty() && source_clock != source.clock) {
            out.rejected.push_back(motion_ref_);
            out.status = FunctionStatus::kFault;
            out.diagnostic = "motion observation combines different source clocks";
            applyAttitude(clock_.valid());
            return out;
        }
        source_clock = source.clock;
    }
    if (stamp.domain == ClockDomain::kDevice && source_clock != motion_clock_) {
        clock_.reset();
        attitude_.forget();
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
                applyAttitude(clock_.valid());
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
        applyAttitude(clock_.valid());
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
        attitude_.forget();
        out.status     = FunctionStatus::kFault;
        out.diagnostic = "source time regression; odometry epoch advanced";
        applyAttitude(false);
        return out;
    }

    if (r.measuredAt.isSet() && sameDomain(r.measuredAt, stamp) &&
        stamp.ms == r.measuredAt.ms) {
        out.rejected.push_back(motion_ref_);
        if (heading_used) out.rejected.push_back(heading_ref_);
        out.status = FunctionStatus::kNoData;
        out.diagnostic = "motion effective time already consumed";
        applyAttitude(clock_.valid());
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

    double lx = 0.0, ly = 0.0;
    chordOfArc(dx, dy, dtheta, lx, ly);

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
    applyAttitude(clock_.valid());
    return out;
}

} // namespace navigatr
