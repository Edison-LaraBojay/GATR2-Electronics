// motion_step.cpp

#include "impl/localization/motion_step.h"

#include <cmath>
#include <string>
#include <typeindex>

namespace navigatr
{

namespace
{

std::string window(MonotonicTime start, MonotonicTime end) {
    return std::to_string(start.ms) + ".." + std::to_string(end.ms);
}

std::string clockName(const std::string& clock) { return clock.empty() ? "(unnamed)" : clock; }

} // namespace

bool sharesMeasurement(const std::vector<Provenance>& a, const std::vector<Provenance>& b) {
    for (const Provenance& pa : a) {
        for (const Provenance& pb : b) {
            if (measurementOf(pa) == measurementOf(pb)) {
                return true;
            }
        }
    }
    return false;
}

bool observationClock(MonotonicTime stamp, const std::vector<Provenance>& sources,
                      std::string& clock, std::string& why) {
    clock.clear();
    if (stamp.domain == ClockDomain::kHost) {
        clock = "host";
        return true;
    }
    if (stamp.domain != ClockDomain::kDevice) {
        why = "stamp is on no clock";
        return false;
    }
    for (const Provenance& source : sources) {
        if (source.clock.empty()) {
            continue;
        }
        if (!clock.empty() && clock != source.clock) {
            why = "sources combine different clocks " + clock + " and " + source.clock;
            return false;
        }
        clock = source.clock;
    }
    return true;
}

bool comparableClocks(const std::string& a, const std::string& b, std::string& why) {
    if (a.empty() || b.empty()) {
        why = "insufficient clock identity (" + clockName(a) + " vs " + clockName(b) + ")";
        return false;
    }
    if (a != b) {
        why = "different clocks " + a + " and " + b;
        return false;
    }
    return true;
}

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

HeadingAligner::Result HeadingAligner::align(const HeadingIncrement*    heading,
                                             const BodyMotionIncrement& motion,
                                             const std::string& motion_clock, MonotonicTime now,
                                             long max_wait_ms) {
    // the deadline belongs to the motion window, from its first sighting
    if (!window_.tracked || window_.start.ms != motion.startAt.ms ||
        window_.end.ms != motion.endAt.ms) {
        window_.tracked = true;
        window_.start   = motion.startAt;
        window_.end     = motion.endAt;
        window_.since   = now;
    }
    const bool expired = now.isSet() && window_.since.isSet() &&
                         sameDomain(now, window_.since) && (now - window_.since) > max_wait_ms;

    Result result  = alignWindow(heading, motion, motion_clock);
    result.expired = expired;
    if (result.outcome == Outcome::kWaiting && expired) {
        result.outcome  = Outcome::kNone;
        result.released = true;
        result.diagnostic += "; heading support " + window(stash_.motion_start, stash_.end) +
                             " never completed motion window " +
                             window(motion.startAt, motion.endAt) +
                             " within max_wait_ms; released without it";
        stash_ = Stash{};
    }
    // kNone may still be held by a caller without rotation, so the deadline
    // stays until the window is fused, released, or expired; a new window
    // replaces it anyway
    if (result.outcome == Outcome::kFused || result.released || expired) {
        window_ = Window{};
    }
    return result;
}

HeadingAligner::Result HeadingAligner::alignWindow(const HeadingIncrement*    heading,
                                                   const BodyMotionIncrement& motion,
                                                   const std::string&         motion_clock) {
    Result result;

    // a stash covers exactly one motion window; anything else is stale
    if (stash_.active && (stash_.motion_start.ms != motion.startAt.ms ||
                          stash_.motion_end.ms != motion.endAt.ms)) {
        stash_ = Stash{};
        result.diagnostic = "partial heading support discarded: motion window changed; ";
    }

    if (heading == nullptr) {
        if (!stash_.active) {
            result.outcome = Outcome::kNone;
            return result;
        }
        result.outcome = Outcome::kWaiting;
        result.diagnostic += "heading support " + window(stash_.motion_start, stash_.end) +
                             " covers part of motion window " +
                             window(stash_.motion_start, stash_.motion_end) + "; holding";
        return result;
    }

    // the offered heading must live on the motion's clock
    std::string heading_clock;
    std::string why;
    if (!observationClock(heading->endAt, heading->sources, heading_clock, why) ||
        !comparableClocks(motion_clock, heading_clock, why)) {
        result.outcome        = stash_.active ? Outcome::kWaiting : Outcome::kNone;
        result.reject_heading = true;
        result.diagnostic += "heading not on the motion clock: " + why + "; rejected";
        return result;
    }
    if (sharesMeasurement(heading->sources, motion.sources)) {
        result.outcome        = stash_.active ? Outcome::kWaiting : Outcome::kNone;
        result.reject_heading = true;
        result.diagnostic +=
            "heading shares a measurement with the motion observation; not counted twice";
        return result;
    }
    if (!heading->startAt.isSet() || !heading->endAt.isSet() ||
        heading->endAt.ms <= heading->startAt.ms) {
        result.outcome        = stash_.active ? Outcome::kWaiting : Outcome::kNone;
        result.reject_heading = true;
        result.diagnostic += "heading without a positive interval; rejected";
        return result;
    }

    const uint64_t epoch = heading->sources.empty() ? 0 : heading->sources.front().epoch;
    if (stash_.active && (stash_.clock != heading_clock || stash_.epoch != epoch)) {
        stash_ = Stash{};
        result.diagnostic += "partial heading support discarded: heading source restarted; ";
    }

    const MonotonicTime expected_start = stash_.active ? stash_.end : motion.startAt;
    if (heading->startAt.ms != expected_start.ms) {
        result.outcome        = stash_.active ? Outcome::kWaiting : Outcome::kNone;
        result.reject_heading = true;
        result.diagnostic += "heading window " + window(heading->startAt, heading->endAt) +
                             " does not continue motion window " +
                             window(motion.startAt, motion.endAt) + " at " +
                             std::to_string(expected_start.ms) + "; rejected";
        return result;
    }
    if (heading->endAt.ms > motion.endAt.ms) {
        // no later heading can start inside the window any more
        result.outcome        = Outcome::kNone;
        result.reject_heading = true;
        result.released       = stash_.active;
        result.diagnostic += "heading window " + window(heading->startAt, heading->endAt) +
                             " overruns motion window " + window(motion.startAt, motion.endAt) +
                             "; rejected" +
                             (stash_.active ? ", partial support released" : "");
        stash_ = Stash{};
        return result;
    }

    if (!stash_.active) {
        stash_.active        = true;
        stash_.motion_start  = motion.startAt;
        stash_.motion_end    = motion.endAt;
        stash_.clock         = heading_clock;
        stash_.epoch         = epoch;
    }
    stash_.end = heading->endAt;
    stash_.dtheta += heading->dtheta_rad;
    stash_.dt += heading->dt_s > 0.0 ? heading->dt_s
                                     : (heading->endAt.ms - heading->startAt.ms) / 1000.0;
    result.accept_heading = true;

    if (stash_.end.ms == motion.endAt.ms) {
        result.outcome    = Outcome::kFused;
        result.dtheta_rad = stash_.dtheta;
        result.dt_s       = stash_.dt;
        stash_            = Stash{};
        return result;
    }
    result.outcome = Outcome::kWaiting;
    result.diagnostic += "heading support " + window(stash_.motion_start, stash_.end) +
                         " covers part of motion window " +
                         window(stash_.motion_start, stash_.motion_end) + "; holding";
    return result;
}

bool configureHeading(const ConfigNode& node, const StateEstimatorInitializationContext& context,
                      ObservationId& ref, long& max_wait_ms, std::string& err) {
    const ConfigNode heading = node.child("Heading");
    ref                      = ObservationId{};
    if (!heading.valid()) {
        return true;
    }
    if (heading.next("Heading").valid()) {
        err = node.path() + ": at most one Heading is supported";
        return false;
    }
    ref = ObservationId{heading.attr("observation_id")};
    if (ref.empty()) {
        err = heading.path() + ": Heading needs observation_id";
        return false;
    }
    const std::type_index heading_type(typeid(HeadingIncrement));
    if (!context.requireObservation(ref, &heading_type, heading.path(), err)) {
        return false;
    }
    if (heading.hasAttr("interval_tolerance_ms")) {
        err = heading.path() + ": interval_tolerance_ms is gone; heading support must equal "
              "the motion window exactly, use max_wait_ms for how long a motion waits for it";
        return false;
    }
    if (!heading.getInt("max_wait_ms", 100, max_wait_ms, err)) {
        return false;
    }
    if (max_wait_ms <= 0) {
        err = heading.path() + ": max_wait_ms must be positive";
        return false;
    }
    return true;
}

bool prepareStep(PlanarStepState& state, const StateEstimatorInput& in,
                 const ObservationId& motion_ref, const ObservationId& heading_ref,
                 long max_wait_ms, StateEstimatorOutput& out, PreparedStep& step) {
    RobotState& r = out.robot;
    // a heading offered for the same window as a motion that dies goes with it
    const auto rejectHeadingFor = [&](const BodyMotionIncrement& motion) {
        if (heading_ref.empty()) {
            return;
        }
        const auto heading_it = in.observations.find(heading_ref);
        if (heading_it == in.observations.end()) {
            return;
        }
        const HeadingIncrement* h = heading_it->second.payload.get<HeadingIncrement>();
        if (h != nullptr && h->startAt.ms == motion.startAt.ms &&
            h->endAt.ms == motion.endAt.ms) {
            out.rejected.push_back(heading_ref);
        }
    };
    for (const auto& observation : in.observations) {
        if (observation.first != motion_ref && observation.first != heading_ref &&
            observation.first != state.attitude.ref()) {
            out.rejected.push_back(observation.first);
            out.diagnostic = "observation not consumed by this estimator";
        }
    }

    // re-anchor only; the odometry pose stays continuous
    state.placement.apply(r, in.requests.placement);

    const auto applyAttitude = [&](bool clock_valid) {
        state.attitude.apply(out, in, state.clock, state.motion_clock, clock_valid);
    };

    const auto motion_it = in.observations.find(motion_ref);
    if (motion_it == in.observations.end()) {
        out.status       = FunctionStatus::kNoData;   // hold
        out.clock_mapped = state.clock.valid();
        applyAttitude(state.clock.valid());
        return false;
    }
    const BodyMotionIncrement* motion = motion_it->second.payload.get<BodyMotionIncrement>();
    if (motion == nullptr) {
        out.rejected.push_back(motion_ref);
        out.status = FunctionStatus::kFault;
        applyAttitude(state.clock.valid());
        return false;
    }
    const MonotonicTime stamp =
        motion->endAt.isSet() ? motion->endAt : motion_it->second.measuredAt;
    if (!std::isfinite(motion->dx_m) || !std::isfinite(motion->dy_m) ||
        !std::isfinite(motion->dtheta_rad) || !std::isfinite(motion->dt_s) ||
        motion->dt_s <= 0.0 || !stamp.isSet() ||
        (motion->has_rotation_coupling && (!std::isfinite(motion->dx_per_dtheta_m_rad) ||
                                           !std::isfinite(motion->dy_per_dtheta_m_rad)))) {
        out.rejected.push_back(motion_ref);
        out.status     = FunctionStatus::kFault;
        out.diagnostic = "motion increment without a positive interval";
        applyAttitude(state.clock.valid());
        return false;
    }

    // one clock per motion, and the state's clock must stay that one
    std::string source_clock;
    std::string why;
    if (!observationClock(stamp, motion->sources, source_clock, why)) {
        out.rejected.push_back(motion_ref);
        out.status     = FunctionStatus::kFault;
        out.diagnostic = "motion observation " + why;
        applyAttitude(state.clock.valid());
        return false;
    }
    if (source_clock != state.motion_clock) {
        const bool discontinuity = r.measuredAt.isSet();
        state.clock.reset();
        state.attitude.forget();
        state.aligner.reset();
        const std::string previous = state.motion_clock;
        state.motion_clock         = source_clock;
        if (discontinuity) {
            // stamps on the new clock share nothing with the old ones
            out.rejected.push_back(motion_ref);
            r.odometry_epoch += 1;
            r.vx_m_s         = 0.0;
            r.vy_m_s         = 0.0;
            r.yaw_rate_rad_s = 0.0;
            r.measuredAt     = stamp;
            r.measuredAtHost = MonotonicTime{};
            out.status       = FunctionStatus::kFault;
            out.diagnostic   = "motion clock changed from " + clockName(previous) + " to " +
                             clockName(source_clock) + "; odometry epoch advanced";
            applyAttitude(false);
            return false;
        }
    }

    // device time running backwards means the source rebooted: the
    // odometry frame is discontinuous, nothing is integrated
    if (r.measuredAt.isSet() && stamp < r.measuredAt) {
        out.rejected.push_back(motion_ref);
        rejectHeadingFor(*motion);
        r.odometry_epoch += 1;
        r.vx_m_s         = 0.0;
        r.vy_m_s         = 0.0;
        r.yaw_rate_rad_s = 0.0;
        r.measuredAt     = stamp;
        r.measuredAtHost = MonotonicTime{};
        state.clock.reset();
        state.attitude.forget();
        state.aligner.reset();
        out.status     = FunctionStatus::kFault;
        out.diagnostic = "source time regression; odometry epoch advanced";
        applyAttitude(false);
        return false;
    }
    if (r.measuredAt.isSet() && stamp.ms == r.measuredAt.ms) {
        out.rejected.push_back(motion_ref);
        rejectHeadingFor(*motion);
        out.status     = FunctionStatus::kNoData;
        out.diagnostic = "motion effective time already consumed";
        applyAttitude(state.clock.valid());
        return false;
    }

    // heading alignment over this motion window
    const HeadingIncrement* heading = nullptr;
    if (!heading_ref.empty()) {
        const auto heading_it = in.observations.find(heading_ref);
        if (heading_it != in.observations.end()) {
            heading = heading_it->second.payload.get<HeadingIncrement>();
            if (heading == nullptr || !std::isfinite(heading->dtheta_rad) ||
                !std::isfinite(heading->dt_s)) {
                out.rejected.push_back(heading_ref);
                out.status = FunctionStatus::kFault;
                applyAttitude(state.clock.valid());
                return false;
            }
        }
    }
    const HeadingAligner::Result aligned =
        state.aligner.align(heading, *motion, source_clock, in.context.now, max_wait_ms);
    if (aligned.accept_heading) {
        out.accepted.push_back(heading_ref);
    }
    if (aligned.reject_heading) {
        out.rejected.push_back(heading_ref);
    }
    if (!aligned.diagnostic.empty()) {
        out.diagnostic = aligned.diagnostic;
    }
    if (aligned.outcome == HeadingAligner::Outcome::kWaiting) {
        out.status       = FunctionStatus::kNoData;
        out.clock_mapped = state.clock.valid();
        applyAttitude(state.clock.valid());
        return false;
    }

    step.motion             = motion;
    step.stamp              = stamp;
    step.heading_used       = aligned.outcome == HeadingAligner::Outcome::kFused;
    step.heading_dtheta_rad = aligned.dtheta_rad;
    step.heading_dt_s       = aligned.dt_s;
    step.heading_released   = aligned.released;

    if (!motion->has_rotation && !step.heading_used) {
        if (aligned.released || aligned.expired) {
            // nothing completed this window in time; do not block on it
            out.rejected.push_back(motion_ref);
            out.diagnostic += "; motion without observed rotation rejected after max_wait_ms";
        } else {
            out.diagnostic =
                "motion increment without observed rotation and no aligned heading";
        }
        out.status = FunctionStatus::kNoData;
        applyAttitude(state.clock.valid());
        return false;
    }
    return true;
}

void finishStep(PlanarStepState& state, const StateEstimatorInput& in,
                const ObservationId& motion_ref, const ObservationId& heading_ref,
                const RobotObservationRecord& motion_record, const PreparedStep& step,
                StateEstimatorOutput& out) {
    (void)heading_ref;
    RobotState& r = out.robot;

    // Pair the source stamp with the actual host receipt of the newest
    // consumed sample; pairing with the loop time would count processing
    // delay as clock offset.
    if (step.stamp.domain == ClockDomain::kDevice &&
        motion_record.receivedAt.domain == ClockDomain::kHost) {
        state.clock.observe(step.stamp, motion_record.receivedAt);
    }

    r.valid      = true;
    r.confidence = 1.0;
    r.measuredAt = step.stamp;
    if (step.stamp.domain == ClockDomain::kHost) {
        r.measuredAtHost = step.stamp;
    } else if (state.clock.valid()) {
        r.measuredAtHost = state.clock.toHost(step.stamp);
    } else {
        r.measuredAtHost = MonotonicTime{};
    }
    out.advanced = true;
    out.accepted.push_back(motion_ref);
    out.clock_mapped = step.stamp.domain == ClockDomain::kHost || state.clock.valid();
    state.attitude.apply(out, in, state.clock, state.motion_clock, state.clock.valid());
}

} // namespace navigatr
