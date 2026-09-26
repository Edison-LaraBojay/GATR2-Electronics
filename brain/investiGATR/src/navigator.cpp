// navigator.cpp

#include "investigatr/navigator.h"

#include <algorithm>
#include <cmath>

namespace investigatr
{

namespace
{

// Beyond this bearing a turn keeps its previous direction.
constexpr Radians kTurnHold = kPi - 5.0 * kPi / 180.0;

bool finite(const Pose& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.heading);
}

bool nonNegative(double value) { return std::isfinite(value) && value >= 0; }

bool validPid(const PidGains& g) {
    return nonNegative(g.kP) && nonNegative(g.kI) && nonNegative(g.kD) &&
           nonNegative(g.integral_limit) && nonNegative(g.output_limit) && g.output_limit > 0;
}

bool validDestination(const Destination& d) {
    if (!finite(d.pose)) {
        return false;
    }
    return d.kind != Destination::Kind::kLandmark || d.landmark != 0;
}

bool beyond(const Pose& a, const Pose& b, Meters distance, Radians heading) {
    return std::hypot(b.x - a.x, b.y - a.y) > distance ||
           std::fabs(wrapAngle(b.heading - a.heading)) > heading;
}

// from moved toward to by at most step meters and turn radians.
Pose approach(const Pose& from, const Pose& to, Meters step, Radians turn) {
    Pose         out = from;
    const double dx  = to.x - from.x;
    const double dy  = to.y - from.y;
    const double d   = std::hypot(dx, dy);
    if (d <= step) {
        out.x = to.x;
        out.y = to.y;
    } else {
        out.x += dx * step / d;
        out.y += dy * step / d;
    }
    const double dh = wrapAngle(to.heading - from.heading);
    out.heading     = wrapAngle(from.heading + std::clamp(dh, -turn, turn));
    return out;
}

// Nonzero demand below minimum raised to it.
double raise(double value, double minimum) {
    if (value > 0 && value < minimum) {
        return minimum;
    }
    if (value < 0 && value > -minimum) {
        return -minimum;
    }
    return value;
}

int sign(double value) {
    if (value > 0) {
        return 1;
    }
    return value < 0 ? -1 : 0;
}

} // namespace

Destination Destination::field(const Pose& pose) {
    Destination d;
    d.kind = Kind::kField;
    d.pose = pose;
    return d;
}

Destination Destination::relative(LandmarkId landmark, const Pose& offset) {
    Destination d;
    d.kind     = Kind::kLandmark;
    d.landmark = landmark;
    d.pose     = offset;
    return d;
}

Destination Destination::robotRelative(const Pose& offset) {
    Destination d;
    d.kind = Kind::kRobot;
    d.pose = offset;
    return d;
}

bool isTerminal(MotionState state) {
    return state == MotionState::kCompleted || state == MotionState::kCancelled ||
           state == MotionState::kFailed;
}

const char* toString(MotionState state) {
    switch (state) {
    case MotionState::kIdle:
        return "idle";
    case MotionState::kWaiting:
        return "waiting";
    case MotionState::kTurning:
        return "turning";
    case MotionState::kDriving:
        return "driving";
    case MotionState::kAligning:
        return "aligning";
    case MotionState::kCompleted:
        return "completed";
    case MotionState::kCancelled:
        return "cancelled";
    case MotionState::kFailed:
        return "failed";
    }
    return "?";
}

const char* toString(MotionReason reason) {
    switch (reason) {
    case MotionReason::kNone:
        return "none";
    case MotionReason::kInvalidCommand:
        return "invalid command";
    case MotionReason::kInputUnavailable:
        return "input unavailable";
    case MotionReason::kInputLost:
        return "input lost";
    case MotionReason::kFrameChanged:
        return "frame changed";
    case MotionReason::kLandmarkUnknown:
        return "landmark unknown";
    case MotionReason::kLandmarkUnsupported:
        return "landmark unsupported";
    case MotionReason::kLandmarkUnavailable:
        return "landmark unavailable";
    case MotionReason::kLandmarkJump:
        return "landmark jump";
    case MotionReason::kTimedOut:
        return "timed out";
    case MotionReason::kSourceChanged:
        return "source changed";
    case MotionReason::kCancelledByCaller:
        return "cancelled by caller";
    }
    return "?";
}

Navigator::Navigator(InputSource& source, const NavigatorConfig& config)
    : source_(&source), config_(config), config_valid_(valid(config)),
      drive_pid_(config.drive_pid), heading_pid_(config.heading_pid, true),
      turn_pid_(config.turn_pid, true) {}

bool Navigator::valid(const NavigatorConfig& c, const char** why) {
    struct Check {
        bool        ok;
        const char* what;
    };
    const Check checks[] = {
        {nonNegative(c.position_tolerance), "position_tolerance must be >= 0"},
        {nonNegative(c.arrive_distance), "arrive_distance must be >= 0"},
        {nonNegative(c.heading_tolerance) && c.heading_tolerance > 0,
         "heading_tolerance must be > 0"},
        {nonNegative(c.settle_time), "settle_time must be >= 0"},
        {nonNegative(c.near_distance), "near_distance must be >= 0"},
        {nonNegative(c.waypoint_pass_radius), "waypoint_pass_radius must be >= 0"},
        {nonNegative(c.turn_in_place_threshold), "turn_in_place_threshold must be >= 0"},
        {nonNegative(c.turn_exit), "turn_exit must be >= 0"},
        {validPid(c.drive_pid), "drive_pid gains must be >= 0 with output_limit > 0"},
        {validPid(c.heading_pid), "heading_pid gains must be >= 0 with output_limit > 0"},
        {validPid(c.turn_pid), "turn_pid gains must be >= 0 with output_limit > 0"},
        {nonNegative(c.max_forward) && c.max_forward > 0 && c.max_forward <= 1,
         "max_forward must be in (0, 1]"},
        {nonNegative(c.max_turn) && c.max_turn > 0 && c.max_turn <= 1,
         "max_turn must be in (0, 1]"},
        {nonNegative(c.forward_slew), "forward_slew must be >= 0"},
        {nonNegative(c.turn_slew), "turn_slew must be >= 0"},
        {nonNegative(c.min_forward) && c.min_forward <= c.max_forward,
         "min_forward must be in [0, max_forward]"},
        {nonNegative(c.min_turn) && c.min_turn <= c.max_turn, "min_turn must be in [0, max_turn]"},
        {nonNegative(c.max_pose_age), "max_pose_age must be >= 0"},
        {nonNegative(c.max_link_age), "max_link_age must be >= 0"},
        {nonNegative(c.input_wait_timeout), "input_wait_timeout must be >= 0"},
        {nonNegative(c.input_loss_timeout), "input_loss_timeout must be >= 0"},
        {nonNegative(c.default_timeout) && c.default_timeout > 0, "default_timeout must be > 0"},
        {nonNegative(c.max_landmark_age), "max_landmark_age must be >= 0"},
        {nonNegative(c.max_landmark_jump), "max_landmark_jump must be >= 0"},
        {nonNegative(c.max_landmark_jump_heading), "max_landmark_jump_heading must be >= 0"},
        {nonNegative(c.max_nominal_correction), "max_nominal_correction must be >= 0"},
        {nonNegative(c.max_nominal_correction_heading),
         "max_nominal_correction_heading must be >= 0"},
        {nonNegative(c.landmark_follow_speed), "landmark_follow_speed must be >= 0"},
        {nonNegative(c.landmark_follow_turn_rate), "landmark_follow_turn_rate must be >= 0"},
        {nonNegative(c.max_dt) && c.max_dt > 0, "max_dt must be > 0"},
        {c.arrive_distance < c.position_tolerance, "arrive_distance must be < position_tolerance"},
        {c.waypoint_pass_radius >= c.near_distance, "waypoint_pass_radius must be >= near_distance"},
        {c.turn_exit < c.turn_in_place_threshold, "turn_exit must be < turn_in_place_threshold"},
    };
    for (const Check& check : checks) {
        if (!check.ok) {
            if (why != nullptr) {
                *why = check.what;
            }
            return false;
        }
    }
    if (why != nullptr) {
        *why = nullptr;
    }
    return true;
}

CommandId Navigator::goTo(const Pose& field_pose, const MotionOptions& options) {
    return follow(Path{Waypoint{Destination::field(field_pose), true}}, options);
}

CommandId Navigator::goToRelative(LandmarkId landmark, const Pose& landmark_local_offset,
                                  const MotionOptions& options) {
    return follow(Path{Waypoint{Destination::relative(landmark, landmark_local_offset), true}},
                  options);
}

CommandId Navigator::goToRobotRelative(const Pose& offset, const MotionOptions& options) {
    return follow(Path{Waypoint{Destination::robotRelative(offset), true}}, options);
}

CommandId Navigator::follow(const Path& path, const MotionOptions& options) {
    last_id_               = last_id_ == UINT32_MAX ? 1 : last_id_ + 1;
    status_                = MotionStatus{};
    status_.command_id     = last_id_;
    status_.waypoint_count = path.size();
    path_                  = path;
    options_               = options;
    started_               = false;
    frame_captured_        = false;
    frame_                 = 0;
    losing_input_          = false;

    bool ok = config_valid_ && !path.empty() && nonNegative(options.timeout);
    for (const Waypoint& waypoint : path) {
        ok = ok && validDestination(waypoint.destination);
    }
    if (!ok) {
        finish(MotionState::kFailed, MotionReason::kInvalidCommand);
        return last_id_;
    }
    status_.state = MotionState::kWaiting;
    activate(0, 0);
    return last_id_;
}

void Navigator::cancel() {
    if (active()) {
        finish(MotionState::kCancelled, MotionReason::kCancelledByCaller);
    }
}

void Navigator::setSource(InputSource& source) {
    source_->request(InputRequest{});
    if (active()) {
        finish(MotionState::kCancelled, MotionReason::kSourceChanged);
    }
    source_ = &source;
}

DriveCommand Navigator::update(Seconds now) {
    Seconds dt  = 0;
    bool    gap = false;
    if (have_last_update_) {
        dt  = now - last_update_;
        gap = dt > config_.max_dt;
        dt  = std::max(0.0, std::min(dt, config_.max_dt));
    }
    have_last_update_ = true;
    last_update_      = now;

    source_->request(need());
    if (!active()) {
        return zero();
    }

    if (!started_) {
        started_      = true;
        start_time_   = now;
        activated_at_ = now;
    } else if (gap) {
        resetControl();
    }
    status_.elapsed = now - start_time_;
    const Seconds timeout = options_.timeout > 0 ? options_.timeout : config_.default_timeout;
    if (status_.elapsed > timeout) {
        finish(MotionState::kFailed, MotionReason::kTimedOut);
        return zero();
    }

    const InputSnapshot snapshot = source_->latest(now);
    if (frame_captured_ && snapshot.frame != 0 && snapshot.frame != frame_) {
        finish(MotionState::kFailed, MotionReason::kFrameChanged);
        return zero();
    }

    if (!robotUsable(snapshot)) {
        if (!frame_captured_) {
            if (now - start_time_ > config_.input_wait_timeout) {
                finish(MotionState::kFailed, MotionReason::kInputUnavailable);
                return zero();
            }
        } else if (!snapshot.connected) {
            finish(MotionState::kFailed, MotionReason::kInputLost);
            return zero();
        } else {
            if (!losing_input_) {
                losing_input_ = true;
                losing_since_ = now;
            }
            if (now - losing_since_ > config_.input_loss_timeout) {
                finish(MotionState::kFailed, MotionReason::kInputLost);
                return zero();
            }
        }
        wait();
        return zero();
    }
    losing_input_ = false;
    if (!frame_captured_) {
        frame_captured_ = true;
        frame_          = snapshot.frame;
    }
    return navigate(snapshot, now, dt);
}

bool Navigator::active() const {
    return status_.state != MotionState::kIdle && !isTerminal(status_.state);
}

bool Navigator::stopsAt(std::size_t index) const {
    return path_[index].stop || index + 1 == path_.size();
}

InputRequest Navigator::need() const {
    InputRequest request;
    if (active()) {
        const Destination& d = path_[status_.waypoint_index].destination;
        if (d.kind == Destination::Kind::kLandmark) {
            request.landmark    = true;
            request.landmark_id = d.landmark;
        }
    }
    return request;
}

bool Navigator::robotUsable(const InputSnapshot& s) const {
    return s.robot.valid && s.frame != 0 && s.connected && finite(s.robot.pose) &&
           s.robot.age <= config_.max_pose_age && s.link_age <= config_.max_link_age;
}

bool Navigator::landmarkUsable(const LandmarkEstimate& landmark, LandmarkId id) const {
    if (landmark.status != LandmarkStatus::kAvailable || landmark.id != id ||
        landmark.source == LandmarkSource::kNone || !finite(landmark.pose)) {
        return false;
    }
    if (landmark.source == LandmarkSource::kNominal && options_.require_observed_landmark) {
        return false;
    }
    if (landmark.source == LandmarkSource::kObserved && landmark.age_known &&
        !(landmark.age <= config_.max_landmark_age)) {
        return false;
    }
    return true;
}

Meters Navigator::remainingPath() const {
    Meters total = 0;
    Pose   from  = destination_;
    for (std::size_t i = status_.waypoint_index + 1; i < path_.size(); ++i) {
        const Destination& d = path_[i].destination;
        if (d.kind != Destination::Kind::kField) {
            break;
        }
        total += std::hypot(d.pose.x - from.x, d.pose.y - from.y);
        from = d.pose;
        if (stopsAt(i)) {
            break;
        }
    }
    return total;
}

Navigator::Errors Navigator::errorsTo(const Pose& robot) const {
    const double dx = destination_.x - robot.x;
    const double dy = destination_.y - robot.y;
    Errors       e;
    e.distance = std::hypot(dx, dy);
    e.bearing  = wrapAngle(std::atan2(dy, dx) - robot.heading);
    e.heading  = wrapAngle(destination_.heading - robot.heading);
    e.along    = e.distance * std::cos(e.bearing);
    return e;
}

void Navigator::finish(MotionState state, MotionReason reason) {
    status_.state  = state;
    status_.reason = reason;
}

void Navigator::activate(std::size_t index, Seconds now) {
    const Destination& d    = path_[index].destination;
    status_.waypoint_index  = index;
    activated_at_           = now;
    resolved_               = d.kind == Destination::Kind::kField;
    destination_            = resolved_ ? d.pose : Pose{};
    landmark_source_        = LandmarkSource::kNone;
    along_was_positive_     = false;
    select_phase_           = true;
    status_.has_destination = resolved_;
    status_.destination     = destination_;
    resetControl();
}

void Navigator::wait() {
    status_.state = MotionState::kWaiting;
    select_phase_ = true;
    settling_     = false;
}

void Navigator::setPhase(MotionState phase) {
    if (status_.state != phase) {
        status_.state = phase;
        resetControl();
    }
}

void Navigator::selectPhase(const Errors& e) {
    if (stopsAt(status_.waypoint_index) && e.distance <= config_.arrive_distance) {
        setPhase(MotionState::kAligning);
    } else if (std::fabs(e.bearing) > config_.turn_exit) {
        setPhase(MotionState::kTurning);
    } else {
        setPhase(MotionState::kDriving);
    }
}

void Navigator::resetControl() {
    drive_pid_.reset();
    heading_pid_.reset();
    turn_pid_.reset();
    turn_sign_ = 0;
    settling_  = false;
    settled_   = 0;
}

double Navigator::holdTurnSign(double error) {
    if (std::fabs(error) > kTurnHold && turn_sign_ != 0 && sign(error) != turn_sign_) {
        error += turn_sign_ * 2.0 * kPi;
    }
    turn_sign_ = sign(error);
    return error;
}

Navigator::Resolve Navigator::resolve(const InputSnapshot& snapshot, Seconds now, Seconds dt) {
    const Destination& d = path_[status_.waypoint_index].destination;
    switch (d.kind) {
    case Destination::Kind::kField:
        return Resolve::kReady;
    case Destination::Kind::kRobot:
        if (!resolved_) {
            destination_ = compose(snapshot.robot.pose, d.pose);
            resolved_    = true;
        }
        return Resolve::kReady;
    case Destination::Kind::kLandmark:
        return followLandmark(snapshot.landmark, now, dt);
    }
    return Resolve::kReady;
}

Navigator::Resolve Navigator::followLandmark(const LandmarkEstimate& landmark, Seconds now,
                                             Seconds dt) {
    const Destination&     d = path_[status_.waypoint_index].destination;
    const NavigatorConfig& c = config_;
    if (landmark.id == d.landmark && landmark.status == LandmarkStatus::kUnknownLandmark) {
        finish(MotionState::kFailed, MotionReason::kLandmarkUnknown);
        return Resolve::kEnded;
    }
    if (landmark.id == d.landmark && landmark.status == LandmarkStatus::kUnsupported) {
        finish(MotionState::kFailed, MotionReason::kLandmarkUnsupported);
        return Resolve::kEnded;
    }
    const bool usable = landmarkUsable(landmark, d.landmark);
    if (!resolved_) {
        if (!usable) {
            if (now - activated_at_ > c.input_wait_timeout) {
                finish(MotionState::kFailed, MotionReason::kLandmarkUnavailable);
                return Resolve::kEnded;
            }
            return Resolve::kWait;
        }
        landmark_        = landmark.pose;
        landmark_source_ = landmark.source;
        destination_     = compose(landmark_, d.pose);
        resolved_        = true;
        return Resolve::kReady;
    }
    if (usable) {
        if (landmark.source == landmark_source_) {
            if (beyond(landmark_, landmark.pose, c.max_landmark_jump,
                       c.max_landmark_jump_heading)) {
                finish(MotionState::kFailed, MotionReason::kLandmarkJump);
                return Resolve::kEnded;
            }
            landmark_ = landmark.pose;
        } else if (landmark_source_ == LandmarkSource::kNominal &&
                   landmark.source == LandmarkSource::kObserved) {
            if (beyond(landmark_, landmark.pose, c.max_nominal_correction,
                       c.max_nominal_correction_heading)) {
                finish(MotionState::kFailed, MotionReason::kLandmarkJump);
                return Resolve::kEnded;
            }
            landmark_        = landmark.pose;
            landmark_source_ = LandmarkSource::kObserved;
        }
    }
    destination_ = approach(destination_, compose(landmark_, d.pose), c.landmark_follow_speed * dt,
                            c.landmark_follow_turn_rate * dt);
    return Resolve::kReady;
}

DriveCommand Navigator::navigate(const InputSnapshot& snapshot, Seconds now, Seconds dt) {
    for (std::size_t pass = 0; pass <= path_.size(); ++pass) {
        const Resolve resolution = resolve(snapshot, now, dt);
        if (resolution == Resolve::kEnded) {
            return zero();
        }
        if (resolution == Resolve::kWait) {
            wait();
            return zero();
        }
        const Errors e          = errorsTo(snapshot.robot.pose);
        status_.has_destination = true;
        status_.destination     = destination_;
        status_.distance_error  = e.distance;
        status_.bearing_error   = e.bearing;
        status_.heading_error   = e.heading;
        if (select_phase_) {
            select_phase_ = false;
            selectPhase(e);
        }

        const std::size_t index   = status_.waypoint_index;
        bool              advance = false;
        if (!stopsAt(index)) {
            advance = e.distance <= config_.waypoint_pass_radius ||
                      (status_.state == MotionState::kDriving && along_was_positive_ &&
                       e.along <= 0);
        }
        if (!advance) {
            const DriveCommand demand = control(e, dt, advance);
            if (!advance) {
                return shape(demand, dt);
            }
        }
        if (index + 1 >= path_.size()) {
            finish(MotionState::kCompleted, MotionReason::kNone);
            return zero();
        }
        activate(index + 1, now);
    }
    return zero();
}

DriveCommand Navigator::control(const Errors& e, Seconds dt, bool& settled) {
    const NavigatorConfig& c    = config_;
    const bool             stop = stopsAt(status_.waypoint_index);
    switch (status_.state) {
    case MotionState::kTurning:
        if (stop && e.distance <= c.arrive_distance) {
            setPhase(MotionState::kAligning);
        } else if (std::fabs(e.bearing) <= c.turn_exit) {
            setPhase(MotionState::kDriving);
        }
        break;
    case MotionState::kDriving:
        if (stop && e.distance <= c.arrive_distance) {
            setPhase(MotionState::kAligning);
        } else if (stop && e.along <= 0) {
            if (e.distance <= c.position_tolerance) {
                setPhase(MotionState::kAligning);
            } else {
                selectPhase(e);
            }
        } else if (std::fabs(e.bearing) > c.turn_in_place_threshold &&
                   e.distance > c.near_distance) {
            setPhase(MotionState::kTurning);
        }
        break;
    case MotionState::kAligning:
        if (e.distance > c.position_tolerance) {
            selectPhase(e);
        }
        break;
    default:
        break;
    }

    DriveCommand demand;
    switch (status_.state) {
    case MotionState::kTurning:
        demand.turn = raise(turn_pid_.update(holdTurnSign(e.bearing), dt), c.min_turn);
        break;
    case MotionState::kDriving: {
        const Meters along   = e.along + (stop ? 0.0 : remainingPath());
        const double scale   = std::max(0.0, std::cos(e.bearing));
        demand.forward       = std::clamp(drive_pid_.update(along, dt) * scale, 0.0, c.max_forward);
        if (e.distance > c.position_tolerance) {
            demand.forward = raise(demand.forward, c.min_forward);
        }
        if (e.distance > c.near_distance) {
            demand.turn = heading_pid_.update(e.bearing, dt);
        } else {
            heading_pid_.reset();
        }
        if (e.along > 0) {
            along_was_positive_ = true;
        }
        break;
    }
    case MotionState::kAligning: {
        const bool inside = e.distance <= c.position_tolerance &&
                            std::fabs(e.heading) <= c.heading_tolerance;
        const double turn = turn_pid_.update(holdTurnSign(e.heading), dt);
        demand.turn       = inside ? turn : raise(turn, c.min_turn);
        if (!inside) {
            settling_ = false;
        } else if (!settling_) {
            settling_ = true;
            settled_  = 0;
        } else {
            settled_ += dt;
        }
        settled = settling_ && settled_ >= c.settle_time;
        break;
    }
    default:
        break;
    }
    return demand;
}

DriveCommand Navigator::shape(const DriveCommand& demand, Seconds dt) {
    const NavigatorConfig& c = config_;
    DriveCommand           out;
    out.forward = slewLimit(output_.forward, std::clamp(demand.forward, -c.max_forward, c.max_forward),
                            c.forward_slew, dt);
    out.turn =
        slewLimit(output_.turn, std::clamp(demand.turn, -c.max_turn, c.max_turn), c.turn_slew, dt);
    output_ = out;
    return out;
}

DriveCommand Navigator::zero() {
    output_ = DriveCommand{};
    return DriveCommand{};
}

} // namespace investigatr
