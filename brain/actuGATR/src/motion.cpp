// motion.cpp

#include "actugatr/motion.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{
namespace
{

using investigatr::PlanStatus;
using investigatr::ResolveStatus;
using investigatr::RobotStatus;
using investigatr::wrapAngle;

bool nonNegative(double v) {
    return std::isfinite(v) && v >= 0;
}

bool positive(double v) {
    return std::isfinite(v) && v > 0;
}

// Resolution failures worth waiting for: the field or an observation may
// still arrive.
bool waitable(ResolveStatus status) {
    return status == ResolveStatus::kNoField || status == ResolveStatus::kNotObserved ||
           status == ResolveStatus::kNoEstimate || status == ResolveStatus::kStale ||
           status == ResolveStatus::kFrameMismatch;
}

MotionReason resolveReason(ResolveStatus status) {
    switch (status) {
    case ResolveStatus::kNoField: return MotionReason::kFieldUnavailable;
    case ResolveStatus::kMapMismatch: return MotionReason::kMapMismatch;
    case ResolveStatus::kUnknownObject: return MotionReason::kUnknownReference;
    case ResolveStatus::kNotReference: return MotionReason::kNotReference;
    case ResolveStatus::kNotObserved:
    case ResolveStatus::kNoEstimate:
    case ResolveStatus::kStale:
    case ResolveStatus::kFrameMismatch: return MotionReason::kReferenceUnavailable;
    case ResolveStatus::kOk: break;
    }
    return MotionReason::kNone;
}

MotionReason planReason(PlanStatus status) {
    switch (status) {
    case PlanStatus::kInvalidRequest: return MotionReason::kInvalidCommand;
    case PlanStatus::kUnsupportedModel: return MotionReason::kUnsupportedModel;
    case PlanStatus::kNoField: return MotionReason::kFieldUnavailable;
    case PlanStatus::kStartOutOfBounds: return MotionReason::kStartOutOfBounds;
    case PlanStatus::kStartBlocked: return MotionReason::kStartBlocked;
    case PlanStatus::kGoalOutOfBounds: return MotionReason::kGoalOutOfBounds;
    case PlanStatus::kGoalBlocked: return MotionReason::kGoalBlocked;
    case PlanStatus::kNoPath: return MotionReason::kNoPath;
    case PlanStatus::kOk: break;
    }
    return MotionReason::kNone;
}

bool beyond(const Pose& a, const Pose& b, Meters distance, Radians heading) {
    return std::hypot(b.x - a.x, b.y - a.y) > distance ||
           std::fabs(wrapAngle(b.heading - a.heading)) > heading;
}

} // namespace

bool isTerminal(MotionState state) {
    return state == MotionState::kCompleted || state == MotionState::kCancelled ||
           state == MotionState::kFailed;
}

const char* toString(MotionState state) {
    switch (state) {
    case MotionState::kIdle: return "idle";
    case MotionState::kWaiting: return "waiting";
    case MotionState::kRunning: return "running";
    case MotionState::kSettling: return "settling";
    case MotionState::kCompleted: return "completed";
    case MotionState::kCancelled: return "cancelled";
    case MotionState::kFailed: return "failed";
    }
    return "?";
}

const char* toString(MotionReason reason) {
    switch (reason) {
    case MotionReason::kNone: return "none";
    case MotionReason::kInvalidCommand: return "invalid command";
    case MotionReason::kInvalidConfig: return "invalid config";
    case MotionReason::kInputUnavailable: return "input unavailable";
    case MotionReason::kNoProfile: return "no robot profile";
    case MotionReason::kCalibrating: return "calibrating";
    case MotionReason::kPlacementRequired: return "placement required";
    case MotionReason::kInputLost: return "input lost";
    case MotionReason::kFrameChanged: return "frame changed";
    case MotionReason::kFieldUnavailable: return "field unavailable";
    case MotionReason::kMapMismatch: return "map mismatch";
    case MotionReason::kUnknownReference: return "unknown reference";
    case MotionReason::kNotReference: return "not a reference";
    case MotionReason::kReferenceUnavailable: return "reference unavailable";
    case MotionReason::kUnsupportedModel: return "unsupported model";
    case MotionReason::kStartOutOfBounds: return "start out of bounds";
    case MotionReason::kStartBlocked: return "start blocked";
    case MotionReason::kGoalOutOfBounds: return "goal out of bounds";
    case MotionReason::kGoalBlocked: return "goal blocked";
    case MotionReason::kNoPath: return "no path";
    case MotionReason::kTrackingError: return "tracking error";
    case MotionReason::kPlanLimit: return "plan limit";
    case MotionReason::kTimedOut: return "timed out";
    case MotionReason::kSourceChanged: return "source changed";
    case MotionReason::kCancelledByCaller: return "cancelled by caller";
    }
    return "?";
}

bool valid(const MotionConfig& c, const char** why) {
    struct Check {
        bool        ok;
        const char* what;
    };
    const Check checks[] = {
        {nonNegative(c.max_pose_age), "max_pose_age must be >= 0"},
        {nonNegative(c.max_link_age), "max_link_age must be >= 0"},
        {nonNegative(c.input_wait_timeout), "input_wait_timeout must be >= 0"},
        {nonNegative(c.input_loss_timeout), "input_loss_timeout must be >= 0"},
        {positive(c.default_timeout), "default_timeout must be > 0"},
        {positive(c.max_dt), "max_dt must be > 0"},
        {nonNegative(c.replan_distance) && nonNegative(c.replan_heading),
         "replan thresholds must be >= 0"},
        {positive(c.final_position_tolerance) && positive(c.final_heading_tolerance),
         "final tolerances must be > 0"},
        {nonNegative(c.halt_speed) && nonNegative(c.halt_omega) && c.halt_cycles >= 1 &&
             nonNegative(c.halt_timeout),
         "halt settings must be >= 0 with halt_cycles >= 1"},
        {c.max_plans >= 1, "max_plans must be >= 1"},
    };
    for (const Check& check : checks) {
        if (!check.ok) {
            if (why != nullptr) {
                *why = check.what;
            }
            return false;
        }
    }
    return investigatr::valid(c.model, why);
}

Motion::Motion(investigatr::StateSource& source, const investigatr::PathPlanner& planner,
               Follower& follower, const MotionConfig& config)
    : source_(&source), planner_(planner), follower_(follower), config_(config),
      config_valid_(valid(config) && follower.holonomic() == config.model.holonomic) {}

CommandId Motion::goToDirect(const Pose& destination, const Reference& relative_to,
                             const MoveOptions& options) {
    return start(PlanMode::kDirect, destination, relative_to, options);
}

CommandId Motion::goToAvoiding(const Pose& destination, const Reference& relative_to,
                               const MoveOptions& options) {
    return start(PlanMode::kAvoiding, destination, relative_to, options);
}

CommandId Motion::start(PlanMode mode, const Pose& destination, const Reference& reference,
                        const MoveOptions& options) {
    if (active()) {
        follower_.stop();
    }
    if (planned_ && sink_ != nullptr) {
        sink_->reportPath(status_.command_id, investigatr::Path{});
    }
    last_id_           = last_id_ == UINT32_MAX ? 1 : last_id_ + 1;
    status_            = MotionStatus{};
    status_.command_id = last_id_;
    status_.mode       = mode;
    relative_          = destination;
    reference_         = reference;
    options_           = options;
    path_              = investigatr::Path{};
    started_           = false;
    frame_captured_    = false;
    frame_             = 0;
    losing_            = false;
    planned_           = false;
    have_resolution_   = false;
    halting_           = false;
    have_last_pose_    = false;

    const bool ok = config_valid_ && investigatr::finite(destination) &&
                    nonNegative(options.timeout) && nonNegative(options.reference_max_age) &&
                    positive(options.speed_scale) && options.speed_scale <= 1.0;
    if (!ok) {
        finish(MotionState::kFailed,
               config_valid_ ? MotionReason::kInvalidCommand : MotionReason::kInvalidConfig);
        return last_id_;
    }
    status_.state = MotionState::kWaiting;
    return last_id_;
}

bool Motion::active() const {
    return status_.state != MotionState::kIdle && !isTerminal(status_.state);
}

void Motion::cancel() {
    if (active()) {
        finish(MotionState::kCancelled, MotionReason::kCancelledByCaller);
    }
}

void Motion::setNextId(CommandId id) {
    if (id != 0) {
        last_id_ = id - 1;
    }
}

void Motion::setSource(investigatr::StateSource& source) {
    if (active()) {
        finish(MotionState::kCancelled, MotionReason::kSourceChanged);
    }
    source_     = &source;
    field_      = investigatr::Field{};
    have_field_ = false;
}

void Motion::finish(MotionState state, MotionReason reason) {
    const bool reported = planned_;
    status_.state       = state;
    status_.reason      = reason;
    follower_.stop();
    planned_ = false;
    if (reported && sink_ != nullptr) {
        sink_->reportPath(status_.command_id, investigatr::Path{});
    }
}

ChassisCommand Motion::zero() {
    return ChassisCommand{};
}

investigatr::MotionModel Motion::scaledModel() const {
    investigatr::MotionModel model = config_.model;
    model.limits.max_speed *= options_.speed_scale;
    model.limits.max_omega *= options_.speed_scale;
    return model;
}

MotionReason Motion::waitReason(const investigatr::RobotState& robot) const {
    switch (robot.status) {
    case RobotStatus::kNoProfile: return MotionReason::kNoProfile;
    case RobotStatus::kCalibrating: return MotionReason::kCalibrating;
    case RobotStatus::kUnplaced: return MotionReason::kPlacementRequired;
    default: return MotionReason::kInputUnavailable;
    }
}

Motion::Step Motion::plan(const investigatr::RobotState& robot, Seconds now) {
    const bool waited = now - start_time_ > config_.input_wait_timeout;

    investigatr::ReferencePolicy policy;
    policy.require_observed = options_.require_observed_reference;
    policy.max_age          = options_.reference_max_age;
    const investigatr::Resolved r =
        investigatr::resolve(reference_, relative_, have_field_ ? &field_ : nullptr,
                             robot_at_start_, robot.frame, policy, now);
    if (r.status != ResolveStatus::kOk) {
        if (!waitable(r.status)) {
            finish(MotionState::kFailed, resolveReason(r.status));
            return Step::kEnded;
        }
        // Replans keep the last resolution when an estimate is briefly missing.
        if (!have_resolution_) {
            if (!waited) {
                return Step::kWait;
            }
            finish(MotionState::kFailed, resolveReason(r.status));
            return Step::kEnded;
        }
    }
    const Pose goal = r.status == ResolveStatus::kOk ? r.destination : resolved_;
    if (r.status == ResolveStatus::kOk) {
        status_.reference_source = r.source;
    }

    if (status_.mode == PlanMode::kAvoiding && !have_field_) {
        if (!waited) {
            return Step::kWait;
        }
        finish(MotionState::kFailed, MotionReason::kFieldUnavailable);
        return Step::kEnded;
    }

    if (status_.plans >= config_.max_plans) {
        finish(MotionState::kFailed, MotionReason::kPlanLimit);
        return Step::kEnded;
    }
    investigatr::PlanRequest request;
    request.mode          = status_.mode;
    request.start         = robot.pose;
    request.goal          = goal;
    request.model         = scaledModel();
    request.allow_reverse = options_.allow_reverse;
    request.field         = have_field_ ? &field_ : nullptr;
    investigatr::PlanResult result = planner_.plan(request);
    ++status_.plans;
    if (result.status != PlanStatus::kOk) {
        status_.blocking = result.blocking;
        finish(MotionState::kFailed, planReason(result.status));
        return Step::kEnded;
    }

    resolved_               = goal;
    have_resolution_        = true;
    path_                   = result.path;
    planned_                = true;
    plan_generation_        = field_.generation;
    status_.has_destination = true;
    status_.destination     = goal;
    status_.segment         = 0;
    status_.segment_count   = path_.segments.size();
    follower_.start(path_, request.model.limits);
    if (sink_ != nullptr) {
        sink_->reportPath(status_.command_id, path_);
    }
    return Step::kPlanned;
}

// The planned segments still ahead, unmodified. Tracking error near them is
// what the model clearance covers, so the measured pose is not substituted:
// routes run just outside the grown boxes and small drift would read as blocked.
investigatr::Path Motion::remaining() const {
    investigatr::Path out;
    out.mode = path_.mode;
    for (std::size_t i = status_.segment; i < path_.segments.size(); ++i) {
        out.segments.push_back(path_.segments[i]);
    }
    return out;
}

bool Motion::needsReplan(const investigatr::RobotState& robot, Seconds now) {
    if (!have_field_ || field_.generation == plan_generation_) {
        return false;
    }
    plan_generation_ = field_.generation;
    if (reference_.kind == Reference::Kind::kObject) {
        investigatr::ReferencePolicy policy;
        policy.require_observed = options_.require_observed_reference;
        policy.max_age          = options_.reference_max_age;
        const investigatr::Resolved r = investigatr::resolve(
            reference_, relative_, &field_, robot_at_start_, robot.frame, policy, now);
        if (r.status == ResolveStatus::kOk &&
            beyond(resolved_, r.destination, config_.replan_distance, config_.replan_heading)) {
            return true;
        }
    }
    return status_.mode == PlanMode::kAvoiding &&
           !planner_.clear(remaining(), field_, config_.model);
}

ChassisCommand Motion::update(Seconds now) {
    Seconds dt = 0;
    if (have_last_update_) {
        dt = std::clamp(now - last_update_, 0.0, config_.max_dt);
    }
    have_last_update_ = true;
    last_update_      = now;

    if (source_->field(field_)) {
        have_field_ = true;
    }
    if (!active()) {
        return zero();
    }
    if (!started_) {
        started_    = true;
        start_time_ = now;
    }
    status_.elapsed       = now - start_time_;
    const Seconds timeout = options_.timeout > 0 ? options_.timeout : config_.default_timeout;
    if (status_.elapsed > timeout) {
        finish(MotionState::kFailed, MotionReason::kTimedOut);
        return zero();
    }

    const investigatr::RobotState robot = source_->robot(now);
    const bool usable = robot.valid() && robot.connected && investigatr::finite(robot.pose) &&
                        robot.age <= config_.max_pose_age &&
                        robot.link_age <= config_.max_link_age;
    if (!usable) {
        if (!frame_captured_) {
            if (now - start_time_ > config_.input_wait_timeout) {
                finish(MotionState::kFailed, waitReason(robot));
                return zero();
            }
        } else if (!robot.connected) {
            finish(MotionState::kFailed, MotionReason::kInputLost);
            return zero();
        } else {
            if (!losing_) {
                losing_       = true;
                losing_since_ = now;
            }
            if (now - losing_since_ > config_.input_loss_timeout) {
                finish(MotionState::kFailed, MotionReason::kInputLost);
                return zero();
            }
        }
        status_.state = MotionState::kWaiting;
        return zero();
    }
    losing_ = false;
    if (frame_captured_ && robot.frame != frame_) {
        finish(MotionState::kFailed, MotionReason::kFrameChanged);
        return zero();
    }
    if (!frame_captured_) {
        frame_captured_ = true;
        frame_          = robot.frame;
        robot_at_start_ = robot.pose;
    }

    // Pose rate for the halt check.
    bool steady = false;
    if (have_last_pose_ && dt > 0) {
        const double speed = std::hypot(robot.pose.x - last_pose_.x, robot.pose.y - last_pose_.y) / dt;
        const double omega = std::fabs(wrapAngle(robot.pose.heading - last_pose_.heading)) / dt;
        steady             = speed <= config_.halt_speed && omega <= config_.halt_omega;
    }
    have_last_pose_ = true;
    last_pose_      = robot.pose;

    if (planned_ && needsReplan(robot, now)) {
        planned_    = false;
        halting_    = true;
        halt_since_ = now;
        steady_     = 0;
        follower_.stop();
    }
    if (halting_) {
        steady_ = steady ? steady_ + 1 : 0;
        if (steady_ < config_.halt_cycles && now - halt_since_ <= config_.halt_timeout) {
            status_.state = MotionState::kRunning;
            return zero();
        }
        halting_ = false;
    }
    if (!planned_) {
        const Step step = plan(robot, now);
        if (step == Step::kEnded) {
            return zero();
        }
        if (step == Step::kWait) {
            status_.state = MotionState::kWaiting;
            return zero();
        }
    }

    status_.distance_error = std::hypot(resolved_.x - robot.pose.x, resolved_.y - robot.pose.y);
    status_.heading_error  = wrapAngle(resolved_.heading - robot.pose.heading);

    const FollowOutput out = follower_.update(robot.pose, dt);
    status_.segment        = out.segment;
    status_.cross_track    = out.cross_track;
    switch (out.state) {
    case FollowState::kDone:
        if (status_.distance_error <= config_.final_position_tolerance &&
            std::fabs(status_.heading_error) <= config_.final_heading_tolerance) {
            finish(MotionState::kCompleted, MotionReason::kNone);
            return zero();
        }
        // Settled short of the destination: plan a correction from here.
        planned_ = false;
        status_.state = MotionState::kRunning;
        return zero();
    case FollowState::kFailed:
        finish(MotionState::kFailed, out.fault == FollowFault::kTrackingError
                                         ? MotionReason::kTrackingError
                                         : MotionReason::kInvalidCommand);
        return zero();
    case FollowState::kSettling: status_.state = MotionState::kSettling; break;
    default: status_.state = MotionState::kRunning; break;
    }
    return out.command;
}

} // namespace actugatr
