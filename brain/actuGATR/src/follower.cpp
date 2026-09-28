// follower.cpp

#include "actugatr/follower.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{
namespace
{

using investigatr::kPi;
using investigatr::SegmentKind;
using investigatr::wrapAngle;

// Beyond this error a turn keeps its previous direction.
constexpr Radians kTurnHold = kPi - 5.0 * kPi / 180.0;

// Translations shorter than this count as done.
constexpr Meters kTinySegment = 1e-6;

int sign(double value) {
    if (value > 0) {
        return 1;
    }
    return value < 0 ? -1 : 0;
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

bool fail(const char** why, const char* what) {
    if (why != nullptr) {
        *why = what;
    }
    return false;
}

} // namespace

bool valid(const FollowerConfig& c, const char** why) {
    const auto positive    = [](double v) { return std::isfinite(v) && v > 0; };
    const auto nonNegative = [](double v) { return std::isfinite(v) && v >= 0; };
    if (!positive(c.position_tolerance) || !positive(c.heading_tolerance)) {
        return fail(why, "tolerances must be > 0");
    }
    if (!nonNegative(c.settle_time)) {
        return fail(why, "settle_time must be >= 0");
    }
    if (!(c.tracking_tolerance > c.position_tolerance)) {
        return fail(why, "tracking_tolerance must exceed position_tolerance");
    }
    if (!(c.max_heading_error > c.heading_tolerance) || c.max_heading_error > kPi) {
        return fail(why, "max_heading_error must be in (heading_tolerance, pi]");
    }
    if (!valid(c.along) || !valid(c.cross) || !valid(c.heading) || !valid(c.turn)) {
        return fail(why, "gains must be >= 0 with output_limit > 0");
    }
    if (!nonNegative(c.min_speed) || !nonNegative(c.min_omega)) {
        return fail(why, "min_speed and min_omega must be >= 0");
    }
    if (why != nullptr) {
        *why = nullptr;
    }
    return true;
}

SegmentFollower::SegmentFollower(const FollowerConfig& config)
    : config_(config), along_(config.along), cross_(config.cross), heading_(config.heading, true),
      turn_(config.turn, true) {}

void SegmentFollower::start(const Path& path, const MotionLimits& limits) {
    path_      = path;
    limits_    = limits;
    index_     = 0;
    settled_   = 0;
    turn_sign_ = 0;
    output_    = ChassisCommand{};
    resetControl();
    state_ = path_.segments.empty() ? FollowState::kDone : FollowState::kRunning;
}

void SegmentFollower::stop() {
    state_  = FollowState::kIdle;
    output_ = ChassisCommand{};
    path_.segments.clear();
}

void SegmentFollower::resetControl() {
    along_.reset();
    cross_.reset();
    heading_.reset();
    turn_.reset();
}

Radians SegmentFollower::turnError(const PathSegment& s, const Pose& robot) {
    double error = wrapAngle(s.end.heading - robot.heading);
    int    hold  = turn_sign_ != 0 ? turn_sign_ : s.turn_direction;
    if (std::fabs(error) > kTurnHold && hold != 0 && sign(error) != hold) {
        error += hold * 2.0 * kPi;
    }
    turn_sign_ = sign(error);
    return error;
}

FollowOutput SegmentFollower::finish(FollowState state, FollowFault fault) {
    state_  = state;
    output_ = ChassisCommand{};
    FollowOutput out;
    out.state   = state;
    out.fault   = fault;
    out.segment = index_;
    return out;
}

ChassisCommand SegmentFollower::limit(const ChassisCommand& in, Seconds dt) {
    ChassisCommand out = in;
    const double   speed = std::hypot(out.vx, out.vy);
    if (speed > limits_.max_speed && speed > 0) {
        const double scale = limits_.max_speed / speed;
        out.vx *= scale;
        out.vy *= scale;
    }
    out.omega = std::clamp(out.omega, -limits_.max_omega, limits_.max_omega);
    out.vx    = slewLimit(output_.vx, out.vx, limits_.max_accel, dt);
    out.vy    = slewLimit(output_.vy, out.vy, limits_.max_accel, dt);
    out.omega = slewLimit(output_.omega, out.omega, limits_.max_alpha, dt);
    output_   = out;
    return out;
}

FollowOutput SegmentFollower::update(const Pose& robot, Seconds dt) {
    if (state_ != FollowState::kRunning && state_ != FollowState::kSettling) {
        FollowOutput out;
        out.state   = state_;
        out.segment = index_;
        return out;
    }
    if (!investigatr::finite(robot)) {
        return finish(FollowState::kFailed, FollowFault::kInvalidPath);
    }

    for (std::size_t pass = 0; pass <= path_.segments.size(); ++pass) {
        const PathSegment& s = path_.segments[index_];
        FollowOutput       out;
        out.segment = index_;

        if (s.kind == SegmentKind::kTurn) {
            const double drift = std::hypot(robot.x - s.start.x, robot.y - s.start.y);
            if (drift > std::hypot(config_.position_tolerance, config_.tracking_tolerance)) {
                FollowOutput failed = finish(FollowState::kFailed, FollowFault::kTrackingError);
                failed.cross_track  = drift;
                return failed;
            }
            const Radians error = turnError(s, robot);
            const bool    inside = std::fabs(error) <= config_.heading_tolerance;
            out.heading_error    = error;
            if (!last() && inside) {
                ++index_;
                turn_sign_ = 0;
                resetControl();
                continue;
            }
            if (last()) {
                if (inside) {
                    settled_ += dt;
                    state_ = FollowState::kSettling;
                    if (settled_ >= config_.settle_time) {
                        return finish(FollowState::kDone, FollowFault::kNone);
                    }
                } else {
                    settled_ = 0;
                    state_   = FollowState::kRunning;
                }
            }
            out.command = limit(turn(s, robot, error, dt), dt);
            out.state   = state_;
            return out;
        }

        Track        t;
        const double dx = s.end.x - s.start.x;
        const double dy = s.end.y - s.start.y;
        t.length        = std::hypot(dx, dy);
        if (t.length < kTinySegment) {
            if (!last()) {
                ++index_;
                resetControl();
                continue;
            }
            t.dir_x = std::cos(s.end.heading);
            t.dir_y = std::sin(s.end.heading);
        } else {
            t.dir_x = dx / t.length;
            t.dir_y = dy / t.length;
        }
        const double rx = robot.x - s.start.x;
        const double ry = robot.y - s.start.y;
        t.along         = rx * t.dir_x + ry * t.dir_y;
        t.cross         = t.dir_x * ry - t.dir_y * rx;
        t.remaining     = t.length - t.along;
        out.cross_track = t.cross;
        out.remaining   = t.remaining;

        if (std::fabs(t.cross) > config_.tracking_tolerance) {
            FollowOutput failed = finish(FollowState::kFailed, FollowFault::kTrackingError);
            failed.cross_track  = t.cross;
            failed.remaining    = t.remaining;
            return failed;
        }
        if (t.remaining <= config_.position_tolerance && !last()) {
            ++index_;
            resetControl();
            continue;
        }

        const double reach = std::sqrt(2.0 * limits_.max_accel * std::fabs(t.remaining));
        const double cap   = std::min(limits_.max_speed, reach);
        const double floor = last() ? -cap : 0.0;
        double       speed = std::clamp(along_.update(t.remaining, dt), floor, cap);
        if (t.remaining > config_.position_tolerance) {
            speed = raise(speed, std::min(config_.min_speed, cap));
        }
        t.speed = speed;

        Radians heading_error = 0;
        out.command           = limit(translate(s, robot, t, dt, heading_error), dt);
        out.heading_error     = heading_error;

        if (last()) {
            const double position = std::hypot(robot.x - s.end.x, robot.y - s.end.y);
            const double final_heading =
                std::fabs(wrapAngle(s.end.heading - robot.heading));
            const bool inside = position <= config_.position_tolerance &&
                                (!holonomic() || final_heading <= config_.heading_tolerance);
            if (inside) {
                settled_ += dt;
                state_ = FollowState::kSettling;
                if (settled_ >= config_.settle_time) {
                    return finish(FollowState::kDone, FollowFault::kNone);
                }
            } else {
                settled_ = 0;
                state_   = FollowState::kRunning;
            }
        }
        out.state = state_;
        return out;
    }
    return finish(FollowState::kDone, FollowFault::kNone);
}

ChassisCommand DifferentialFollower::translate(const PathSegment& s, const Pose& robot,
                                               const Track& t, Seconds dt,
                                               Radians& heading_error) {
    const double travel = std::atan2(t.dir_y, t.dir_x) + (s.reverse ? kPi : 0.0);
    heading_error       = wrapAngle(travel - robot.heading);

    ChassisCommand out;
    out.omega = heading_.update(heading_error, dt) - cross_.update(t.cross, dt);
    double speed = t.speed;
    if (std::fabs(heading_error) > config_.max_heading_error) {
        speed = 0;
    } else {
        speed *= std::max(0.0, std::cos(heading_error));
    }
    out.vx = s.reverse ? -speed : speed;
    return out;
}

ChassisCommand DifferentialFollower::turn(const PathSegment&, const Pose&, Radians error,
                                          Seconds dt) {
    ChassisCommand out;
    const double   omega = turn_.update(error, dt);
    out.omega = std::fabs(error) <= config_.heading_tolerance ? omega
                                                              : raise(omega, config_.min_omega);
    return out;
}

ChassisCommand HolonomicFollower::translate(const PathSegment& s, const Pose& robot,
                                            const Track& t, Seconds dt,
                                            Radians& heading_error) {
    const double fraction =
        t.length > kTinySegment ? std::clamp(t.along / t.length, 0.0, 1.0) : 1.0;
    const double target =
        s.start.heading + wrapAngle(s.end.heading - s.start.heading) * fraction;
    heading_error = wrapAngle(target - robot.heading);

    const double   correction = -cross_.update(t.cross, dt);
    ChassisCommand field;
    field.frame = ChassisFrame::kField;
    field.vx    = t.dir_x * t.speed - t.dir_y * correction;
    field.vy    = t.dir_y * t.speed + t.dir_x * correction;
    field.omega = heading_.update(heading_error, dt);
    return toBody(field, robot.heading);
}

ChassisCommand HolonomicFollower::turn(const PathSegment& s, const Pose& robot, Radians error,
                                       Seconds dt) {
    ChassisCommand field;
    field.frame = ChassisFrame::kField;
    const double hold = config_.along.kP;
    field.vx          = hold * (s.start.x - robot.x);
    field.vy          = hold * (s.start.y - robot.y);
    const double omega = turn_.update(error, dt);
    field.omega = std::fabs(error) <= config_.heading_tolerance ? omega
                                                                : raise(omega, config_.min_omega);
    return toBody(field, robot.heading);
}

} // namespace actugatr
