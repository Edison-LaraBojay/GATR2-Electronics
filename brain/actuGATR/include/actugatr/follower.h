// follower.h
// Executes a planned path segment by segment from pose feedback. A segment
// is finished (position along it, or heading for a turn) before the next one
// starts, and translations slow toward their end, so the executed motion
// stays on the planned segments instead of cutting corners. Cross-track
// beyond tracking_tolerance fails the path, because the planner's clearance
// only covers motion near the segments. A turn allows drift from its point up
// to hypot(position_tolerance, tracking_tolerance), the most a finished
// translation can leave.

#pragma once
#include <cstddef>
#include <cstdint>

#include "actugatr/chassis.h"
#include "actugatr/pid.h"
#include "investigatr/motion_model.h"
#include "investigatr/path.h"

namespace actugatr
{

using investigatr::MotionLimits;
using investigatr::Path;
using investigatr::PathSegment;

struct FollowerConfig {
    Meters  position_tolerance = 0.02;
    Radians heading_tolerance  = 0.03;
    Seconds settle_time        = 0.2;  // inside both tolerances at the end
    Meters  tracking_tolerance = 0.04; // keep below the planner clearance
    Radians max_heading_error  = 0.5;  // translation pauses beyond this

    PidGains along{3.0, 0.0, 0.0, 0.0, 100.0};   // m/s per m remaining
    PidGains cross{3.0, 0.0, 0.0, 0.0, 100.0};   // differential rad/s per m, holonomic m/s per m
    PidGains heading{4.0, 0.0, 0.1, 0.0, 100.0}; // rad/s per rad while translating
    PidGains turn{4.0, 0.0, 0.2, 0.0, 100.0};    // rad/s per rad turning in place

    MetersPerSecond  min_speed = 0.0; // raise nonzero demands to overcome static friction
    RadiansPerSecond min_omega = 0.0;
};

bool valid(const FollowerConfig& config, const char** why = nullptr);

enum class FollowState : uint8_t { kIdle, kRunning, kSettling, kDone, kFailed };

enum class FollowFault : uint8_t { kNone, kInvalidPath, kTrackingError };

struct FollowOutput {
    ChassisCommand command; // body frame
    FollowState    state = FollowState::kIdle;
    FollowFault    fault = FollowFault::kNone;
    std::size_t    segment       = 0;
    Meters         cross_track   = 0; // left of the segment positive
    Meters         remaining     = 0; // along the current translation
    Radians        heading_error = 0;
};

class Follower {
public:
    virtual ~Follower() = default;

    virtual bool holonomic() const = 0;

    // limits are the model limits, already scaled for the command.
    virtual void         start(const Path& path, const MotionLimits& limits) = 0;
    virtual FollowOutput update(const Pose& robot, Seconds dt)               = 0;
    virtual void         stop()                                              = 0;
};

// Shared segment sequencing, settling, tracking checks and output limits.
class SegmentFollower : public Follower {
public:
    explicit SegmentFollower(const FollowerConfig& config);

    void         start(const Path& path, const MotionLimits& limits) override;
    FollowOutput update(const Pose& robot, Seconds dt) override;
    void         stop() override;

    const FollowerConfig& config() const { return config_; }

protected:
    struct Track {
        double  dir_x     = 1; // unit direction of travel along the segment
        double  dir_y     = 0;
        Meters  length    = 0;
        Meters  along     = 0;
        Meters  remaining = 0;
        Meters  cross     = 0;
        MetersPerSecond speed = 0; // along-track demand, >= 0
    };

    // Body command for a translation. speed is already profiled and limited.
    virtual ChassisCommand translate(const PathSegment& segment, const Pose& robot,
                                     const Track& track, Seconds dt, Radians& heading_error) = 0;

    // Body command for a turn in place at segment.start.
    virtual ChassisCommand turn(const PathSegment& segment, const Pose& robot, Radians error,
                                Seconds dt) = 0;

    void resetControl();

    FollowerConfig config_;
    MotionLimits   limits_;
    Pid            along_;
    Pid            cross_;
    Pid            heading_;
    Pid            turn_;

private:
    bool         last() const { return index_ + 1 >= path_.segments.size(); }
    Radians      turnError(const PathSegment& segment, const Pose& robot);
    FollowOutput finish(FollowState state, FollowFault fault);
    ChassisCommand limit(const ChassisCommand& command, Seconds dt);

    Path           path_;
    std::size_t    index_   = 0;
    FollowState    state_   = FollowState::kIdle;
    int            turn_sign_ = 0;
    Seconds        settled_ = 0;
    ChassisCommand output_;
};

// Tank and other drives that cannot move sideways.
class DifferentialFollower final : public SegmentFollower {
public:
    explicit DifferentialFollower(const FollowerConfig& config = {}) : SegmentFollower(config) {}

    bool holonomic() const override { return false; }

protected:
    ChassisCommand translate(const PathSegment& segment, const Pose& robot, const Track& track,
                             Seconds dt, Radians& heading_error) override;
    ChassisCommand turn(const PathSegment& segment, const Pose& robot, Radians error,
                        Seconds dt) override;
};

// Mecanum and other drives with independent translation and heading.
class HolonomicFollower final : public SegmentFollower {
public:
    explicit HolonomicFollower(const FollowerConfig& config = {}) : SegmentFollower(config) {}

    bool holonomic() const override { return true; }

protected:
    ChassisCommand translate(const PathSegment& segment, const Pose& robot, const Track& track,
                             Seconds dt, Radians& heading_error) override;
    ChassisCommand turn(const PathSegment& segment, const Pose& robot, Radians error,
                        Seconds dt) override;
};

} // namespace actugatr
