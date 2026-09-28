// motion.h
// Movement commands. Two commands share one reference argument; omitting it
// means the field origin:
//
//   motion.goToDirect({1.0, 0.5, 0.0});
//   motion.goToAvoiding({-0.4, 0.0, 0.0}, Field::SomeLandmark);
//
// The destination is relative to the reference; the reference translation
// and rotation are applied once. Direct plans a straight approach shaped for
// the drivetrain and never looks at obstacles. Avoiding needs a complete
// field and reports blocked or unreachable goals; it never falls back to
// direct. A command keeps its id and its deadline through replans. update()
// is called by the single drive owner every cycle.

#pragma once
#include <cstdint>

#include "actugatr/chassis.h"
#include "actugatr/follower.h"
#include "investigatr/field.h"
#include "investigatr/motion_model.h"
#include "investigatr/path.h"
#include "investigatr/planner.h"
#include "investigatr/reference.h"
#include "investigatr/state_source.h"

namespace actugatr
{

using investigatr::CommandId;
using investigatr::EstimateSource;
using investigatr::MotionModel;
using investigatr::ObjectId;
using investigatr::PlanMode;
using investigatr::Reference;

struct MoveOptions {
    Seconds timeout                    = 0;    // 0 = MotionConfig::default_timeout
    bool    require_observed_reference = true; // object references need an observed estimate
    Seconds reference_max_age          = 0;    // observed estimate age limit, 0 = none
    bool    allow_reverse              = true; // non-holonomic drives may back up
    double  speed_scale                = 1.0;  // (0, 1] of the model speed limits
};

enum class MotionState : uint8_t {
    kIdle,
    kWaiting, // for usable state, the field, or the reference
    kRunning,
    kSettling,
    kCompleted,
    kCancelled,
    kFailed,
};

enum class MotionReason : uint8_t {
    kNone,
    kInvalidCommand,
    kInvalidConfig,
    kInputUnavailable,
    kNoProfile,
    kCalibrating,
    kPlacementRequired,
    kInputLost,
    kFrameChanged,
    kFieldUnavailable,
    kMapMismatch,
    kUnknownReference,
    kNotReference,
    kReferenceUnavailable,
    kUnsupportedModel,
    kStartOutOfBounds,
    kStartBlocked,
    kGoalOutOfBounds,
    kGoalBlocked,
    kNoPath,
    kTrackingError,
    kPlanLimit,
    kTimedOut,
    kSourceChanged,
    kCancelledByCaller,
};

bool        isTerminal(MotionState state);
const char* toString(MotionState state);
const char* toString(MotionReason reason);

struct MotionStatus {
    CommandId      command_id = 0;
    MotionState    state      = MotionState::kIdle;
    MotionReason   reason     = MotionReason::kNone;
    PlanMode       mode       = PlanMode::kDirect;
    bool           has_destination = false;
    Pose           destination; // field frame, latest resolution
    EstimateSource reference_source = EstimateSource::kNone;
    std::size_t    segment          = 0;
    std::size_t    segment_count    = 0;
    Meters         distance_error   = 0; // robot to destination
    Radians        heading_error    = 0; // destination heading minus robot heading
    Meters         cross_track      = 0;
    Seconds        elapsed          = 0; // since the command's first update
    uint32_t       plans            = 0; // first plan, replans and corrections
    ObjectId       blocking         = 0; // object blocking the start or goal
};

struct MotionConfig {
    MotionModel model;

    Seconds max_pose_age       = 0.25;
    Seconds max_link_age       = 0.25;
    Seconds input_wait_timeout = 2.0; // before the first plan
    Seconds input_loss_timeout = 0.3; // stale while connected, after the first plan
    Seconds default_timeout    = 10.0;
    Seconds max_dt             = 0.1;

    // A field correction moving the destination beyond these replans; smaller
    // moves keep the current path.
    Meters  replan_distance = 0.03;
    Radians replan_heading  = 0.05;

    // After the follower settles, larger errors plan a correction.
    Meters  final_position_tolerance = 0.03;
    Radians final_heading_tolerance  = 0.05;

    // A replan while moving first brakes until the pose is steady (below
    // both rates for halt_cycles updates) or halt_timeout passes, then plans
    // from where the robot actually stopped.
    MetersPerSecond  halt_speed   = 0.02;
    RadiansPerSecond halt_omega   = 0.05;
    int              halt_cycles  = 3;
    Seconds          halt_timeout = 1.0;

    uint32_t max_plans = 12; // per command
};

bool valid(const MotionConfig& config, const char** why = nullptr);

class Motion {
public:
    // follower.holonomic() must match config.model.holonomic.
    Motion(investigatr::StateSource& source, const investigatr::PathPlanner& planner,
           Follower& follower, const MotionConfig& config);

    // Each command replaces the previous one at once and gets a new id.
    CommandId goToDirect(const Pose& destination, const Reference& relative_to = Reference::origin(),
                         const MoveOptions& options = {});
    CommandId goToAvoiding(const Pose&        destination,
                           const Reference&   relative_to = Reference::origin(),
                           const MoveOptions& options     = {});

    // Active command -> kCancelled. No effect once terminal.
    void cancel();

    // Cancels an active command with kSourceChanged.
    void setSource(investigatr::StateSource& source);

    // The next command gets this id (0 ignored); later ones continue from it.
    // For owners that hand out ids before the command reaches Motion.
    void setNextId(CommandId id);

    // Receives every plan and a clear when the command ends. nullptr = none.
    void setPathSink(investigatr::PathSink* sink) { sink_ = sink; }

    const MotionStatus& status() const { return status_; }
    const investigatr::Path& path() const { return path_; }
    const MotionConfig&  config() const { return config_; }
    bool                 active() const;

    // Body frame command for this cycle; zero when idle, waiting or ended.
    ChassisCommand update(Seconds now);

private:
    enum class Step : uint8_t { kPlanned, kWait, kEnded };

    CommandId start(PlanMode mode, const Pose& destination, const Reference& reference,
                    const MoveOptions& options);
    void      finish(MotionState state, MotionReason reason);
    Step      plan(const investigatr::RobotState& robot, Seconds now);
    bool      needsReplan(const investigatr::RobotState& robot, Seconds now);
    investigatr::Path remaining() const;
    investigatr::MotionModel scaledModel() const;
    MotionReason        waitReason(const investigatr::RobotState& robot) const;
    ChassisCommand      zero();

    investigatr::StateSource*       source_;
    const investigatr::PathPlanner& planner_;
    Follower&                       follower_;
    MotionConfig                    config_;
    bool                            config_valid_ = false;
    investigatr::PathSink*          sink_         = nullptr;

    MotionStatus      status_;
    CommandId         last_id_ = 0;
    Pose              relative_;
    Reference         reference_;
    MoveOptions       options_;
    investigatr::Path path_;

    bool    have_last_update_ = false;
    Seconds last_update_      = 0;
    bool    started_          = false;
    Seconds start_time_       = 0;

    bool                         frame_captured_ = false;
    investigatr::FrameGeneration frame_          = 0;
    Pose                         robot_at_start_;
    bool                         losing_       = false;
    Seconds                      losing_since_ = 0;

    bool    halting_     = false;
    Seconds halt_since_  = 0;
    int     steady_      = 0;
    bool    have_last_pose_ = false;
    Pose    last_pose_;

    investigatr::Field field_;
    bool               have_field_       = false;
    bool               planned_          = false;
    bool               have_resolution_  = false;
    uint32_t           plan_generation_  = 0;
    Pose               resolved_;
};

} // namespace actugatr
