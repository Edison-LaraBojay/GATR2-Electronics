// navigator.h
// Waypoint navigation for a tank drive over an InputSource. Commands set the
// objective and return at once; update() samples the source and returns the
// drive demand. Forward driving only, no obstacle routing.

#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "investigatr/drive.h"
#include "investigatr/geometry.h"
#include "investigatr/input.h"
#include "investigatr/pid.h"

namespace investigatr
{

// Desired robot pose. Relative offsets are expressed in the landmark frame or
// in the robot frame at the time the waypoint activates.
struct Destination {
    enum class Kind : uint8_t { kField, kLandmark, kRobot };

    Kind       kind     = Kind::kField;
    LandmarkId landmark = 0;
    Pose       pose; // field pose, or the offset for relative kinds

    static Destination field(const Pose& pose);
    static Destination relative(LandmarkId landmark, const Pose& offset);
    static Destination robotRelative(const Pose& offset);
};

struct Waypoint {
    Destination destination;
    bool        stop = false; // the last waypoint always stops
};

using Path      = std::vector<Waypoint>;
using CommandId = uint32_t;

struct MotionOptions {
    Seconds timeout                   = 0; // 0 = NavigatorConfig::default_timeout
    bool    require_observed_landmark = true;
};

enum class MotionState : uint8_t {
    kIdle,
    kWaiting,
    kTurning,
    kDriving,
    kAligning,
    kCompleted,
    kCancelled,
    kFailed,
};

enum class MotionReason : uint8_t {
    kNone,
    kInvalidCommand,
    kInputUnavailable,
    kInputLost,
    kFrameChanged,
    kLandmarkUnknown,
    kLandmarkUnsupported,
    kLandmarkUnavailable,
    kLandmarkJump,
    kTimedOut,
    kSourceChanged,
    kCancelledByCaller,
};

bool        isTerminal(MotionState state);
const char* toString(MotionState state);
const char* toString(MotionReason reason);

struct MotionStatus {
    CommandId    command_id      = 0;
    MotionState  state           = MotionState::kIdle;
    MotionReason reason          = MotionReason::kNone;
    std::size_t  waypoint_index  = 0;
    std::size_t  waypoint_count  = 0;
    bool         has_destination = false;
    Pose         destination;        // current waypoint, field frame
    Meters       distance_error = 0; // robot to destination
    Radians      bearing_error  = 0; // direction to destination minus heading
    Radians      heading_error  = 0; // destination heading minus heading
    Seconds      elapsed        = 0; // since the command's first update
};

struct NavigatorConfig {
    Meters   position_tolerance      = 0.03;
    Meters   arrive_distance         = 0.02;
    Radians  heading_tolerance       = 0.035;
    Seconds  settle_time             = 0.25;
    Meters   near_distance           = 0.10;
    Meters   waypoint_pass_radius    = 0.15;
    Radians  turn_in_place_threshold = 0.5;
    Radians  turn_exit               = 0.1;
    PidGains drive_pid{2.0, 0.0, 0.1, 0.0, 1.0};    // per m of along-track error
    PidGains heading_pid{1.5, 0.0, 0.05, 0.0, 1.0}; // per rad, while driving
    PidGains turn_pid{1.2, 0.0, 0.06, 0.0, 1.0};    // per rad, turning in place
    double   max_forward                    = 0.8;
    double   max_turn                       = 0.7;
    double   forward_slew                   = 2.0; // per s, 0 = none
    double   turn_slew                      = 3.0; // per s, 0 = none
    double   min_forward                    = 0.0;
    double   min_turn                       = 0.0;
    Seconds  max_pose_age                   = 0.25;
    Seconds  max_link_age                   = 0.25;
    Seconds  input_wait_timeout             = 2.0;
    Seconds  input_loss_timeout             = 0.3;
    Seconds  default_timeout                = 10.0;
    Seconds  max_landmark_age               = 1.5;
    Meters   max_landmark_jump              = 0.15;
    Radians  max_landmark_jump_heading      = 0.25;
    Meters   max_nominal_correction         = 0.5;
    Radians  max_nominal_correction_heading = 0.5;
    double   landmark_follow_speed          = 0.3; // m/s
    double   landmark_follow_turn_rate      = 0.6; // rad/s
    Seconds  max_dt                         = 0.1;
};

class Navigator {
public:
    explicit Navigator(InputSource& source, const NavigatorConfig& config = {});

    // Each command replaces the previous one at once and gets a new id.
    CommandId goTo(const Pose& field_pose, const MotionOptions& options = {});
    CommandId goToRelative(LandmarkId landmark, const Pose& landmark_local_offset,
                           const MotionOptions& options = {});
    CommandId goToRobotRelative(const Pose& offset, const MotionOptions& options = {});
    CommandId follow(const Path& path, const MotionOptions& options = {});

    // Active command -> kCancelled. No effect once terminal.
    void cancel();

    // Releases the old source and cancels an active command.
    void setSource(InputSource& source);

    const MotionStatus&    status() const { return status_; }
    const NavigatorConfig& config() const { return config_; }

    DriveCommand update(Seconds now);

    static bool valid(const NavigatorConfig& config, const char** why = nullptr);

private:
    struct Errors {
        Meters  distance = 0;
        Radians bearing  = 0;
        Radians heading  = 0;
        Meters  along    = 0;
    };

    enum class Resolve : uint8_t { kReady, kWait, kEnded };

    bool         active() const;
    bool         stopsAt(std::size_t index) const;
    InputRequest need() const;
    bool         robotUsable(const InputSnapshot& snapshot) const;
    bool         landmarkUsable(const LandmarkEstimate& landmark, LandmarkId id) const;
    Meters       remainingPath() const;
    Errors       errorsTo(const Pose& robot) const;

    void         finish(MotionState state, MotionReason reason);
    void         activate(std::size_t index, Seconds now);
    void         wait();
    void         setPhase(MotionState phase);
    void         selectPhase(const Errors& e);
    void         resetControl();
    double       holdTurnSign(double error);
    Resolve      resolve(const InputSnapshot& snapshot, Seconds now, Seconds dt);
    Resolve      followLandmark(const LandmarkEstimate& landmark, Seconds now, Seconds dt);
    DriveCommand navigate(const InputSnapshot& snapshot, Seconds now, Seconds dt);
    DriveCommand control(const Errors& e, Seconds dt, bool& settled);
    DriveCommand shape(const DriveCommand& demand, Seconds dt);
    DriveCommand zero();

    InputSource*    source_;
    NavigatorConfig config_;
    bool            config_valid_ = false;
    MotionStatus    status_;
    Path            path_;
    MotionOptions   options_;
    CommandId       last_id_ = 0;

    bool    have_last_update_ = false;
    Seconds last_update_      = 0;
    bool    started_          = false;
    Seconds start_time_       = 0;
    Seconds activated_at_     = 0;

    bool            frame_captured_ = false;
    FrameGeneration frame_          = 0;
    bool            losing_input_   = false;
    Seconds         losing_since_   = 0;

    bool           resolved_ = false;
    Pose           destination_;
    Pose           landmark_; // latest accepted landmark candidate
    LandmarkSource landmark_source_    = LandmarkSource::kNone;
    bool           select_phase_       = true;
    bool           along_was_positive_ = false;

    Pid          drive_pid_;
    Pid          heading_pid_;
    Pid          turn_pid_;
    int          turn_sign_ = 0;
    bool         settling_  = false;
    Seconds      settled_   = 0;
    DriveCommand output_;
};

} // namespace investigatr
