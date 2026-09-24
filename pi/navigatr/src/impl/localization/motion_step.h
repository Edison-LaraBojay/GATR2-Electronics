// motion_step.h
// Everything the planar state estimators share before and after
// integrating one step: the placement edge, clock identity, the heading
// aligner, the attitude fold, and the arc chord. The estimators differ
// only in how they combine the aligned rotation and what they say about
// uncertainty.
//
// Clock identity: a device stamp is comparable with another only on one
// named clock (Provenance.clock, the acquiring link); a host stamp is on
// the host clock. Domain alone identifies nothing. Two observations are
// combined only on a verified common clock, and a motion whose clock
// identity changes is a discontinuity, like a source reboot.
//
// Lineage: Provenance.measurement names the acquisition output behind a
// sample. Two contributors overlap when any of their measurements match,
// whatever sensor ids they were configured under; distinct outputs of one
// device stay distinct.
//
// Alignment: a heading is combined with a motion only when its support
// equals the motion window exactly on the common clock. Headings that
// cover the window from its start contiguously accumulate here, each
// accepted as it arrives, until the support reaches the motion end; the
// motion stays pending meanwhile. A heading that starts elsewhere,
// overruns the window, or arrives on another clock is rejected with the
// reason. Angles are summed, never scaled by duration. Every held motion
// window has a deadline from the call that first saw it, checked on every
// call whether headings keep arriving or not: past max_wait_ms a window
// with rotation goes on without the gyro and one without is rejected.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "contracts/localization.h"
#include "core/clock_sync.h"
#include "payloads/robot_observations.h"

namespace navigatr
{

// Acquisition identity, falling back to the configured id when a record
// never carried one.
inline const std::string& measurementOf(const Provenance& p) {
    return p.measurement.empty() ? p.source : p.measurement;
}

// Any physical measurement in common.
bool sharesMeasurement(const std::vector<Provenance>& a, const std::vector<Provenance>& b);

// The one clock an observation's sources put its stamps on: "host" for
// host stamps, the named device clock otherwise, "" when device sources
// name none. False when the sources disagree.
bool observationClock(MonotonicTime stamp, const std::vector<Provenance>& sources,
                      std::string& clock, std::string& why);

// Whether stamps on these two clocks may be compared.
bool comparableClocks(const std::string& a, const std::string& b, std::string& why);

// Edge triggered per (origin, sequence): re-anchors the field frame around
// the untouched odometry pose and bumps anchor_revision.
struct PlacementEdge {
    std::string origin;
    uint64_t    sequence = 0;

    bool apply(RobotState& r, const PlacementRequest& placement);

    void reset() {
        origin.clear();
        sequence = 0;
    }
};

// Chord of the constant-curvature arc across a body-frame increment.
void chordOfArc(double dx, double dy, double dtheta, double& lx, double& ly);

// <Attitude observation_id="attitude" max_age_ms="200"/>, optional. The
// newest observation is retained and aged against its own host time; when
// fresh its tilt combines with the planar heading, otherwise the state
// carries an explicitly assumed level attitude. The source's own yaw is
// never applied: tilt-only attitude never becomes absolute yaw.
class AttitudeFold
{
public:
    bool configure(const ConfigNode& node, const StateEstimatorInitializationContext& context,
                   std::string& err);

    bool                 configured() const { return !ref_.empty(); }
    const ObservationId& ref() const { return ref_; }

    void apply(StateEstimatorOutput& out, const StateEstimatorInput& in,
               const DeviceToHostClock& clock, const std::string& motion_clock,
               bool clock_valid);

    void forget() { retained_ = Retained{}; }
    void reset() { forget(); }

private:
    struct Retained {
        bool                valid = false;
        AttitudeObservation observation;
        MonotonicTime       hostAt;
    };

    ObservationId ref_;
    long          max_age_ms_ = 200;
    Retained      retained_;
};

// Accumulates heading support over one motion window. See the file
// comment for the rules.
class HeadingAligner
{
public:
    enum class Outcome {
        kNone,      // no usable heading for this motion; the motion stands alone
        kFused,     // dtheta/dt cover the motion window exactly
        kWaiting,   // support is partial; hold the motion
    };

    struct Result {
        Outcome     outcome        = Outcome::kNone;
        double      dtheta_rad     = 0.0;
        double      dt_s           = 0.0;
        bool        accept_heading = false;   // the offered heading was consumed
        bool        reject_heading = false;
        bool        released       = false;   // partial support was given up
        bool        expired        = false;   // this window has waited past max_wait_ms
        std::string diagnostic;
    };

    // Every motion window gets a deadline from the first call that sees
    // it, checked on every call whether or not a heading is offered: once
    // past it, a waiting window is released and a caller holding for a
    // missing rotation is told to give the motion up.
    Result align(const HeadingIncrement* heading, const BodyMotionIncrement& motion,
                 const std::string& motion_clock, MonotonicTime now, long max_wait_ms);

    bool active() const { return stash_.active; }
    void reset() {
        stash_  = Stash{};
        window_ = Window{};
    }

private:
    Result alignWindow(const HeadingIncrement* heading, const BodyMotionIncrement& motion,
                       const std::string& motion_clock);

    // the motion window currently being served and when it was first seen
    struct Window {
        bool          tracked = false;
        MonotonicTime start, end;
        MonotonicTime since;   // host clock
    };
    Window window_;
    struct Stash {
        bool          active = false;
        MonotonicTime motion_start, motion_end;   // the window being covered
        MonotonicTime end;                        // support reached so far
        double        dtheta = 0.0;
        double        dt     = 0.0;
        std::string   clock;
        uint64_t      epoch = 0;
    };
    Stash stash_;
};

// Persistent state of one estimator's step handling.
struct PlanarStepState {
    PlacementEdge     placement;
    AttitudeFold      attitude;
    HeadingAligner    aligner;
    DeviceToHostClock clock;          // motion device clock onto the host clock
    std::string       motion_clock;   // identity of the clock the state's measuredAt is on

    void reset() {
        placement.reset();
        attitude.reset();
        aligner.reset();
        clock.reset();
        motion_clock.clear();
    }
};

// What prepareStep hands back when the motion is ready to integrate.
struct PreparedStep {
    const BodyMotionIncrement* motion = nullptr;
    MonotonicTime              stamp;   // effective time, motion clock
    bool                       heading_used = false;
    double                     heading_dtheta_rad = 0.0;
    double                     heading_dt_s       = 0.0;
    bool                       heading_released   = false;
};

// Rejects unlisted observations, applies the placement edge, finds and
// validates the motion, verifies its clock identity, detects regression
// and repeats, aligns the heading, and folds the attitude on every early
// return. True means integrate step now; false means out is final.
bool prepareStep(PlanarStepState& state, const StateEstimatorInput& in,
                 const ObservationId& motion_ref, const ObservationId& heading_ref,
                 long max_wait_ms, StateEstimatorOutput& out, PreparedStep& step);

// After integration: clock mapping, effective times, dispositions, and
// the attitude fold.
void finishStep(PlanarStepState& state, const StateEstimatorInput& in,
                const ObservationId& motion_ref, const ObservationId& heading_ref,
                const RobotObservationRecord& motion_record, const PreparedStep& step,
                StateEstimatorOutput& out);

// Reads <Heading observation_id=".." max_wait_ms="100"> and refuses the
// retired interval_tolerance_ms.
bool configureHeading(const ConfigNode& node, const StateEstimatorInitializationContext& context,
                      ObservationId& ref, long& max_wait_ms, std::string& err);

} // namespace navigatr
