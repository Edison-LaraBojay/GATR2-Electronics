// motion_step.h
// Pieces shared by the planar state estimators: the placement edge, the
// arc chord, and the attitude fold that puts a measured tilt under the
// planar heading and ages it on its own clock.

#pragma once
#include <cstdint>
#include <string>

#include "contracts/localization.h"
#include "core/clock_sync.h"
#include "payloads/robot_observations.h"

namespace navigatr
{

// Edge triggered per (origin, sequence): re-anchors the field frame around
// the untouched odometry pose and bumps anchor_revision.
struct PlacementEdge {
    std::string origin;
    uint64_t    sequence = 0;

    // True when the request was a new edge and was applied.
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
    // node is the estimator element; a missing Attitude child is fine.
    bool configure(const ConfigNode& node, const StateEstimatorInitializationContext& context,
                   std::string& err);

    bool                 configured() const { return !ref_.empty(); }
    const ObservationId& ref() const { return ref_; }

    // clock maps device stamps carrying motion_clock onto the host clock.
    void apply(StateEstimatorOutput& out, const StateEstimatorInput& in,
               const DeviceToHostClock& clock, const std::string& motion_clock,
               bool clock_valid);

    // The motion clock changed or restarted: a retained stamp means nothing.
    void forget() { retained_ = Retained{}; }
    void reset() { forget(); }

private:
    struct Retained {
        bool                valid = false;
        AttitudeObservation observation;
        MonotonicTime       hostAt;   // set once the source time maps
    };

    ObservationId ref_;
    long          max_age_ms_ = 200;
    Retained      retained_;
};

} // namespace navigatr
