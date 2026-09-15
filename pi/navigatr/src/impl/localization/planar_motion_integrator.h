// planar_motion_integrator.h
// State estimator: integrates body motion increments into the continuous
// odometry-frame pose using the constant-curvature chord, maps source
// stamps onto the host clock, applies placement requests as field
// re-anchors, and folds an optional heading increment and an optional
// attitude observation.
//
//   <Estimator type="planar_motion_integrator">
//       <Motion observation_id="tracking_motion"/>
//       <Heading observation_id="imu_heading" interval_tolerance_ms="20"/>   optional
//       <Attitude observation_id="attitude" max_age_ms="200"/>             optional
//   </Estimator>
//
// Rules:
//   placement    edge triggered per (origin, sequence); re-anchors the field
//                frame around the untouched odometry pose, bumps
//                anchor_revision
//   motion       integrated only over a positive interval on the same source
//                clock; device time running backwards means the source
//                rebooted, so the odometry epoch moves on and the step is
//                not integrated
//   heading      replaces the motion's rotation only when its interval
//                matches the motion interval within the tolerance and it
//                shares no source with the motion observation (an IMU already
//                folded by the wheel model is not independent); otherwise
//                the motion's own rotation stands and the mismatch is noted
//   partial      a motion increment without observed rotation is refused
//                unless an aligned heading supplies it; nothing is fabricated
//   attitude     the newest observation, aged against its own host time; when
//                fresh its tilt combines with the planar heading, otherwise
//                the state carries an explicitly assumed level attitude
//   hold         without motion the estimate keeps its previous effective
//                time; loop iterations never refresh it

#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "contracts/localization.h"
#include "core/clock_sync.h"
#include "payloads/robot_observations.h"

namespace navigatr
{

class PlanarMotionIntegrator : public StateEstimator
{
public:
    static std::unique_ptr<StateEstimator> create(const ConfigNode& node,
                                                  StateEstimatorInitializationContext& context,
                                                  std::string& err);

    StateEstimatorOutput run(const StateEstimatorInput& in) override;

    const std::string& type() const override { return type_; }

    void reset() override;

private:
    struct RetainedAttitude {
        bool                valid = false;
        AttitudeObservation observation;
        MonotonicTime       hostAt;   // set once the source time maps
    };

    void applyAttitude(StateEstimatorOutput& out, const StateEstimatorInput& in,
                       bool clock_valid);

    std::string   type_ = "planar_motion_integrator";
    ObservationId motion_ref_;
    ObservationId heading_ref_;    // empty when not configured
    ObservationId attitude_ref_;   // empty when not configured
    long          heading_tolerance_ms_ = 20;
    long          attitude_max_age_ms_  = 200;

    std::string last_placement_origin_;
    uint64_t    last_placement_sequence_ = 0;

    RetainedAttitude attitude_;

    // Maps the motion source's device stamps onto the host clock so pose
    // history carries the time the pose was physically true, not the loop
    // time it was computed at.
    DeviceToHostClock clock_;
    std::string motion_clock_;
};

} // namespace navigatr
