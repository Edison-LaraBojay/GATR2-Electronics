// planar_motion_integrator.h
// State estimator: integrates body motion increments into the continuous
// odometry-frame pose using the constant-curvature chord, maps source
// stamps onto the host clock, applies placement requests as field
// re-anchors, and folds an optional heading increment and an optional
// attitude observation. No uncertainty model; weighted_planar_fusion is
// the estimator that carries one.
//
//   <Estimator type="planar_motion_integrator">
//       <Motion observation_id="tracking_motion"/>
//       <Heading observation_id="imu_heading" max_wait_ms="100"/>   optional
//       <Attitude observation_id="attitude" max_age_ms="200"/>     optional
//   </Estimator>
//
// Rules (shared with weighted_planar_fusion through motion_step):
//   placement    edge triggered per (origin, sequence); re-anchors the field
//                frame around the untouched odometry pose, bumps
//                anchor_revision
//   motion       integrated only over a positive interval on one named
//                source clock; device time running backwards, or a change
//                of clock identity, means the source is discontinuous: the
//                odometry epoch moves on and the step is not integrated
//   heading      replaces the motion's rotation only when its support
//                equals the motion window exactly on the same clock and it
//                shares no measurement with the motion (an IMU already
//                folded by the wheel model is not independent, whatever
//                sensor id it was configured under); partial support from
//                the window start accumulates while the motion waits, up
//                to max_wait_ms; anything else is rejected with the reason
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
#include "impl/localization/motion_step.h"
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

    void reset() override { state_.reset(); }

private:
    std::string     type_ = "planar_motion_integrator";
    ObservationId   motion_ref_;
    ObservationId   heading_ref_;   // empty when not configured
    long            max_wait_ms_ = 100;
    PlanarStepState state_;
};

} // namespace navigatr
