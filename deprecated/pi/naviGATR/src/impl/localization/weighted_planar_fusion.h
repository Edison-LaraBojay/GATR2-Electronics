// weighted_planar_fusion.h
// State estimator: covariance-weighted fusion of the supported robot
// observations into the odometry pose, with the pose covariance carried
// through every step. Same contract, placement, clock, alignment and
// disposition rules as planar_motion_integrator (motion_step); the
// difference is that every measurement enters with an explicit variance
// and the estimate reports one.
//
//   <Estimator type="weighted_planar_fusion">
//       <Motion observation_id="tracking_motion">
//           <Noise translation_floor_m="0.0005" translation_per_m="0.02"
//                  rotation_floor_rad="0.0005" rotation_per_rad="0.02"
//                  rotation_per_m="0.005"/>
//       </Motion>
//       <Heading observation_id="imu_heading" max_wait_ms="100">          optional
//           <Noise angle_random_walk_rad_per_sqrt_s="0.002"
//                  bias_rad_per_s="0.0005"/>
//       </Heading>
//       <Attitude observation_id="attitude" max_age_ms="200"/>             optional
//   </Estimator>
//
// Supported observations: one BodyMotionIncrement (required), one
// HeadingIncrement whose support equals the motion window on the same
// named clock (optional; partial support from the window start
// accumulates while the motion waits, up to max_wait_ms), one
// AttitudeObservation (optional, tilt only, never yaw). Nothing else is a
// robot observation here: landmark evidence is derived through this pose
// and never feeds back into it.
//
// Noise, all one-sigma, per accepted step over distance |d| and interval
// dt (the values are configuration, not measurements of a sensor):
//   translation   s_d^2 = floor_m^2 + (per_m |d|)^2, per body axis
//   wheel rotation s_w^2 = floor_rad^2 + (per_rad |dtheta_w|)^2 + (per_m |d|)^2
//   gyro rotation s_g^2 = ARW^2 dt + (bias dt)^2
//
// Per step, body frame:
//   rotation      precision weighted when an independent, aligned heading
//                 exists: dtheta = dtheta_w + w (dtheta_g - dtheta_w),
//                 w = s_w^2 / (s_w^2 + s_g^2), var = s_w^2 s_g^2 / (s_w^2 + s_g^2)
//   translation   re-solved through the producer's geometric coupling
//                 J = d(d)/d(dtheta): d = d_w + J (dtheta - dtheta_w), which
//                 adds J J' w^2 (s_w^2 + s_g^2) to its variance; without a
//                 coupling the translation stands as published
//   partial       a motion without observed rotation takes the aligned
//                 heading's rotation and, through J, its translation
//                 d = d_w + J dtheta_g with the cross term J s_g^2; without
//                 an aligned heading the step is refused
//   chord         l = C(dtheta) d with the Jacobian applied to the
//                 covariance
//   pose          x' = x + R(h) l, h' = h + dtheta,
//                 P' = F P F' + G Q G' with F carrying the heading to
//                 position coupling (-g_y, g_x)
//
// A heading that shares a measurement with the motion is not independent
// and is rejected, never counted twice (a wheel model with a
// HeadingConstraint already folded that gyro, under any sensor id).
// Covariance starts at zero at the odometry origin, which is exact by
// definition, and restarts there on reset.

#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "contracts/localization.h"
#include "impl/localization/motion_step.h"
#include "payloads/robot_observations.h"

namespace navigatr
{

class WeightedPlanarFusion : public StateEstimator
{
public:
    static std::unique_ptr<StateEstimator> create(const ConfigNode& node,
                                                  StateEstimatorInitializationContext& context,
                                                  std::string& err);

    StateEstimatorOutput run(const StateEstimatorInput& in) override;

    const std::string& type() const override { return type_; }

    void reset() override { state_.reset(); }

private:
    struct MotionNoise {
        double translation_floor_m = 0.0;
        double translation_per_m   = 0.0;
        double rotation_floor_rad  = 0.0;
        double rotation_per_rad    = 0.0;
        double rotation_per_m      = 0.0;
    };

    struct HeadingNoise {
        double angle_random_walk_rad_per_sqrt_s = 0.0;
        double bias_rad_per_s                   = 0.0;
    };

    std::string     type_ = "weighted_planar_fusion";
    ObservationId   motion_ref_;
    ObservationId   heading_ref_;   // empty when not configured
    long            max_wait_ms_ = 100;
    MotionNoise     motion_noise_;
    HeadingNoise    heading_noise_;
    PlanarStepState state_;
};

} // namespace navigatr
