// register_localization.cpp

#include "impl/localization/attitude_reference.h"
#include "impl/localization/imu_heading_increment.h"
#include "impl/localization/planar_motion_integrator.h"
#include "impl/localization/tracking_wheel_motion.h"
#include "impl/localization/weighted_planar_fusion.h"
#include "impl/noop/noops.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_localization(FunctionRegistry& functions) {
    registerOrDie<RobotObservationMakeFunction>(functions, "tracking_wheel_motion",
                                                &TrackingWheelMotion::create);
    registerOrDie<RobotObservationMakeFunction>(functions, "imu_heading_increment",
                                                &ImuHeadingIncrement::create);
    registerOrDie<RobotObservationMakeFunction>(functions, "attitude_reference",
                                                &AttitudeReference::create);

    registerOrDie<StateEstimatorMakeFunction>(functions, "noop", &makeNoopStateEstimator);
    registerOrDie<StateEstimatorMakeFunction>(functions, "planar_motion_integrator",
                                              &PlanarMotionIntegrator::create);
    registerOrDie<StateEstimatorMakeFunction>(functions, "weighted_planar_fusion",
                                              &WeightedPlanarFusion::create);
}

} // namespace navigatr
