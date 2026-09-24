// planar_motion_integrator.cpp

#include "impl/localization/planar_motion_integrator.h"

#include <cmath>
#include <typeindex>

#include "math/angles.h"

namespace navigatr
{

std::unique_ptr<StateEstimator>
PlanarMotionIntegrator::create(const ConfigNode& node, StateEstimatorInitializationContext& context,
                               std::string& err) {
    auto estimator = std::make_unique<PlanarMotionIntegrator>();

    estimator->motion_ref_ = ObservationId{node.child("Motion").attr("observation_id")};
    if (estimator->motion_ref_.empty()) {
        err = node.path() + ": needs <Motion observation_id=.../>";
        return nullptr;
    }
    const std::type_index motion_type(typeid(BodyMotionIncrement));
    if (!context.requireObservation(estimator->motion_ref_, &motion_type, node.path(), err)) {
        return nullptr;
    }
    if (!configureHeading(node, context, estimator->heading_ref_, estimator->max_wait_ms_,
                          err)) {
        return nullptr;
    }
    if (!estimator->state_.attitude.configure(node, context, err)) {
        return nullptr;
    }
    return estimator;
}

StateEstimatorOutput PlanarMotionIntegrator::run(const StateEstimatorInput& in) {
    StateEstimatorOutput out;
    out.robot = in.previous;

    PreparedStep step;
    if (!prepareStep(state_, in, motion_ref_, heading_ref_, max_wait_ms_, out, step)) {
        return out;
    }
    const BodyMotionIncrement& motion = *step.motion;
    const double dtheta = step.heading_used ? step.heading_dtheta_rad : motion.dtheta_rad;

    double lx = 0.0, ly = 0.0;
    chordOfArc(motion.dx_m, motion.dy_m, dtheta, lx, ly);

    RobotState&  r  = out.robot;
    const double h  = r.odom_pose.heading_rad;
    const double gx = lx * std::cos(h) - ly * std::sin(h);
    const double gy = lx * std::sin(h) + ly * std::cos(h);

    r.odom_pose.x_m += gx;
    r.odom_pose.y_m += gy;
    r.odom_pose.heading_rad = wrapAngle(h + dtheta);

    r.vx_m_s         = gx / motion.dt_s;
    r.vy_m_s         = gy / motion.dt_s;
    r.yaw_rate_rad_s = dtheta / motion.dt_s;

    finishStep(state_, in, motion_ref_, heading_ref_, in.observations.at(motion_ref_), step,
               out);
    return out;
}

} // namespace navigatr
