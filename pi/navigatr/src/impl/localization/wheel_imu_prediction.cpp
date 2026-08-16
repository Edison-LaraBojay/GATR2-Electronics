// wheel_imu_prediction.cpp

#include "impl/localization/wheel_imu_prediction.h"

#include <cmath>

#include "math/angles.h"
#include "payloads/preprocessing_products.h"

namespace navigatr
{

std::unique_ptr<Localization> WheelImuPrediction::create(const ConfigNode& node,
                                                         SlotInitializationContext& context,
                                                         std::string& err) {
    auto prediction = std::make_unique<WheelImuPrediction>();

    prediction->motion_ref_ = ArtifactId{node.child("Motion").attr("artifact_id")};
    if (prediction->motion_ref_.empty()) {
        err = node.path() + ": needs <Motion artifact_id=.../>";
        return nullptr;
    }
    const std::type_index motion_type(typeid(PlanarMotionDelta));
    if (!context.requireArtifact(prediction->motion_ref_, &motion_type, node.path(), err)) {
        return nullptr;
    }

    const ConfigNode orientation = node.child("Orientation");
    if (orientation.valid()) {
        prediction->orientation_ref_ = ArtifactId{orientation.attr("artifact_id")};
        if (prediction->orientation_ref_.empty()) {
            err = orientation.path() + ": Orientation needs artifact_id";
            return nullptr;
        }
        const std::type_index imu_type(typeid(ImuDelta));
        if (!context.requireArtifact(prediction->orientation_ref_, &imu_type, node.path(),
                                     err)) {
            return nullptr;
        }
    }
    return prediction;
}

void WheelImuPrediction::reset() {
    last_applied_init_ = 0;
    clock_.reset();
}

LocalizationOutput WheelImuPrediction::run(const LocalizationInput& in) {
    LocalizationOutput out;
    out.robot     = in.previous;
    RobotState& r = out.robot;

    // A commanded pose init re-anchors the odometry frame in the field
    // frame. The continuous odometry pose is untouched, so anything latched
    // in the odometry frame keeps its physical meaning.
    if (in.command.init_sequence != 0 && in.command.init_sequence != last_applied_init_) {
        last_applied_init_ = in.command.init_sequence;
        r.field_from_odom  = compose(in.command.init_pose, inverse(r.odom_pose));
        r.valid            = true;
        r.initialized      = true;
    }

    const auto motion_it = in.artifacts.find(motion_ref_);
    if (motion_it == in.artifacts.end()) {
        out.status = FunctionStatus::kNoData;   // pose holds
        return out;
    }
    const PlanarMotionDelta* motion =
        motion_it->second.payload.get<PlanarMotionDelta>();
    if (motion == nullptr) {
        out.status = FunctionStatus::kFault;
        return out;
    }

    double        dx     = motion->dx_m;
    double        dy     = motion->dy_m;
    double        dtheta = motion->dtheta_rad;
    double        dt     = motion->dt_s;
    MonotonicTime stamp  = motion_it->second.measuredAt;

    if (!orientation_ref_.empty()) {
        const auto orientation_it = in.artifacts.find(orientation_ref_);
        if (orientation_it != in.artifacts.end()) {
            const ImuDelta* imu = orientation_it->second.payload.get<ImuDelta>();
            if (imu == nullptr) {
                out.status = FunctionStatus::kFault;
                return out;
            }
            dtheta = imu->delta_rad;
            dt     = std::max(dt, imu->dt_s);
            if (orientation_it->second.measuredAt > stamp) {
                stamp = orientation_it->second.measuredAt;
            }
        }
    }

    // Device time running backwards means the source rebooted: the odometry
    // frame is discontinuous. Bump the epoch so odometry-anchored latches
    // are invalidated, and do not integrate the garbage step.
    if (r.measuredAt.domain == ClockDomain::kDevice &&
        stamp.domain == ClockDomain::kDevice && stamp < r.measuredAt) {
        r.odometry_epoch += 1;
        r.vx_m_s         = 0.0;
        r.vy_m_s         = 0.0;
        r.yaw_rate_rad_s = 0.0;
        r.measuredAt     = stamp;
        clock_.reset();   // the device clock restarted; old offsets are void
        out.status       = FunctionStatus::kFault;
        return out;
    }

    // Pair the device stamp with the actual host receipt of the newest
    // consumed sample; pairing with the pipeline loop time would count
    // preprocessing and cycle delay as clock offset.
    clock_.observe(stamp, motion_it->second.receivedAt.isSet()
                              ? motion_it->second.receivedAt
                              : in.now);

    // chord of the constant-curvature arc across this step
    double lx = dx;
    double ly = dy;
    if (std::fabs(dtheta) > 1e-9) {
        const double s = std::sin(dtheta) / dtheta;
        const double c = (1.0 - std::cos(dtheta)) / dtheta;
        lx             = dx * s - dy * c;
        ly             = dx * c + dy * s;
    }

    const double h  = r.odom_pose.heading_rad;
    const double gx = lx * std::cos(h) - ly * std::sin(h);
    const double gy = lx * std::sin(h) + ly * std::cos(h);

    r.odom_pose.x_m += gx;
    r.odom_pose.y_m += gy;
    r.odom_pose.heading_rad = wrapAngle(h + dtheta);

    if (dt > 1e-6) {
        r.vx_m_s         = gx / dt;
        r.vy_m_s         = gy / dt;
        r.yaw_rate_rad_s = dtheta / dt;
    }

    r.valid      = true;
    r.confidence = 1.0;
    r.measuredAt = stamp;
    if (clock_.valid()) {
        r.measuredAtHost = clock_.toHost(stamp);
    }
    return out;
}

} // namespace navigatr
