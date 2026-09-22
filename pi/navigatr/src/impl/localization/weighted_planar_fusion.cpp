// weighted_planar_fusion.cpp

#include "impl/localization/weighted_planar_fusion.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <typeindex>

#include "math/angles.h"

namespace navigatr
{

namespace
{

struct Mat3 {
    double m[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    static Mat3 identity() {
        Mat3 I;
        I.m[0][0] = I.m[1][1] = I.m[2][2] = 1.0;
        return I;
    }
};

Mat3 multiply(const Mat3& a, const Mat3& b) {
    Mat3 out;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) {
                s += a.m[i][k] * b.m[k][j];
            }
            out.m[i][j] = s;
        }
    }
    return out;
}

Mat3 transpose(const Mat3& a) {
    Mat3 out;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out.m[i][j] = a.m[j][i];
        }
    }
    return out;
}

// A B A'
Mat3 sandwich(const Mat3& a, const Mat3& b) { return multiply(multiply(a, b), transpose(a)); }

Mat3 fromCovariance(const PoseCovariance& c) {
    Mat3 out;
    out.m[0][0] = c.xx;
    out.m[0][1] = out.m[1][0] = c.xy;
    out.m[0][2] = out.m[2][0] = c.xh;
    out.m[1][1]               = c.yy;
    out.m[1][2] = out.m[2][1] = c.yh;
    out.m[2][2]               = c.hh;
    return out;
}

// symmetrized on the way out; rounding never leaves it lopsided
PoseCovariance toCovariance(const Mat3& a) {
    PoseCovariance c;
    c.xx = a.m[0][0];
    c.yy = a.m[1][1];
    c.hh = a.m[2][2];
    c.xy = 0.5 * (a.m[0][1] + a.m[1][0]);
    c.xh = 0.5 * (a.m[0][2] + a.m[2][0]);
    c.yh = 0.5 * (a.m[1][2] + a.m[2][1]);
    return c;
}

double square(double v) { return v * v; }

bool parseNoise(const ConfigNode& parent, std::initializer_list<const char*> names,
                double* values[], std::string& err) {
    const ConfigNode noise = parent.child("Noise");
    if (!noise.valid()) {
        err = parent.path() + ": needs <Noise .../> with explicit one-sigma values";
        return false;
    }
    if (noise.next("Noise").valid()) {
        err = parent.path() + " has more than one Noise";
        return false;
    }
    std::size_t i = 0;
    for (const char* name : names) {
        if (!noise.requireDouble(name, *values[i], err)) {
            return false;
        }
        if (*values[i] < 0.0) {
            err = noise.path() + ": " + name + " cannot be negative";
            return false;
        }
        ++i;
    }
    return true;
}

bool onlyChildren(const ConfigNode& parent, std::initializer_list<const char*> allowed,
                  std::string& err) {
    for (ConfigNode c = parent.child(); c.valid(); c = c.next()) {
        bool known = false;
        for (const char* name : allowed) {
            if (std::string(c.name()) == name) {
                known = true;
            }
        }
        if (!known) {
            err = parent.path() + " has unknown element " + c.name();
            return false;
        }
    }
    return true;
}

} // namespace

std::unique_ptr<StateEstimator>
WeightedPlanarFusion::create(const ConfigNode& node, StateEstimatorInitializationContext& context,
                             std::string& err) {
    auto estimator = std::make_unique<WeightedPlanarFusion>();
    if (!onlyChildren(node, {"Motion", "Heading", "Attitude"}, err)) {
        return nullptr;
    }

    const ConfigNode motion = node.child("Motion");
    estimator->motion_ref_  = ObservationId{motion.attr("observation_id")};
    if (estimator->motion_ref_.empty()) {
        err = node.path() + ": needs <Motion observation_id=.../>";
        return nullptr;
    }
    if (motion.next("Motion").valid()) {
        err = node.path() + ": exactly one Motion is supported";
        return nullptr;
    }
    const std::type_index motion_type(typeid(BodyMotionIncrement));
    if (!context.requireObservation(estimator->motion_ref_, &motion_type, motion.path(), err)) {
        return nullptr;
    }
    if (!onlyChildren(motion, {"Noise"}, err)) {
        return nullptr;
    }
    {
        MotionNoise& n        = estimator->motion_noise_;
        double*      values[] = {&n.translation_floor_m, &n.translation_per_m,
                                 &n.rotation_floor_rad, &n.rotation_per_rad, &n.rotation_per_m};
        if (!parseNoise(motion,
                        {"translation_floor_m", "translation_per_m", "rotation_floor_rad",
                         "rotation_per_rad", "rotation_per_m"},
                        values, err)) {
            return nullptr;
        }
        if (n.translation_floor_m <= 0.0 || n.rotation_floor_rad <= 0.0) {
            err = motion.path() + ": translation_floor_m and rotation_floor_rad must be "
                  "positive; a measurement with zero variance is not a measurement";
            return nullptr;
        }
    }

    const ConfigNode heading = node.child("Heading");
    if (heading.valid()) {
        estimator->heading_ref_ = ObservationId{heading.attr("observation_id")};
        if (estimator->heading_ref_.empty()) {
            err = heading.path() + ": Heading needs observation_id";
            return nullptr;
        }
        if (heading.next("Heading").valid()) {
            err = node.path() + ": at most one Heading is supported";
            return nullptr;
        }
        const std::type_index heading_type(typeid(HeadingIncrement));
        if (!context.requireObservation(estimator->heading_ref_, &heading_type, heading.path(),
                                        err)) {
            return nullptr;
        }
        if (!heading.getInt("interval_tolerance_ms", 20, estimator->heading_tolerance_ms_,
                            err)) {
            return nullptr;
        }
        if (estimator->heading_tolerance_ms_ < 0) {
            err = heading.path() + ": interval_tolerance_ms cannot be negative";
            return nullptr;
        }
        if (!onlyChildren(heading, {"Noise"}, err)) {
            return nullptr;
        }
        HeadingNoise& n        = estimator->heading_noise_;
        double*       values[] = {&n.angle_random_walk_rad_per_sqrt_s, &n.bias_rad_per_s};
        if (!parseNoise(heading, {"angle_random_walk_rad_per_sqrt_s", "bias_rad_per_s"},
                        values, err)) {
            return nullptr;
        }
        if (n.angle_random_walk_rad_per_sqrt_s <= 0.0) {
            err = heading.path() + ": angle_random_walk_rad_per_sqrt_s must be positive";
            return nullptr;
        }
    }

    if (!estimator->attitude_.configure(node, context, err)) {
        return nullptr;
    }
    return estimator;
}

void WeightedPlanarFusion::reset() {
    placement_.reset();
    attitude_.reset();
    clock_.reset();
    motion_clock_.clear();
}

StateEstimatorOutput WeightedPlanarFusion::run(const StateEstimatorInput& in) {
    StateEstimatorOutput out;
    out.robot     = in.previous;
    RobotState& r = out.robot;
    for (const auto& observation : in.observations) {
        if (observation.first != motion_ref_ && observation.first != heading_ref_ &&
            observation.first != attitude_.ref()) {
            out.rejected.push_back(observation.first);
            out.diagnostic = "observation not consumed by this estimator";
        }
    }

    // re-anchor only; the odometry pose and its covariance are untouched
    placement_.apply(r, in.requests.placement);

    const auto applyAttitude = [&](bool clock_valid) {
        attitude_.apply(out, in, clock_, motion_clock_, clock_valid);
    };

    const auto motion_it = in.observations.find(motion_ref_);
    if (motion_it == in.observations.end()) {
        out.status       = FunctionStatus::kNoData;   // hold
        out.clock_mapped = clock_.valid();
        applyAttitude(clock_.valid());
        return out;
    }
    const BodyMotionIncrement* motion = motion_it->second.payload.get<BodyMotionIncrement>();
    if (motion == nullptr) {
        out.rejected.push_back(motion_ref_);
        out.status = FunctionStatus::kFault;
        applyAttitude(clock_.valid());
        return out;
    }

    double              dx            = motion->dx_m;
    double              dy            = motion->dy_m;
    const double        dtheta_w      = motion->dtheta_rad;
    double              dtheta        = dtheta_w;
    bool                have_rotation = motion->has_rotation;
    const double        dt            = motion->dt_s;
    const MonotonicTime stamp =
        motion->endAt.isSet() ? motion->endAt : motion_it->second.measuredAt;

    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dtheta_w) ||
        !std::isfinite(dt) || dt <= 0.0 || !stamp.isSet() ||
        (motion->has_rotation_coupling && (!std::isfinite(motion->dx_per_dtheta_m_rad) ||
                                           !std::isfinite(motion->dy_per_dtheta_m_rad)))) {
        out.rejected.push_back(motion_ref_);
        out.status     = FunctionStatus::kFault;
        out.diagnostic = "motion increment without a positive interval";
        applyAttitude(clock_.valid());
        return out;
    }

    // one source clock per motion; the attitude fold keys off it
    std::string source_clock;
    for (const auto& source : motion->sources) {
        if (stamp.domain != ClockDomain::kDevice) break;
        if (source.clock.empty()) continue;
        if (!source_clock.empty() && source_clock != source.clock) {
            out.rejected.push_back(motion_ref_);
            out.status     = FunctionStatus::kFault;
            out.diagnostic = "motion observation combines different source clocks";
            applyAttitude(clock_.valid());
            return out;
        }
        source_clock = source.clock;
    }
    if (stamp.domain == ClockDomain::kDevice && source_clock != motion_clock_) {
        clock_.reset();
        attitude_.forget();
        motion_clock_ = source_clock;
    }

    // wheel side variances
    const double dist  = std::hypot(dx, dy);
    const double var_d = square(motion_noise_.translation_floor_m) +
                         square(motion_noise_.translation_per_m * dist);
    const double var_w = square(motion_noise_.rotation_floor_rad) +
                         square(motion_noise_.rotation_per_rad * dtheta_w) +
                         square(motion_noise_.rotation_per_m * dist);

    // gyro side: only an independent, aligned heading takes part
    bool   heading_used = false;
    double dtheta_g     = 0.0;
    double var_g        = 0.0;
    if (!heading_ref_.empty()) {
        const auto heading_it = in.observations.find(heading_ref_);
        if (heading_it != in.observations.end()) {
            const HeadingIncrement* heading = heading_it->second.payload.get<HeadingIncrement>();
            if (heading == nullptr || !std::isfinite(heading->dtheta_rad) ||
                !std::isfinite(heading->dt_s)) {
                out.rejected.push_back(heading_ref_);
                out.status = FunctionStatus::kFault;
                applyAttitude(clock_.valid());
                return out;
            }
            bool independent = true;
            for (const Provenance& hs : heading->sources) {
                for (const Provenance& ms : motion->sources) {
                    if (hs.source == ms.source) {
                        independent = false;
                    }
                }
            }
            const bool aligned =
                heading->startAt.isSet() && motion->startAt.isSet() &&
                sameDomain(heading->startAt, motion->startAt) &&
                sameDomain(heading->endAt, motion->endAt) &&
                std::llabs(heading->startAt - motion->startAt) <= heading_tolerance_ms_ &&
                std::llabs(heading->endAt - motion->endAt) <= heading_tolerance_ms_;
            if (!independent) {
                out.rejected.push_back(heading_ref_);
                out.diagnostic =
                    "heading shares a source with the motion observation; not counted twice";
            } else if (!aligned) {
                out.diagnostic = "heading interval does not match the motion interval; ignored";
                if (!sameDomain(heading->endAt, motion->endAt) ||
                    heading->endAt.ms <= motion->endAt.ms) {
                    out.rejected.push_back(heading_ref_);
                } else if (!have_rotation) {
                    out.rejected.push_back(motion_ref_);
                }
            } else {
                const double dt_g = heading->dt_s > 0.0 ? heading->dt_s : dt;
                heading_used      = true;
                dtheta_g          = heading->dtheta_rad;
                var_g = square(heading_noise_.angle_random_walk_rad_per_sqrt_s) * dt_g +
                        square(heading_noise_.bias_rad_per_s * dt_g);
            }
        }
    }

    if (!have_rotation && !heading_used) {
        out.status     = FunctionStatus::kNoData;
        out.diagnostic = "motion increment without observed rotation and no aligned heading";
        applyAttitude(clock_.valid());
        return out;
    }

    // source reboot: epoch moves on, nothing integrated
    if (r.measuredAt.isSet() && sameDomain(r.measuredAt, stamp) && stamp < r.measuredAt) {
        out.rejected.push_back(motion_ref_);
        if (heading_used) out.rejected.push_back(heading_ref_);
        r.odometry_epoch += 1;
        r.vx_m_s         = 0.0;
        r.vy_m_s         = 0.0;
        r.yaw_rate_rad_s = 0.0;
        r.measuredAt     = stamp;
        r.measuredAtHost = MonotonicTime{};
        clock_.reset();
        attitude_.forget();
        out.status     = FunctionStatus::kFault;
        out.diagnostic = "source time regression; odometry epoch advanced";
        applyAttitude(false);
        return out;
    }

    if (r.measuredAt.isSet() && sameDomain(r.measuredAt, stamp) &&
        stamp.ms == r.measuredAt.ms) {
        out.rejected.push_back(motion_ref_);
        if (heading_used) out.rejected.push_back(heading_ref_);
        out.status     = FunctionStatus::kNoData;
        out.diagnostic = "motion effective time already consumed";
        applyAttitude(clock_.valid());
        return out;
    }

    if (stamp.domain == ClockDomain::kDevice &&
        motion_it->second.receivedAt.domain == ClockDomain::kHost) {
        clock_.observe(stamp, motion_it->second.receivedAt);
    }

    // Body increment z = (d, dtheta) with covariance Qz. J carries the
    // producer's translation to rotation coupling; jj scales J J' in the
    // translation block and jc scales J in the cross term.
    const double Jx = motion->has_rotation_coupling ? motion->dx_per_dtheta_m_rad : 0.0;
    const double Jy = motion->has_rotation_coupling ? motion->dy_per_dtheta_m_rad : 0.0;
    double       var_theta = var_w;
    double       jj        = 0.0;
    double       jc        = 0.0;
    double       weight    = 0.0;
    if (have_rotation && heading_used) {
        weight    = var_w / (var_w + var_g);
        dtheta    = dtheta_w + weight * (dtheta_g - dtheta_w);
        var_theta = var_w * var_g / (var_w + var_g);
        dx += Jx * (dtheta - dtheta_w);
        dy += Jy * (dtheta - dtheta_w);
        jj = weight * weight * (var_w + var_g);
    } else if (!have_rotation) {
        // published translation is the solve at zero rotation
        dtheta    = dtheta_g;
        var_theta = var_g;
        dx += Jx * dtheta;
        dy += Jy * dtheta;
        jj = var_g;
        jc = var_g;
    }
    Mat3 Qz;
    Qz.m[0][0] = var_d + jj * Jx * Jx;
    Qz.m[1][1] = var_d + jj * Jy * Jy;
    Qz.m[0][1] = Qz.m[1][0] = jj * Jx * Jy;
    Qz.m[0][2] = Qz.m[2][0] = jc * Jx;
    Qz.m[1][2] = Qz.m[2][1] = jc * Jy;
    Qz.m[2][2]              = var_theta;

    // chord l = C(dtheta) d and its Jacobian B = [[C, C'(dtheta) d], [0, 1]]
    double s = 1.0, c = 0.0, ds = 0.0, dc = 0.5;
    if (std::fabs(dtheta) > 1e-4) {
        s  = std::sin(dtheta) / dtheta;
        c  = (1.0 - std::cos(dtheta)) / dtheta;
        ds = (dtheta * std::cos(dtheta) - std::sin(dtheta)) / (dtheta * dtheta);
        dc = (dtheta * std::sin(dtheta) - (1.0 - std::cos(dtheta))) / (dtheta * dtheta);
    } else {
        s  = 1.0 - dtheta * dtheta / 6.0;
        c  = dtheta / 2.0 - dtheta * dtheta * dtheta / 24.0;
        ds = -dtheta / 3.0;
        dc = 0.5 - dtheta * dtheta / 8.0;
    }
    const double lx = dx * s - dy * c;
    const double ly = dx * c + dy * s;
    Mat3         B  = Mat3::identity();
    B.m[0][0]       = s;
    B.m[0][1]       = -c;
    B.m[0][2]       = ds * dx - dc * dy;
    B.m[1][0]       = c;
    B.m[1][1]       = s;
    B.m[1][2]       = dc * dx + ds * dy;
    const Mat3 Qu   = sandwich(B, Qz);

    // pose update and P' = F P F' + G Qu G'
    const double h  = r.odom_pose.heading_rad;
    const double ch = std::cos(h);
    const double sh = std::sin(h);
    const double gx = lx * ch - ly * sh;
    const double gy = lx * sh + ly * ch;

    Mat3 F    = Mat3::identity();
    F.m[0][2] = -gy;
    F.m[1][2] = gx;
    Mat3 G    = Mat3::identity();
    G.m[0][0] = ch;
    G.m[0][1] = -sh;
    G.m[1][0] = sh;
    G.m[1][1] = ch;
    const Mat3 P_prev = r.has_covariance ? fromCovariance(r.odom_covariance) : Mat3{};
    const Mat3 FPF    = sandwich(F, P_prev);
    const Mat3 GQG    = sandwich(G, Qu);
    Mat3       P;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            P.m[i][j] = FPF.m[i][j] + GQG.m[i][j];
        }
    }
    r.odom_covariance = toCovariance(P);
    r.has_covariance  = true;

    r.odom_pose.x_m += gx;
    r.odom_pose.y_m += gy;
    r.odom_pose.heading_rad = wrapAngle(h + dtheta);

    r.vx_m_s         = gx / dt;
    r.vy_m_s         = gy / dt;
    r.yaw_rate_rad_s = dtheta / dt;

    r.valid      = true;
    r.confidence = 1.0;
    r.measuredAt = stamp;
    if (stamp.domain == ClockDomain::kHost) {
        r.measuredAtHost = stamp;
    } else if (clock_.valid()) {
        r.measuredAtHost = clock_.toHost(stamp);
    } else {
        r.measuredAtHost = MonotonicTime{};
    }
    out.advanced = true;
    out.accepted.push_back(motion_ref_);
    if (heading_used) {
        out.accepted.push_back(heading_ref_);
        if (have_rotation) {
            char text[96];
            std::snprintf(text, sizeof(text), "fused heading, gyro weight %.3f", weight);
            out.diagnostic = text;
        } else {
            out.diagnostic = "rotation from heading only";
        }
    }
    out.clock_mapped = stamp.domain == ClockDomain::kHost || clock_.valid();
    applyAttitude(clock_.valid());
    return out;
}

} // namespace navigatr
