// weighted_planar_fusion.cpp

#include "impl/localization/weighted_planar_fusion.h"

#include <cmath>
#include <cstdio>
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

    if (!configureHeading(node, context, estimator->heading_ref_, estimator->max_wait_ms_,
                          err)) {
        return nullptr;
    }
    if (!estimator->heading_ref_.empty()) {
        const ConfigNode heading = node.child("Heading");
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

    if (!estimator->state_.attitude.configure(node, context, err)) {
        return nullptr;
    }
    return estimator;
}

StateEstimatorOutput WeightedPlanarFusion::run(const StateEstimatorInput& in) {
    StateEstimatorOutput out;
    out.robot = in.previous;

    PreparedStep step;
    if (!prepareStep(state_, in, motion_ref_, heading_ref_, max_wait_ms_, out, step)) {
        return out;
    }
    const BodyMotionIncrement& motion   = *step.motion;
    const bool                 have_rotation = motion.has_rotation;
    const double               dtheta_w = motion.dtheta_rad;
    double                     dx       = motion.dx_m;
    double                     dy       = motion.dy_m;
    double                     dtheta   = dtheta_w;

    // wheel side variances
    const double dist  = std::hypot(dx, dy);
    const double var_d = square(motion_noise_.translation_floor_m) +
                         square(motion_noise_.translation_per_m * dist);
    const double var_w = square(motion_noise_.rotation_floor_rad) +
                         square(motion_noise_.rotation_per_rad * dtheta_w) +
                         square(motion_noise_.rotation_per_m * dist);

    // gyro side, over the heading support that equals the motion window
    double var_g = 0.0;
    if (step.heading_used) {
        const double dt_g = step.heading_dt_s > 0.0 ? step.heading_dt_s : motion.dt_s;
        var_g = square(heading_noise_.angle_random_walk_rad_per_sqrt_s) * dt_g +
                square(heading_noise_.bias_rad_per_s * dt_g);
    }

    // Body increment z = (d, dtheta) with covariance Qz. J carries the
    // producer's translation to rotation coupling; jj scales J J' in the
    // translation block and jc scales J in the cross term.
    const double Jx = motion.has_rotation_coupling ? motion.dx_per_dtheta_m_rad : 0.0;
    const double Jy = motion.has_rotation_coupling ? motion.dy_per_dtheta_m_rad : 0.0;
    double       var_theta = var_w;
    double       jj        = 0.0;
    double       jc        = 0.0;
    double       weight    = 0.0;
    if (have_rotation && step.heading_used) {
        weight    = var_w / (var_w + var_g);
        dtheta    = dtheta_w + weight * (step.heading_dtheta_rad - dtheta_w);
        var_theta = var_w * var_g / (var_w + var_g);
        dx += Jx * (dtheta - dtheta_w);
        dy += Jy * (dtheta - dtheta_w);
        jj = weight * weight * (var_w + var_g);
    } else if (!have_rotation) {
        // published translation is the solve at zero rotation
        dtheta    = step.heading_dtheta_rad;
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
    RobotState&  r  = out.robot;
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

    r.vx_m_s         = gx / motion.dt_s;
    r.vy_m_s         = gy / motion.dt_s;
    r.yaw_rate_rad_s = dtheta / motion.dt_s;

    if (step.heading_used) {
        if (have_rotation) {
            char text[96];
            std::snprintf(text, sizeof(text), "fused heading, gyro weight %.3f", weight);
            out.diagnostic = text;
        } else {
            out.diagnostic = "rotation from heading only";
        }
    }
    finishStep(state_, in, motion_ref_, heading_ref_, in.observations.at(motion_ref_), step,
               out);
    return out;
}

} // namespace navigatr
