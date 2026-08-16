// tracking_wheel_odometry.cpp

#include "impl/preprocessing/tracking_wheel_odometry.h"

#include <cmath>

#include "math/angles.h"

namespace navigatr
{

namespace
{

// dtheta known: least squares for d over the wheels.
bool solvePlanar(const std::vector<double>& ux, const std::vector<double>& uy,
                 const std::vector<double>& m, double& dx, double& dy) {
    double a00 = 0.0, a01 = 0.0, a11 = 0.0, b0 = 0.0, b1 = 0.0;
    for (std::size_t i = 0; i < m.size(); ++i) {
        a00 += ux[i] * ux[i];
        a01 += ux[i] * uy[i];
        a11 += uy[i] * uy[i];
        b0 += ux[i] * m[i];
        b1 += uy[i] * m[i];
    }
    const double det = a00 * a11 - a01 * a01;
    if (std::fabs(det) < 1e-6) {
        return false;
    }
    dx = (a11 * b0 - a01 * b1) / det;
    dy = (a00 * b1 - a01 * b0) / det;
    return true;
}

// dtheta unknown: least squares for (d, dtheta) together.
bool solveFull(const std::vector<double>& ux, const std::vector<double>& uy,
               const std::vector<double>& k, const std::vector<double>& m, double& dx,
               double& dy, double& dtheta) {
    double M[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double v[3]    = {0, 0, 0};
    for (std::size_t i = 0; i < m.size(); ++i) {
        const double r[3] = {ux[i], uy[i], k[i]};
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) {
                M[a][b] += r[a] * r[b];
            }
            v[a] += r[a] * m[i];
        }
    }
    const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                       M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                       M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (std::fabs(det) < 1e-9) {
        return false;
    }
    const auto solveCol = [&](int col) {
        double A[3][3];
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b) {
                A[a][b] = (b == col) ? v[a] : M[a][b];
            }
        }
        return (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) /
               det;
    };
    dx     = solveCol(0);
    dy     = solveCol(1);
    dtheta = solveCol(2);
    return true;
}

} // namespace

std::unique_ptr<PreprocessorExecutable> TrackingWheelOdometry::create(
    const ConfigNode& node, PreprocessorInitializationContext& context, std::string& err) {
    if (context.sensors == nullptr) {
        err = "tracking_wheel_odometry needs the sensor catalog";
        return nullptr;
    }
    auto odom = std::make_unique<TrackingWheelOdometry>();
    odom->id_ = PreprocessorId{node.attr("id")};

    const std::string who = node.path();

    bool ok = true;
    node.forEach("TrackingWheel", [&](const ConfigNode& w) {
        if (!ok) {
            return;
        }
        Wheel          wheel;
        const SensorId sensor_id{w.attr("sensor_id")};
        if (sensor_id.empty()) {
            err = w.path() + ": TrackingWheel needs sensor_id";
            ok  = false;
            return;
        }
        for (const Wheel& seen : odom->wheels_) {
            if (seen.binding.id == sensor_id) {
                err = w.path() + ": sensor " + sensor_id.value +
                      " is referenced by more than one TrackingWheel";
                ok = false;
                return;
            }
        }
        if (!context.sensors->bind<EncoderSample>(sensor_id, w.path(), wheel.binding,
                                                  err)) {
            ok = false;
            return;
        }
        wheel.label = w.attr("label");

        // Calibration-critical geometry: every value measured, none
        // defaulted. An unmeasured wheel must fail the build, not run as
        // zero.
        double x = 0.0, y = 0.0, angle_deg = 0.0;
        if (!w.requireDouble("radius_m", wheel.radius_m, err) ||
            !w.requireDouble("position_x_m", x, err) ||
            !w.requireDouble("position_y_m", y, err) ||
            !w.requireDouble("measurement_angle_deg", angle_deg, err)) {
            ok = false;
            return;
        }
        if (wheel.radius_m <= 0.0) {
            err = w.path() + ": radius_m must be positive";
            ok  = false;
            return;
        }
        std::string direction;
        if (!w.requireAttr("direction", direction, err)) {
            ok = false;
            return;
        }
        if (direction == "positive") {
            wheel.sign = 1.0;
        } else if (direction == "negative") {
            wheel.sign = -1.0;
        } else {
            err = w.path() + ": direction must be positive or negative";
            ok  = false;
            return;
        }
        const double angle = degToRad(angle_deg);
        wheel.ux           = std::cos(angle);
        wheel.uy           = std::sin(angle);
        wheel.k_m          = x * wheel.uy - y * wheel.ux;
        odom->wheels_.push_back(wheel);
    });
    if (!ok) {
        return nullptr;
    }
    if (odom->wheels_.empty()) {
        err = who + ": tracking_wheel_odometry needs at least one TrackingWheel";
        return nullptr;
    }

    const ConfigNode constraint = node.child("HeadingConstraint");
    if (constraint.valid()) {
        odom->heading_.configured = true;
        const SensorId imu_id{constraint.attr("sensor_id")};
        if (imu_id.empty()) {
            err = constraint.path() + ": HeadingConstraint needs sensor_id";
            return nullptr;
        }
        if (!context.sensors->bind<ImuSample>(imu_id, constraint.path(),
                                              odom->heading_.binding, err)) {
            return nullptr;
        }
        if (!constraint.getInt("bias_samples", 200, odom->heading_.bias_samples, err) ||
            !constraint.getInt("max_gap_ms", 250, odom->heading_.max_gap_ms, err) ||
            !constraint.getDouble("max_calibration_travel_m", 0.005,
                                  odom->heading_.max_calibration_travel_m, err)) {
            return nullptr;
        }
        if (odom->heading_.bias_samples < 0 || odom->heading_.max_gap_ms <= 0 ||
            odom->heading_.max_calibration_travel_m <= 0.0) {
            err = constraint.path() + ": bias_samples cannot be negative; max_gap_ms "
                  "and max_calibration_travel_m must be positive";
            return nullptr;
        }
        odom->heading_.calibrated = odom->heading_.bias_samples == 0;
    }

    // Observability from the configured geometry, checked before runtime.
    {
        std::vector<double> ux, uy, k, m;
        for (const Wheel& w : odom->wheels_) {
            ux.push_back(w.ux);
            uy.push_back(w.uy);
            k.push_back(w.k_m);
            m.push_back(0.0);
        }
        double dx = 0.0, dy = 0.0, dtheta = 0.0;
        if (odom->heading_.configured) {
            if (!solvePlanar(ux, uy, m, dx, dy)) {
                err = who + ": wheel geometry cannot observe planar translation even "
                      "with a heading constraint; wheels are collinear in direction";
                return nullptr;
            }
        } else {
            if (!solveFull(ux, uy, k, m, dx, dy, dtheta)) {
                err = who + ": wheel geometry cannot observe planar motion without a "
                      "HeadingConstraint; add one or add a suitably placed wheel";
                return nullptr;
            }
        }
    }

    const ConfigNode output = node.child("Output");
    odom->output_           = ArtifactId{output.attr("artifact_id")};
    if (odom->output_.empty()) {
        err = who + ": needs <Output artifact_id=.../>";
        return nullptr;
    }
    if (output.next("Output").valid()) {
        err = who + ": tracking_wheel_odometry produces exactly one Output";
        return nullptr;
    }
    return odom;
}

std::vector<ArtifactOutputDecl> TrackingWheelOdometry::outputs() const {
    return {ArtifactOutputDecl{
        output_,
        PayloadDescriptor::of<PlanarMotionDelta>(payload_names::kPlanarMotionDelta)}};
}

void TrackingWheelOdometry::reset() {
    for (Wheel& w : wheels_) {
        w.have_prev        = false;
        w.last_sequence    = 0;
        w.pending          = false;
        w.pending_travel_m = 0.0;
        w.pending_dt_s     = 0.0;
    }
    heading_.calibrated     = heading_.bias_samples == 0;
    heading_.cal_count      = 0;
    heading_.cal_sum        = 0.0;
    heading_.cal_have_accum = false;
    heading_.cal_travel_m   = 0.0;
    heading_.bias_rad_s     = 0.0;
    heading_.have_prev      = false;
    heading_.prev_has_accum = false;
    heading_.last_sequence  = 0;
    heading_.pending        = false;
    heading_.pending_dtheta = 0.0;
    heading_.pending_dt_s   = 0.0;
    last_received_          = MonotonicTime{};
}

FunctionStatus TrackingWheelOdometry::run(const PreprocessingInput& in, ArtifactMap& out) {
    bool progressed = false;

    // While gyro bias collection runs, wheel baselines rebase continuously
    // instead of accumulating: the first fused solve must never combine
    // travel from before calibration with a short gyro interval.
    const bool calibrating = heading_.configured && !heading_.calibrated;

    for (Wheel& w : wheels_) {
        // only healthy sources are consumed; fault and unavailable sensors
        // hold their history without feeding the solve
        const StoredSensorSample* stored = w.binding.freshStored(in.sensorResults);
        if (stored == nullptr || stored->sequence == w.last_sequence) {
            continue;
        }
        const EncoderSample* sample = stored->payload.get<EncoderSample>();
        if (sample == nullptr) {
            return FunctionStatus::kFault;   // wiring bug, payload changed underneath
        }
        w.last_sequence = stored->sequence;
        progressed      = true;
        if (!last_received_.isSet() || stored->receivedAt > last_received_) {
            last_received_ = stored->receivedAt;
        }

        if (!w.have_prev) {
            w.have_prev      = true;
            w.prev_angle_rad = sample->angle_rad;
            w.prev_stamp     = stored->measuredAt;
            continue;
        }
        if (calibrating) {
            heading_.cal_travel_m +=
                std::fabs((sample->angle_rad - w.prev_angle_rad) * w.radius_m);
            w.prev_angle_rad = sample->angle_rad;
            w.prev_stamp     = stored->measuredAt;
            continue;   // rebase only; travel during calibration is not motion data
        }
        w.pending_travel_m +=
            (sample->angle_rad - w.prev_angle_rad) * w.radius_m * w.sign;
        w.pending_dt_s += secondsBetween(stored->measuredAt, w.prev_stamp);
        w.pending       = true;
        w.pending_stamp = stored->measuredAt;
        w.prev_angle_rad = sample->angle_rad;
        w.prev_stamp     = stored->measuredAt;
    }

    if (heading_.configured) {
        const StoredSensorSample* stored =
            heading_.binding.freshStored(in.sensorResults);
        if (stored != nullptr && stored->sequence != heading_.last_sequence) {
            const ImuSample* sample = stored->payload.get<ImuSample>();
            if (sample == nullptr) {
                return FunctionStatus::kFault;
            }
            heading_.last_sequence = stored->sequence;
            progressed             = true;
            if (!last_received_.isSet() || stored->receivedAt > last_received_) {
                last_received_ = stored->receivedAt;
            }

            if (!heading_.calibrated) {
                // Bias collection is valid only while stationary; motion
                // restarts it.
                if (heading_.cal_travel_m > heading_.max_calibration_travel_m) {
                    heading_.cal_count      = 0;
                    heading_.cal_sum        = 0.0;
                    heading_.cal_have_accum = false;
                    heading_.cal_travel_m   = 0.0;
                }
                if (sample->has_accumulated && !heading_.cal_have_accum) {
                    heading_.cal_have_accum       = true;
                    heading_.cal_accum_start      = sample->accumulated_angle_rad;
                    heading_.cal_accum_start_stamp = stored->measuredAt;
                }
                heading_.cal_sum += sample->yaw_rate_rad_s;
                ++heading_.cal_count;
                if (heading_.cal_count >= heading_.bias_samples) {
                    const double elapsed =
                        heading_.cal_have_accum
                            ? secondsBetween(stored->measuredAt,
                                             heading_.cal_accum_start_stamp)
                            : 0.0;
                    if (heading_.cal_have_accum && elapsed > 1e-6) {
                        // total accumulated angle over the stationary window
                        // beats a mean of sampled rates
                        heading_.bias_rad_s = (sample->accumulated_angle_rad -
                                               heading_.cal_accum_start) /
                                              elapsed;
                    } else {
                        heading_.bias_rad_s = heading_.cal_sum / heading_.cal_count;
                    }
                    heading_.calibrated   = true;
                    heading_.cal_travel_m = 0.0;
                }
            } else {
                const double rate = sample->yaw_rate_rad_s - heading_.bias_rad_s;
                if (heading_.have_prev) {
                    const double dt = secondsBetween(stored->measuredAt,
                                                     heading_.prev_stamp);
                    if (dt * 1000.0 > static_cast<double>(heading_.max_gap_ms)) {
                        // rate integration across an outage is garbage;
                        // reseed and drop the interval
                        heading_.have_prev = false;
                    } else {
                        // the accumulated angle keeps rotation that packet
                        // batching would drop from a latest-rate sample; a
                        // difference across accumulator epochs spans a
                        // producer-side discontinuity and falls back to the
                        // endpoint trapezoid
                        double dtheta;
                        if (sample->has_accumulated && heading_.prev_has_accum &&
                            sample->accumulated_epoch == heading_.prev_accum_epoch) {
                            dtheta = (sample->accumulated_angle_rad -
                                      heading_.prev_accum) -
                                     heading_.bias_rad_s * dt;
                        } else {
                            dtheta = 0.5 * (heading_.prev_rate + rate) * dt;
                        }
                        heading_.pending_dtheta += dtheta;
                        heading_.pending_dt_s += dt;
                        heading_.pending = true;
                    }
                }
                heading_.have_prev        = true;
                heading_.prev_rate        = rate;
                heading_.prev_accum       = sample->accumulated_angle_rad;
                heading_.prev_has_accum   = sample->has_accumulated;
                heading_.prev_accum_epoch = sample->accumulated_epoch;
                heading_.prev_stamp       = stored->measuredAt;
            }
        }
    }

    // Solve only when every configured input has a pending contribution, so
    // nothing is dropped when sources tick on different cycles.
    bool ready = true;
    for (const Wheel& w : wheels_) {
        ready = ready && w.pending;
    }
    if (heading_.configured) {
        ready = ready && heading_.pending;
    }
    if (!ready) {
        return progressed ? FunctionStatus::kOk : FunctionStatus::kNoData;
    }

    std::vector<double> ux, uy, k, m;
    double              dt_s = 0.0;
    MonotonicTime       newest;
    for (Wheel& w : wheels_) {
        ux.push_back(w.ux);
        uy.push_back(w.uy);
        k.push_back(w.k_m);
        m.push_back(w.pending_travel_m);
        dt_s = std::max(dt_s, w.pending_dt_s);
        if (!newest.isSet() || w.pending_stamp > newest) {
            newest = w.pending_stamp;
        }
    }

    PlanarMotionDelta delta;
    bool              solved = false;
    if (heading_.configured) {
        delta.dtheta_rad = heading_.pending_dtheta;
        dt_s             = std::max(dt_s, heading_.pending_dt_s);
        for (std::size_t i = 0; i < m.size(); ++i) {
            m[i] -= k[i] * delta.dtheta_rad;
        }
        solved = solvePlanar(ux, uy, m, delta.dx_m, delta.dy_m);
    } else {
        solved = solveFull(ux, uy, k, m, delta.dx_m, delta.dy_m, delta.dtheta_rad);
    }
    delta.dt_s = dt_s;

    for (Wheel& w : wheels_) {
        w.pending          = false;
        w.pending_travel_m = 0.0;
        w.pending_dt_s     = 0.0;
    }
    heading_.pending        = false;
    heading_.pending_dtheta = 0.0;
    heading_.pending_dt_s   = 0.0;

    if (!solved) {
        return FunctionStatus::kNoData;   // this step's data could not resolve
    }

    ArtifactRecord record;
    record.measuredAt = newest;
    record.receivedAt = last_received_;
    record.payload =
        TypedPayload::store(delta, payload_names::kPlanarMotionDelta);
    out[output_] = std::move(record);
    return FunctionStatus::kOk;
}

} // namespace navigatr
