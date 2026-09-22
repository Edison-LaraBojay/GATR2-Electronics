// tracking_wheel_motion.cpp

#include "impl/localization/tracking_wheel_motion.h"

#include <algorithm>
#include <cmath>

#include "math/angles.h"
#include "resources/resource_store.h"
#include "resources/wheel_geometry.h"

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

Provenance provenanceOf(const SensorId& id, const StoredSample& stored) {
    Provenance p;
    p.source   = id.value;
    p.clock    = stored.upstream.clock;
    p.sequence = stored.sequence;
    p.epoch    = stored.epoch;
    return p;
}

} // namespace

std::unique_ptr<RobotObservationFunction>
TrackingWheelMotion::create(const ConfigNode& node, RobotObservationInitializationContext& context,
                            std::string& err) {
    if (context.sensors == nullptr) {
        err = "tracking_wheel_motion needs the sensor catalog";
        return nullptr;
    }
    auto model = std::make_unique<TrackingWheelMotion>();
    model->id_ = ObservationFunctionId{node.attr("id")};

    const std::string who = node.path();

    const auto addWheel = [&](const SensorId& sensor_id, const std::string& label,
                              double radius_m, double x, double y, double angle_deg,
                              bool positive, const std::string& where) {
        for (const Wheel& seen : model->wheels_) {
            if (seen.binding.id == sensor_id) {
                err = where + ": sensor " + sensor_id.value +
                      " is referenced by more than one wheel";
                return false;
            }
        }
        Wheel wheel;
        if (!context.sensors->bind<EncoderSample>(sensor_id, where, wheel.binding, err)) {
            return false;
        }
        wheel.label = label;
        if (radius_m <= 0.0) {
            err = where + ": radius_m must be positive";
            return false;
        }
        wheel.radius_m     = radius_m;
        wheel.sign         = positive ? 1.0 : -1.0;
        const double angle = degToRad(angle_deg);
        wheel.ux           = std::cos(angle);
        wheel.uy           = std::sin(angle);
        wheel.k_m          = x * wheel.uy - y * wheel.ux;
        model->wheels_.push_back(wheel);
        return true;
    };

    // Inline form: this model owns the measured geometry.
    bool ok = true;
    node.forEach("TrackingWheel", [&](const ConfigNode& w) {
        if (!ok) {
            return;
        }
        const SensorId sensor_id{w.attr("sensor_id")};
        if (sensor_id.empty()) {
            err = w.path() + ": TrackingWheel needs sensor_id";
            ok  = false;
            return;
        }
        // calibration-critical geometry: every value measured, none defaulted
        double      radius_m = 0.0, x = 0.0, y = 0.0, angle_deg = 0.0;
        std::string direction;
        if (!w.requireDouble("radius_m", radius_m, err) ||
            !w.requireDouble("position_x_m", x, err) ||
            !w.requireDouble("position_y_m", y, err) ||
            !w.requireDouble("measurement_angle_deg", angle_deg, err) ||
            !w.requireAttr("direction", direction, err)) {
            ok = false;
            return;
        }
        if (direction != "positive" && direction != "negative") {
            err = w.path() + ": direction must be positive or negative";
            ok  = false;
            return;
        }
        ok = addWheel(sensor_id, w.attr("label"), radius_m, x, y, angle_deg,
                      direction == "positive", w.path());
    });
    if (!ok) {
        return nullptr;
    }

    // Reference form: geometry lives in a shared wheel_geometry resource.
    const ConfigNode wheels_ref = node.child("Wheels");
    if (wheels_ref.valid()) {
        if (!model->wheels_.empty()) {
            err = wheels_ref.path() + ": use either inline TrackingWheel elements or "
                  "a Wheels reference, not both";
            return nullptr;
        }
        if (context.resources == nullptr) {
            err = wheels_ref.path() + ": no resources available";
            return nullptr;
        }
        const ResourceId geometry_id{wheels_ref.attr("resource_id")};
        if (geometry_id.empty()) {
            err = wheels_ref.path() + ": Wheels needs resource_id";
            return nullptr;
        }
        std::string inner;
        const auto  geometry =
            context.resources->require<const WheelGeometryMap>(geometry_id, inner);
        if (geometry == nullptr) {
            err = wheels_ref.path() + ": " + inner;
            return nullptr;
        }
        bool any = false;
        ok       = true;
        wheels_ref.forEach("Use", [&](const ConfigNode& u) {
            if (!ok) {
                return;
            }
            std::string wheel_id;
            if (!u.requireAttr("wheel_id", wheel_id, err)) {
                ok = false;
                return;
            }
            const WheelDecl* decl = geometry->find(wheel_id);
            if (decl == nullptr) {
                err = u.path() + ": wheel " + wheel_id + " does not exist in resource " +
                      geometry_id.value;
                ok = false;
                return;
            }
            any = true;
            ok  = addWheel(decl->sensor, decl->label.empty() ? decl->id : decl->label,
                           decl->radius_m, decl->position_x_m, decl->position_y_m,
                           decl->measurement_angle_deg, decl->direction_positive,
                           u.path());
        });
        if (!ok) {
            return nullptr;
        }
        if (!any) {
            err = wheels_ref.path() + ": Wheels needs at least one <Use wheel_id=.../>";
            return nullptr;
        }
    }

    if (model->wheels_.empty()) {
        err = who + ": tracking_wheel_motion needs TrackingWheel elements or a Wheels "
              "reference";
        return nullptr;
    }

    const ConfigNode constraint = node.child("HeadingConstraint");
    if (constraint.valid()) {
        model->heading_.configured = true;
        const SensorId imu_id{constraint.attr("sensor_id")};
        if (imu_id.empty()) {
            err = constraint.path() + ": HeadingConstraint needs sensor_id";
            return nullptr;
        }
        if (!context.sensors->bind<ImuSample>(imu_id, constraint.path(),
                                              model->heading_.binding, err)) {
            return nullptr;
        }
        if (!constraint.getInt("bias_samples", 200, model->heading_.bias_samples, err) ||
            !constraint.getInt("max_gap_ms", 250, model->heading_.max_gap_ms, err) ||
            !constraint.getDouble("max_calibration_travel_m", 0.005,
                                  model->heading_.max_calibration_travel_m, err)) {
            return nullptr;
        }
        if (model->heading_.bias_samples < 0 || model->heading_.max_gap_ms <= 0 ||
            model->heading_.max_calibration_travel_m <= 0.0) {
            err = constraint.path() + ": bias_samples cannot be negative; max_gap_ms "
                  "and max_calibration_travel_m must be positive";
            return nullptr;
        }
        model->heading_.calibrated = model->heading_.bias_samples == 0;
    }

    const ConfigNode timing = node.child("Timing");
    if (!timing.getInt("interval_tolerance_ms", 20, model->interval_tolerance_ms_, err) ||
        !timing.getInt("max_pending_ms", 500, model->max_pending_ms_, err)) {
        return nullptr;
    }
    if (model->interval_tolerance_ms_ < 0 || model->max_pending_ms_ <= 0) {
        err = who + ": interval_tolerance_ms cannot be negative and max_pending_ms must "
              "be positive";
        return nullptr;
    }

    // Observability from the configured geometry, checked before runtime.
    {
        std::vector<double> ux, uy, k, m;
        for (const Wheel& w : model->wheels_) {
            ux.push_back(w.ux);
            uy.push_back(w.uy);
            k.push_back(w.k_m);
            m.push_back(0.0);
        }
        double dx = 0.0, dy = 0.0, dtheta = 0.0;
        if (model->heading_.configured) {
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
        // translation sensitivity to rotation: d = (U'U)^-1 U' (m - k dtheta)
        {
            std::vector<double> negk;
            for (double ki : k) {
                negk.push_back(-ki);
            }
            model->has_coupling_ =
                solvePlanar(ux, uy, negk, model->coupling_x_, model->coupling_y_);
        }
    }

    const ConfigNode output = node.child("Output");
    model->output_          = ObservationId{output.attr("observation_id")};
    if (model->output_.empty()) {
        err = who + ": needs <Output observation_id=.../>";
        return nullptr;
    }
    if (output.next("Output").valid()) {
        err = who + ": tracking_wheel_motion produces exactly one Output";
        return nullptr;
    }
    return model;
}

std::vector<RobotObservationOutputDecl> TrackingWheelMotion::outputs() const {
    return {RobotObservationOutputDecl{
        output_,
        PayloadDescriptor::of<BodyMotionIncrement>(payload_names::kBodyMotionIncrement)}};
}

ObservationReadiness TrackingWheelMotion::readiness() const {
    ObservationReadiness r;
    if (heading_.configured && !heading_.calibrated) {
        r.ready = false;
        r.note  = "gyro bias calibrating " + std::to_string(heading_.cal_count) + "/" +
                 std::to_string(heading_.bias_samples);
        return r;
    }
    r.ready = true;
    r.note  = last_drop_reason_.empty() ? "" : "last dropped window: " + last_drop_reason_;
    return r;
}

void TrackingWheelMotion::reset() {
    offered_ = false;
    for (Wheel& w : wheels_) {
        w.have_prev          = false;
        w.last_sequence      = 0;
        w.last_epoch         = 0;
        w.prev_discontinuity = 0;
        w.pending            = false;
        w.pending_travel_m   = 0.0;
        w.baselined          = true;
    }
    awaiting_baselines_ = false;
    heading_.calibrated     = heading_.bias_samples == 0;
    heading_.cal_count      = 0;
    heading_.cal_sum        = 0.0;
    heading_.cal_have_accum = false;
    heading_.cal_travel_m   = 0.0;
    heading_.bias_rad_s     = 0.0;
    heading_.have_prev      = false;
    heading_.prev_has_accum = false;
    heading_.last_sequence  = 0;
    heading_.last_epoch     = 0;
    heading_.pending        = false;
    heading_.pending_dtheta = 0.0;
    heading_.baselined      = true;
    last_received_          = MonotonicTime{};
    last_drop_reason_.clear();
}

void TrackingWheelMotion::dropWindow(const char* why) {
    for (Wheel& w : wheels_) {
        w.pending          = false;
        w.pending_travel_m = 0.0;
        w.baselined        = false;
    }
    heading_.pending        = false;
    heading_.pending_dtheta = 0.0;
    heading_.baselined      = false;
    awaiting_baselines_     = true;
    last_drop_reason_       = why;
    ++drops_;
}

void TrackingWheelMotion::restartCalibration() {
    heading_.cal_count      = 0;
    heading_.cal_sum        = 0.0;
    heading_.cal_have_accum = false;
    heading_.cal_travel_m   = 0.0;
}

FunctionStatus TrackingWheelMotion::run(const RobotObservationInput& in,
                                        RobotObservationMap&         out) {
    bool progressed = false;

    // While gyro bias collection runs, wheel baselines rebase continuously
    // instead of accumulating: the first fused solve must never combine
    // travel from before calibration with a short gyro interval.
    const bool calibrating = heading_.configured && !heading_.calibrated;

    MonotonicTime newest_stamp;

    for (Wheel& w : wheels_) {
        // only healthy sources are consumed; fault and unavailable sensors
        // hold their history without feeding the solve
        const StoredSample* stored = w.binding.freshStored(in.sensors);
        if (stored == nullptr ||
            (stored->sequence == w.last_sequence && stored->epoch == w.last_epoch)) {
            continue;   // nothing new: a retained record is not a second measurement
        }
        const EncoderSample* sample = stored->payload.get<EncoderSample>();
        if (sample == nullptr) {
            return FunctionStatus::kFault;   // wiring bug, payload changed underneath
        }
        const bool record_restart = w.have_prev && stored->epoch != w.last_epoch;
        w.last_sequence           = stored->sequence;
        w.last_epoch              = stored->epoch;
        w.provenance              = provenanceOf(w.binding.id, *stored);
        progressed                = true;
        if (!last_received_.isSet() || stored->receivedAt > last_received_) {
            last_received_ = stored->receivedAt;
        }
        if (!newest_stamp.isSet() || stored->measuredAt > newest_stamp) {
            newest_stamp = stored->measuredAt;
        }

        if (!w.have_prev) {
            w.have_prev          = true;
            w.prev_angle_rad     = sample->angle_rad;
            w.prev_stamp         = stored->measuredAt;
            w.prev_discontinuity = sample->discontinuity_epoch;
            continue;
        }
        if (record_restart || sample->discontinuity_epoch != w.prev_discontinuity ||
            !sameDomain(stored->measuredAt, w.prev_stamp)) {
            // the interval up to this sample measured nothing usable: rebase
            // the wheel and discard every pending contribution with it
            w.prev_angle_rad     = sample->angle_rad;
            w.prev_stamp         = stored->measuredAt;
            w.prev_discontinuity = sample->discontinuity_epoch;
            dropWindow("encoder discontinuity");
            w.baselined = true;   // this sample is the wheel's new baseline
            if (calibrating) {
                restartCalibration();
            }
            continue;
        }
        const int64_t dt_ms = stored->measuredAt - w.prev_stamp;
        if (dt_ms <= 0) {
            w.prev_angle_rad = sample->angle_rad;
            w.prev_stamp     = stored->measuredAt;
            dropWindow("nonpositive encoder interval");
            w.baselined = true;
            continue;
        }
        if (awaiting_baselines_) {
            // baseline only: travel before this sample may span an
            // invalid stretch
            w.prev_angle_rad = sample->angle_rad;
            w.prev_stamp     = stored->measuredAt;
            w.baselined      = true;
            continue;
        }
        if (calibrating) {
            heading_.cal_travel_m +=
                std::fabs((sample->angle_rad - w.prev_angle_rad) * w.radius_m);
            w.prev_angle_rad = sample->angle_rad;
            w.prev_stamp     = stored->measuredAt;
            continue;   // rebase only; travel during calibration is not motion data
        }
        if (!w.pending) {
            w.pending_start    = w.prev_stamp;
            w.pending_travel_m = 0.0;
        }
        w.pending_travel_m += (sample->angle_rad - w.prev_angle_rad) * w.radius_m * w.sign;
        w.pending_end    = stored->measuredAt;
        w.pending        = true;
        w.prev_angle_rad = sample->angle_rad;
        w.prev_stamp     = stored->measuredAt;
    }

    if (heading_.configured) {
        const StoredSample* stored = heading_.binding.freshStored(in.sensors);
        if (stored != nullptr && (stored->sequence != heading_.last_sequence ||
                                  stored->epoch != heading_.last_epoch)) {
            const ImuSample* sample = stored->payload.get<ImuSample>();
            if (sample == nullptr) {
                return FunctionStatus::kFault;
            }
            const bool record_restart = heading_.have_prev && stored->epoch != heading_.last_epoch;
            heading_.last_sequence    = stored->sequence;
            heading_.last_epoch       = stored->epoch;
            heading_.provenance       = provenanceOf(heading_.binding.id, *stored);
            progressed                = true;
            if (!last_received_.isSet() || stored->receivedAt > last_received_) {
                last_received_ = stored->receivedAt;
            }
            if (!newest_stamp.isSet() || stored->measuredAt > newest_stamp) {
                newest_stamp = stored->measuredAt;
            }

            if (!heading_.calibrated) {
                // Bias collection is valid only while stationary; motion
                // restarts it.
                if (heading_.cal_travel_m > heading_.max_calibration_travel_m) {
                    restartCalibration();
                }
                if (sample->has_accumulated && !heading_.cal_have_accum) {
                    heading_.cal_have_accum        = true;
                    heading_.cal_accum_start       = sample->accumulated_angle_rad;
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
                if (heading_.have_prev && !record_restart &&
                    sameDomain(stored->measuredAt, heading_.prev_stamp)) {
                    const int64_t dt_ms = stored->measuredAt - heading_.prev_stamp;
                    if (dt_ms <= 0) {
                        dropWindow("nonpositive gyro interval");
                        heading_.baselined = true;
                    } else if (dt_ms > heading_.max_gap_ms) {
                        // rate integration across an outage is garbage; the
                        // wheel travel over the same window goes with it
                        dropWindow("gyro gap");
                        heading_.baselined = true;
                    } else if (sample->has_accumulated && heading_.prev_has_accum &&
                               sample->accumulated_epoch != heading_.prev_accum_epoch) {
                        // the producer dropped an interval inside this span;
                        // nothing bridges it
                        dropWindow("gyro accumulator discontinuity");
                        heading_.baselined = true;
                    } else if (awaiting_baselines_) {
                        heading_.baselined = true;   // baseline only
                    } else {
                        const double dt = dt_ms / 1000.0;
                        double       dtheta;
                        if (sample->has_accumulated && heading_.prev_has_accum) {
                            dtheta = (sample->accumulated_angle_rad - heading_.prev_accum) -
                                     heading_.bias_rad_s * dt;
                        } else {
                            // no producer accumulator: endpoint trapezoid is
                            // the only integration available
                            dtheta = 0.5 * (heading_.prev_rate + rate) * dt;
                        }
                        if (!heading_.pending) {
                            heading_.pending_start  = heading_.prev_stamp;
                            heading_.pending_dtheta = 0.0;
                        }
                        heading_.pending_dtheta += dtheta;
                        heading_.pending_end = stored->measuredAt;
                        heading_.pending     = true;
                    }
                } else if (record_restart) {
                    dropWindow("gyro source restart");
                    heading_.baselined = true;
                } else if (awaiting_baselines_) {
                    heading_.baselined = true;
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

    if (awaiting_baselines_) {
        bool all_baselined = heading_.configured ? heading_.baselined : true;
        for (const Wheel& w : wheels_) {
            all_baselined = all_baselined && w.baselined;
        }
        if (all_baselined) {
            awaiting_baselines_ = false;   // accumulation resumes next cycle
        }
        return progressed ? FunctionStatus::kOk : FunctionStatus::kNoData;
    }

    // The previous solve is retained by the stage until disposition. New
    // sensor samples have still accumulated above, including gyro samples
    // without a producer accumulator. Do not overwrite the offered interval.
    if (offered_) {
        return progressed ? FunctionStatus::kOk : FunctionStatus::kNoData;
    }

    // Solve only when every configured input has a pending contribution and
    // those contributions describe one common interval.
    bool          all_pending = true;
    bool          any_pending = false;
    MonotonicTime start, end;
    for (const Wheel& w : wheels_) {
        all_pending = all_pending && w.pending;
        any_pending = any_pending || w.pending;
        if (w.pending) {
            if (!start.isSet() || w.pending_start < start) {
                start = w.pending_start;
            }
            if (!end.isSet() || w.pending_end > end) {
                end = w.pending_end;
            }
        }
    }
    if (heading_.configured) {
        all_pending = all_pending && heading_.pending;
        any_pending = any_pending || heading_.pending;
        if (heading_.pending) {
            if (!start.isSet() || heading_.pending_start < start) {
                start = heading_.pending_start;
            }
            if (!end.isSet() || heading_.pending_end > end) {
                end = heading_.pending_end;
            }
        }
    }
    if (!all_pending) {
        if (any_pending && newest_stamp.isSet() && sameDomain(newest_stamp, start) &&
            (newest_stamp - start) > max_pending_ms_) {
            dropWindow("sources misaligned longer than max_pending_ms");
        }
        return progressed ? FunctionStatus::kOk : FunctionStatus::kNoData;
    }

    bool aligned = true;
    for (const Wheel& w : wheels_) {
        aligned = aligned && std::llabs(w.pending_start - start) <= interval_tolerance_ms_ &&
                  std::llabs(w.pending_end - end) <= interval_tolerance_ms_;
    }
    if (heading_.configured) {
        aligned = aligned &&
                  std::llabs(heading_.pending_start - start) <= interval_tolerance_ms_ &&
                  std::llabs(heading_.pending_end - end) <= interval_tolerance_ms_;
    }
    if (!aligned) {
        if ((end - start) > max_pending_ms_) {
            dropWindow("sources misaligned longer than max_pending_ms");
        }
        return progressed ? FunctionStatus::kOk : FunctionStatus::kNoData;
    }
    const int64_t span_ms = end - start;
    if (span_ms <= 0) {
        dropWindow("nonpositive solve interval");
        return FunctionStatus::kNoData;
    }

    std::vector<double> ux, uy, k, m;
    for (const Wheel& w : wheels_) {
        ux.push_back(w.ux);
        uy.push_back(w.uy);
        k.push_back(w.k_m);
        m.push_back(w.pending_travel_m);
    }

    BodyMotionIncrement increment;
    bool                solved = false;
    if (heading_.configured) {
        increment.dtheta_rad = heading_.pending_dtheta;
        for (std::size_t i = 0; i < m.size(); ++i) {
            m[i] -= k[i] * increment.dtheta_rad;
        }
        solved = solvePlanar(ux, uy, m, increment.dx_m, increment.dy_m);
    } else {
        solved = solveFull(ux, uy, k, m, increment.dx_m, increment.dy_m,
                           increment.dtheta_rad);
    }
    increment.startAt = start;
    increment.endAt   = end;
    increment.dt_s    = span_ms / 1000.0;
    for (const Wheel& w : wheels_) {
        increment.sources.push_back(w.provenance);
    }
    if (heading_.configured) {
        increment.sources.push_back(heading_.provenance);
    }
    increment.has_rotation_coupling = has_coupling_;
    increment.dx_per_dtheta_m_rad   = coupling_x_;
    increment.dy_per_dtheta_m_rad   = coupling_y_;

    if (!solved) {
        dropWindow("wheel solve failed");
        return FunctionStatus::kNoData;
    }

    for (Wheel& w : wheels_) {
        w.pending          = false;
        w.pending_travel_m = 0.0;
    }
    heading_.pending        = false;
    heading_.pending_dtheta = 0.0;

    RobotObservationRecord record;
    record.measuredAt = end;
    record.receivedAt = last_received_;
    record.payload = TypedPayload::store(std::move(increment), payload_names::kBodyMotionIncrement);
    out[output_]   = std::move(record);
    offered_       = true;
    return FunctionStatus::kOk;
}

} // namespace navigatr
