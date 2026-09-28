// imu_heading_increment.cpp

#include "impl/localization/imu_heading_increment.h"

namespace navigatr
{

std::unique_ptr<RobotObservationFunction>
ImuHeadingIncrement::create(const ConfigNode& node, RobotObservationInitializationContext& context,
                            std::string& err) {
    if (context.sensors == nullptr) {
        err = "imu_heading_increment needs the sensor catalog";
        return nullptr;
    }
    auto fn = std::make_unique<ImuHeadingIncrement>();
    fn->id_ = ObservationFunctionId{node.attr("id")};

    const SensorId sensor_id{node.child("Input").attr("sensor_id")};
    if (sensor_id.empty()) {
        err = node.path() + ": needs <Input sensor_id=.../>";
        return nullptr;
    }
    if (!context.sensors->bind<ImuSample>(sensor_id, node.path(), fn->binding_, err)) {
        return nullptr;
    }

    const ConfigNode            calibration  = node.child("Calibration");
    long                        bias_samples = 200;
    StillnessConfig             still;
    GyroBiasCalibration::Config bias;
    still.window_ms = 0;
    if (!calibration.getInt("bias_samples", 200, bias_samples, err) ||
        !calibration.getInt("max_gap_ms", 250, fn->max_gap_ms_, err) ||
        !calibration.getDouble("still_travel_m", still.still_travel_m, still.still_travel_m,
                               err) ||
        !readStillness(calibration, still, err) ||
        !GyroBiasCalibration::read(calibration, bias, err)) {
        return nullptr;
    }
    if (bias_samples < 0 || fn->max_gap_ms_ <= 0 || !(still.still_travel_m > 0.0)) {
        err = node.path() + ": bias_samples and window_ms cannot be negative; max_gap_ms and "
                            "still_travel_m must be positive";
        return nullptr;
    }
    still.min_samples = bias_samples > 0 ? bias_samples : 20;
    bias.enabled      = bias_samples > 0;
    fn->window_.configure(still);

    bool ok = true;
    calibration.forEach("Wheel", [&](const ConfigNode& w) {
        if (!ok) {
            return;
        }
        StillWheel  wheel;
        std::string id;
        ok = w.requireAttr("sensor_id", id, err) &&
             w.requireDouble("radius_m", wheel.radius_m, err) &&
             context.sensors->bind<EncoderSample>(SensorId{id}, w.path(), wheel.binding, err);
        if (ok && !(wheel.radius_m > 0.0)) {
            err = w.path() + ": radius_m must be positive";
            ok  = false;
        }
        if (ok) {
            wheel.source = fn->window_.addSource(StationaryWindow::Kind::kWheel, "wheel " + id);
            fn->wheels_.push_back(std::move(wheel));
        }
    });
    if (!ok) {
        return nullptr;
    }
    fn->gyro_source_ =
        fn->window_.addSource(StationaryWindow::Kind::kGyro, "gyro " + sensor_id.value);
    fn->bias_.configure(bias);

    fn->output_ = ObservationId{node.child("Output").attr("observation_id")};
    if (fn->output_.empty()) {
        err = node.path() + ": needs <Output observation_id=.../>";
        return nullptr;
    }
    return fn;
}

std::vector<RobotObservationOutputDecl> ImuHeadingIncrement::outputs() const {
    return {RobotObservationOutputDecl{
        output_, PayloadDescriptor::of<HeadingIncrement>(payload_names::kHeadingIncrement)}};
}

ObservationReadiness ImuHeadingIncrement::readiness() const {
    ObservationReadiness r;
    r.stillness = stillnessOf(window_, &bias_);
    r.ready     = bias_.calibrated();
    r.note      = r.ready ? last_note_
                          : std::string("gyro bias ") + toString(r.stillness.calibration) + ": " +
                                r.stillness.reason;
    return r;
}

bool ImuHeadingIncrement::recalibrate() {
    if (!bias_.enabled()) {
        return false;
    }
    pending_.reset();
    window_.restart(StationaryWindow::Phase::kWaitingData, "recalibration requested");
    bias_.start("recalibration requested");
    have_prev_ = false;   // nothing integrates across the calibration
    last_note_.clear();
    return true;
}

void ImuHeadingIncrement::reset() {
    pending_.reset();
    offered_        = false;
    seen_           = false;
    last_sequence_  = 0;
    last_epoch_     = 0;
    last_upstream_  = 0;
    have_prev_      = false;
    prev_has_accum_ = false;
    window_.restart(StationaryWindow::Phase::kWaitingData, "reset");
    bias_.start("reset");
    last_note_.clear();
}

void ImuHeadingIncrement::takeWindow(MonotonicTime now) {
    StationaryWindow::Qualified q;
    if (window_.takeQualified(q) && q.gyro) {
        bias_.qualified(q.gyro_rate, now);
    }
}

FunctionStatus ImuHeadingIncrement::run(const RobotObservationInput& in,
                                        RobotObservationMap&         out) {
    // wheel evidence for the stationary window only
    for (const StillWheel& w : wheels_) {
        const StoredSample* stored = w.binding.freshStored(in.sensors);
        if (stored == nullptr) {
            continue;
        }
        const EncoderSample* sample = stored->payload.get<EncoderSample>();
        if (sample == nullptr) {
            return FunctionStatus::kFault;
        }
        StillSample evidence;
        evidence.sequence      = stored->sequence;
        evidence.epoch         = stored->epoch;
        evidence.discontinuity = sample->discontinuity_epoch;
        evidence.at            = stored->measuredAt;
        evidence.received      = stored->receivedAt;
        evidence.value         = sample->angle_rad * w.radius_m;
        window_.add(w.source, evidence);
        takeWindow(in.context.now);
    }

    RobotObservationMap  produced;
    const FunctionStatus status = ingest(in, produced);
    window_.poll(in.context.now);
    bias_.poll(in.context.now);

    const auto found = produced.find(output_);
    if (found != produced.end()) {
        RobotObservationRecord record = found->second;
        HeadingIncrement delta = *record.payload.get<HeadingIncrement>();
        if (pending_) {
            const HeadingIncrement& previous = *pending_->payload.get<HeadingIncrement>();
            if (sameDomain(previous.endAt, delta.startAt) &&
                previous.endAt.ms == delta.startAt.ms &&
                previous.sources.front().epoch == delta.sources.front().epoch) {
                delta.startAt = previous.startAt;
                delta.dt_s += previous.dt_s;
                delta.dtheta_rad += previous.dtheta_rad;
            } else {
                last_note_ = "pending heading interval invalidated by a discontinuity";
            }
        }
        record.payload = TypedPayload::store(std::move(delta), payload_names::kHeadingIncrement);
        pending_ = std::move(record);
    }
    if (!offered_ && pending_) {
        out[output_] = std::move(*pending_);
        pending_.reset();
        offered_ = true;
        return FunctionStatus::kOk;
    }
    return status;
}

FunctionStatus ImuHeadingIncrement::ingest(const RobotObservationInput& in,
                                           RobotObservationMap& out) {
    // only a currently healthy source is consumed
    const StoredSample* stored = binding_.freshStored(in.sensors);
    if (stored == nullptr ||
        (stored->sequence == last_sequence_ && stored->epoch == last_epoch_)) {
        return FunctionStatus::kNoData;
    }
    const ImuSample* sample = stored->payload.get<ImuSample>();
    if (sample == nullptr) {
        return FunctionStatus::kFault;
    }
    const bool record_restart = seen_ && stored->epoch != last_epoch_;
    const bool source_restart = seen_ && stored->upstream.epoch != last_upstream_;
    seen_                     = true;
    last_sequence_            = stored->sequence;
    last_epoch_               = stored->epoch;
    last_upstream_            = stored->upstream.epoch;

    StillSample evidence;
    evidence.sequence          = stored->sequence;
    evidence.epoch             = stored->epoch;
    evidence.discontinuity     = stored->upstream.epoch;
    evidence.at                = stored->measuredAt;
    evidence.received          = stored->receivedAt;
    evidence.value             = sample->yaw_rate_rad_s;
    evidence.has_accumulated   = sample->has_accumulated;
    evidence.accumulated       = sample->accumulated_angle_rad;
    evidence.accumulated_epoch = sample->accumulated_epoch;
    const bool bias_ready      = bias_.calibrated();
    window_.add(gyro_source_, evidence);
    if ((record_restart || source_restart) && bias_.enabled()) {
        // a restarted IMU invalidates its bias; calibration starts again
        pending_.reset();
        bias_.start("IMU source restarted");
    }
    takeWindow(in.context.now);

    if (!bias_ready || !bias_.calibrated()) {
        have_prev_ = false;   // calibration data; the first sample after it seeds
        return FunctionStatus::kOk;
    }

    const double bias = bias_.bias();
    const double rate = sample->yaw_rate_rad_s - bias;
    const auto   seed = [&] {
        have_prev_        = true;
        prev_rate_        = rate;
        prev_accum_       = sample->accumulated_angle_rad;
        prev_has_accum_   = sample->has_accumulated;
        prev_accum_epoch_ = sample->accumulated_epoch;
        prev_stamp_       = stored->measuredAt;
    };
    if (!have_prev_ || record_restart || source_restart ||
        !sameDomain(stored->measuredAt, prev_stamp_)) {
        pending_.reset();
        last_note_ = record_restart || source_restart ? "reseeded after source restart" : "";
        seed();
        return FunctionStatus::kOk;
    }

    const int64_t dt_ms = stored->measuredAt - prev_stamp_;
    if (dt_ms <= 0) {
        pending_.reset();
        last_note_ = "nonpositive interval dropped";
        seed();
        return FunctionStatus::kOk;
    }
    if (dt_ms > max_gap_ms_) {
        pending_.reset();
        last_note_ = "gap dropped";   // integrating across an outage is garbage
        seed();
        return FunctionStatus::kOk;
    }
    if (sample->has_accumulated && prev_has_accum_ &&
        sample->accumulated_epoch != prev_accum_epoch_) {
        pending_.reset();
        last_note_ = "accumulator discontinuity dropped";
        seed();
        return FunctionStatus::kOk;
    }

    HeadingIncrement delta;
    delta.dt_s       = dt_ms / 1000.0;
    delta.rate_rad_s = rate;
    if (sample->has_accumulated && prev_has_accum_) {
        delta.dtheta_rad = (sample->accumulated_angle_rad - prev_accum_) - bias * delta.dt_s;
    } else {
        delta.dtheta_rad = 0.5 * (prev_rate_ + rate) * delta.dt_s;
    }
    delta.startAt = prev_stamp_;
    delta.endAt   = stored->measuredAt;
    Provenance p;
    p.source   = binding_.id.value;
    p.measurement = stored->upstream.measurement.empty() ? stored->upstream.source
                                                           : stored->upstream.measurement;
    p.clock    = stored->upstream.clock;
    p.sequence = stored->sequence;
    p.epoch    = stored->epoch;
    delta.sources.push_back(std::move(p));

    seed();
    last_note_.clear();

    RobotObservationRecord record;
    record.measuredAt = stored->measuredAt;
    record.receivedAt = stored->receivedAt;
    record.payload    = TypedPayload::store(std::move(delta), payload_names::kHeadingIncrement);
    out[output_]      = std::move(record);
    return FunctionStatus::kOk;
}

} // namespace navigatr
