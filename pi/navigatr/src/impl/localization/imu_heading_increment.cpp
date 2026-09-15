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

    const ConfigNode calibration = node.child("Calibration");
    if (!calibration.getInt("bias_samples", 200, fn->bias_samples_, err) ||
        !calibration.getInt("max_gap_ms", 250, fn->max_gap_ms_, err)) {
        return nullptr;
    }
    if (fn->bias_samples_ < 0 || fn->max_gap_ms_ <= 0) {
        err = node.path() +
              ": bias_samples cannot be negative and max_gap_ms must be positive";
        return nullptr;
    }
    fn->calibrated_ = fn->bias_samples_ == 0;

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
    r.ready = calibrated_;
    r.note  = calibrated_ ? last_note_
                          : "gyro bias calibrating " + std::to_string(cal_count_) + "/" +
                                std::to_string(bias_samples_);
    return r;
}

void ImuHeadingIncrement::reset() {
    pending_.reset();
    offered_        = false;
    last_sequence_  = 0;
    last_epoch_     = 0;
    calibrated_     = bias_samples_ == 0;
    cal_count_      = 0;
    cal_sum_        = 0.0;
    bias_rad_s_     = 0.0;
    have_prev_      = false;
    prev_has_accum_ = false;
    last_note_.clear();
}

FunctionStatus ImuHeadingIncrement::run(const RobotObservationInput& in,
                                        RobotObservationMap&         out) {
    RobotObservationMap produced;
    const FunctionStatus status = ingest(in, produced);
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
    const bool record_restart = have_prev_ && stored->epoch != last_epoch_;
    last_sequence_            = stored->sequence;
    last_epoch_               = stored->epoch;

    if (!calibrated_) {
        cal_sum_ += sample->yaw_rate_rad_s;
        ++cal_count_;
        if (cal_count_ >= bias_samples_) {
            bias_rad_s_ = cal_sum_ / cal_count_;
            calibrated_ = true;
        }
        return FunctionStatus::kOk;
    }

    const double rate = sample->yaw_rate_rad_s - bias_rad_s_;
    const auto   seed = [&] {
        have_prev_        = true;
        prev_rate_        = rate;
        prev_accum_       = sample->accumulated_angle_rad;
        prev_has_accum_   = sample->has_accumulated;
        prev_accum_epoch_ = sample->accumulated_epoch;
        prev_stamp_       = stored->measuredAt;
    };
    if (!have_prev_ || record_restart || !sameDomain(stored->measuredAt, prev_stamp_)) {
        pending_.reset();
        last_note_ = record_restart ? "reseeded after source restart" : "";
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
        delta.dtheta_rad =
            (sample->accumulated_angle_rad - prev_accum_) - bias_rad_s_ * delta.dt_s;
    } else {
        delta.dtheta_rad = 0.5 * (prev_rate_ + rate) * delta.dt_s;
    }
    delta.startAt = prev_stamp_;
    delta.endAt   = stored->measuredAt;
    Provenance p;
    p.source   = binding_.id.value;
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
