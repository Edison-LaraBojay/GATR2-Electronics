// imu_normalization.cpp

#include "impl/preprocessing/imu_normalization.h"

namespace navigatr
{

std::unique_ptr<PreprocessorExecutable> ImuNormalization::create(
    const ConfigNode& node, PreprocessorInitializationContext& context, std::string& err) {
    if (context.sensors == nullptr) {
        err = "imu_normalization needs the sensor catalog";
        return nullptr;
    }
    auto fn = std::make_unique<ImuNormalization>();
    fn->id_ = PreprocessorId{node.attr("id")};

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

    fn->output_ = ArtifactId{node.child("Output").attr("artifact_id")};
    if (fn->output_.empty()) {
        err = node.path() + ": needs <Output artifact_id=.../>";
        return nullptr;
    }
    return fn;
}

std::vector<ArtifactOutputDecl> ImuNormalization::outputs() const {
    return {ArtifactOutputDecl{output_,
                               PayloadDescriptor::of<ImuDelta>(payload_names::kImuDelta)}};
}

void ImuNormalization::reset() {
    last_sequence_ = 0;
    calibrated_    = bias_samples_ == 0;
    cal_count_     = 0;
    cal_sum_       = 0.0;
    bias_rad_s_    = 0.0;
    have_prev_     = false;
}

FunctionStatus ImuNormalization::run(const PreprocessingInput& in, ArtifactMap& out) {
    // only a currently healthy source is consumed
    const StoredSensorSample* stored = binding_.freshStored(in.sensorResults);
    if (stored == nullptr || stored->sequence == last_sequence_) {
        return FunctionStatus::kNoData;
    }
    const ImuSample* sample = stored->payload.get<ImuSample>();
    if (sample == nullptr) {
        return FunctionStatus::kFault;
    }
    last_sequence_ = stored->sequence;

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
    if (!have_prev_) {
        have_prev_  = true;
        prev_rate_  = rate;
        prev_stamp_ = stored->measuredAt;
        return FunctionStatus::kOk;
    }

    const double dt_s = secondsBetween(stored->measuredAt, prev_stamp_);
    if (dt_s * 1000.0 > static_cast<double>(max_gap_ms_)) {
        // rate integration across an outage is garbage; reseed instead
        prev_rate_  = rate;
        prev_stamp_ = stored->measuredAt;
        return FunctionStatus::kOk;
    }

    ImuDelta delta;
    delta.dt_s       = dt_s;
    delta.rate_rad_s = rate;
    delta.delta_rad  = 0.5 * (prev_rate_ + rate) * delta.dt_s;

    prev_rate_  = rate;
    prev_stamp_ = stored->measuredAt;

    ArtifactRecord record;
    record.measuredAt = stored->measuredAt;
    record.payload    = TypedPayload::store(delta, payload_names::kImuDelta);
    out[output_]      = std::move(record);
    return FunctionStatus::kOk;
}

} // namespace navigatr
