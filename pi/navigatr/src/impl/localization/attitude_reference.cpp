// attitude_reference.cpp

#include "impl/localization/attitude_reference.h"

namespace navigatr
{

std::unique_ptr<RobotObservationFunction>
AttitudeReference::create(const ConfigNode& node, RobotObservationInitializationContext& context,
                          std::string& err) {
    if (context.sensors == nullptr) {
        err = "attitude_reference needs the sensor catalog";
        return nullptr;
    }
    auto fn = std::make_unique<AttitudeReference>();
    fn->id_ = ObservationFunctionId{node.attr("id")};

    const SensorId sensor_id{node.child("Input").attr("sensor_id")};
    if (sensor_id.empty()) {
        err = node.path() + ": needs <Input sensor_id=.../>";
        return nullptr;
    }
    if (!context.sensors->bind<AttitudeSample>(sensor_id, node.path(), fn->binding_, err)) {
        return nullptr;
    }
    if (!node.child("Freshness").getInt("max_age_ms", 100, fn->max_age_ms_, err)) {
        return nullptr;
    }
    if (fn->max_age_ms_ <= 0) {
        err = node.path() + ": max_age_ms must be positive";
        return nullptr;
    }
    fn->output_ = ObservationId{node.child("Output").attr("observation_id")};
    if (fn->output_.empty()) {
        err = node.path() + ": needs <Output observation_id=.../>";
        return nullptr;
    }
    return fn;
}

std::vector<RobotObservationOutputDecl> AttitudeReference::outputs() const {
    return {RobotObservationOutputDecl{
        output_,
        PayloadDescriptor::of<AttitudeObservation>(payload_names::kAttitudeObservation)}};
}

ObservationReadiness AttitudeReference::readiness() const {
    ObservationReadiness r;
    r.ready = true;   // nothing to calibrate here; availability is per sample
    r.note  = seen_ ? "" : "no attitude sample yet";
    return r;
}

FunctionStatus AttitudeReference::run(const RobotObservationInput& in,
                                      RobotObservationMap&         out) {
    // Attitude is an instantaneous sample, not an increment. Latest-sample
    // replacement in SensorMap is appropriate while the prior offer waits.
    if (offered_) return FunctionStatus::kNoData;
    const StoredSample* stored = binding_.freshStored(in.sensors);
    if (stored == nullptr ||
        (stored->sequence == last_sequence_ && stored->epoch == last_epoch_)) {
        return FunctionStatus::kNoData;
    }
    const AttitudeSample* sample = stored->payload.get<AttitudeSample>();
    if (sample == nullptr) {
        return FunctionStatus::kFault;
    }
    last_sequence_ = stored->sequence;
    last_epoch_    = stored->epoch;
    seen_          = true;

    // a sample that sat in the record longer than max_age_ms before this
    // model saw it is not current attitude
    if (in.context.now.isSet() && stored->receivedAt.isSet() &&
        (in.context.now - stored->receivedAt) > max_age_ms_) {
        return FunctionStatus::kNoData;
    }
    if (!isUnit(sample->q_reference_body, 1e-3)) {
        return FunctionStatus::kFault;   // not a rotation
    }

    AttitudeObservation obs;
    obs.q_reference_body = normalized(sample->q_reference_body);
    obs.reference        = sample->reference;
    obs.has_yaw          = sample->has_yaw;
    obs.measuredAt       = stored->measuredAt;
    obs.quality          = sample->quality;
    obs.source.source    = binding_.id.value;
    obs.source.measurement = stored->upstream.measurement.empty()
                                 ? stored->upstream.source
                                 : stored->upstream.measurement;
    obs.source.clock     = stored->upstream.clock;
    obs.source.sequence  = stored->sequence;
    obs.source.epoch     = stored->epoch + sample->epoch;

    RobotObservationRecord record;
    record.measuredAt = stored->measuredAt;
    record.receivedAt = stored->receivedAt;
    record.payload = TypedPayload::store(std::move(obs), payload_names::kAttitudeObservation);
    out[output_]   = std::move(record);
    offered_       = true;
    return FunctionStatus::kOk;
}

} // namespace navigatr
