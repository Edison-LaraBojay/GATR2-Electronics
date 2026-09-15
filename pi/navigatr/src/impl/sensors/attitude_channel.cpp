// attitude_channel.cpp

#include "impl/sensors/attitude_channel.h"

#include <memory>

#include "math/angles.h"
#include "math/quaternion.h"
#include "payloads/attitude_samples.h"
#include "runtime/resource_catalog.h"

namespace navigatr
{

std::optional<SensorExecutable> make_attitude_channel(const ConfigNode&            node,
                                                      SensorInitializationContext& context,
                                                      std::string&                 err) {
    const ConfigNode source = node.child("Source");
    const ResourceId resource{source.attr("resource_id")};
    const OutputId   output{source.attr("output_id")};
    if (resource.empty() || output.empty()) {
        err = node.path() + ": needs <Source resource_id=... output_id=.../>";
        return std::nullopt;
    }
    if (context.outputs == nullptr) {
        err = node.path() + ": no resource outputs available";
        return std::nullopt;
    }

    struct State {
        TypedOutputBinding<AttitudeSample> binding;
        Quaternion                         q_sensor_body;   // inverse of the mounting
        long                               stale_after_ms = 250;
        uint64_t                           last_sequence  = 0;
        uint64_t                           last_epoch     = 0;
        bool                               has_new        = false;
        bool                               invalid_attitude = false;
        MonotonicTime                      last_new_at;
    };
    auto state = std::make_shared<State>();
    if (!context.outputs->bind<AttitudeSample>(resource, output, node.path(), state->binding,
                                               err)) {
        return std::nullopt;
    }

    const ConfigNode mounting = node.child("Mounting");
    if (mounting.valid()) {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        if (!mounting.requireDouble("roll_deg", roll, err) ||
            !mounting.requireDouble("pitch_deg", pitch, err) ||
            !mounting.requireDouble("yaw_deg", yaw, err)) {
            return std::nullopt;
        }
        state->q_sensor_body =
            conjugate(quaternionFromEuler(degToRad(roll), degToRad(pitch), degToRad(yaw)));
    }
    if (!node.child("Freshness").getInt("stale_after_ms", 250, state->stale_after_ms, err)) {
        return std::nullopt;
    }
    if (state->stale_after_ms < 0) {
        err = node.path() + ": stale_after_ms cannot be negative";
        return std::nullopt;
    }

    SensorExecutable executable;
    executable.outputPayload =
        PayloadDescriptor::of<AttitudeSample>(payload_names::kAttitudeSample);
    executable.execute = [state](const ResourceMap&      resources,
                                 const ExecutionContext& context) {
        PollResult               result;
        const MeasurementRecord* record = state->binding.record(resources);
        if (record == nullptr) {
            result.state      = SourceState::kFault;
            result.diagnostic = "bound resource output is missing";
            return result;
        }
        if (record->latest.has_value() &&
            (record->latest->sequence != state->last_sequence ||
             record->latest->epoch != state->last_epoch)) {
            const StoredSample&   stored = *record->latest;
            const AttitudeSample* raw    = stored.payload.get<AttitudeSample>();
            if (raw == nullptr) {
                result.state      = SourceState::kFault;
                result.diagnostic = "bound output changed payload";
                return result;
            }
            state->last_sequence = stored.sequence;
            state->last_epoch    = stored.epoch;
            state->has_new       = true;
            state->last_new_at   = context.now;

            const Quaternion& q = raw->q_reference_body;
            const double length = norm(q);
            state->invalid_attitude =
                !std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
                !std::isfinite(q.z) || !std::isfinite(length) || length < 1e-12;
            if (state->invalid_attitude) {
                result.state      = SourceState::kFault;
                result.diagnostic = "attitude quaternion must be finite and have nonzero norm";
                return result;
            }

            AttitudeSample body = *raw;
            body.q_reference_body =
                normalized(multiply(raw->q_reference_body, state->q_sensor_body));
            body.epoch = raw->epoch + stored.upstream.epoch;

            Publication p;
            p.measuredAt        = stored.measuredAt;
            p.receivedAt        = stored.receivedAt;
            p.upstream.source   = state->binding.resource.value + "." + state->binding.output.value;
            p.upstream.clock    = stored.upstream.clock;
            p.upstream.sequence = stored.sequence;
            p.upstream.epoch    = stored.epoch + stored.upstream.epoch;
            p.payload           = TypedPayload::store(body, payload_names::kAttitudeSample);
            result.state        = SourceState::kValid;
            result.publication  = std::move(p);
            return result;
        }
        if (record->state == SourceState::kFault || record->state == SourceState::kUnavailable) {
            result.state      = record->state;
            result.diagnostic = record->diagnostic;
            return result;
        }
        if (!record->latest.has_value()) {
            result.state = SourceState::kNoDataYet;
            return result;
        }
        if (state->invalid_attitude) {
            result.state      = SourceState::kFault;
            result.diagnostic = "attitude quaternion must be finite and have nonzero norm";
            return result;
        }
        if (state->stale_after_ms != 0 && state->has_new &&
            (context.now - state->last_new_at) > state->stale_after_ms) {
            result.state      = SourceState::kUnavailable;
            result.diagnostic = "no new attitude within stale_after_ms";
            return result;
        }
        result.state = SourceState::kValid;
        return result;
    };
    executable.reset = [state] {
        state->last_sequence = 0;
        state->last_epoch    = 0;
        state->has_new       = false;
        state->invalid_attitude = false;
    };
    return executable;
}

} // namespace navigatr
