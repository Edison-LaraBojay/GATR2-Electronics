// pico_channels.cpp

#include "impl/sensors/pico_channels.h"

#include "math/angles.h"
#include "payloads/pico_telemetry_samples.h"
#include "runtime/resource_catalog.h"

namespace navigatr
{

namespace
{

// Source binding plus the shared per-output health classification: which
// output this sensor reads and what it has already consumed from it.
//
// Health rules, in order:
//   1. an unseen stored sample is consumed; it was Valid when the stage
//      stored it, so it is evidence even if the output has since faulted
//   2. otherwise the output's current Fault or Unavailable state propagates
//   3. otherwise no sample yet is NoDataYet
//   4. otherwise silence longer than stale_after_ms is Unavailable
//   5. otherwise a healthy quiet cycle
// A record epoch change or an upstream source restart is reported so the
// caller rebases its baseline instead of differencing across it.
template <typename Payload>
struct ChannelInput {
    TypedOutputBinding<Payload> binding;
    long                        stale_after_ms = 250;   // 0 disables

    uint64_t      last_sequence  = 0;
    uint64_t      last_epoch     = 0;
    uint64_t      last_upstream  = 0;   // upstream restart epoch
    bool          has_new        = false;
    MonotonicTime last_new_at;   // host clock

    bool bind(const ConfigNode& node, SensorInitializationContext& context,
              std::string& err) {
        const ConfigNode source = node.child("Source");
        const ResourceId resource{source.attr("resource_id")};
        const OutputId   output{source.attr("output_id")};
        if (resource.empty() || output.empty()) {
            err = node.path() + ": needs <Source resource_id=... output_id=.../>";
            return false;
        }
        if (context.outputs == nullptr) {
            err = node.path() + ": no resource outputs available";
            return false;
        }
        if (!context.outputs->bind<Payload>(resource, output, node.path(), binding, err)) {
            return false;
        }
        if (!node.child("Freshness").getInt("stale_after_ms", 250, stale_after_ms, err)) {
            return false;
        }
        if (stale_after_ms < 0) {
            err = node.path() + ": stale_after_ms cannot be negative";
            return false;
        }
        return true;
    }

    // Classifies this poll. Returns the sample only when it is new, so a
    // retained record read twice never becomes two measurements. restart
    // is set when the sample belongs to a new epoch of the record or of
    // the upstream source.
    const StoredSample* fresh(const ResourceMap& resources, const ExecutionContext& context,
                              PollResult& result, bool& restart) {
        restart                         = false;
        const MeasurementRecord* record = binding.record(resources);
        if (record == nullptr) {
            result.state      = SourceState::kFault;
            result.diagnostic = "bound resource output is missing";
            return nullptr;
        }
        if (record->latest.has_value() &&
            (record->latest->sequence != last_sequence ||
             record->latest->epoch != last_epoch)) {
            restart = has_new && (record->latest->epoch != last_epoch ||
                                  record->latest->upstream.epoch != last_upstream);
            last_sequence = record->latest->sequence;
            last_epoch    = record->latest->epoch;
            last_upstream = record->latest->upstream.epoch;
            has_new       = true;
            last_new_at   = context.now;
            result.state  = SourceState::kValid;
            if (record->state != SourceState::kValid) {
                result.diagnostic = std::string("source now ") + sourceStateName(record->state);
            }
            return &*record->latest;
        }
        if (record->state == SourceState::kFault || record->state == SourceState::kUnavailable) {
            result.state      = record->state;
            result.diagnostic = record->diagnostic;
            return nullptr;
        }
        if (!record->latest.has_value()) {
            result.state = SourceState::kNoDataYet;
            return nullptr;
        }
        if (stale_after_ms != 0 && has_new && (context.now - last_new_at) > stale_after_ms) {
            result.state      = SourceState::kUnavailable;
            result.diagnostic = "no new data within stale_after_ms";
            return nullptr;
        }
        result.state = SourceState::kValid;   // healthy, no new sample
        return nullptr;
    }

    // Carries the upstream receipt and identity forward.
    Publication publicationFor(const StoredSample& stored, TypedPayload payload) const {
        Publication p;
        p.measuredAt        = stored.measuredAt;
        p.receivedAt        = stored.receivedAt;
        p.upstream.source   = binding.resource.value + "." + binding.output.value;
        p.upstream.measurement = stored.upstream.measurement.empty()
                                     ? p.upstream.source
                                     : stored.upstream.measurement;
        p.upstream.clock    = stored.upstream.clock;
        p.upstream.sequence = stored.sequence;
        p.upstream.epoch    = stored.epoch + stored.upstream.epoch;
        p.payload           = std::move(payload);
        return p;
    }

    void reset() {
        last_sequence = 0;
        last_epoch    = 0;
        last_upstream = 0;
        has_new       = false;
    }
};

} // namespace

std::optional<SensorExecutable> make_pico_encoder_channel(
    const ConfigNode& node, SensorInitializationContext& context, std::string& err) {
    ChannelInput<PicoEncoderCounts> input;
    if (!input.bind(node, context, err)) {
        return std::nullopt;
    }

    const ConfigNode calibration = node.child("Calibration");
    double           cpr         = 0.0;
    bool             invert      = false;
    if (!calibration.getDouble("counts_per_revolution", 0.0, cpr, err) ||
        !calibration.getBool("invert", false, invert, err)) {
        return std::nullopt;
    }
    if (cpr <= 0.0) {
        err = node.path() + ": Calibration needs positive counts_per_revolution";
        return std::nullopt;
    }
    const double radians_per_count = (2.0 * kPi / cpr) * (invert ? -1.0 : 1.0);

    struct State {
        ChannelInput<PicoEncoderCounts> input;
        double                          radians_per_count = 0.0;
        bool                            have_prev     = false;
        int32_t                         prev_counts   = 0;
        double                          angle_rad     = 0.0;
        uint64_t                        discontinuity = 0;
    };
    auto state               = std::make_shared<State>();
    state->input             = std::move(input);
    state->radians_per_count = radians_per_count;

    SensorExecutable executable;
    executable.outputPayload =
        PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample);
    executable.execute = [state](const ResourceMap&      resources,
                                 const ExecutionContext& context) {
        PollResult          result;
        bool                restart = false;
        const StoredSample* stored  = state->input.fresh(resources, context, result, restart);
        if (stored == nullptr) {
            return result;
        }
        const PicoEncoderCounts* raw = stored->payload.get<PicoEncoderCounts>();
        if (raw == nullptr) {
            result.state      = SourceState::kFault;
            result.diagnostic = "bound output changed payload";
            return result;
        }

        const int32_t counts = raw->counts;
        if (restart) {
            // the counter restarted with its source: rebase the baseline,
            // keep the angle continuous, and mark the span as unmeasured
            state->discontinuity += 1;
        } else if (state->have_prev) {
            // modular unsigned subtraction survives count wraparound
            const int32_t delta = static_cast<int32_t>(
                static_cast<uint32_t>(counts) - static_cast<uint32_t>(state->prev_counts));
            state->angle_rad += delta * state->radians_per_count;
        }
        state->have_prev   = true;
        state->prev_counts = counts;

        EncoderSample sample;
        sample.angle_rad           = state->angle_rad;
        sample.discontinuity_epoch = state->discontinuity;
        result.publication         = state->input.publicationFor(
            *stored, TypedPayload::store(sample, payload_names::kEncoderSample));
        return result;
    };
    executable.reset = [state] {
        state->input.reset();
        state->have_prev     = false;
        state->angle_rad     = 0.0;
        state->discontinuity = 0;
    };
    return executable;
}

std::optional<SensorExecutable> make_pico_imu_channel(const ConfigNode& node,
                                                      SensorInitializationContext& context,
                                                      std::string& err) {
    ChannelInput<PicoGyroRate> input;
    if (!input.bind(node, context, err)) {
        return std::nullopt;
    }
    bool invert = false;
    if (!node.child("Calibration").getBool("invert", false, invert, err)) {
        return std::nullopt;
    }

    struct State {
        ChannelInput<PicoGyroRate> input;
        double                     sign = 1.0;
        uint64_t                   restarts = 0;
    };
    auto state   = std::make_shared<State>();
    state->input = std::move(input);
    state->sign  = invert ? -1.0 : 1.0;

    SensorExecutable executable;
    executable.outputPayload = PayloadDescriptor::of<ImuSample>(payload_names::kImuSample);
    executable.execute = [state](const ResourceMap&      resources,
                                 const ExecutionContext& context) {
        PollResult          result;
        bool                restart = false;
        const StoredSample* stored  = state->input.fresh(resources, context, result, restart);
        if (stored == nullptr) {
            return result;
        }
        const PicoGyroRate* raw = stored->payload.get<PicoGyroRate>();
        if (raw == nullptr) {
            result.state      = SourceState::kFault;
            result.diagnostic = "bound output changed payload";
            return result;
        }
        if (restart) {
            state->restarts += 1;
        }

        // wire unit is millidegrees per second; a source restart shifts the
        // accumulator epoch so consumers never difference across it
        ImuSample sample;
        sample.yaw_rate_rad_s        = state->sign * degToRad(raw->rate_mdps / 1000.0);
        sample.accumulated_angle_rad = state->sign * degToRad(raw->accumulated_mdeg / 1000.0);
        sample.accumulated_epoch     = raw->accumulated_epoch + (state->restarts << 32);
        sample.has_accumulated       = true;

        result.publication = state->input.publicationFor(
            *stored, TypedPayload::store(sample, payload_names::kImuSample));
        return result;
    };
    executable.reset = [state] {
        state->input.reset();
        state->restarts = 0;
    };
    return executable;
}

} // namespace navigatr
