// pico_channels.cpp

#include "impl/sensors/pico_channels.h"

#include "impl/resources/pico_telemetry.h"
#include "math/angles.h"
#include "resources/resource_map.h"

namespace navigatr
{

namespace
{

struct ChannelSetup {
    std::shared_ptr<PicoTelemetry> telemetry;
    long                           stale_after_ms = 250;   // 0 disables
};

bool setupFrom(const ConfigNode& node, SensorInitializationContext& context,
               ConfigNode& source_out, ChannelSetup& out, std::string& err) {
    source_out = node.child("Source");
    const ResourceId telemetry_id{source_out.attr("resource_id")};
    if (telemetry_id.empty()) {
        err = node.path() + ": needs <Source resource_id=.../>";
        return false;
    }
    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return false;
    }
    std::string inner;
    out.telemetry = context.resources->require<PicoTelemetry>(telemetry_id, inner);
    if (out.telemetry == nullptr) {
        err = node.path() + ": " + inner;
        return false;
    }
    if (!node.child("Freshness").getInt("stale_after_ms", 250, out.stale_after_ms, err)) {
        return false;
    }
    if (out.stale_after_ms < 0) {
        err = node.path() + ": stale_after_ms cannot be negative";
        return false;
    }
    return true;
}

// Shared polling shell: refresh once per cycle, then classify freshness.
// Returns true when the caller should decode a new sample from the channel.
struct ChannelPollState {
    uint64_t      last_updates = 0;
    bool          has_new      = false;
    MonotonicTime last_new_at;   // host clock

    bool classify(const ChannelSetup& setup, const PicoTelemetry::Channel& channel,
                  const SensorExecutionInput& input, SensorPollResult& result) {
        if (channel.updates != last_updates) {
            last_updates = channel.updates;
            has_new      = true;
            last_new_at  = input.now;
            result.state = SensorState::kValid;
            return true;
        }
        // data decoded before a link death still counted; the fault shows on
        // the first poll with nothing new
        if (setup.telemetry->linkDead()) {
            result.state      = SensorState::kFault;
            result.diagnostic = "telemetry link dead";
            return false;
        }
        if (!channel.present) {
            result.state = SensorState::kNoDataYet;
            return false;
        }
        if (setup.stale_after_ms != 0 && has_new &&
            (input.now - last_new_at) > setup.stale_after_ms) {
            result.state      = SensorState::kUnavailable;
            result.diagnostic = "no new data within stale_after_ms";
            return false;
        }
        result.state = SensorState::kValid;   // healthy, no new sample
        return false;
    }
};

} // namespace

std::optional<SensorExecutable> make_pico_encoder_channel(
    const ConfigNode& node, SensorInitializationContext& context, std::string& err) {
    ConfigNode   source;
    ChannelSetup setup;
    if (!setupFrom(node, context, source, setup, err)) {
        return std::nullopt;
    }

    long channel = -1;
    if (!source.getInt("channel", -1, channel, err)) {
        return std::nullopt;
    }
    if (channel < 0 || channel >= PicoTelemetry::kEncoderChannels) {
        err = source.path() + ": channel must be 0.." +
              std::to_string(PicoTelemetry::kEncoderChannels - 1);
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
        ChannelSetup     setup;
        int              channel = -1;
        double           radians_per_count = 0.0;
        ChannelPollState poll;
        bool             have_prev   = false;
        int32_t          prev_counts = 0;
        double           angle_rad   = 0.0;
    };
    auto state               = std::make_shared<State>();
    state->setup             = std::move(setup);
    state->channel           = static_cast<int>(channel);
    state->radians_per_count = radians_per_count;

    SensorExecutable executable;
    executable.outputPayload =
        PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample);
    executable.execute = [state](const SensorExecutionInput& input) {
        state->setup.telemetry->refresh(input.cycle, input.diagnostics);
        SensorPollResult result;
        const PicoTelemetry::Channel& channel =
            state->setup.telemetry->encoder(state->channel);
        if (!state->poll.classify(state->setup, channel, input, result)) {
            return result;
        }

        const int32_t counts = channel.value[0];
        if (state->have_prev) {
            // modular unsigned subtraction survives count wraparound
            const int32_t delta = static_cast<int32_t>(
                static_cast<uint32_t>(counts) - static_cast<uint32_t>(state->prev_counts));
            state->angle_rad += delta * state->radians_per_count;
        }
        state->have_prev   = true;
        state->prev_counts = counts;

        result.publication =
            SensorPublication{channel.measuredAt,
                              TypedPayload::store(EncoderSample{state->angle_rad},
                                                  payload_names::kEncoderSample)};
        return result;
    };
    executable.reset = [state] {
        state->poll      = ChannelPollState{};
        state->have_prev = false;
        state->angle_rad = 0.0;
    };
    return executable;
}

std::optional<SensorExecutable> make_pico_imu_channel(const ConfigNode& node,
                                                      SensorInitializationContext& context,
                                                      std::string& err) {
    ConfigNode   source;
    ChannelSetup setup;
    if (!setupFrom(node, context, source, setup, err)) {
        return std::nullopt;
    }
    if (source.attr("channel") != "imu") {
        err = source.path() + ": pico_imu_channel supports channel=\"imu\"";
        return std::nullopt;
    }
    bool invert = false;
    if (!node.child("Calibration").getBool("invert", false, invert, err)) {
        return std::nullopt;
    }

    struct State {
        ChannelSetup     setup;
        double           sign = 1.0;
        ChannelPollState poll;
    };
    auto state   = std::make_shared<State>();
    state->setup = std::move(setup);
    state->sign  = invert ? -1.0 : 1.0;

    SensorExecutable executable;
    executable.outputPayload = PayloadDescriptor::of<ImuSample>(payload_names::kImuSample);
    executable.execute = [state](const SensorExecutionInput& input) {
        state->setup.telemetry->refresh(input.cycle, input.diagnostics);
        SensorPollResult              result;
        const PicoTelemetry::Channel& gyro = state->setup.telemetry->gyro();
        if (!state->poll.classify(state->setup, gyro, input, result)) {
            return result;
        }

        // wire unit is millidegrees per second
        ImuSample sample;
        sample.yaw_rate_rad_s = state->sign * degToRad(gyro.value[0] / 1000.0);
        sample.accumulated_angle_rad =
            state->sign *
            degToRad(state->setup.telemetry->gyroAccumulatedRaw() / 1000.0);
        sample.accumulated_epoch = state->setup.telemetry->gyroAccumulatedEpoch();
        sample.has_accumulated   = true;

        result.publication =
            SensorPublication{gyro.measuredAt,
                              TypedPayload::store(sample, payload_names::kImuSample)};
        return result;
    };
    executable.reset = [state] {
        state->poll = ChannelPollState{};
    };
    return executable;
}

} // namespace navigatr
