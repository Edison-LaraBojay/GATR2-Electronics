// pico_telemetry.cpp

#include "impl/resources/pico_telemetry.h"

#include <vector>

#include "core/diagnostics.h"
#include "payloads/pico_telemetry_samples.h"

namespace navigatr
{

PicoTelemetry::PicoTelemetry(std::shared_ptr<SerialLink> link, std::string diagnostics_id)
    : link_(std::move(link)), diagnostics_id_(std::move(diagnostics_id)) {}

void PicoTelemetry::reset() {
    reader_.reset();
    have_seq_    = false;
    have_stamp_  = false;
    polled_once_ = false;
    packets_decoded_ = 0;
    for (Channel& c : encoders_) {
        c = Channel{};
    }
    gyro_             = Channel{};
    accel_            = Channel{};
    gyro_accum_raw_   = 0.0;
    gyro_accum_epoch_ = 0;
    gyro_have_prev_   = false;
}

void PicoTelemetry::refresh(uint64_t cycle, Diagnostics* diagnostics) {
    if (polled_once_ && cycle == last_poll_cycle_) {
        return;   // already drained this cycle
    }
    polled_once_     = true;
    last_poll_cycle_ = cycle;
    link_dead_       = false;

    LinkStats* stats =
        diagnostics != nullptr ? &diagnostics->links[diagnostics_id_] : nullptr;

    uint8_t buf[256];
    for (;;) {
        const SerialReadResult read =
            link_->readAvailable(MutableByteSpan{buf, sizeof(buf)});
        if (read.closed) {
            link_dead_ = true;
            break;
        }
        if (read.bytes == 0) {
            break;
        }
        if (stats != nullptr) {
            stats->bytes += static_cast<uint32_t>(read.bytes);
        }
        for (std::size_t i = 0; i < read.bytes; ++i) {
            if (!reader_.push(buf[i])) {
                continue;
            }
            if (reader_.frameType() != gatr2::kFrameSensor) {
                continue;
            }
            gatr2::SensorSample s{};
            if (!gatr2::decodeSensorFrame(reader_.frame(), reader_.frameLen(), s)) {
                if (stats != nullptr) {
                    ++stats->decode_errors;
                }
                continue;
            }
            applyPacket(s, diagnostics);
        }
    }
}

void PicoTelemetry::applyPacket(const gatr2::SensorSample& s, Diagnostics* diagnostics) {
    if (diagnostics != nullptr) {
        LinkStats& stats = diagnostics->links[diagnostics_id_];
        ++stats.packets;
        if (have_seq_) {
            stats.seq_gaps += static_cast<uint8_t>(s.seq - last_seq_ - 1);
        }
    }
    have_seq_ = true;
    last_seq_ = s.seq;
    ++packets_decoded_;

    const MonotonicTime stamp = deviceTime(static_cast<int64_t>(s.stamp_ms));
    if (have_stamp_ && stamp < last_stamp_) {
        // the device clock restarted: nothing before this packet shares a
        // baseline with anything after it
        ++device_epoch_;
        gyro_have_prev_ = false;
        ++gyro_accum_epoch_;
    }
    have_stamp_ = true;
    last_stamp_ = stamp;

    const auto update = [&](Channel& c, int32_t v0, int32_t v1) {
        c.present    = true;
        c.value[0]   = v0;
        c.value[1]   = v1;
        c.measuredAt = stamp;
        ++c.updates;
    };

    for (int ch = 0; ch < kEncoderChannels; ++ch) {
        if (s.mask & (1u << ch)) {
            update(encoders_[ch], s.enc[ch], 0);
        }
    }
    if (s.mask & gatr2::kSensorGyroZ) {
        // Integrate every decoded packet so batching drops no rotation.
        // Device reboots (stamp regression) and long gaps reseed instead of
        // integrating garbage.
        if (gyro_have_prev_) {
            const int64_t gap_ms = stamp.ms - gyro_prev_stamp_.ms;
            if (gap_ms > 0 && gap_ms <= kGyroGapMs) {
                gyro_accum_raw_ += 0.5 *
                                   (static_cast<double>(gyro_prev_raw_) +
                                    static_cast<double>(s.gyro_z)) *
                                   (static_cast<double>(gap_ms) / 1000.0);
            } else {
                ++gyro_accum_epoch_;   // dropped interval, not zero rotation
            }
        }
        gyro_have_prev_  = true;
        gyro_prev_raw_   = s.gyro_z;
        gyro_prev_stamp_ = stamp;
        update(gyro_, s.gyro_z, 0);
    }
    if (s.mask & gatr2::kSensorAccelXY) {
        update(accel_, s.accel[0], s.accel[1]);
    }
}

namespace
{

// One configured <Output>: which decoded channel it publishes under which
// id, plus what has already been published.
struct ConfiguredOutput {
    OutputId id;
    int      encoder = -1;   // 0..2, or -1 for the gyro
    uint64_t last_updates = 0;
};

} // namespace

ResourceInstance make_pico_telemetry(const ConfigNode& node,
                                     ResourceInitializationContext& context,
                                     std::string& err) {
    const ConfigNode serial = node.child("Serial");
    const ResourceId link_id{serial.attr("resource_id")};
    if (link_id.empty()) {
        err = "pico_telemetry needs <Serial resource_id=.../>";
        return ResourceInstance{};
    }
    auto link = context.require<SerialLink>(link_id, err);
    if (link == nullptr) {
        return ResourceInstance{};
    }

    auto outputs = std::make_shared<std::vector<ConfiguredOutput>>();
    ResourceExecutable executable;
    bool               ok = true;
    node.forEach("Output", [&](const ConfigNode& o) {
        if (!ok) {
            return;
        }
        ConfiguredOutput out;
        std::string      channel;
        out.id = OutputId{o.attr("id")};
        if (out.id.empty() || !o.requireAttr("channel", channel, err)) {
            if (err.empty()) {
                err = o.path() + ": Output needs id and channel";
            }
            ok = false;
            return;
        }
        for (const ConfiguredOutput& seen : *outputs) {
            if (seen.id == out.id) {
                err = o.path() + ": duplicate Output id " + out.id.value;
                ok  = false;
                return;
            }
        }
        if (channel == "imu") {
            executable.outputs.push_back(ResourceOutputDecl{
                out.id, PayloadDescriptor::of<PicoGyroRate>(payload_names::kPicoGyroRate)});
        } else {
            long index = -1;
            if (!o.getInt("channel", -1, index, err)) {
                ok = false;
                return;
            }
            if (index < 0 || index >= PicoTelemetry::kEncoderChannels) {
                err = o.path() + ": channel must be 0.." +
                      std::to_string(PicoTelemetry::kEncoderChannels - 1) + " or imu";
                ok = false;
                return;
            }
            out.encoder = static_cast<int>(index);
            executable.outputs.push_back(
                ResourceOutputDecl{out.id, PayloadDescriptor::of<PicoEncoderCounts>(
                                               payload_names::kPicoEncoderCounts)});
        }
        outputs->push_back(out);
    });
    if (!ok) {
        return ResourceInstance{};
    }

    auto telemetry = std::make_shared<PicoTelemetry>(std::move(link), link_id.value);

    executable.execute = [telemetry, outputs](const ExecutionContext& context) {
        telemetry->refresh(context.cycle, context.diagnostics);

        ResourcePollResult result;
        if (telemetry->linkDead()) {
            result.state      = SourceState::kFault;
            result.diagnostic = "telemetry link dead";
        } else {
            result.state = telemetry->anyPacket() ? SourceState::kValid
                                                  : SourceState::kNoDataYet;
        }

        for (ConfiguredOutput& out : *outputs) {
            const PicoTelemetry::Channel& channel =
                out.encoder >= 0 ? telemetry->encoder(out.encoder) : telemetry->gyro();
            OutputPoll poll;
            poll.id = out.id;
            if (channel.updates != out.last_updates) {
                out.last_updates  = channel.updates;
                poll.result.state = SourceState::kValid;
                Publication publication;
                publication.measuredAt        = channel.measuredAt;
                publication.upstream.source   = "pico:" + telemetry->clockId();
                publication.upstream.clock    = telemetry->clockId();
                publication.upstream.sequence = telemetry->packetsDecoded();
                publication.upstream.epoch    = telemetry->deviceEpoch();
                if (out.encoder >= 0) {
                    publication.payload = TypedPayload::store(
                        PicoEncoderCounts{channel.value[0]},
                        payload_names::kPicoEncoderCounts);
                } else {
                    PicoGyroRate rate;
                    rate.rate_mdps         = channel.value[0];
                    rate.accumulated_mdeg  = telemetry->gyroAccumulatedRaw();
                    rate.accumulated_epoch = telemetry->gyroAccumulatedEpoch();
                    publication.payload =
                        TypedPayload::store(rate, payload_names::kPicoGyroRate);
                }
                poll.result.publication = std::move(publication);
            } else if (telemetry->linkDead()) {
                // data decoded before a link death still counted; the fault
                // shows on the first poll with nothing new
                poll.result.state      = SourceState::kFault;
                poll.result.diagnostic = "telemetry link dead";
            } else if (!channel.present) {
                poll.result.state = SourceState::kNoDataYet;
            } else {
                poll.result.state = SourceState::kValid;   // healthy, nothing new
            }
            result.outputs.push_back(std::move(poll));
        }
        return result;
    };
    executable.reset = [telemetry, outputs] {
        telemetry->reset();
        for (ConfiguredOutput& out : *outputs) {
            out.last_updates = 0;
        }
    };

    auto instance = ResourceInstance::asContract<PicoTelemetry>(telemetry);
    instance.setExecutable(std::move(executable));
    return instance;
}

} // namespace navigatr
