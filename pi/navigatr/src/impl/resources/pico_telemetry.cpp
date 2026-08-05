// pico_telemetry.cpp

#include "impl/resources/pico_telemetry.h"

#include "core/diagnostics.h"

namespace navigatr
{

PicoTelemetry::PicoTelemetry(std::shared_ptr<SerialLink> link, std::string diagnostics_id)
    : link_(std::move(link)), diagnostics_id_(std::move(diagnostics_id)) {}

void PicoTelemetry::reset() {
    reader_.reset();
    have_seq_    = false;
    polled_once_ = false;
    for (Channel& c : encoders_) {
        c = Channel{};
    }
    gyro_  = Channel{};
    accel_ = Channel{};
}

void PicoTelemetry::refresh(uint64_t cycle, Diagnostics* diagnostics) {
    if (polled_once_ && cycle == last_poll_cycle_) {
        return;   // another channel sensor already drained this cycle
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

    const MonotonicTime stamp = deviceTime(static_cast<int64_t>(s.stamp_ms));

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
        update(gyro_, s.gyro_z, 0);
    }
    if (s.mask & gatr2::kSensorAccelXY) {
        update(accel_, s.accel[0], s.accel[1]);
    }
}

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
    return ResourceInstance::asContract<PicoTelemetry>(
        std::make_shared<PicoTelemetry>(std::move(link), link_id.value));
}

} // namespace navigatr
