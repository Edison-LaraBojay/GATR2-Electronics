// pico_telemetry.cpp

#include "impl/resources/pico_telemetry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "core/diagnostics.h"
#include "payloads/pico_telemetry_samples.h"

namespace navigatr
{

namespace
{

bool settled(const PicoRequestStatus& s) {
    return s.state == PicoRequestState::kCompleted || s.state == PicoRequestState::kFailed;
}

// Pico failure detail as a Brain link CONTROL detail.
uint8_t controlDetail(uint8_t pico_detail) {
    switch (pico_detail) {
    case gatr2::kPicoDetailWrongTarget: return gatr2::kControlDetailPicoLink;
    case gatr2::kPicoDetailImuAbsent: return gatr2::kControlDetailImuAbsent;
    default: return gatr2::kControlDetailPicoRefused;
    }
}

uint16_t randomRequestId() {
    std::random_device device;
    const uint64_t     now = static_cast<uint64_t>(steadyNowUs());
    std::seed_seq      seed{device(), device(), static_cast<unsigned>(now),
                       static_cast<unsigned>(now >> 32)};
    std::mt19937       random(seed);
    return static_cast<uint16_t>(random());
}

MonotonicTime later(MonotonicTime t, int64_t ms) { return MonotonicTime{t.ms + ms, t.domain}; }

constexpr double kMaxTimeoutS = 3600.0; // longer bounds are clamped

} // namespace

PicoTelemetry::PicoTelemetry(std::shared_ptr<SerialLink> link, std::string diagnostics_id,
                             uint16_t first_request_id)
    : link_(std::move(link)), diagnostics_id_(std::move(diagnostics_id)) {
    next_request_id_ = first_request_id != 0 ? first_request_id : randomRequestId();
    if (next_request_id_ == 0) {
        next_request_id_ = 1;
    }
}

void PicoTelemetry::reset() {
    reader_.reset();
    have_seq_        = false;
    polled_once_     = false;
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

PicoLinkState PicoTelemetry::link() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return link_state_;
}

uint32_t PicoTelemetry::submit(uint8_t op, uint8_t arg, MonotonicTime now, double timeout_s) {
    if (gatr2::picoCommandLen(op) == 0 || !(timeout_s > 0.0) || !now.isSet()) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!link_state_.identity || !link_state_.frames_fresh) {
        return 0;
    }
    Request r;
    r.handle = next_handle_++;
    if (next_handle_ == 0) {
        next_handle_ = 1;
    }
    r.request_id   = nextRequestId();
    r.op           = op;
    r.arg          = arg;
    r.target       = link_state_.boot_id;
    r.deadline     = later(now, std::llround(std::min(timeout_s, kMaxTimeoutS) * 1000.0));
    r.status.state = PicoRequestState::kSending;
    requests_.push_back(r);

    // oldest settled record first; an unsettled one only when all are
    while (requests_.size() > kRequestsKept) {
        auto victim = requests_.begin();
        for (auto it = requests_.begin(); it != requests_.end(); ++it) {
            if (settled(it->status)) {
                victim = it;
                break;
            }
        }
        requests_.erase(victim);
    }
    return r.handle;
}

PicoRequestStatus PicoTelemetry::request(uint32_t handle) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (const Request& r : requests_) {
        if (r.handle == handle) {
            return r.status;
        }
    }
    return PicoRequestStatus{};
}

uint16_t PicoTelemetry::nextRequestId() {
    const uint16_t id = next_request_id_;
    next_request_id_  = static_cast<uint16_t>(next_request_id_ + 1);
    if (next_request_id_ == 0) {
        next_request_id_ = 1;
    }
    return id;
}

void PicoTelemetry::refresh(const ExecutionContext& context) {
    if (polled_once_ && context.cycle == last_poll_cycle_) {
        return;   // already drained this cycle
    }
    polled_once_     = true;
    last_poll_cycle_ = context.cycle;
    link_dead_       = false;

    LinkStats* stats =
        context.diagnostics != nullptr ? &context.diagnostics->links[diagnostics_id_] : nullptr;

    uint8_t buf[256];
    for (;;) {
        const SerialReadResult read =
            link_->readAvailable(MutableByteSpan{buf, sizeof(buf)});
        if (read.closed) {
            link_dead_ = true;
            reader_.reset();   // a partial frame never joins bytes from a reopened device
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
            // one rescan can buffer several whole frames
            do {
                const uint8_t type = reader_.frameType();
                bool          bad  = false;
                if (type == gatr2::kFrameSensor || type == gatr2::kFrameSensorV2) {
                    gatr2::SensorSample s{};
                    bad = !gatr2::decodeSensorFrame(reader_.frame(), reader_.frameLen(), s);
                    if (!bad) {
                        applyPacket(s, context.now, stats);
                    }
                } else if (type == gatr2::kFramePicoStatus) {
                    gatr2::PicoStatus s;
                    bad = !gatr2::decodePicoStatus(reader_.frame(), reader_.frameLen(), s);
                    if (!bad) {
                        applyStatus(s, context.now);
                    }
                }
                if (bad && stats != nullptr) {
                    ++stats->decode_errors;
                }
            } while (reader_.next());
        }
    }

    sendDue(context.now, stats);

    std::lock_guard<std::mutex> lock(state_mutex_);
    publishLinkState(context.now);
}

void PicoTelemetry::applyPacket(const gatr2::SensorSample& s, MonotonicTime now,
                                LinkStats* stats) {
    const MonotonicTime stamp = deviceTime(static_cast<int64_t>(s.stamp_ms));

    // A reboot is a new boot_id, a changed frame version (new firmware), or
    // the device clock running backwards (v1, or a repeated 16-bit boot_id).
    bool reboot = false, acquisition = false, imu = false;
    if (have_frame_) {
        if (s.identity != last_identity_ || (s.identity && s.boot_id != last_boot_) ||
            stamp < last_stamp_) {
            reboot = true;
        } else if (s.identity) {
            acquisition = s.acq_epoch != last_acq_;
            imu         = s.imu_epoch != last_imu_;
        }
    }

    if (stats != nullptr) {
        ++stats->packets;
        if (have_seq_ && !reboot) {
            stats->seq_gaps += static_cast<uint8_t>(s.seq - last_seq_ - 1);
        }
    }
    have_seq_ = true;
    last_seq_ = s.seq;
    ++packets_decoded_;

    if (reboot) {
        ++reboots_;
        ++encoder_epoch_;
        ++imu_epoch_;
    }
    if (acquisition) {
        ++restarts_;
        ++encoder_epoch_;   // counters zeroed: rebase, never a displacement
    }
    if (imu) {
        ++imu_restarts_;
        ++imu_epoch_;       // encoders stay continuous
    }
    if (reboot || imu) {
        // nothing before this packet shares a gyro baseline with it
        gyro_have_prev_ = false;
        ++gyro_accum_epoch_;
    }
    have_frame_    = true;
    last_identity_ = s.identity;
    last_boot_     = s.boot_id;
    last_acq_      = s.acq_epoch;
    last_imu_      = s.imu_epoch;
    last_stamp_    = stamp;
    last_frame_at_ = now;
    if (reboot) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        failStale(s.identity, s.boot_id);
    }

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
        // Restarts and long gaps reseed instead of integrating garbage.
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

void PicoTelemetry::applyStatus(const gatr2::PicoStatus& s, MonotonicTime now) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    have_status_ = true;
    status_      = s;
    status_at_   = now;
    for (Request& r : requests_) {
        if (settled(r.status) || r.target != s.boot_id || r.request_id != s.last_request_id ||
            r.op != s.last_op) {
            continue;
        }
        switch (s.last_status) {
        case gatr2::kPicoCommandRunning:
            r.reported     = true;
            r.status.state = PicoRequestState::kRunning;
            break;
        case gatr2::kPicoCommandCompleted:
            r.reported = true;
            r.status   = PicoRequestStatus{PicoRequestState::kCompleted, gatr2::kControlDetailNone};
            break;
        case gatr2::kPicoCommandFailed:
            r.reported = true;
            r.status =
                PicoRequestStatus{PicoRequestState::kFailed, controlDetail(s.last_detail)};
            break;
        default: break;
        }
    }
}

void PicoTelemetry::failStale(bool identity, uint16_t boot_id) {
    for (Request& r : requests_) {
        if (!settled(r.status) && (!identity || r.target != boot_id)) {
            // the Pico it was meant for is gone; never resent to a new boot
            r.status = PicoRequestStatus{PicoRequestState::kFailed, gatr2::kControlDetailPicoLink};
        }
    }
}

bool PicoTelemetry::namedByStatus(const Request& r) const {
    return have_status_ && status_.boot_id == r.target &&
           status_.last_request_id == r.request_id && status_.last_op == r.op;
}

void PicoTelemetry::sendDue(MonotonicTime now, LinkStats* stats) {
    uint8_t  frame[gatr2::kMaxFrameLen];
    uint16_t len = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (Request& r : requests_) {
            if (!settled(r.status) && now >= r.deadline) {
                r.status = PicoRequestStatus{PicoRequestState::kFailed,
                                             r.reported ? gatr2::kControlDetailTimedOut
                                                        : gatr2::kControlDetailPicoLink};
            }
        }
        if (last_command_at_.isSet() && now - last_command_at_ < kCommandGapMs) {
            return;
        }
        // Unsent first, then the longest unsent. A running command the newest
        // status already names is left alone: the Pico reports it unasked.
        Request* due      = nullptr;
        int64_t  due_sent = 0;
        for (Request& r : requests_) {
            if (settled(r.status) ||
                (r.status.state == PicoRequestState::kRunning && namedByStatus(r))) {
                continue;
            }
            if (r.last_sent.isSet() && now - r.last_sent < kResendMs) {
                continue;
            }
            const int64_t sent =
                r.last_sent.isSet() ? r.last_sent.ms : std::numeric_limits<int64_t>::min();
            if (due == nullptr || sent < due_sent) {
                due      = &r;
                due_sent = sent;
            }
        }
        if (due == nullptr) {
            return;
        }
        gatr2::PicoCommand command;
        command.op             = due->op;
        command.request_id     = due->request_id;
        command.target_boot_id = due->target;
        command.imu_enabled    = due->arg;
        command.imu_port       = due->arg;
        len                    = gatr2::encodePicoCommand(command, frame, sizeof(frame));
        due->last_sent         = now;
        last_command_at_       = now;
    }
    if (len == 0) {
        return;
    }
    const SerialWriteResult written = link_->write(ByteSpan{frame, len});
    if (!written.ok && stats != nullptr) {
        ++stats->tx_errors;
    }
}

void PicoTelemetry::publishLinkState(MonotonicTime now) {
    PicoLinkState& l = link_state_;
    l.frames_fresh   = have_frame_ && !link_dead_ && now.isSet() &&
                     now - last_frame_at_ <= kFrameFreshMs;
    l.identity     = have_frame_ && last_identity_;
    l.boot_id      = l.identity ? last_boot_ : 0;
    l.acq_epoch    = l.identity ? last_acq_ : 0;
    l.imu_epoch    = l.identity ? last_imu_ : 0;
    l.reboots      = reboots_;
    l.restarts     = restarts_;
    l.imu_restarts = imu_restarts_;
    l.status_known = have_status_ && l.identity && status_.boot_id == last_boot_;
    l.status       = l.status_known ? status_ : gatr2::PicoStatus{};
    l.last_frame   = last_frame_at_;
    l.last_status  = status_at_;
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
        telemetry->refresh(context);

        ResourcePollResult result;
        if (telemetry->linkDead()) {
            result.state      = SourceState::kFault;
            result.diagnostic = "telemetry link dead";
        } else {
            result.state = telemetry->anyPacket() ? SourceState::kValid
                                                  : SourceState::kNoDataYet;
            if (telemetry->anyPacket() && !telemetry->identity()) {
                result.diagnostic = "v1 sensor frames: no Pico identity, reboots found by clock "
                                    "regression, commands unavailable";
            }
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
                publication.upstream.epoch    = out.encoder >= 0 ? telemetry->encoderEpoch()
                                                                 : telemetry->imuEpoch();
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
