// pico_telemetry.cpp

#include "impl/resources/pico_telemetry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include "core/diagnostics.h"
#include "core/host_clock.h"
#include "diagnostics/hub.h"
#include "impl/resources/serial_links.h"
#include "payloads/pico_telemetry_samples.h"

namespace navigatr
{

const char* picoDiagStateName(PicoDiagState s) {
    switch (s) {
    case PicoDiagState::kNotRequested: return "not requested";
    case PicoDiagState::kNoLink: return "no link";
    case PicoDiagState::kNoIdentity: return "no identity";
    case PicoDiagState::kRequesting: return "requesting";
    case PicoDiagState::kActive: return "active";
    case PicoDiagState::kFirmware: return "firmware";
    case PicoDiagState::kRefused: return "refused";
    }
    return "unknown";
}

namespace
{

const char* picoOpName(uint8_t op) {
    switch (op) {
    case translagatr::kPicoOpConfigure: return "CONFIGURE";
    case translagatr::kPicoOpReinitImu: return "REINIT_IMU";
    case translagatr::kPicoOpRestartAcquisition: return "RESTART_ACQUISITION";
    case translagatr::kPicoOpDiagnostics: return "DIAGNOSTICS";
    default: return "command";
    }
}

bool settled(const PicoRequestStatus& s) {
    return s.state == PicoRequestState::kCompleted || s.state == PicoRequestState::kFailed;
}

// Pico failure detail as a Brain link CONTROL detail.
uint8_t controlDetail(uint8_t pico_detail) {
    switch (pico_detail) {
    case translagatr::kPicoDetailWrongTarget: return translagatr::kControlDetailPicoLink;
    case translagatr::kPicoDetailImuAbsent: return translagatr::kControlDetailImuAbsent;
    default: return translagatr::kControlDetailPicoRefused;
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
    reopening_       = dynamic_cast<ReopeningLink*>(link_.get());
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
    for (EncoderHistory& h : encoder_history_) {
        h.samples.clear();
    }
}

void PicoTelemetry::attachDiagnostics(DiagnosticsHub* hub) {
    hub_ = hub;
    if (hub_ == nullptr) {
        monitor_.reset();
        return;
    }
    source_id_ = hub_->sourceId(diagnostics_id_);
    monitor_   = hub_->links().monitor(diagnostics_id_, "pico_uart");
}

void PicoTelemetry::setDiagnosticsRate(uint8_t hz) {
    diag_hz_    = std::min(hz, kMaxDiagHz);
    diag_state_ = diag_hz_ == 0 ? PicoDiagState::kNotRequested : PicoDiagState::kNoLink;
}

PicoInstrumentation PicoTelemetry::instrumentation() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return view_;
}

void PicoTelemetry::noteFrame(const char* name, const std::string& fields) {
    if (monitor_ != nullptr) {
        monitor_->frame(true, name, fields);
    }
}

void PicoTelemetry::postSensor(const translagatr::SensorSample& s) {
    if (hub_ == nullptr || !hub_->wants(DiagKind::kPicoSensor)) {
        return;
    }
    DiagPicoSensor p;
    p.version     = s.identity ? 2 : 1;
    p.boot_id     = s.boot_id;
    p.acq_epoch   = s.acq_epoch;
    p.imu_epoch   = s.imu_epoch;
    p.seq         = s.seq;
    p.mask        = static_cast<uint8_t>(s.mask);
    p.stamp_ms    = s.stamp_ms;
    p.enc[0]      = s.enc[0];
    p.enc[1]      = s.enc[1];
    p.enc[2]      = s.enc[2];
    p.gyro_z_mdps = s.gyro_z;
    DiagRecord r;
    r.kind    = DiagKind::kPicoSensor;
    r.source  = source_id_;
    r.payload = p;
    hub_->post(std::move(r));
}

// Keeps about kEncoderWindowMs of (host ms, counts) per updated channel.
// A rebase (new boot or acquisition epoch) drops the history: counts were
// zeroed, so a difference across it is not motion. So does a silence longer
// than the window: a change across it is not a one second change.
void PicoTelemetry::noteEncoders(MonotonicTime now, uint16_t mask, bool rebase) {
    const int64_t host_us = HostClock::nowUs();
    for (int ch = 0; ch < kEncoderChannels; ++ch) {
        EncoderHistory& h = encoder_history_[ch];
        if (rebase) {
            h.samples.clear();
        }
        if ((mask & (1u << ch)) == 0) {
            continue;
        }
        if (h.updated_ms >= 0 && now.ms - h.updated_ms > kEncoderWindowMs) {
            h.samples.clear();
        }
        h.updated_ms      = now.ms;
        h.updated_host_us = host_us;
        if (!h.samples.empty() && now.ms - h.samples.back().first < 50) {
            h.samples.back().second = encoders_[ch].value[0];   // thin to 20 per second
        } else {
            h.samples.emplace_back(now.ms, encoders_[ch].value[0]);
        }
        while (h.samples.size() > 2 && now.ms - h.samples[1].first >= kEncoderWindowMs) {
            h.samples.pop_front();
        }
        if (h.samples.size() > 64) {
            h.samples.pop_front();
        }
    }
}

PicoLinkState PicoTelemetry::link() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return link_state_;
}

uint32_t PicoTelemetry::submit(uint8_t op, uint8_t arg, MonotonicTime now, double timeout_s) {
    if (translagatr::picoCommandLen(op) == 0 || !(timeout_s > 0.0) || !now.isSet()) {
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

    const bool decoded = monitor_ != nullptr && monitor_->decodedOn();
    uint8_t    buf[256];
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
        if (monitor_ != nullptr) {
            monitor_->rx(buf, read.bytes);
        }
        for (std::size_t i = 0; i < read.bytes; ++i) {
            if (!reader_.push(buf[i])) {
                continue;
            }
            // one rescan can buffer several whole frames
            do {
                const uint8_t type = reader_.frameType();
                const char*   bad  = nullptr;
                char          fields[160];
                fields[0] = 0;
                if (type == translagatr::kFrameSensor || type == translagatr::kFrameSensorV2) {
                    translagatr::SensorSample s{};
                    if (!translagatr::decodeSensorFrame(reader_.frame(), reader_.frameLen(), s)) {
                        bad = "sensor frame decode";
                    } else {
                        applyPacket(s, context.now, stats);
                        ++sensor_frames_;
                        postSensor(s);
                        if (decoded) {
                            std::snprintf(fields, sizeof(fields),
                                          "seq=%u t=%lu mask=0x%02X enc=%ld,%ld,%ld gyro_mdps=%ld",
                                          static_cast<unsigned>(s.seq),
                                          static_cast<unsigned long>(s.stamp_ms),
                                          static_cast<unsigned>(s.mask), static_cast<long>(s.enc[0]),
                                          static_cast<long>(s.enc[1]), static_cast<long>(s.enc[2]),
                                          static_cast<long>(s.gyro_z));
                        }
                        noteFrame(s.identity ? "sensor v2" : "sensor v1", fields);
                    }
                } else if (type == translagatr::kFramePicoStatus) {
                    translagatr::PicoStatus s;
                    if (!translagatr::decodePicoStatus(reader_.frame(), reader_.frameLen(), s)) {
                        bad = "status frame decode";
                    } else {
                        applyStatus(s, context.now);
                        ++status_frames_;
                        if (hub_ != nullptr) {
                            DiagRecord r;
                            r.kind    = DiagKind::kPicoStatus;
                            r.source  = source_id_;
                            r.payload = DiagPicoStatus{s};
                            hub_->post(std::move(r));
                        }
                        if (decoded) {
                            std::snprintf(fields, sizeof(fields),
                                          "boot=%04X imu_state=%u reason=%u last=%u/%u/%u/%u",
                                          static_cast<unsigned>(s.boot_id),
                                          static_cast<unsigned>(s.imu_state),
                                          static_cast<unsigned>(s.imu_reason),
                                          static_cast<unsigned>(s.last_request_id),
                                          static_cast<unsigned>(s.last_op),
                                          static_cast<unsigned>(s.last_status),
                                          static_cast<unsigned>(s.last_detail));
                        }
                        noteFrame("status", fields);
                    }
                } else if (type == translagatr::kFramePicoDiag) {
                    translagatr::PicoDiag d;
                    if (!translagatr::decodePicoDiag(reader_.frame(), reader_.frameLen(), d)) {
                        bad = "diag frame decode";
                    } else {
                        applyDiag(d, reader_.frame() + 4);
                        if (decoded) {
                            std::snprintf(fields, sizeof(fields),
                                          "seq=%u pins=0x%04X known=0x%04X imu_rx=%u bad=%u "
                                          "rx_bad=%u skipped=%u",
                                          static_cast<unsigned>(d.seq),
                                          static_cast<unsigned>(d.pins),
                                          static_cast<unsigned>(d.pins_known),
                                          static_cast<unsigned>(d.imu_rx),
                                          static_cast<unsigned>(d.imu_bad),
                                          static_cast<unsigned>(d.link_rx_bad),
                                          static_cast<unsigned>(d.ticks_skipped));
                        }
                        noteFrame("diag", fields);
                    }
                } else {
                    bad = "not a Pico frame";
                }
                if (bad != nullptr) {
                    if (stats != nullptr) {
                        ++stats->decode_errors;
                    }
                    if (monitor_ != nullptr) {
                        monitor_->rejected(bad);
                    }
                }
            } while (reader_.next());
        }
    }
    if (monitor_ != nullptr) {
        monitor_->readerStats(reader_.stats());
    }

    sendDue(context.now, stats);

    std::lock_guard<std::mutex> lock(state_mutex_);
    publishLinkState(context.now);
    requestDiagnostics(context.now);
    view_.diag_hz       = diag_hz_;
    view_.diag_state    = diag_state_;
    view_.have_diag     = have_diag_;
    view_.diag          = diag_;
    view_.diag_host_us  = diag_host_us_;
    view_.diag_frames   = diag_frames_;
    view_.sensor_frames = sensor_frames_;
    view_.status_frames = status_frames_;
    // the link is single threaded: copied here, on its own thread
    view_.serial_reopening = reopening_ != nullptr;
    if (reopening_ != nullptr) {
        view_.serial_open     = reopening_->isOpen();
        view_.serial_attempts = reopening_->attempts();
        view_.serial_reopens  = reopening_->reopens();
        view_.serial_closes   = reopening_->closes();
        view_.serial_error    = reopening_->lastError();
    }
    for (int ch = 0; ch < kEncoderChannels; ++ch) {
        PicoEncoderView&      v = view_.encoders[ch];
        const EncoderHistory& h = encoder_history_[ch];
        v.present               = encoders_[ch].present;
        v.counts                = encoders_[ch].value[0];
        v.updated_host_us       = h.updated_host_us;
        // a change is current only while the channel still updates
        v.fresh                 = h.updated_ms >= 0 && context.now.isSet() &&
                                  context.now.ms - h.updated_ms <= kFrameFreshMs;
        v.delta_known           = v.fresh && h.samples.size() >= 2;
        v.delta   = v.delta_known ? h.samples.back().second - h.samples.front().second : 0;
        v.span_ms = v.delta_known ? h.samples.back().first - h.samples.front().first : 0;
    }
}

void PicoTelemetry::applyDiag(const translagatr::PicoDiag& d, const uint8_t* payload) {
    have_diag_    = true;
    diag_         = d;
    diag_host_us_ = HostClock::nowUs();
    ++diag_frames_;
    if (hub_ != nullptr) {
        DiagPicoDiag p;
        p.len = translagatr::kPicoDiagLen;
        std::memcpy(p.payload, payload, translagatr::kPicoDiagLen);
        DiagRecord r;
        r.kind    = DiagKind::kPicoDiag;
        r.source  = source_id_;
        r.payload = p;
        hub_->post(std::move(r));
    }
}

// Caller holds state_mutex_, after publishLinkState. A Pico keeps the rate
// its boot was given until it reboots, so with no rate configured here,
// diagnostic frames of the current boot were asked for by an earlier Pi
// process (a restart with another configuration): that boot is asked once,
// the same way, for rate 0. That is not a request, so the state stays
// "not requested".
void PicoTelemetry::requestDiagnostics(MonotonicTime now) {
    const PicoLinkState& l = link_state_;
    const bool unasked = diag_hz_ == 0 && have_diag_ && l.identity && diag_.boot_id == l.boot_id;
    if (diag_hz_ != 0 || unasked) {
        driveDiagnostics(now);
    }
    if (diag_hz_ == 0) {
        diag_state_ = PicoDiagState::kNotRequested;
    }
}

// Asks each Pico boot once for rate diag_hz_; an unanswered attempt is
// retried after kDiagRetryMs, a refusal waits for the next boot.
void PicoTelemetry::driveDiagnostics(MonotonicTime now) {
    const PicoLinkState& l = link_state_;
    if (diag_handle_ != 0) {
        const Request* r = nullptr;
        for (const Request& q : requests_) {
            if (q.handle == diag_handle_) {
                r = &q;
            }
        }
        if (r == nullptr || r->target != (l.identity ? l.boot_id : 0)) {
            diag_handle_ = 0;   // evicted, or failed by a reboot: ask the new boot
        } else if (r->status.state == PicoRequestState::kCompleted) {
            diag_handle_  = 0;
            diag_settled_ = true;
            diag_state_   = PicoDiagState::kActive;
        } else if (r->status.state == PicoRequestState::kFailed) {
            diag_handle_ = 0;
            if (r->reported && r->pico_detail == translagatr::kPicoDetailUnknownOp) {
                diag_settled_ = true;
                diag_state_   = PicoDiagState::kFirmware;
            } else if (r->reported && r->status.detail != translagatr::kControlDetailTimedOut) {
                diag_settled_ = true;
                diag_state_   = PicoDiagState::kRefused;
            } else {
                diag_retry_at_ = later(now, kDiagRetryMs);   // never answered
            }
        }
    }
    if (!l.frames_fresh) {
        if (diag_handle_ == 0 && !diag_settled_) {
            diag_state_ = PicoDiagState::kNoLink;
        }
        return;
    }
    if (!l.identity) {
        diag_state_ = PicoDiagState::kNoIdentity;
        return;
    }
    if (diag_settled_ && diag_boot_ != l.boot_id) {
        diag_settled_ = false;   // a new boot starts with diagnostics off
    }
    if (diag_handle_ != 0 || diag_settled_ ||
        (diag_retry_at_.isSet() && now < diag_retry_at_)) {
        if (!diag_settled_) {
            diag_state_ = PicoDiagState::kRequesting;
        }
        return;
    }
    // submit() takes state_mutex_; this runs with it held, so add directly.
    Request q;
    q.handle = next_handle_++;
    if (next_handle_ == 0) {
        next_handle_ = 1;
    }
    q.request_id   = nextRequestId();
    q.op           = translagatr::kPicoOpDiagnostics;
    q.arg          = diag_hz_;
    q.target       = l.boot_id;
    q.deadline     = later(now, std::llround(kDiagTimeoutS * 1000.0));
    q.status.state = PicoRequestState::kSending;
    requests_.push_back(q);
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
    diag_handle_   = q.handle;
    diag_boot_     = l.boot_id;
    diag_retry_at_ = MonotonicTime{};
    diag_state_    = PicoDiagState::kRequesting;
}

void PicoTelemetry::applyPacket(const translagatr::SensorSample& s, MonotonicTime now,
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
    if (s.mask & translagatr::kSensorGyroZ) {
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
    if (s.mask & translagatr::kSensorAccelXY) {
        update(accel_, s.accel[0], s.accel[1]);
    }
    noteEncoders(now, static_cast<uint16_t>(s.mask & 0x7u), reboot || acquisition);
}

void PicoTelemetry::applyStatus(const translagatr::PicoStatus& s, MonotonicTime now) {
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
        case translagatr::kPicoCommandRunning:
            r.reported     = true;
            r.status.state = PicoRequestState::kRunning;
            break;
        case translagatr::kPicoCommandCompleted:
            r.reported = true;
            r.status   = PicoRequestStatus{PicoRequestState::kCompleted, translagatr::kControlDetailNone};
            break;
        case translagatr::kPicoCommandFailed:
            r.reported    = true;
            r.pico_detail = s.last_detail;
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
            r.status = PicoRequestStatus{PicoRequestState::kFailed, translagatr::kControlDetailPicoLink};
        }
    }
}

bool PicoTelemetry::namedByStatus(const Request& r) const {
    return have_status_ && status_.boot_id == r.target &&
           status_.last_request_id == r.request_id && status_.last_op == r.op;
}

void PicoTelemetry::sendDue(MonotonicTime now, LinkStats* stats) {
    uint8_t  frame[translagatr::kMaxFrameLen];
    uint16_t len = 0;
    uint8_t  op  = 0;
    uint16_t rid = 0;
    uint8_t  arg = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (Request& r : requests_) {
            if (!settled(r.status) && now >= r.deadline) {
                r.status = PicoRequestStatus{PicoRequestState::kFailed,
                                             r.reported ? translagatr::kControlDetailTimedOut
                                                        : translagatr::kControlDetailPicoLink};
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
        translagatr::PicoCommand command;
        command.op             = due->op;
        command.request_id     = due->request_id;
        command.target_boot_id = due->target;
        command.imu_enabled    = due->arg;
        command.imu_port       = due->arg;
        command.diag_hz        = due->arg;
        len                    = translagatr::encodePicoCommand(command, frame, sizeof(frame));
        due->last_sent         = now;
        last_command_at_       = now;
        op                     = due->op;
        rid                    = due->request_id;
        arg                    = due->arg;
    }
    if (len == 0) {
        return;
    }
    const SerialWriteResult written = link_->write(ByteSpan{frame, len});
    if (!written.ok && stats != nullptr) {
        ++stats->tx_errors;
    }
    if (monitor_ != nullptr) {
        // a SerialLink write is whole or nothing
        monitor_->tx(frame, len, written.ok ? len : 0);
        std::string fields;
        if (monitor_->decodedOn()) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "rid=%u arg=%u%s", static_cast<unsigned>(rid),
                          static_cast<unsigned>(arg), written.ok ? "" : " write failed");
            fields = buf;
        }
        monitor_->frame(false, picoOpName(op), fields);
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
    l.status       = l.status_known ? status_ : translagatr::PicoStatus{};
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
    long diag_hz = 0;
    if (!node.atMostOne("Diagnostics", err)) {
        return ResourceInstance{};
    }
    const ConfigNode diagnostics = node.child("Diagnostics");
    if (diagnostics.valid()) {
        if (!diagnostics.onlyAttributes({"hz"}, err) || !diagnostics.onlyChildren({}, err) ||
            !diagnostics.requireInt("hz", diag_hz, err)) {
            return ResourceInstance{};
        }
        if (diag_hz < 0 || diag_hz > PicoTelemetry::kMaxDiagHz) {
            err = diagnostics.path() + ": hz must be 0.." +
                  std::to_string(PicoTelemetry::kMaxDiagHz);
            return ResourceInstance{};
        }
    }

    auto telemetry = std::make_shared<PicoTelemetry>(std::move(link), link_id.value);
    telemetry->attachDiagnostics(context.diagnostics);
    telemetry->setDiagnosticsRate(static_cast<uint8_t>(diag_hz));

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
