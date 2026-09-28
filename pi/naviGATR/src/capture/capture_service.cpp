// capture_service.cpp
// The capture recorder (capture_recorder.h) and its System host.

#include "capture/capture_recorder.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "core/host_clock.h"
#include "inspection/json_writer.h"
#include "runtime/brain_profile_builder.h"
#include "runtime/system.h"

namespace navigatr
{
namespace
{

constexpr int64_t kHourUs          = 3600LL * 1000000LL;
constexpr int64_t kHubSampleUs     = 100000; // hub counter samples, for drops since a window start
constexpr int64_t kHostSampleUs    = 1000000; // host metadata samples, for the state at a window start
constexpr std::size_t kFaultsQueued = 16;    // fault notes between two pumps
constexpr std::size_t kResetsKept   = 64;

int64_t toUs(double s) { return static_cast<int64_t>(std::llround(s * 1e6)); }

DiagKindCounts minus(const DiagKindCounts& a, const DiagKindCounts& b) {
    DiagKindCounts d{};
    for (int k = 0; k < kDiagKindCount; ++k) {
        d[k] = a[k] >= b[k] ? a[k] - b[k] : 0;
    }
    return d;
}

uint64_t total(const DiagKindCounts& c) {
    uint64_t n = 0;
    for (uint64_t v : c) {
        n += v;
    }
    return n;
}

void writeKinds(JsonWriter& w, const char* key, uint32_t kinds) {
    w.key(key);
    w.beginArray();
    for (int k = 0; k < kDiagKindCount; ++k) {
        if ((kinds & (1u << k)) != 0) {
            w.value(diagKindName(static_cast<DiagKind>(k)));
        }
    }
    w.endArray();
}

void writeCountsByKind(JsonWriter& w, const char* key, const DiagKindCounts& c) {
    w.key(key);
    w.beginObject();
    for (int k = 0; k < kDiagKindCount; ++k) {
        if (c[k] != 0) {
            w.field(diagKindName(static_cast<DiagKind>(k)), c[k]);
        }
    }
    w.endObject();
}

// ---- the System as the recorder sees it ------------------------------------------

const char* imuSourceName(uint8_t s) {
    switch (s) {
    case translagatr::kImuSourceNone: return "none";
    case translagatr::kImuSourcePico: return "pico";
    case translagatr::kImuSourceBrainVex: return "brain_vex";
    default: return "unknown";
    }
}

class SystemCaptureHost final : public CaptureHost
{
public:
    explicit SystemCaptureHost(const System& system) : system_(system) {}

    std::string sessionId() const override { return system_.sessionId(); }
    uint64_t    resetCount() const override { return system_.resetCount(); }

    void writeMetadata(JsonWriter& w) const override {
        char digest[20];
        std::snprintf(digest, sizeof(digest), "%016llx",
                      static_cast<unsigned long long>(system_.configurationDigest()));
        w.key("configuration");
        w.beginObject();
        w.field("id", system_.configurationId());
        w.field("name", system_.configurationName());
        w.field("digest", digest);
        w.field("loop_rate_hz", system_.loopRateHz());
        w.field("commands", system_.commandsType());
        const std::shared_ptr<const BindingView> view = system_.bindingView();
        if (view != nullptr) {
            w.field("estimator", view->estimator_type);
            w.field("takes_brain_profile", view->brain_profile);
            w.key("warnings");
            w.beginArray();
            for (const std::string& s : view->warnings) {
                w.value(s);
            }
            w.endArray();
        }
        w.endObject();

        writeProfile(w, system_.profileBinding());
        writeLocalization(w);
        writePico(w);

        w.key("software");
        w.beginObject();
        w.field("program", "naviGATR");
        w.field("capture_module_built", __DATE__ " " __TIME__);
#if defined(__VERSION__)
        w.field("compiler", __VERSION__);
#endif
        w.key("contracts");
        w.beginObject();
        w.field("capture", kCaptureSchema);
        w.field("inspection", "navigatr.inspect/2");
        w.field("brain_link_version", static_cast<int>(translagatr::kBrainLinkVersion));
        w.field("pico_link_version", static_cast<int>(translagatr::kPicoLinkVersion));
        w.endObject();
        w.endObject();
    }

private:
    void writeProfile(JsonWriter& w, const std::shared_ptr<const ProfileBinding>& b) const {
        if (b == nullptr) {
            w.fieldNull("profile");
            return;
        }
        const translagatr::RobotProfileDoc& p = b->profile;
        char id[12];
        std::snprintf(id, sizeof(id), "%08x", static_cast<unsigned>(b->id));
        w.key("profile");
        w.beginObject();
        w.field("id", id);
        w.field("generation", b->generation);
        w.field("summary", b->summary);
        w.field("topology", profileTopologyName(p.topology));
        w.field("imu_source", imuSourceName(p.imu_source));
        w.field("imu_port", static_cast<int>(p.imu_port));
        w.field("vex_smart_port", static_cast<int>(p.vex_smart_port));
        w.field("imu_inverted", (p.imu_flags & translagatr::kImuInvert) != 0);
        w.key("footprint_m");
        w.beginObject();
        w.field("front", p.footprint_front_um * 1e-6);
        w.field("back", p.footprint_back_um * 1e-6);
        w.field("left", p.footprint_left_um * 1e-6);
        w.field("right", p.footprint_right_um * 1e-6);
        w.endObject();
        w.key("calibration_settings");
        w.beginObject();
        w.field("window_ms", static_cast<int>(p.calibration_window_ms));
        w.field("still_rate_deg_s", p.still_rate_cdps / 100.0);
        w.field("still_travel_m", p.still_travel_um * 1e-6);
        w.field("zero_means", "the Pi default");
        w.endObject();
        w.key("wheels");
        w.beginArray();
        for (uint8_t i = 0; i < p.wheel_count && i < translagatr::kProfileMaxWheels; ++i) {
            const translagatr::ProfileWheel& wh = p.wheels[i];
            w.beginObject();
            w.field("encoder_port", static_cast<int>(wh.encoder_port));
            w.field("counts_per_rev", static_cast<uint64_t>(wh.counts_per_rev));
            w.field("gear", wh.gear_micro * 1e-6);
            w.field("radius_m", wh.radius_um * 1e-6);
            w.field("x_m", wh.x_um * 1e-6);
            w.field("y_m", wh.y_um * 1e-6);
            w.field("angle_deg", wh.angle_mdeg * 1e-3);
            w.field("reversed", (wh.flags & translagatr::kWheelReversed) != 0);
            w.field("travel_scale", wh.travel_scale_ppm * 1e-6);
            if (i < b->encoders.size()) {
                w.field("sensor", b->encoders[i].value);
            }
            w.endObject();
        }
        w.endArray();
        w.key("cameras");
        w.beginArray();
        for (uint8_t i = 0; i < p.camera_count && i < translagatr::kProfileMaxCameras; ++i) {
            const translagatr::ProfileCamera& c = p.cameras[i];
            w.beginObject();
            w.field("slot", static_cast<int>(c.slot));
            w.field("x_m", c.x_um * 1e-6);
            w.field("y_m", c.y_um * 1e-6);
            w.field("z_m", c.z_um * 1e-6);
            w.field("roll_deg", c.roll_mdeg * 1e-3);
            w.field("pitch_deg", c.pitch_mdeg * 1e-3);
            w.field("yaw_deg", c.yaw_mdeg * 1e-3);
            w.endObject();
        }
        w.endArray();
        w.field("bias_function", b->bias_function);
        w.endObject();
    }

    void writeLocalization(JsonWriter& w) const {
        const std::shared_ptr<RobotStateFeed> feed = system_.robotFeed();
        if (feed == nullptr) {
            w.fieldNull("localization");
            return;
        }
        const LocalizationStatus s = feed->status();
        w.key("localization");
        w.beginObject();
        w.field("estimator", s.estimator_type);
        w.field("all_ready", s.allReady());
        w.field("stationary", s.stationary());
        w.field("clock_mapped", s.clock_mapped);
        w.field("continuity_breaks", s.continuity_breaks);
        w.field("last_break", s.last_break);
        w.key("functions");
        w.beginArray();
        for (const ObservationFunctionStatus& f : s.functions) {
            w.beginObject();
            w.field("id", f.id);
            w.field("type", f.type);
            w.field("ready", f.ready);
            w.field("note", f.note);
            w.field("calibration", toString(f.stillness.calibration));
            w.field("calibration_attempts", static_cast<uint64_t>(f.stillness.attempts));
            if (f.stillness.has_bias) {
                w.field("gyro_bias_deg_s", f.stillness.bias_rad_s * 57.29577951308232);
            } else {
                w.fieldNull("gyro_bias_deg_s");
            }
            w.field("dropped_intervals", f.dropped_intervals);
            w.field("dropped_why", f.dropped_why);
            w.endObject();
        }
        w.endArray();
        w.endObject();
    }

    void writePico(JsonWriter& w) const {
        const std::shared_ptr<PicoControl>& pico = system_.picoControl();
        if (pico == nullptr) {
            w.fieldNull("pico_link");
            return;
        }
        const PicoLinkState l = pico->link();
        w.key("pico_link");
        w.beginObject();
        w.field("frames_fresh", l.frames_fresh);
        w.field("identity", l.identity);
        w.field("boot_id", static_cast<int>(l.boot_id));
        w.field("acq_epoch", static_cast<int>(l.acq_epoch));
        w.field("imu_epoch", static_cast<int>(l.imu_epoch));
        w.field("reboots", l.reboots);
        w.field("restarts", l.restarts);
        w.field("imu_restarts", l.imu_restarts);
        w.field("firmware", static_cast<int>(l.status_known ? l.status.firmware : 0));
        w.endObject();
    }

    const System& system_;
};

const char* stateName(bool enabled, bool stopped, int state) {
    if (!enabled) {
        return "disabled";
    }
    if (stopped) {
        return "stopped";
    }
    switch (state) {
    case 1: return "recording";
    case 2: return "finalizing";
    default: return "idle";
    }
}

} // namespace

// ---- recorder -------------------------------------------------------------------

CaptureRecorder::CaptureRecorder(const CaptureConfig& config, DiagnosticsHub& hub,
                                 std::shared_ptr<const CaptureHost> host, Options options)
    : config_(config), hub_(hub), host_(std::move(host)), options_(std::move(options)) {
    const std::size_t by_records = static_cast<std::size_t>(std::max<long>(config_.max_records, 1));
    const std::size_t by_memory =
        static_cast<std::size_t>(std::max<long>(config_.max_mb, 1)) * (1u << 20) / sizeof(DiagRecord);
    record_cap_    = std::max<std::size_t>(1, std::min(by_records, by_memory));
    memory_bound_  = by_memory < by_records;
    rolling_kinds_ = config_.enabled && config_.rolling_s > 0.0 ? kCaptureDefaultKinds : 0;
    const std::string session = host_ != nullptr ? host_->sessionId() : std::string();
    id_prefix_ = session.empty() ? std::string("capture") : session.substr(0, 8);
    resets_.push_back({0, host_ != nullptr ? host_->resetCount() : 0});
    hub_.setWanted(rolling_kinds_);
    if (config_.enabled && options_.thread) {
        thread_ = std::thread([this] {
            std::unique_lock<std::mutex> lock(mutex_);
            while (!stopped_) {
                lock.unlock();
                pump();
                lock.lock();
                wake_.wait_for(lock, std::chrono::milliseconds(options_.pump_ms),
                               [this] { return stopped_ || woken_; });
                woken_ = false;
            }
        });
    }
}

CaptureRecorder::~CaptureRecorder() { stop(); }

int64_t CaptureRecorder::now() const {
    return options_.clock ? options_.clock() : HostClock::nowUs();
}

int64_t CaptureRecorder::wallMs() const {
    if (options_.wall_ms) {
        return options_.wall_ms();
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

bool CaptureRecorder::start(const CaptureRequest& request, std::string& id, std::string& err) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!startLocked(request, std::string(), now(), id, err)) {
            return false;
        }
        woken_ = true;
    }
    wake_.notify_all();
    return true;
}

bool CaptureRecorder::startLocked(const CaptureRequest& request, const std::string& detail,
                                  int64_t trigger_us, std::string& id, std::string& err) {
    if (!config_.enabled) {
        err = "capture is disabled in the configuration";
        return false;
    }
    if (stopped_) {
        err = "capture is stopped";
        return false;
    }
    if (state_ != State::kIdle || pending_ != nullptr) {
        err = "busy: capture " + active_id_ +
              (state_ == State::kFinalizing ? " is finalizing" : " is recording");
        return false;
    }
    if (!std::isfinite(request.pre_s) || !std::isfinite(request.post_s) || request.pre_s < 0.0 ||
        request.post_s <= 0.0) {
        err = "pre_s must be at least 0 and post_s more than 0";
        return false;
    }
    const uint32_t kinds = request.kinds & kDiagAllKinds;
    if (kinds == 0) {
        err = "no streams selected";
        return false;
    }
    auto t              = std::make_unique<Trigger>();
    t->request          = request;
    t->request.kinds    = kinds;
    t->pre_s_requested  = request.pre_s;
    t->post_s_requested = request.post_s;
    // the pre-trigger interval can only come from the rolling window
    t->request.pre_s  = std::min({request.pre_s, config_.max_pre_s,
                                  rolling_kinds_ != 0 ? config_.rolling_s : 0.0});
    t->request.post_s = std::min(request.post_s, config_.max_post_s);
    if (t->request.reason.empty()) {
        t->request.reason = "manual";
    }
    if (t->request.requester.empty()) {
        t->request.requester = "unknown";
    }
    t->detail     = detail;
    t->trigger_us = trigger_us;
    const int64_t wall = wallMs();
    t->unix_ms    = wall >= 0 ? wall - (now() - trigger_us) / 1000 : -1;
    t->id         = id_prefix_ + "-" + std::to_string(++next_seq_);
    id            = t->id;

    state_            = State::kRecording;
    active_id_        = t->id;
    active_trigger_   = *t;
    active_end_us_    = trigger_us + toUs(t->request.post_s);
    active_records_   = 0;
    active_dropped_   = 0;
    last_progress_us_ = trigger_us;
    // selected-only streams (raw bytes) start flowing now
    hub_.setWanted(rolling_kinds_ | kinds);
    pending_ = std::move(t);
    bump();
    return true;
}

bool CaptureRecorder::cancel(const std::string& id, std::string& err) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (state_ == State::kIdle || id.empty() || id != active_id_) {
            err = id.empty() ? "no capture id given" : "no active capture " + id;
            return false;
        }
        cancel_id_ = id;
        woken_     = true;
    }
    wake_.notify_all();
    return true;
}

void CaptureRecorder::fault(CaptureFault fault, const std::string& detail) {
    if ((config_.auto_faults & captureFaultBit(fault)) == 0 || !config_.enabled) {
        return; // not an automatic trigger here: nothing to queue
    }
    const int64_t at = now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (faults_.size() >= kFaultsQueued) {
            ++faults_dropped_;
            return;
        }
        faults_.push_back(FaultNote{fault, detail, at});
        woken_ = true;
    }
    wake_.notify_all();
}

void CaptureRecorder::noteReset(uint64_t reset_count) {
    const int64_t               at = now();
    std::lock_guard<std::mutex> lock(mutex_);
    resets_.push_back({at, reset_count});
    if (resets_.size() > kResetsKept) {
        resets_.erase(resets_.begin());
    }
}

std::shared_ptr<const std::string> CaptureRecorder::bundle(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const Finished& f : finished_) {
        if (f.id == id) {
            return f.zip;
        }
    }
    return nullptr;
}

void CaptureRecorder::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopped_) {
            return;
        }
        stopped_ = true;
        woken_   = true;
    }
    wake_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    // the last records and a start nobody picked up yet, then close what is
    // recording so it stays downloadable while the System lives
    pump();
    if (active_ != nullptr) {
        finish("shutdown", true);
    }
    hub_.setWanted(0);
}

void CaptureRecorder::sampleHub(int64_t now_us) {
    if (!hub_samples_.empty() && now_us - hub_samples_.back().at_us < kHubSampleUs) {
        return;
    }
    const DiagHubStats s = hub_.stats();
    HubSample          h;
    h.at_us   = now_us;
    h.posted  = s.posted;
    h.dropped = s.dropped;
    hub_samples_.push_back(h);
    const int64_t span = toUs(std::max(config_.rolling_s, config_.max_pre_s)) + 2 * kHubSampleUs;
    while (hub_samples_.size() > 2 && hub_samples_.front().at_us < now_us - span) {
        hub_samples_.pop_front();
    }
}

std::string CaptureRecorder::hostJson(int64_t at_us) const {
    JsonWriter w;
    w.beginObject();
    w.field("sampled_pi_host_us", at_us);
    host_->writeMetadata(w);
    w.endObject();
    return w.take();
}

void CaptureRecorder::sampleHost(int64_t now_us) {
    // without a rolling window there is no pre-window: begin() reads the host
    if (host_ == nullptr || rolling_kinds_ == 0) {
        return;
    }
    if (!host_samples_.empty() && now_us - host_samples_.back().at_us < kHostSampleUs) {
        return;
    }
    host_samples_.push_back(HostSample{now_us, hostJson(now_us)});
    // one sample at or before the oldest window start a trigger can reach
    const int64_t oldest = now_us - toUs(config_.rolling_s);
    while (host_samples_.size() > 1 && host_samples_[1].at_us <= oldest) {
        host_samples_.pop_front();
    }
}

void CaptureRecorder::begin(Trigger trigger) {
    auto a               = std::make_unique<Active>();
    a->trigger           = std::move(trigger);
    a->window_start_us   = a->trigger.trigger_us - toUs(a->trigger.request.pre_s);
    a->end_us            = a->trigger.trigger_us + toUs(a->trigger.request.post_s);
    a->rolling_oldest_us = rolling_.empty() ? -1 : rolling_.front().host_us;
    // evicted records stamped inside this window; the bucket of the window
    // start counts whole
    const int64_t from_ms = a->window_start_us / 1000;
    for (const auto& e : evicted_) {
        if (e.first >= from_ms) {
            a->rolling_evicted += e.second;
        }
    }
    // the host state in force at the window start: the newest sample at or
    // before it, else the oldest, else read now (the time says which)
    if (host_ != nullptr) {
        const HostSample* pick = host_samples_.empty() ? nullptr : &host_samples_.front();
        for (const HostSample& h : host_samples_) {
            if (h.at_us <= a->window_start_us) {
                pick = &h;
            }
        }
        a->host_at_start = pick != nullptr ? pick->json : hostJson(now());
    }
    // hub counters as of the window start: the newest sample not after it
    // (the oldest one when the window reaches back past every sample)
    if (!hub_samples_.empty()) {
        const HubSample* base = &hub_samples_.front();
        for (const HubSample& h : hub_samples_) {
            if (h.at_us <= a->window_start_us) {
                base = &h;
            }
        }
        a->hub_posted0  = base->posted;
        a->hub_dropped0 = base->dropped;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        a->faults_dropped0 = faults_dropped_;
    }
    active_ = std::move(a);
    for (const DiagRecord& r : rolling_) {
        route(r);
    }
}

void CaptureRecorder::route(const DiagRecord& r) {
    Active& a = *active_;
    if (r.host_us < a.window_start_us || r.host_us > a.end_us) {
        return;
    }
    const auto k = static_cast<std::size_t>(r.kind);
    if (k >= static_cast<std::size_t>(kDiagKindCount)) {
        return;
    }
    if ((a.trigger.request.kinds & (1u << k)) == 0) {
        ++a.not_selected[k];
        return;
    }
    if (a.limit_hit || a.records.size() >= record_cap_) {
        a.limit_hit = true;
        ++a.limit_dropped[k];
        return;
    }
    a.records.push_back(r);
}

void CaptureRecorder::trimRolling(int64_t now_us) {
    const int64_t from = now_us - toUs(config_.rolling_s);
    while (!rolling_.empty() && rolling_.front().host_us < from) {
        rolling_.pop_front();
    }
    // the rolling window and the active capture share one record bound
    const std::size_t active = active_ != nullptr ? active_->records.size() : 0;
    while (!rolling_.empty() && rolling_.size() + active > record_cap_) {
        // by the evicted record's own stamp, so a later window counts only
        // what it lost; stamps a few us out of order join the newest bucket
        const int64_t ms = rolling_.front().host_us / 1000;
        if (!evicted_.empty() && evicted_.back().first >= ms) {
            ++evicted_.back().second;
        } else {
            evicted_.push_back({ms, 1});
        }
        rolling_.pop_front();
    }
    const int64_t oldest_ms = from / 1000;
    while (!evicted_.empty() && evicted_.front().first < oldest_ms) {
        evicted_.pop_front();
    }
}

void CaptureRecorder::handleFault(const FaultNote& f) {
    std::unique_ptr<Trigger> started;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++auto_seen_[static_cast<std::size_t>(f.fault)];
        while (!auto_times_.empty() && auto_times_.front() <= f.at_us - kHourUs) {
            auto_times_.pop_front();
        }
        if (state_ != State::kIdle || pending_ != nullptr) {
            ++auto_busy_;
            bump();
            return;
        }
        if (auto_last_us_ >= 0 && f.at_us - auto_last_us_ < toUs(config_.auto_cooldown_s)) {
            ++auto_cooldown_;
            bump();
            return;
        }
        if (static_cast<long>(auto_times_.size()) >= config_.auto_max_per_hour) {
            ++auto_capped_;
            bump();
            return;
        }
        CaptureRequest request;
        request.pre_s     = config_.default_pre_s;
        request.post_s    = config_.default_post_s;
        request.kinds     = kCaptureDefaultKinds;
        request.reason    = captureFaultName(f.fault);
        request.requester = "auto";
        std::string id, err;
        if (!startLocked(request, f.detail, f.at_us, id, err)) {
            return;
        }
        ++auto_fired_;
        auto_times_.push_back(f.at_us);
        auto_last_us_     = f.at_us;
        auto_last_reason_ = request.reason;
        started           = std::move(pending_);
    }
    begin(std::move(*started));
}

void CaptureRecorder::pump() {
    const int64_t t = now();
    batch_.clear();
    hub_.drain(batch_);
    sampleHub(t);
    sampleHost(t);

    std::unique_ptr<Trigger> start;
    std::vector<FaultNote>   faults;
    std::string              cancel;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        start = std::move(pending_);
        faults.swap(faults_);
        cancel.swap(cancel_id_);
    }
    // a start, then fault triggers, before this batch: records drained now
    // but stamped before the trigger still belong to its pre-window
    if (start != nullptr) {
        begin(std::move(*start));
    }
    for (const FaultNote& f : faults) {
        handleFault(f);
    }
    if (!cancel.empty() && active_ != nullptr && active_->trigger.id == cancel) {
        active_.reset();
        hub_.setWanted(rolling_kinds_);
        std::lock_guard<std::mutex> lock(mutex_);
        state_        = State::kIdle;
        last_id_      = cancel;
        last_outcome_ = "cancelled";
        last_error_.clear();
        active_id_.clear();
        bump();
    }

    for (DiagRecord& r : batch_) {
        if (active_ != nullptr) {
            route(r);
        }
        if ((rolling_kinds_ & (1u << static_cast<uint8_t>(r.kind))) != 0) {
            rolling_.push_back(std::move(r));
        }
    }
    batch_.clear();
    trimRolling(t);

    if (active_ != nullptr) {
        if (active_->limit_hit) {
            finish(memory_bound_ ? "memory_limit" : "record_limit", true);
        } else if (t >= active_->end_us) {
            finish("post_window", false);
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ != nullptr) {
        active_records_ = active_->records.size();
        active_dropped_ = total(active_->limit_dropped);
        if (t - last_progress_us_ >= 1000000) {
            last_progress_us_ = t; // progress for viewers, at most 1 Hz
            bump();
        }
    }
}

void CaptureRecorder::finish(const char* stop_reason, bool truncated) {
    std::unique_ptr<Active> a = std::move(active_);
    hub_.setWanted(rolling_kinds_);
    const int64_t end_us = truncated ? std::min(now(), a->end_us) : a->end_us;
    std::vector<std::pair<int64_t, uint64_t>> resets;
    uint64_t                                  faults_dropped = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = State::kFinalizing;
        resets = resets_;
        faults_dropped = faults_dropped_ - a->faults_dropped0;
        bump();
    }

    CaptureBundleInput in;
    in.id               = a->trigger.id;
    in.pi_session       = host_ != nullptr ? host_->sessionId() : std::string();
    in.records          = &a->records;
    in.kinds            = a->trigger.request.kinds;
    in.reason           = a->trigger.request.reason;
    in.requester        = a->trigger.request.requester;
    in.detail           = a->trigger.detail;
    in.pre_s_requested  = a->trigger.pre_s_requested;
    in.post_s_requested = a->trigger.post_s_requested;
    in.pre_s            = a->trigger.request.pre_s;
    in.post_s           = a->trigger.request.post_s;
    in.trigger_us       = a->trigger.trigger_us;
    in.window_start_us  = a->window_start_us;
    in.end_us           = end_us;
    in.rolling_oldest_us = a->rolling_oldest_us;
    in.created_unix_ms  = a->trigger.unix_ms;
    in.truncated        = truncated;
    in.stop_reason      = stop_reason;
    const DiagHubStats hs = hub_.stats();
    in.hub_posted       = minus(hs.posted, a->hub_posted0);
    in.hub_dropped      = minus(hs.dropped, a->hub_dropped0);
    in.not_selected     = a->not_selected;
    in.limit_dropped    = a->limit_dropped;
    in.rolling_evicted  = a->rolling_evicted;
    in.faults_dropped   = faults_dropped;
    in.hub_capacity     = hs.capacity;
    in.record_cap       = record_cap_;
    in.source_names     = hub_.sourceNames();
    in.max_bytes        = static_cast<std::size_t>(config_.max_mb) << 20;
    // the reset in force at the window start, then those inside it
    std::size_t first = 0;
    for (std::size_t i = 0; i < resets.size(); ++i) {
        if (resets[i].first <= in.window_start_us) {
            first = i;
        }
    }
    for (std::size_t i = first; i < resets.size(); ++i) {
        if (i == first || resets[i].first <= end_us) {
            in.resets.push_back(resets[i]);
        }
    }
    in.markers.push_back({in.trigger_us, "capture " + in.id + " trigger: " + in.reason + " by " +
                                             in.requester +
                                             (in.detail.empty() ? "" : " (" + in.detail + ")")});
    for (std::size_t i = 1; i < in.resets.size(); ++i) {
        if (in.resets[i].first > in.window_start_us) {
            in.markers.push_back({in.resets[i].first, "Pi reset: reset count " +
                                                          std::to_string(in.resets[i].second)});
        }
    }
    in.markers.push_back({end_us, "capture " + in.id + " end: " + stop_reason});
    std::stable_sort(in.markers.begin(), in.markers.end(),
                     [](const CaptureMarker& x, const CaptureMarker& y) {
                         return x.host_us < y.host_us;
                     });
    if (host_ != nullptr) {
        const std::shared_ptr<const CaptureHost> host = host_;
        in.write_host    = [host](JsonWriter& w) { host->writeMetadata(w); };
        in.host_end_us   = now();
        in.host_at_start = std::move(a->host_at_start);
    }

    // the rows hold the records once written: free them before the ZIP
    const uint64_t recorded = a->records.size();
    Active* const  held     = a.get();
    in.rows_written         = [held] { std::deque<DiagRecord>().swap(held->records); };

    CaptureBundleOutput out;
    std::string         err;
    const bool          built = buildCaptureBundle(in, out, err);
    a.reset();

    const auto idle = [&](const char* outcome, const std::string& error) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_        = State::kIdle;
        last_id_      = in.id;
        last_outcome_ = outcome;
        last_error_   = error;
        active_id_.clear();
        bump();
    };
    const auto cancelled = [&] {
        std::lock_guard<std::mutex> lock(mutex_);
        if (cancel_id_ != in.id) {
            return false;
        }
        cancel_id_.clear();
        return true;
    };
    if (!built) {
        idle("failed", "bundle: " + err);
        return;
    }
    if (cancelled()) {
        idle("cancelled", std::string());
        return;
    }

    Finished f;
    f.id          = in.id;
    f.reason      = in.reason;
    f.requester   = in.requester;
    f.detail      = in.detail;
    f.stop_reason = in.stop_reason;
    f.trigger_us  = in.trigger_us;
    f.end_us      = end_us;
    f.pre_s       = in.pre_s;
    f.post_s      = in.post_s;
    f.kinds       = in.kinds;
    f.truncated   = out.truncated; // the same verdict as metadata.json
    f.truncated_by = out.truncated_by;
    f.records     = recorded;
    f.rows        = out.rows;
    for (int k = 0; k < kDiagKindCount; ++k) {
        f.dropped[k] = in.hub_dropped[k] + in.limit_dropped[k] + out.rows_omitted[k];
    }
    f.zip = std::make_shared<const std::string>(std::move(out.zip));
    if (!config_.directory.empty()) {
        std::string path, ferr;
        if (writeFile(f.id, *f.zip, path, ferr)) {
            f.file = path;
        } else {
            f.file_error = ferr; // still downloadable from memory
        }
    }
    std::vector<std::string> remove;
    {
        // publishing ready and honoring a late cancel under one lock: once
        // idle, a cancel of this id is refused, never silently ignored
        std::lock_guard<std::mutex> lock(mutex_);
        const bool discard = cancel_id_ == in.id;
        last_id_           = in.id;
        last_outcome_      = discard ? "cancelled" : "ready";
        last_error_        = discard ? std::string() : f.file_error;
        if (discard) {
            cancel_id_.clear();
            if (!f.file.empty()) {
                remove.push_back(f.file);
            }
        } else {
            finished_.push_back(std::move(f));
        }
        while (finished_.size() > static_cast<std::size_t>(config_.keep)) {
            if (!finished_.front().file.empty()) {
                remove.push_back(finished_.front().file);
            }
            finished_.pop_front();
        }
        state_ = State::kIdle;
        active_id_.clear();
        bump();
    }
    // the directory keeps the same newest captures of this run; files of
    // earlier runs are never touched
    for (const std::string& path : remove) {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
}

bool CaptureRecorder::writeFile(const std::string& id, const std::string& zip, std::string& path,
                                std::string& err) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path  dir(config_.directory);
    fs::create_directories(dir, ec);
    if (!fs::is_directory(dir, ec)) {
        err = "not a directory: " + config_.directory;
        return false;
    }
    const fs::path target = dir / (id + ".zip");
    const fs::path part   = dir / (id + ".zip.part");
    {
        std::ofstream o(part, std::ios::binary | std::ios::trunc);
        if (!o) {
            err = "cannot open " + part.string();
            return false;
        }
        o.write(zip.data(), static_cast<std::streamsize>(zip.size()));
        o.close();
        if (!o) {
            err = "write failed: " + part.string();
            fs::remove(part, ec);
            return false;
        }
    }
    fs::rename(part, target, ec);
    if (ec) {
        err = "rename failed: " + ec.message();
        fs::remove(part, ec);
        return false;
    }
    path = target.string();
    return true;
}

void CaptureRecorder::writeActive(JsonWriter& w, int64_t now_us) const {
    // caller holds mutex_
    if (state_ == State::kIdle) {
        w.fieldNull("active");
        return;
    }
    const Trigger& t = active_trigger_;
    w.key("active");
    w.beginObject();
    w.field("id", t.id);
    w.field("state", state_ == State::kFinalizing ? "finalizing" : "recording");
    w.field("reason", t.request.reason);
    w.field("requester", t.request.requester);
    if (!t.detail.empty()) {
        w.field("detail", t.detail);
    }
    writeKinds(w, "streams", t.request.kinds);
    w.field("pre_s_requested", t.pre_s_requested);
    w.field("post_s_requested", t.post_s_requested);
    w.field("pre_s", t.request.pre_s);
    w.field("post_s", t.request.post_s);
    w.field("trigger_pi_host_us", t.trigger_us);
    w.field("end_pi_host_us", active_end_us_);
    w.field("elapsed_s", static_cast<double>(now_us - t.trigger_us) * 1e-6);
    w.field("remaining_s", static_cast<double>(std::max<int64_t>(0, active_end_us_ - now_us)) * 1e-6);
    w.field("records", active_records_);
    w.field("limit_dropped", active_dropped_);
    w.endObject();
}

void CaptureRecorder::writeStatus(JsonWriter& w) const {
    const int64_t               t = now();
    const DiagHubStats          hs = hub_.stats();
    std::lock_guard<std::mutex> lock(mutex_);
    w.beginObject();
    w.field("available", config_.enabled && !stopped_);
    w.field("schema", kCaptureSchema);
    w.field("state", stateName(config_.enabled, stopped_, static_cast<int>(state_)));
    w.field("pi_host_us", t);
    writeActive(w, t);

    w.key("captures");
    w.beginArray();
    for (auto it = finished_.rbegin(); it != finished_.rend(); ++it) {
        const Finished& f = *it;
        w.beginObject();
        w.field("id", f.id);
        w.field("state", "ready");
        w.field("url", "/api/capture/" + f.id + ".zip");
        w.field("reason", f.reason);
        w.field("requester", f.requester);
        if (!f.detail.empty()) {
            w.field("detail", f.detail);
        }
        writeKinds(w, "streams", f.kinds);
        w.field("pre_s", f.pre_s);
        w.field("post_s", f.post_s);
        w.field("trigger_pi_host_us", f.trigger_us);
        w.field("end_pi_host_us", f.end_us);
        w.field("truncated", f.truncated);
        w.key("truncated_by");
        w.beginArray();
        for (const std::string& why : f.truncated_by) {
            w.value(why);
        }
        w.endArray();
        w.field("stop_reason", f.stop_reason);
        w.field("records", f.records);
        writeCountsByKind(w, "rows", f.rows);
        writeCountsByKind(w, "dropped", f.dropped);
        w.field("size_bytes", static_cast<uint64_t>(f.zip != nullptr ? f.zip->size() : 0));
        if (!f.file.empty()) {
            w.field("file", f.file);
        } else {
            w.fieldNull("file");
        }
        if (!f.file_error.empty()) {
            w.field("file_error", f.file_error);
        } else {
            w.fieldNull("file_error");
        }
        w.endObject();
    }
    w.endArray();

    if (!last_id_.empty()) {
        w.key("last");
        w.beginObject();
        w.field("id", last_id_);
        w.field("outcome", last_outcome_);
        w.field("error", last_error_);
        w.endObject();
    } else {
        w.fieldNull("last");
    }

    w.key("limits");
    w.beginObject();
    w.field("rolling_s", config_.rolling_s);
    w.field("max_pre_s", std::min(config_.max_pre_s, rolling_kinds_ != 0 ? config_.rolling_s : 0.0));
    w.field("max_post_s", config_.max_post_s);
    w.field("default_pre_s", config_.default_pre_s);
    w.field("default_post_s", config_.default_post_s);
    w.field("max_records", static_cast<int64_t>(config_.max_records));
    w.field("max_mb", static_cast<int64_t>(config_.max_mb));
    w.field("record_cap", static_cast<uint64_t>(record_cap_));
    w.field("keep", static_cast<int64_t>(config_.keep));
    w.field("directory", config_.directory);
    w.endObject();

    w.key("streams");
    w.beginObject();
    writeKinds(w, "default", kCaptureDefaultKinds);
    writeKinds(w, "all", kDiagAllKinds);
    writeKinds(w, "rolling", rolling_kinds_);
    w.endObject();

    w.key("auto");
    w.beginObject();
    w.key("triggers");
    w.beginArray();
    for (int i = 0; i < kCaptureFaultCount; ++i) {
        const auto f = static_cast<CaptureFault>(i);
        if ((config_.auto_faults & captureFaultBit(f)) != 0) {
            w.value(captureFaultName(f));
        }
    }
    w.endArray();
    w.field("cooldown_s", config_.auto_cooldown_s);
    w.field("max_per_hour", static_cast<int64_t>(config_.auto_max_per_hour));
    w.field("fired", auto_fired_);
    std::size_t last_hour = 0;
    for (int64_t at : auto_times_) {
        if (at > t - kHourUs) {
            ++last_hour;
        }
    }
    w.field("fired_last_hour", static_cast<uint64_t>(last_hour));
    w.key("suppressed");
    w.beginObject();
    w.field("busy", auto_busy_);
    w.field("cooldown", auto_cooldown_);
    w.field("hourly_cap", auto_capped_);
    w.endObject();
    w.key("seen");
    w.beginObject();
    for (int i = 0; i < kCaptureFaultCount; ++i) {
        w.field(captureFaultName(static_cast<CaptureFault>(i)), auto_seen_[static_cast<std::size_t>(i)]);
    }
    w.endObject();
    if (auto_last_us_ >= 0) {
        w.field("last_reason", auto_last_reason_);
        w.field("last_pi_host_us", auto_last_us_);
    } else {
        w.fieldNull("last_reason");
        w.fieldNull("last_pi_host_us");
    }
    w.field("notices_dropped", faults_dropped_);
    w.endObject();

    w.key("hub");
    w.beginObject();
    w.field("capacity", static_cast<uint64_t>(hs.capacity));
    w.field("queued", static_cast<uint64_t>(hs.queued));
    writeCountsByKind(w, "dropped", hs.dropped);
    w.endObject();
    w.endObject();
}

std::unique_ptr<CaptureRecorder> makeCaptureRecorder(System& system, const CaptureConfig& config) {
    return std::make_unique<CaptureRecorder>(config, system.diagHub(),
                                             std::make_shared<SystemCaptureHost>(system),
                                             CaptureRecorder::Options{});
}

std::unique_ptr<CaptureService> makeCaptureService(System& system, std::string&) {
    return makeCaptureRecorder(system, system.captureConfig());
}

} // namespace navigatr
