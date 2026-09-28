// stationary_window.cpp

#include "impl/localization/stationary_window.h"

#include <algorithm>
#include <cmath>

namespace navigatr
{

bool readStillness(const ConfigNode& node, StillnessConfig& c, std::string& err) {
    long   window_ms = static_cast<long>(c.window_ms);
    long   gap_ms    = static_cast<long>(c.max_gap_ms);
    double still_dps = radToDeg(c.still_rate_rad_s);
    double max_dps   = radToDeg(c.max_rate_rad_s);
    if (!node.getInt("window_ms", window_ms, window_ms, err) ||
        !node.getInt("evidence_gap_ms", gap_ms, gap_ms, err) ||
        !node.getDouble("still_rate_dps", still_dps, still_dps, err) ||
        !node.getDouble("max_rate_dps", max_dps, max_dps, err)) {
        return false;
    }
    if (window_ms < 0 || gap_ms <= 0 || !(still_dps > 0.0) || !(max_dps > 0.0)) {
        err = node.path() + ": window_ms cannot be negative; evidence_gap_ms, still_rate_dps "
                            "and max_rate_dps must be positive";
        return false;
    }
    c.window_ms        = window_ms;
    c.max_gap_ms       = gap_ms;
    c.still_rate_rad_s = degToRad(still_dps);
    c.max_rate_rad_s   = degToRad(max_dps);
    return true;
}

// ---- StationaryWindow --------------------------------------------------------

void StationaryWindow::configure(const StillnessConfig& config) {
    config_        = config;
    sources_.clear();
    restart_phase_ = Phase::kWaitingData;
    stationary_    = false;
    reason_        = "no samples yet";
    qualified_     = false;
}

size_t StationaryWindow::addSource(Kind kind, const std::string& name) {
    Source s;
    s.kind = kind;
    s.name = name;
    sources_.push_back(s);
    return sources_.size() - 1;
}

void StationaryWindow::restart(Phase phase, const std::string& why) {
    for (Source& s : sources_) {
        s.count = 0;
    }
    restart_phase_ = phase;
    stationary_    = false;
    reason_        = why;
    ++restarts_;
}

void StationaryWindow::begin(Source& s, const StillSample& sample) {
    s.count       = 1;
    s.first_at    = sample.at;
    s.last_at     = sample.at;
    s.lo          = sample.value;
    s.hi          = sample.value;
    s.sum         = sample.value;
    s.last_value  = sample.value;
    s.integral    = 0.0;
    s.accumulated = sample.has_accumulated;
    s.first_accum = sample.accumulated;
    s.last_accum  = sample.accumulated;
    s.accum_epoch = sample.accumulated_epoch;
}

void StationaryWindow::append(Source& s, const StillSample& sample) {
    if (s.count == 0) {
        begin(s, sample);
        return;
    }
    if (s.kind == Kind::kGyro) {
        s.integral += 0.5 * (s.last_value + sample.value) * secondsBetween(sample.at, s.last_at);
    }
    ++s.count;
    s.last_at    = sample.at;
    s.lo         = std::min(s.lo, sample.value);
    s.hi         = std::max(s.hi, sample.value);
    s.sum       += sample.value;
    s.last_value = sample.value;
    s.accumulated =
        s.accumulated && sample.has_accumulated && sample.accumulated_epoch == s.accum_epoch;
    s.last_accum = sample.accumulated;
}

bool StationaryWindow::moves(const Source& s, const StillSample& sample, std::string& why) const {
    const double v = sample.value;
    if (s.kind == Kind::kGyro && std::fabs(v) > config_.max_rate_rad_s) {
        why = s.name + " turning";
        return true;
    }
    if (s.count == 0) {
        return false;
    }
    const double lo = std::min(s.lo, v);
    const double hi = std::max(s.hi, v);
    switch (s.kind) {
    case Kind::kWheel:
        if (hi - lo > config_.still_travel_m) {
            why = s.name + " moved";
            return true;
        }
        return false;
    case Kind::kGyro: {
        const double mean = (s.sum + v) / static_cast<double>(s.count + 1);
        if (hi - mean > config_.still_rate_rad_s || mean - lo > config_.still_rate_rad_s) {
            why = s.name + " rate varies";
            return true;
        }
        return false;
    }
    case Kind::kRotation: {
        const double elapsed = std::max(config_.window_ms / 1000.0,
                                        sameDomain(sample.at, s.first_at)
                                            ? secondsBetween(sample.at, s.first_at)
                                            : 0.0);
        if (hi - lo > config_.still_rate_rad_s * elapsed) {
            why = s.name + " turning";
            return true;
        }
        return false;
    }
    }
    return false;
}

int64_t StationaryWindow::span(const Source& s) const {
    if (s.count == 0 || !sameDomain(s.last_at, s.first_at)) {
        return 0;
    }
    return s.last_at - s.first_at;
}

bool StationaryWindow::add(size_t index, const StillSample& sample) {
    if (index >= sources_.size()) {
        return false;
    }
    Source& s = sources_[index];
    if (s.seen && sample.sequence == s.sequence && sample.epoch == s.epoch) {
        return false;   // a retained record is not new evidence
    }
    const bool restarted =
        s.seen && (sample.epoch != s.epoch || sample.discontinuity != s.discontinuity);
    s.seen          = true;
    s.sequence      = sample.sequence;
    s.epoch         = sample.epoch;
    s.discontinuity = sample.discontinuity;
    s.received      = sample.received;
    s.stale         = false;

    if (restarted) {
        restart(Phase::kWaitingData, s.name + " restarted");
        begin(s, sample);
        return true;
    }
    if (s.count > 0) {
        const bool    same = sameDomain(sample.at, s.last_at);
        const int64_t dt   = same ? sample.at - s.last_at : 0;
        if (!same || dt <= 0 || dt > config_.max_gap_ms) {
            restart(Phase::kWaitingData,
                    s.name + (same && dt > 0 ? " gap" : " interval out of order"));
            begin(s, sample);
            return true;
        }
        if (s.kind == Kind::kGyro && s.accumulated && sample.has_accumulated &&
            sample.accumulated_epoch != s.accum_epoch) {
            restart(Phase::kWaitingData, s.name + " accumulator discontinuity");
            begin(s, sample);
            return true;
        }
    }
    std::string why;
    if (moves(s, sample, why)) {
        restart(Phase::kWaitingStill, why);
        begin(s, sample);   // the newest sample is the new baseline
        return true;
    }
    append(s, sample);
    qualify();
    return true;
}

void StationaryWindow::qualify() {
    if (sources_.empty()) {
        return;
    }
    const long samples = std::max(config_.min_samples, 1L);
    for (const Source& s : sources_) {
        if (s.count < samples || span(s) < config_.window_ms) {
            return;
        }
    }
    Qualified q;
    for (const Source& s : sources_) {
        if (s.kind != Kind::kGyro) {
            continue;
        }
        q.gyro      = true;
        q.elapsed_s = secondsBetween(s.last_at, s.first_at);
        if (s.accumulated && q.elapsed_s > 1e-6) {
            q.gyro_rate = (s.last_accum - s.first_accum) / q.elapsed_s;
        } else if (q.elapsed_s > 1e-6) {
            q.gyro_rate = s.integral / q.elapsed_s;
        } else {
            q.gyro_rate = s.sum / static_cast<double>(s.count);
        }
        break;
    }
    last_          = q;
    qualified_     = true;
    stationary_    = true;
    restart_phase_ = Phase::kCollecting;
    reason_        = "stationary";
    ++windows_;
    // the next window starts from the newest samples
    for (Source& s : sources_) {
        s.count       = 1;
        s.first_at    = s.last_at;
        s.lo          = s.last_value;
        s.hi          = s.last_value;
        s.sum         = s.last_value;
        s.integral    = 0.0;
        s.first_accum = s.last_accum;
    }
}

void StationaryWindow::poll(MonotonicTime now) {
    for (Source& s : sources_) {
        if (!s.seen) {
            if (!stationary_) {
                reason_ = s.name + " has no samples yet";
            }
            continue;
        }
        if (!s.stale && sameDomain(now, s.received) && now - s.received > config_.max_gap_ms) {
            s.stale = true;
            restart(Phase::kWaitingData, s.name + " stale");
        }
    }
}

StationaryWindow::Phase StationaryWindow::phase() const {
    for (const Source& s : sources_) {
        if (!s.seen || (s.stale && s.count == 0)) {
            return Phase::kWaitingData;
        }
    }
    if (restart_phase_ == Phase::kCollecting) {
        return Phase::kCollecting;
    }
    if (restart_phase_ == Phase::kWaitingData) {
        for (const Source& s : sources_) {
            if (s.count == 0) {
                return Phase::kWaitingData;
            }
        }
        return Phase::kCollecting;
    }
    // a still sample after the movement, over a quarter window: collecting
    const int64_t settle = config_.window_ms / 4;
    for (const Source& s : sources_) {
        if (s.count < 2 || span(s) < settle) {
            return Phase::kWaitingStill;
        }
    }
    return Phase::kCollecting;
}

int64_t StationaryWindow::progressMs() const {
    if (sources_.empty()) {
        return 0;
    }
    int64_t least = span(sources_.front());
    for (const Source& s : sources_) {
        least = std::min(least, span(s));
    }
    return least;
}

bool StationaryWindow::takeQualified(Qualified& out) {
    if (!qualified_) {
        return false;
    }
    out        = last_;
    qualified_ = false;
    return true;
}

// ---- GyroBiasCalibration -----------------------------------------------------

bool GyroBiasCalibration::read(const ConfigNode& node, Config& c, std::string& err) {
    double attempt_s = c.attempt_ms / 1000.0;
    if (!node.getDouble("attempt_s", attempt_s, attempt_s, err)) {
        return false;
    }
    if (!(attempt_s > 0.0)) {
        err = node.path() + ": attempt_s must be positive";
        return false;
    }
    c.attempt_ms = static_cast<int64_t>(std::llround(attempt_s * 1000.0));
    return true;
}

void GyroBiasCalibration::configure(const Config& config) {
    config_     = config;
    attempts_   = 0;
    steps_      = 0;
    bias_       = 0.0;
    calibrated_ = !config_.enabled;
    failed_     = false;
    note_       = config_.enabled ? "" : "calibration off; bias zero";
    if (config_.enabled) {
        start("waiting for a stationary window");
    }
}

void GyroBiasCalibration::start(const std::string& why) {
    if (!config_.enabled) {
        return;
    }
    calibrated_    = false;
    failed_        = false;
    bias_          = 0.0;
    attempt_start_ = MonotonicTime{};
    ++attempts_;
    note_ = why;
}

void GyroBiasCalibration::qualified(double rate_rad_s, MonotonicTime now) {
    if (!config_.enabled) {
        return;
    }
    if (!attempt_start_.isSet()) {
        attempt_start_ = now;
    }
    if (!calibrated_) {
        if (failed_) {
            return;   // bounded: only a new start retries
        }
        bias_       = rate_rad_s;
        calibrated_ = true;
        note_       = "bias from a qualified stationary window";
        return;
    }
    const double step = std::clamp(config_.maintain_weight * (rate_rad_s - bias_),
                                   -config_.maintain_step_rad_s, config_.maintain_step_rad_s);
    bias_ += step;
    ++steps_;
}

void GyroBiasCalibration::poll(MonotonicTime now) {
    if (!config_.enabled || calibrated_ || failed_) {
        return;
    }
    if (!attempt_start_.isSet() || !sameDomain(now, attempt_start_)) {
        attempt_start_ = now;
        return;
    }
    if (now - attempt_start_ > config_.attempt_ms) {
        failed_ = true;
        note_   = "no qualified stationary window within " +
                std::to_string(config_.attempt_ms / 1000) + " s; recalibrate retries";
    }
}

BiasCalibration GyroBiasCalibration::state(StationaryWindow::Phase phase) const {
    if (calibrated_) {
        return BiasCalibration::kDone;
    }
    if (failed_) {
        return BiasCalibration::kFailed;
    }
    switch (phase) {
    case StationaryWindow::Phase::kWaitingData: return BiasCalibration::kWaitingData;
    case StationaryWindow::Phase::kWaitingStill: return BiasCalibration::kWaitingStill;
    case StationaryWindow::Phase::kCollecting: return BiasCalibration::kRunning;
    }
    return BiasCalibration::kRunning;
}

StillnessStatus stillnessOf(const StationaryWindow& window, const GyroBiasCalibration* calibration) {
    StillnessStatus s;
    s.monitored   = !window.empty();
    s.stationary  = window.stationary();
    s.reason      = window.reason();
    s.progress_ms = window.progressMs();
    s.window_ms   = window.config().window_ms;
    s.windows     = window.windows();
    s.restarts    = window.restarts();
    if (calibration != nullptr) {
        s.calibration = calibration->state(window.phase());
        s.attempts    = calibration->attempts();
        s.steps       = calibration->steps();
        s.has_bias    = calibration->enabled() && calibration->calibrated();
        s.bias_rad_s  = calibration->bias();
        if (calibration->failed()) {
            s.reason = calibration->note();
        }
    }
    return s;
}

} // namespace navigatr
