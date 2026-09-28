// hub.cpp

#include "diagnostics/hub.h"

#include <algorithm>

#include "core/host_clock.h"

namespace navigatr
{

const char* diagKindName(DiagKind k) {
    switch (k) {
    case DiagKind::kRobotState: return "robot_state";
    case DiagKind::kPicoSensor: return "pico_sensor";
    case DiagKind::kPicoStatus: return "pico_status";
    case DiagKind::kPicoDiag: return "pico_diag";
    case DiagKind::kVexImu: return "vex_imu";
    case DiagKind::kBrainRequest: return "brain_request";
    case DiagKind::kBrainTelemetry: return "brain_telemetry";
    case DiagKind::kPath: return "path";
    case DiagKind::kEvent: return "event";
    case DiagKind::kBytes: return "bytes";
    }
    return "unknown";
}

DiagnosticsHub::DiagnosticsHub(std::size_t capacity)
    : links_(std::make_unique<LinkMonitorRegistry>(this)), ring_(capacity > 0 ? capacity : 1) {
    stats_.capacity = ring_.size();
}

uint16_t DiagnosticsHub::sourceId(const std::string& name) {
    std::lock_guard<std::mutex> lock(names_mutex_);
    for (std::size_t i = 0; i < names_.size(); ++i) {
        if (names_[i] == name) {
            return static_cast<uint16_t>(i + 1);
        }
    }
    if (names_.size() >= 0xFFFE) {
        return 0;
    }
    names_.push_back(name);
    return static_cast<uint16_t>(names_.size());
}

std::string DiagnosticsHub::sourceName(uint16_t id) const {
    std::lock_guard<std::mutex> lock(names_mutex_);
    return id >= 1 && id <= names_.size() ? names_[id - 1] : std::string();
}

std::vector<std::string> DiagnosticsHub::sourceNames() const {
    std::lock_guard<std::mutex> lock(names_mutex_);
    return names_;
}

bool DiagnosticsHub::post(DiagRecord record) {
    if (record.host_us <= 0) {
        record.host_us = std::max<int64_t>(1, HostClock::nowUs());
    }
    const auto k = static_cast<std::size_t>(record.kind);
    if (k >= stats_.posted.size()) {
        return false;
    }
    const bool wanted = wants(record.kind);
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.posted[k];
    if (record.kind != DiagKind::kBytes) {
        latest_[k] = record;
        ++latest_seq_[k];
    }
    if (!wanted) {
        return false;
    }
    if (count_ == ring_.size()) {
        ++stats_.dropped[k];
        return false;
    }
    ring_[(head_ + count_) % ring_.size()] = std::move(record);
    ++count_;
    return true;
}

std::size_t DiagnosticsHub::drain(std::vector<DiagRecord>& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t n = count_;
    out.reserve(out.size() + n);
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back(std::move(ring_[(head_ + i) % ring_.size()]));
    }
    head_  = (head_ + n) % ring_.size();
    count_ = 0;
    stats_.drained += n;
    return n;
}

bool DiagnosticsHub::latest(DiagKind k, DiagRecord& out, uint64_t* seq) const {
    const auto i = static_cast<std::size_t>(k);
    if (i >= latest_.size()) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (seq != nullptr) {
        *seq = latest_seq_[i];
    }
    if (latest_seq_[i] == 0) {
        return false;
    }
    out = latest_[i];
    return true;
}

DiagHubStats DiagnosticsHub::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    DiagHubStats s = stats_;
    s.queued       = count_;
    return s;
}

} // namespace navigatr
