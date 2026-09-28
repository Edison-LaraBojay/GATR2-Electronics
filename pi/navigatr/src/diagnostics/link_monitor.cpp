// link_monitor.cpp

#include "diagnostics/link_monitor.h"

#include <algorithm>
#include <cstring>

#include "core/host_clock.h"
#include "diagnostics/hub.h"

namespace navigatr
{
namespace
{

constexpr std::size_t kRawKeptBytes = 4096; // per link, newest kept
constexpr std::size_t kDecodedKept  = 64;
constexpr std::size_t kErrorsKept   = 32;

} // namespace

LinkMonitor::LinkMonitor(std::string id, std::string kind, DiagnosticsHub* hub)
    : id_(std::move(id)), kind_(std::move(kind)), hub_(hub) {
    if (hub_ != nullptr) {
        source_id_ = hub_->sourceId(id_);
    }
}

void LinkMonitor::rate(int64_t now_us) {
    // caller holds mutex_
    if (window_start_us_ < 0) {
        window_start_us_ = now_us;
        return;
    }
    const int64_t span = now_us - window_start_us_;
    if (span >= 1000000) {
        const double s   = static_cast<double>(span) * 1e-6;
        rx_bytes_per_s_  = static_cast<double>(window_rx_bytes_) / s;
        tx_bytes_per_s_  = static_cast<double>(window_tx_bytes_) / s;
        rx_frames_per_s_ = static_cast<double>(window_rx_frames_) / s;
        window_start_us_ = now_us;
        window_rx_bytes_ = window_tx_bytes_ = window_rx_frames_ = 0;
    }
}

void LinkMonitor::pushRaw(DiagDirection dir, const uint8_t* data, std::size_t n, int64_t now_us) {
    // caller holds mutex_
    LinkRawChunk c;
    c.host_us = now_us;
    c.dir     = dir;
    c.bytes.assign(data, data + n);
    raw_bytes_ += n;
    raw_.push_back(std::move(c));
    while (raw_bytes_ > kRawKeptBytes && !raw_.empty()) {
        raw_bytes_ -= raw_.front().bytes.size();
        raw_.pop_front();
    }
}

void LinkMonitor::rx(const uint8_t* data, std::size_t n) {
    if (n == 0) {
        return;
    }
    const int64_t now = HostClock::nowUs();
    rx_bytes_.fetch_add(n, std::memory_order_relaxed);
    last_rx_us_.store(now, std::memory_order_relaxed);
    const bool raw      = rawOn();
    const bool to_hub   = hub_ != nullptr && hub_->wants(DiagKind::kBytes);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        window_rx_bytes_ += n;
        rate(now);
        if (raw) {
            pushRaw(DiagDirection::kRx, data, n, now);
        }
    }
    if (to_hub) {
        for (std::size_t off = 0; off < n; off += 64) {
            DiagRecord r;
            r.kind    = DiagKind::kBytes;
            r.source  = source_id_;
            r.host_us = now;
            DiagBytes b;
            b.dir = DiagDirection::kRx;
            b.len = static_cast<uint8_t>(std::min<std::size_t>(64, n - off));
            std::memcpy(b.data, data + off, b.len);
            r.payload = b;
            hub_->post(r);
        }
    }
}

void LinkMonitor::tx(const uint8_t* data, std::size_t attempted, std::size_t accepted) {
    const int64_t now = HostClock::nowUs();
    tx_attempted_.fetch_add(attempted, std::memory_order_relaxed);
    tx_accepted_.fetch_add(accepted, std::memory_order_relaxed);
    last_tx_us_.store(now, std::memory_order_relaxed);
    const bool raw    = rawOn();
    const bool to_hub = hub_ != nullptr && hub_->wants(DiagKind::kBytes);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        window_tx_bytes_ += accepted;
        rate(now);
        if (raw && attempted > 0) {
            pushRaw(accepted == attempted ? DiagDirection::kTxAccepted : DiagDirection::kTxAttempted,
                    data, attempted, now);
        }
    }
    if (to_hub && attempted > 0) {
        for (std::size_t off = 0; off < attempted; off += 64) {
            DiagRecord r;
            r.kind    = DiagKind::kBytes;
            r.source  = source_id_;
            r.host_us = now;
            DiagBytes b;
            b.dir = off < accepted ? DiagDirection::kTxAccepted : DiagDirection::kTxAttempted;
            b.len = static_cast<uint8_t>(std::min<std::size_t>(64, attempted - off));
            std::memcpy(b.data, data + off, b.len);
            r.payload = b;
            hub_->post(r);
        }
    }
}

void LinkMonitor::frame(bool rx, const char* name, const std::string& fields) {
    const int64_t now = HostClock::nowUs();
    if (rx) {
        rx_frames_.fetch_add(1, std::memory_order_relaxed);
        last_valid_rx_us_.store(now, std::memory_order_relaxed);
    } else {
        tx_frames_.fetch_add(1, std::memory_order_relaxed);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (rx) {
        ++window_rx_frames_;
    }
    rate(now);
    if (decodedOn()) {
        decoded_.push_back(LinkDecodedEntry{now, rx, name != nullptr ? name : "", fields});
        while (decoded_.size() > kDecodedKept) {
            decoded_.pop_front();
        }
    }
}

void LinkMonitor::rejected(const char* reason) {
    rejected_.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(mutex_);
    errors_.push_back(LinkErrorEntry{HostClock::nowUs(), reason != nullptr ? reason : ""});
    while (errors_.size() > kErrorsKept) {
        errors_.pop_front();
    }
}

void LinkMonitor::readerStats(const translagatr::FrameReaderStats& stats) {
    std::lock_guard<std::mutex> lock(mutex_);
    reader_ = stats;
}

void LinkMonitor::stageRequest(const DiagBrainRequest& r) {
    DiagBrainRequest unsent;
    bool             post = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (staged_valid_) {
            unsent           = staged_;
            unsent.reply_len = 0;
            post             = true;
        }
        staged_       = r;
        staged_valid_ = true;
    }
    if (post && hub_ != nullptr) {
        DiagRecord record;
        record.kind    = DiagKind::kBrainRequest;
        record.source  = source_id_;
        record.payload = unsent;
        hub_->post(std::move(record));
    }
}

bool LinkMonitor::takeStagedRequest(DiagBrainRequest& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!staged_valid_) {
        return false;
    }
    out           = staged_;
    staged_valid_ = false;
    return true;
}

LinkMonitorSnapshot LinkMonitor::snapshot(std::size_t max_raw_bytes, std::size_t max_decoded) const {
    LinkMonitorSnapshot s;
    s.id               = id_;
    s.kind             = kind_;
    s.rx_bytes         = rx_bytes_.load(std::memory_order_relaxed);
    s.tx_attempted     = tx_attempted_.load(std::memory_order_relaxed);
    s.tx_accepted      = tx_accepted_.load(std::memory_order_relaxed);
    s.rx_frames        = rx_frames_.load(std::memory_order_relaxed);
    s.tx_frames        = tx_frames_.load(std::memory_order_relaxed);
    s.rejected         = rejected_.load(std::memory_order_relaxed);
    s.last_rx_us       = last_rx_us_.load(std::memory_order_relaxed);
    s.last_tx_us       = last_tx_us_.load(std::memory_order_relaxed);
    s.last_valid_rx_us = last_valid_rx_us_.load(std::memory_order_relaxed);
    s.raw_on           = rawOn();
    const int64_t now  = HostClock::nowUs();
    std::lock_guard<std::mutex> lock(mutex_);
    s.reader          = reader_;
    s.rx_bytes_per_s  = rx_bytes_per_s_;
    s.tx_bytes_per_s  = tx_bytes_per_s_;
    s.rx_frames_per_s = rx_frames_per_s_;
    // Rates only roll over on traffic; a quiet link would keep its last
    // rate. Past two seconds use the open window, which falls toward zero.
    if (window_start_us_ >= 0 && now - window_start_us_ >= 2000000) {
        const double span = static_cast<double>(now - window_start_us_) * 1e-6;
        s.rx_bytes_per_s  = static_cast<double>(window_rx_bytes_) / span;
        s.tx_bytes_per_s  = static_cast<double>(window_tx_bytes_) / span;
        s.rx_frames_per_s = static_cast<double>(window_rx_frames_) / span;
    }
    std::size_t taken = 0;
    for (auto it = raw_.rbegin(); it != raw_.rend() && taken < max_raw_bytes; ++it) {
        s.raw.push_back(*it);
        taken += it->bytes.size();
    }
    std::reverse(s.raw.begin(), s.raw.end());
    const std::size_t from = decoded_.size() > max_decoded ? decoded_.size() - max_decoded : 0;
    s.decoded.assign(decoded_.begin() + static_cast<std::ptrdiff_t>(from), decoded_.end());
    s.errors.assign(errors_.begin(), errors_.end());
    return s;
}

std::shared_ptr<LinkMonitor> LinkMonitorRegistry::monitor(const std::string& id,
                                                          const std::string& kind) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& m : monitors_) {
        if (m->id() == id) {
            return m;
        }
    }
    auto m = std::make_shared<LinkMonitor>(id, kind, hub_);
    m->setRaw(raw_.load());
    m->setDecoded(decoded_.load());
    monitors_.push_back(m);
    return m;
}

std::vector<std::shared_ptr<LinkMonitor>> LinkMonitorRegistry::all() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return monitors_;
}

void LinkMonitorRegistry::setRaw(bool on) {
    raw_.store(on);
    for (const auto& m : all()) {
        m->setRaw(on);
    }
}

void LinkMonitorRegistry::setDecoded(bool on) {
    decoded_.store(on);
    for (const auto& m : all()) {
        m->setDecoded(on);
    }
}

} // namespace navigatr
