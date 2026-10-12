// link_monitor.h
// Live transport instrumentation for one serial link, fed by the code that
// already reads and writes it (never a second reader). Counters are atomics
// and always on. Raw bytes and decoded-frame summaries are kept only while
// someone asked for them (the viewer's instrumentation panel or a capture),
// in bounded rings, so an idle monitor costs one atomic load per call.
//
// Attempted and accepted transmit bytes are counted apart: a write can be
// refused or cut short, and only accepted bytes can reach the other side.

#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostics/records.h"
#include "translaGATR/frame_codec.h"

namespace navigatr
{

class DiagnosticsHub;

struct LinkRawChunk {
    int64_t              host_us = 0;
    DiagDirection        dir     = DiagDirection::kRx;
    std::vector<uint8_t> bytes;
};

struct LinkDecodedEntry {
    int64_t     host_us = 0;
    bool        rx      = true;
    std::string name;   // message name, e.g. "sensor v2", "GET_STATE"
    std::string fields; // short "key=value ..." summary
};

struct LinkErrorEntry {
    int64_t     host_us = 0;
    std::string reason;
};

struct LinkMonitorSnapshot {
    std::string id;
    std::string kind; // "pico_uart", "brain_usb", "brain_rs485", ...
    uint64_t    rx_bytes       = 0;
    uint64_t    tx_attempted   = 0;
    uint64_t    tx_accepted    = 0;
    uint64_t    rx_frames      = 0; // valid decoded frames received
    uint64_t    tx_frames      = 0;
    uint64_t    rejected       = 0; // decode rejections reported by the owner
    translagatr::FrameReaderStats reader; // latest counters of the owner's FrameReader
    int64_t     last_rx_us       = -1;
    int64_t     last_tx_us       = -1;
    int64_t     last_valid_rx_us = -1;
    double      rx_bytes_per_s  = 0; // over the last full second
    double      tx_bytes_per_s  = 0;
    double      rx_frames_per_s = 0;
    bool        raw_on          = false;
    std::vector<LinkRawChunk>     raw;     // oldest first
    std::vector<LinkDecodedEntry> decoded; // oldest first
    std::vector<LinkErrorEntry>   errors;  // oldest first
};

class LinkMonitor
{
public:
    LinkMonitor(std::string id, std::string kind, DiagnosticsHub* hub);

    const std::string& id() const { return id_; }

    // Owner side, on the owner's thread.
    void rx(const uint8_t* data, std::size_t n);
    void tx(const uint8_t* data, std::size_t attempted, std::size_t accepted);
    void frame(bool rx, const char* name, const std::string& fields);
    void rejected(const char* reason);
    void readerStats(const translagatr::FrameReaderStats& stats);

    // Brain link: the request the commands slot processed this cycle, left
    // for the publishing slot to complete with the reply it actually sent.
    // One slot; staging over an uncompleted record posts that one as sent
    // with nothing (reply_len 0).
    void stageRequest(const DiagBrainRequest& r);
    bool takeStagedRequest(DiagBrainRequest& out);

    // Build a decoded summary string only when this is true.
    bool decodedOn() const { return decoded_on_.load(std::memory_order_relaxed); }
    bool rawOn() const { return raw_on_.load(std::memory_order_relaxed); }

    void setRaw(bool on) { raw_on_.store(on, std::memory_order_relaxed); }
    void setDecoded(bool on) { decoded_on_.store(on, std::memory_order_relaxed); }

    LinkMonitorSnapshot snapshot(std::size_t max_raw_bytes = 512,
                                 std::size_t max_decoded   = 32) const;

private:
    void rate(int64_t now_us);
    void pushRaw(DiagDirection dir, const uint8_t* data, std::size_t n, int64_t now_us);

    std::string     id_;
    std::string     kind_;
    DiagnosticsHub* hub_       = nullptr;
    uint16_t        source_id_ = 0;

    std::atomic<bool>     raw_on_{false};
    std::atomic<bool>     decoded_on_{false};
    std::atomic<uint64_t> rx_bytes_{0};
    std::atomic<uint64_t> tx_attempted_{0};
    std::atomic<uint64_t> tx_accepted_{0};
    std::atomic<uint64_t> rx_frames_{0};
    std::atomic<uint64_t> tx_frames_{0};
    std::atomic<uint64_t> rejected_{0};
    std::atomic<int64_t>  last_rx_us_{-1};
    std::atomic<int64_t>  last_tx_us_{-1};
    std::atomic<int64_t>  last_valid_rx_us_{-1};

    mutable std::mutex           mutex_; // rings, reader stats, rate window, staged request
    bool                         staged_valid_ = false;
    DiagBrainRequest             staged_;
    translagatr::FrameReaderStats reader_;
    std::deque<LinkRawChunk>     raw_;
    std::size_t                  raw_bytes_ = 0;
    std::deque<LinkDecodedEntry> decoded_;
    std::deque<LinkErrorEntry>   errors_;
    int64_t                      window_start_us_ = -1;
    uint64_t                     window_rx_bytes_ = 0;
    uint64_t                     window_tx_bytes_ = 0;
    uint64_t                     window_rx_frames_ = 0;
    double                       rx_bytes_per_s_  = 0;
    double                       tx_bytes_per_s_  = 0;
    double                       rx_frames_per_s_ = 0;
};

// Every link monitor of one System. Raw and decoded capture are switched
// for all links at once.
class LinkMonitorRegistry
{
public:
    explicit LinkMonitorRegistry(DiagnosticsHub* hub) : hub_(hub) {}

    // The monitor for id, created on first use (resource build time).
    std::shared_ptr<LinkMonitor> monitor(const std::string& id, const std::string& kind);
    std::vector<std::shared_ptr<LinkMonitor>> all() const;

    void setRaw(bool on);
    void setDecoded(bool on);
    bool raw() const { return raw_.load(std::memory_order_relaxed); }
    bool decoded() const { return decoded_.load(std::memory_order_relaxed); }

private:
    DiagnosticsHub*                           hub_ = nullptr;
    mutable std::mutex                        mutex_;
    std::vector<std::shared_ptr<LinkMonitor>> monitors_;
    std::atomic<bool>                         raw_{false};
    std::atomic<bool>                         decoded_{false};
};

} // namespace navigatr
