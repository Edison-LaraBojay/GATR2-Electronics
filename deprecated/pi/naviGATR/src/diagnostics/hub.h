// hub.h
// The DiagnosticsHub: producers on the estimation and I/O threads post small
// DiagRecords; one consumer (the capture recorder) drains them on its own
// thread. Posting takes one short mutex and copies one record into a bounded
// ring; a kind nobody wants costs one atomic load. When the ring is full the
// new record is dropped and counted per kind, so a stalled consumer can
// never slow a producer or grow memory.
//
// Owned by System and kept across reset() and profile swaps; resources and
// pipeline slots receive a pointer through their initialization contexts
// (null in unit tests that build them alone: check before use).

#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostics/link_monitor.h"
#include "diagnostics/records.h"

namespace navigatr
{

struct DiagHubStats {
    std::array<uint64_t, kDiagKindCount> posted  = {};
    std::array<uint64_t, kDiagKindCount> dropped = {}; // ring full
    uint64_t    drained  = 0;
    std::size_t capacity = 0;
    std::size_t queued   = 0;
};

class DiagnosticsHub
{
public:
    explicit DiagnosticsHub(std::size_t capacity = 16384);

    DiagnosticsHub(const DiagnosticsHub&)            = delete;
    DiagnosticsHub& operator=(const DiagnosticsHub&) = delete;

    // A stable small id for a producer name (resource or link id). Call at
    // build time, not per record. Ids start at 1; 0 means unnamed.
    uint16_t    sourceId(const std::string& name);
    std::string sourceName(uint16_t id) const;
    std::vector<std::string> sourceNames() const; // index = id - 1

    // Producer side. wants() first for frequent kinds: build a record only
    // when a consumer asked for it. Low-rate kinds (Pico status and
    // diagnostics, Brain telemetry, paths, events) may post unconditionally.
    bool wants(DiagKind k) const {
        return (wanted_.load(std::memory_order_relaxed) & diagBit(k)) != 0;
    }
    // Stamps host_us when it is not positive (never 0 after stamping). Keeps it as the latest of its kind (except
    // raw bytes) and queues it when its kind is wanted. False when dropped
    // or not wanted.
    bool post(DiagRecord record);

    // Newest record of a kind, for live views. seq counts records of that
    // kind posted so far (0 = none yet).
    bool latest(DiagKind k, DiagRecord& out, uint64_t* seq = nullptr) const;

    // Transport monitors of this System.
    LinkMonitorRegistry&       links() { return *links_; }
    const LinkMonitorRegistry& links() const { return *links_; }

    // Consumer side (one consumer).
    void     setWanted(uint32_t kind_mask) { wanted_.store(kind_mask, std::memory_order_relaxed); }
    uint32_t wanted() const { return wanted_.load(std::memory_order_relaxed); }
    // Appends everything posted since the last call, oldest first.
    std::size_t drain(std::vector<DiagRecord>& out);

    DiagHubStats stats() const;

private:
    std::atomic<uint32_t> wanted_{0};

    std::unique_ptr<LinkMonitorRegistry> links_;

    mutable std::mutex      mutex_;
    std::vector<DiagRecord> ring_;
    std::array<DiagRecord, kDiagKindCount> latest_;
    std::array<uint64_t, kDiagKindCount>   latest_seq_ = {};
    std::size_t             head_  = 0; // next to drain
    std::size_t             count_ = 0;
    DiagHubStats            stats_;

    mutable std::mutex       names_mutex_;
    std::vector<std::string> names_;
};

} // namespace navigatr
