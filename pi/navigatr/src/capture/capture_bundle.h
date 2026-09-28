// capture_bundle.h
// Turns the records of one capture into the gatr2.capture/1 bundle: a
// store-only ZIP of metadata.json, README.txt and one CSV per selected
// stream (docs/capture.md). Pure: no clocks, no threads, no System, so the
// same records always give the same bytes.
//
// Rows keep each record's own values: nothing is resampled onto a common
// grid, missing values are empty fields, and every row carries the
// identifiers (Pi session and reset count, odometry epoch, Pico boot and
// epochs, Brain session) that say which segment it belongs to. Segments are
// listed in the metadata and never joined.

#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "diagnostics/records.h"

namespace navigatr
{

class JsonWriter;

constexpr const char* kCaptureSchema = "gatr2.capture/1";

using DiagKindCounts = std::array<uint64_t, kDiagKindCount>;

// A note on the capture's own timeline (trigger, end, Pi reset), written to
// events.csv with kind "capture" and to the metadata.
struct CaptureMarker {
    int64_t     host_us = 0;
    std::string text;
};

struct CaptureBundleInput {
    std::string id;
    std::string pi_session;
    // (from host_us, reset count), ascending; a record takes the count of
    // the last entry at or before its host_us, the first entry's otherwise.
    std::vector<std::pair<int64_t, uint64_t>> resets;
    std::vector<std::string> source_names; // DiagnosticsHub::sourceNames(), id - 1
    const std::deque<DiagRecord>* records = nullptr;
    uint32_t kinds = 0; // selected streams, diagBit mask

    std::string reason;    // "manual" or an automatic trigger name
    std::string requester; // "http", "viewer ...", "auto"
    std::string detail;    // the fault text for an automatic trigger
    double pre_s_requested  = 0;
    double post_s_requested = 0;
    double pre_s            = 0; // after clamping to the configuration
    double post_s           = 0;
    int64_t trigger_us      = 0; // Pi host clock
    int64_t window_start_us = 0; // trigger - pre_s
    int64_t end_us          = 0; // when recording stopped
    int64_t rolling_oldest_us = -1; // oldest rolling record at the trigger, -1 none
    int64_t created_unix_ms   = -1; // Pi system clock at the trigger, -1 unknown
    bool        truncated = false;
    std::string stop_reason; // post_window, record_limit, memory_limit, shutdown

    DiagKindCounts hub_posted{};    // posted to the hub during the window
    DiagKindCounts hub_dropped{};   // hub ring full during the window
    DiagKindCounts not_selected{};  // drained but not a selected stream
    DiagKindCounts limit_dropped{}; // refused by the capture limits
    // records stamped inside the window (to the millisecond) that the record
    // bound pushed out of the rolling window before the trigger
    uint64_t    rolling_evicted = 0;
    uint64_t    faults_dropped  = 0; // fault notices over the queue bound
    std::size_t hub_capacity    = 0;
    std::size_t record_cap      = 0; // records one capture may hold
    std::vector<CaptureMarker> markers;

    // Writes the host's keys (configuration, profile, calibration,
    // localization, software) into the open metadata object; they are read
    // when the bundle is built, at host_end_us. Optional.
    std::function<void(JsonWriter&)> write_host;
    int64_t host_end_us = -1;
    // The same keys as one JSON object read at or before the window start,
    // with its own sampled_pi_host_us; empty when none was read.
    std::string host_at_start;

    std::size_t max_bytes = 64u << 20; // CSV text bound; rows past it are omitted and counted

    // Called once every row is written, before the ZIP is built: the owner
    // may release the records then, so they and the ZIP are never held
    // together. Optional.
    std::function<void()> rows_written;
};

struct CaptureBundleOutput {
    std::string              zip;
    DiagKindCounts           rows{};
    DiagKindCounts           rows_omitted{}; // max_bytes reached
    // Records were left out: in.truncated, or rows over max_bytes.
    // truncated_by names each cause (the stop reason, bundle_size_limit).
    bool                     truncated = false;
    std::vector<std::string> truncated_by;
    std::vector<std::string> files;
    std::size_t              csv_bytes = 0;
};

bool buildCaptureBundle(const CaptureBundleInput& in, CaptureBundleOutput& out, std::string& err);

// The CSV file of a stream, e.g. "robot_state.csv".
const char* captureStreamFile(DiagKind k);

// The stream mask for "default", "all" or a comma list of kind names; false
// with bad naming the first unknown item.
bool parseCaptureStreams(const std::string& text, uint32_t& kinds, std::string& bad);
constexpr uint32_t kCaptureDefaultKinds = kDiagAllKinds & ~diagBit(DiagKind::kBytes);

// RFC 4180 quoting when the text holds a comma, quote or line break.
std::string csvEscape(const std::string& s);

} // namespace navigatr
