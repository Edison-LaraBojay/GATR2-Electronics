// inspection_stats.h
// Counters the inspection service publishes about itself, read through a
// copy. Rates are measured, never assumed; a stalled service reports the
// stall. "Queued" means handed to a client's outgoing queue; the per-client
// channel stats say what was actually written.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "inspection/http_server.h"

namespace navigatr
{

struct InspectionServiceStats {
    bool     running = false;
    int      port    = 0;   // bound port
    uint64_t clients = 0;   // connected websocket clients right now
    uint64_t clients_total = 0;
    uint64_t client_disconnects = 0;

    // inspect/1 names kept for existing readers: state messages queued, and
    // state messages replaced unsent or refused
    uint64_t snapshots_sent    = 0;
    uint64_t snapshots_skipped = 0;
    uint64_t frames_sent    = 0;   // previews queued
    uint64_t frames_skipped = 0;   // previews replaced unsent or refused
    uint64_t bytes_sent     = 0;

    uint64_t states_queued          = 0;
    uint64_t states_replaced        = 0;
    uint64_t diags_queued           = 0;
    uint64_t diags_replaced         = 0;
    uint64_t diags_skipped          = 0;   // content unchanged since the client's last diag
    uint64_t hellos_queued          = 0;
    uint64_t histories_queued       = 0;
    uint64_t events_queued          = 0;
    uint64_t captures_queued        = 0;
    uint64_t pongs_queued           = 0;
    uint64_t telemetry_queued       = 0;
    uint64_t instrumentation_queued = 0;
    uint64_t messages_replaced      = 0;   // any replaceable channel, server count
    uint64_t messages_refused       = 0;
    uint64_t closed_stalled           = 0;
    uint64_t closed_reliable_overflow = 0;
    uint64_t closed_protocol          = 0;
    uint64_t closed_peer              = 0;
    uint64_t flow_control_clients     = 0;   // open clients answering pings right now

    uint64_t encodes         = 0;
    double   last_encode_ms  = 0.0;
    double   mean_encode_ms  = 0.0;
    double   snapshot_rate_hz = 0.0;   // state messages queued per second, last window
    double   frame_rate_hz    = 0.0;
    double   diag_rate_hz     = 0.0;
};

// Build time and size of one document kind.
struct DocBuildStats {
    uint64_t count          = 0;
    double   last_us        = 0.0;
    double   mean_us        = 0.0;   // over every build
    double   recent_mean_us = 0.0;   // exponential, weight 0.1
    double   max_us         = 0.0;
    uint64_t last_bytes     = 0;
    double   mean_bytes     = 0.0;

    void add(double us, uint64_t bytes) {
        ++count;
        last_us        = us;
        last_bytes     = bytes;
        mean_us        += (us - mean_us) / static_cast<double>(count);
        mean_bytes     += (static_cast<double>(bytes) - mean_bytes) / static_cast<double>(count);
        recent_mean_us = count == 1 ? us : 0.9 * recent_mean_us + 0.1 * us;
        if (us > max_us) {
            max_us = us;
        }
    }
};

// What one feed client asked for.
struct FeedSubscription {
    double state_hz        = 30.0;
    bool   diag            = true;
    bool   instrumentation = false;
    bool   raw             = false;
    bool   decoded         = false;
    bool   telemetry       = true;
    double preview_hz      = 0.0;
};

struct FeedClientStats {
    WsClientStats    queue;
    FeedSubscription subscription;
    uint64_t         diag_skipped = 0;
};

// Per-client queues and document costs, for diag.inspection.
struct InspectionFeedStats {
    double        state_hz = 0.0;   // configured default
    double        diag_hz  = 0.0;
    DocBuildStats state;
    DocBuildStats diag;
    DocBuildStats history;
    DocBuildStats instrumentation;
    std::vector<FeedClientStats> clients;
    std::vector<WsCloseRecord>   recent_closes;
};

} // namespace navigatr
