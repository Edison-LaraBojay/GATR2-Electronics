// inspection_stats.h
// Counters the inspection service publishes about itself, read through a
// copy. Rates are measured, never assumed; a stalled service reports the
// stall.

#pragma once
#include <cstdint>

namespace navigatr
{

struct InspectionServiceStats {
    bool     running = false;
    int      port    = 0;   // bound port
    uint64_t clients = 0;   // connected websocket clients right now
    uint64_t clients_total = 0;
    uint64_t client_disconnects = 0;

    uint64_t snapshots_sent = 0;
    uint64_t frames_sent    = 0;
    uint64_t frames_skipped = 0;   // a client's buffer was full: frame not queued for it
    uint64_t snapshots_skipped = 0;
    uint64_t bytes_sent     = 0;

    uint64_t encodes         = 0;
    double   last_encode_ms  = 0.0;
    double   mean_encode_ms  = 0.0;
    double   snapshot_rate_hz = 0.0;   // measured over the last window
    double   frame_rate_hz    = 0.0;
};

} // namespace navigatr
