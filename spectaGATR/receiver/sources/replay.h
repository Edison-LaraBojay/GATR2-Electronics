// replay.h
// The file source: bytes from a file or stdin ("-") handed on in small chunks at a modest pace,
// whatever they hold (raw console text, PROS COBS packets, captures). For integration without
// hardware: a host build of the Brain publisher writes lines to a file or pipe, the receiver
// shows them. Link open while bytes flow; searching at the end (so the feed turns stale).

#pragma once
#include <cstddef>
#include <string>

#include "receiver/platform/stop_signal.h"
#include "receiver/sources/source.h"

namespace spectagatr::receiver::sources
{

constexpr const char* kStandardInputPath = "-";

struct ReplayConfig {
    std::string  path        = kStandardInputPath;
    std::size_t  chunk_bytes = 64;                 // per interval
    Milliseconds interval    = Milliseconds(5);    // 64 B / 5 ms = 12.8 kB/s
    bool         loop        = false;              // files only: rewind at the end
    Milliseconds retry_wait  = Milliseconds(1000); // file missing: look again
};

// The chunk size for bytes_per_second at the config's interval, at least 1.
std::size_t replayChunkBytes(double bytes_per_second, Milliseconds interval);

// Returns once stop is requested; a blocked stdin read returns only with input or its end.
void runReplaySource(const ReplayConfig& config, const SourceHooks& hooks,
                     platform::StopSignal& stop);

} // namespace spectagatr::receiver::sources
