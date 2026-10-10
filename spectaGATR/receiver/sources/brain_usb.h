// brain_usb.h
// The Brain's USB user port read raw, as the Brain writes stdout (PROS COBS on or off). Never
// writes to the Brain: stdin bytes such as "pRc" change PROS serial settings for every program.

#pragma once
#include <cstddef>
#include <optional>
#include <string>

#include "receiver/platform/stop_signal.h"
#include "receiver/sources/source.h"

namespace spectagatr::receiver::sources
{

using CloseReason = std::string;

struct BrainUsbConfig {
    Milliseconds read_wait       = Milliseconds(100); // stop latency
    std::size_t  read_bytes      = 4096;
    DurationUs   stats_period_us = 1000000; // status with stats at most this often
};

// Reads one open user port until it closes (the reason) or stop (nullopt). Reports kOpen first.
std::optional<CloseReason> runBrainUsbSession(Port& port, const BrainUsbConfig& config,
                                              const SourceHooks& hooks, platform::StopSignal& stop,
                                              TransportStats& stats);

} // namespace spectagatr::receiver::sources
