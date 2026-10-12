// controller.h
// The controller source: Brain stdout bytes through a V5 controller, by CDC2 USER_READ polling on
// channel 1. It sends SYSTEM_VERSION, USER_READ and RADIO_STATUS and never writes to the Brain's
// stdin. It switches the radio channel only with radio_download (--radio-download).

#pragma once
#include <cstdint>
#include <optional>

#include "receiver/platform/stop_signal.h"
#include "receiver/sources/brain_usb.h"
#include "receiver/sources/cdc2.h"
#include "receiver/sources/source.h"

namespace spectagatr::receiver::sources
{

struct ControllerConfig {
    Milliseconds reply_timeout      = Milliseconds(250); // SYSTEM_VERSION, RADIO_STATUS
    Milliseconds poll_timeout       = Milliseconds(500); // USER_READ over the radio
    Milliseconds empty_backoff      = Milliseconds(15);  // after an empty poll
    Milliseconds link_retry         = Milliseconds(250); // no Brain linked
    Milliseconds write_budget       = Milliseconds(100);
    uint32_t     timeouts_to_relink = 3;       // in a row, then SYSTEM_VERSION again
    DurationUs   idle_relink_us     = 2000000; // no Brain bytes this long: SYSTEM_VERSION again
    DurationUs   radio_period_us    = 10000000; // RADIO_STATUS while linked; 0 never
    Byte         read_length        = kUserReadLength;
    uint32_t     pipeline           = 1; // USER_READs outstanding; over 1 is experimental
    DurationUs   stats_period_us    = 1000000; // status with stats at most this often

    // Opt-in: move a radio-linked controller to the download channel after linking, back to
    // pit before the port closes. The Brain link drops for a few seconds while it moves.
    bool         radio_download     = false;
    Milliseconds channel_ack_wait   = Milliseconds(2000);
    Milliseconds channel_relink     = Milliseconds(8000); // wait for the link to return
    Milliseconds channel_poll       = Milliseconds(250);
};

// Identifies and polls one open controller port until it closes (the reason) or stop (nullopt).
// Waits are port reads with a timeout, never sleeps.
std::optional<CloseReason> runControllerSession(Port& port, const ControllerConfig& config,
                                                const SourceHooks&    hooks,
                                                platform::StopSignal& stop, TransportStats& stats);

} // namespace spectagatr::receiver::sources
