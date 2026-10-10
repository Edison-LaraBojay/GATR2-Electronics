// serial_source.h
// The serial source loop: find a port, run the session its kind needs (Brain user port read raw,
// or controller polling), and after a loss go back to searching. The program never restarts.

#pragma once

#include "receiver/platform/stop_signal.h"
#include "receiver/sources/brain_usb.h"
#include "receiver/sources/controller.h"
#include "receiver/sources/source.h"

namespace spectagatr::receiver::sources
{

struct SerialSourceConfig {
    BrainUsbConfig   brain;
    ControllerConfig controller;
    Milliseconds     reconnect_wait = Milliseconds(1000); // between searches
    SourceKind       searching_kind = SourceKind::kNone;  // status src while nothing is open
};

// Returns once stop is requested. Thread: the caller's, one per source.
void runSerialSource(const SerialSourceConfig& config, const SourceHooks& hooks,
                     platform::StopSignal& stop);

} // namespace spectagatr::receiver::sources
