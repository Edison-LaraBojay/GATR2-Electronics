// run_source.h
// The one source the options name, run on the caller's thread until stop.

#pragma once
#include <string>

#include "receiver/app/command_line.h"
#include "receiver/platform/stop_signal.h"
#include "receiver/sources/source.h"

namespace spectagatr::receiver::app
{

// hooks.open_port is set here for the serial sources.
void runSource(const Options& options, sources::SourceHooks hooks, platform::StopSignal& stop);

// --list-ports: every USB serial port with its VEX role, and what auto would pick.
std::string describePorts();

} // namespace spectagatr::receiver::app
