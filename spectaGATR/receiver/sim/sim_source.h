// sim_source.h
// The sim source: trajectory.h's bytes for tick k, handed on at k / rate on the receiver clock,
// as a serial source hands on what it reads. The bytes then take the same LineScanner, decode and
// tracker path as the real sources.

#pragma once

#include "receiver/platform/stop_signal.h"
#include "receiver/sim/trajectory.h"
#include "receiver/sources/source.h"

namespace spectagatr::receiver::sim
{

// Returns once stop is requested. Thread: the caller's.
void runSimSource(const SimConfig& config, const sources::SourceHooks& hooks,
                  platform::StopSignal& stop);

} // namespace spectagatr::receiver::sim
