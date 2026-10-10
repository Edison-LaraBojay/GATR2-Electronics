// clock.h
// Host time: the one steady clock and duration aliases.

#pragma once
#include <chrono>

namespace spectagatr::receiver::platform
{

using SteadyClock  = std::chrono::steady_clock;
using Milliseconds = std::chrono::milliseconds;

} // namespace spectagatr::receiver::platform
