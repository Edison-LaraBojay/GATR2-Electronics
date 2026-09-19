// host_clock.h
// The one host clock every worker reads: steady_clock milliseconds since
// process start, tagged as the host domain. Foreign clocks must be
// correlated explicitly: libcamera specifies CLOCK_BOOTTIME, which differs
// from Linux steady_clock/CLOCK_MONOTONIC by time spent suspended.

#pragma once
#include <chrono>

#include "core/time.h"

namespace navigatr
{

class HostClock
{
public:
    // Milliseconds since the process origin, host domain.
    static MonotonicTime now() {
        const auto elapsed = std::chrono::steady_clock::now() - origin();
        return hostTime(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
    }

private:
    static std::chrono::steady_clock::time_point origin() {
        static const std::chrono::steady_clock::time_point kOrigin =
            std::chrono::steady_clock::now();
        return kOrigin;
    }
};

} // namespace navigatr
