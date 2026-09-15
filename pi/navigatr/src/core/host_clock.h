// host_clock.h
// The one host clock every worker reads: steady_clock milliseconds since
// process start, tagged as the host domain. Foreign clocks must be
// correlated explicitly: libcamera specifies CLOCK_BOOTTIME, which differs
// from Linux steady_clock/CLOCK_MONOTONIC by time spent suspended.

#pragma once
#include <chrono>
#include <cstdint>

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

    // A raw steady_clock reading (nanoseconds since the clock's own epoch)
    // onto the host axis. This does not convert CLOCK_BOOTTIME or device
    // timestamps; those need a measured correspondence between clocks.
    static MonotonicTime fromSteadyNanoseconds(int64_t ns) {
        const auto origin_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(origin().time_since_epoch())
                .count();
        return hostTime((ns - origin_ns) / 1000000);
    }

    // Host axis back to steady_clock, for sleeping until a host time.
    static std::chrono::steady_clock::time_point toSteady(MonotonicTime host) {
        return origin() + std::chrono::milliseconds(host.ms);
    }

private:
    static std::chrono::steady_clock::time_point origin() {
        static const std::chrono::steady_clock::time_point kOrigin =
            std::chrono::steady_clock::now();
        return kOrigin;
    }
};

} // namespace navigatr
