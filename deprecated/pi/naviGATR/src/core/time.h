// time.h
// Monotonic millisecond count tagged with its clock domain. Device time
// counts from Pico boot, host time from process start; they drift and must
// never be differenced against each other. Debug builds assert on it.
//
// Camera observations use the host clock. Localization maps device-stamped
// wheel measurements onto that clock through DeviceToHostClock before
// publishing pose history for camera exposure-time lookups.

#pragma once
#include <cassert>
#include <cstdint>

namespace navigatr
{

enum class ClockDomain : uint8_t {
    kUnset = 0,   // default constructed, e.g. a NO_DATA_YET record
    kDevice,      // Pico clock
    kHost,        // Pi steady clock
};

struct MonotonicTime {
    int64_t     ms     = 0;
    ClockDomain domain = ClockDomain::kUnset;

    bool isSet() const { return domain != ClockDomain::kUnset; }
};

inline MonotonicTime deviceTime(int64_t ms) {
    return MonotonicTime{ms, ClockDomain::kDevice};
}

inline MonotonicTime hostTime(int64_t ms) { return MonotonicTime{ms, ClockDomain::kHost}; }

inline bool sameDomain(MonotonicTime a, MonotonicTime b) {
    return a.domain == b.domain || a.domain == ClockDomain::kUnset ||
           b.domain == ClockDomain::kUnset;
}

inline int64_t operator-(MonotonicTime a, MonotonicTime b) {
    assert(sameDomain(a, b));
    return a.ms - b.ms;
}

inline bool operator==(MonotonicTime a, MonotonicTime b) { return a.ms == b.ms; }
inline bool operator!=(MonotonicTime a, MonotonicTime b) { return a.ms != b.ms; }
inline bool operator<(MonotonicTime a, MonotonicTime b) {
    assert(sameDomain(a, b));
    return a.ms < b.ms;
}
inline bool operator<=(MonotonicTime a, MonotonicTime b) {
    assert(sameDomain(a, b));
    return a.ms <= b.ms;
}
inline bool operator>(MonotonicTime a, MonotonicTime b) {
    assert(sameDomain(a, b));
    return a.ms > b.ms;
}
inline bool operator>=(MonotonicTime a, MonotonicTime b) {
    assert(sameDomain(a, b));
    return a.ms >= b.ms;
}

inline double secondsBetween(MonotonicTime later, MonotonicTime earlier) {
    return (later - earlier) / 1000.0;
}

} // namespace navigatr
