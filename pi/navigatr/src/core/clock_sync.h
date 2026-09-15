// clock_sync.h
// Device-to-host clock mapping. Device-stamped data (Pico packets) arrives
// on the host after UART transfer and cycle batching, so the host receive
// time overestimates the true host time of the measurement by a variable
// latency. Tracking the minimum observed (host - device) offset over a
// sliding window converges on the transfer with the least latency, giving a
// stable mapping that survives batching jitter. Device clock drift is
// bounded by the window naturally: old minima age out.
//
// Observations must pair a device stamp with the actual host RECEIPT time
// of that data (StoredSample.receivedAt), never a later pipeline
// time; downstream delay would otherwise masquerade as clock offset.
//
// The mapping declares itself valid only after a warm-up of several
// observations, so one unlucky first pairing cannot anchor it. A window
// minimum can still revise downward as better pairings arrive; consumers
// of mapped times that require monotonicity handle that revision
// explicitly (pose history clamps to its newest entry).
//
// One mapper instance serves one device clock. A device reboot restarts its
// clock; the owner must reset the mapper when it detects the discontinuity
// (the same event that bumps the odometry epoch).

#pragma once
#include <cstddef>
#include <cstdint>

#include "core/time.h"

namespace navigatr
{

class DeviceToHostClock
{
public:
    // One (device stamp, host receive) pair. host is an upper bound on the
    // true host time of the measurement.
    void observe(MonotonicTime device, MonotonicTime host) {
        if (device.domain != ClockDomain::kDevice || host.domain != ClockDomain::kHost) {
            return;
        }
        offsets_[next_] = host.ms - device.ms;
        next_           = (next_ + 1) % kWindow;
        if (count_ < kWindow) {
            ++count_;
        }
        int64_t best = offsets_[0];
        for (std::size_t i = 1; i < count_; ++i) {
            if (offsets_[i] < best) {
                best = offsets_[i];
            }
        }
        best_ = best;
    }

    // Valid only after warm-up; a single pairing is not a clock model.
    bool valid() const { return count_ >= kMinObservations; }

    // Host time of a device-stamped measurement. Only meaningful after at
    // least one observation; callers check valid().
    MonotonicTime toHost(MonotonicTime device) const {
        return hostTime(device.ms + best_);
    }

    void reset() {
        count_ = 0;
        next_  = 0;
        best_  = 0;
    }

private:
    static constexpr std::size_t kWindow          = 64;
    static constexpr std::size_t kMinObservations = 8;

    int64_t     offsets_[kWindow] = {};
    std::size_t count_            = 0;
    std::size_t next_             = 0;
    int64_t     best_             = 0;
};

} // namespace navigatr
