// delivery_meter.h
// What the viewer actually receives, per window: accepted poses, rate, the longest gap between
// poses, lines lost in transport (seq gaps), gaps long enough to go stale, and lag growth.
// Lag is rx_ms - t_ms against the smallest value seen this session: the clocks are unrelated,
// but a growing difference means poses arrive older and older (a queue somewhere). Pure.

#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>

#include "receiver/feed/pose_tracker.h"

namespace spectagatr::receiver::feed
{

struct DeliveryWindow {
    double   seconds     = 0;
    uint32_t poses       = 0;
    double   rate_hz     = 0;
    double   max_gap_ms  = 0; // includes the gap still open at the end of the window
    uint32_t seq_lost    = 0;
    uint32_t stale_gaps  = 0; // gaps over stale_ms
    double   max_lag_ms  = 0; // above the session's smallest lag
    uint64_t bytes       = 0; // source bytes
    uint32_t chunks      = 0; // source deliveries with bytes (controller: replies with data)
    double   bytes_per_s = 0;
};

class DeliveryMeter {
  public:
    explicit DeliveryMeter(double stale_ms) : stale_ms_(stale_ms) {}

    void onBytes(std::size_t count) {
        bytes_ += count;
        ++chunks_;
    }
    void onPose(const TrackedPose& pose);

    // The window since the previous take (or the first call), then a new window.
    DeliveryWindow take(RxMillis now);

  private:
    double                  stale_ms_;
    std::optional<RxMillis> window_start_;
    std::optional<RxMillis> last_rx_;
    uint32_t                poses_      = 0;
    double                  max_gap_ms_ = 0;
    uint32_t                seq_lost_   = 0;
    uint32_t                stale_gaps_ = 0;
    double                  max_lag_ms_ = 0;
    uint64_t                bytes_      = 0;
    uint32_t                chunks_     = 0;

    bool                 have_session_ = false;
    protocol::SessionId  session_      = 0;
    protocol::Epoch      epoch_        = 0;
    protocol::Sequence   last_seq_     = 0;
    double               min_lag_ms_   = 0;
};

} // namespace spectagatr::receiver::feed
