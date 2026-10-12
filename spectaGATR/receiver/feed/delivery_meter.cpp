// delivery_meter.cpp

#include "receiver/feed/delivery_meter.h"

#include <algorithm>

namespace spectagatr::receiver::feed
{

void DeliveryMeter::onPose(const TrackedPose& pose) {
    const RxMillis rx = pose.rx_ms;
    if (!window_start_) {
        window_start_ = rx;
    }
    if (last_rx_) {
        const double gap = rx - *last_rx_;
        max_gap_ms_      = std::max(max_gap_ms_, gap);
        if (gap > stale_ms_) {
            ++stale_gaps_;
        }
    }
    last_rx_ = rx;
    ++poses_;

    const protocol::PoseRecord& r   = pose.record;
    const double                lag = rx - static_cast<double>(r.t_ms);
    if (!have_session_ || pose.restart || r.session != session_ || r.epoch != epoch_) {
        have_session_ = true;
        session_      = r.session;
        epoch_        = r.epoch;
        min_lag_ms_   = lag;
    } else {
        if (r.seq > last_seq_ + 1) {
            seq_lost_ += r.seq - last_seq_ - 1;
        }
        min_lag_ms_ = std::min(min_lag_ms_, lag);
    }
    last_seq_   = r.seq;
    max_lag_ms_ = std::max(max_lag_ms_, lag - min_lag_ms_);
}

DeliveryWindow DeliveryMeter::take(RxMillis now) {
    if (!window_start_) {
        window_start_ = now;
    }
    DeliveryWindow w;
    w.seconds    = std::max(0.0, (now - *window_start_) / 1000.0);
    w.poses      = poses_;
    w.rate_hz    = w.seconds > 0 ? poses_ / w.seconds : 0;
    w.max_gap_ms = max_gap_ms_;
    if (last_rx_) {
        w.max_gap_ms = std::max(w.max_gap_ms, now - *last_rx_);
    }
    w.seq_lost    = seq_lost_;
    w.stale_gaps  = stale_gaps_;
    w.max_lag_ms  = max_lag_ms_;
    w.bytes       = bytes_;
    w.chunks      = chunks_;
    w.bytes_per_s = w.seconds > 0 ? bytes_ / w.seconds : 0;

    window_start_ = now;
    poses_        = 0;
    max_gap_ms_   = 0;
    seq_lost_     = 0;
    stale_gaps_   = 0;
    max_lag_ms_   = 0;
    bytes_        = 0;
    chunks_       = 0;
    return w;
}

} // namespace spectagatr::receiver::feed
