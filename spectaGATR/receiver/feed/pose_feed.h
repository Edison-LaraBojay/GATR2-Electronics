// pose_feed.h
// Source bytes to accepted poses: LineScanner -> decodeLine -> PoseTracker, the one path every
// source (controller, Brain USB, sim, replay) goes through. Pure.

#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "receiver/feed/pose_tracker.h"
#include "spectagatr/protocol/pose_line.h"

namespace spectagatr::receiver::feed
{

struct FeedCounts {
    uint64_t poses        = 0;
    uint64_t bad_crc      = 0;
    uint64_t malformed    = 0;
    uint64_t out_of_range = 0;
    uint64_t repeated     = 0;
    uint64_t out_of_order = 0;
    uint64_t truncated    = 0;
    uint64_t too_long     = 0;
    uint64_t garbage      = 0;
    uint64_t sessions     = 0;
    uint64_t bytes        = 0;
};

bool operator==(const FeedCounts& a, const FeedCounts& b);

class PoseFeed {
  public:
    // Bytes as they arrived at rx_ms; on_pose(const TrackedPose&) per accepted pose.
    template <typename OnPose>
    void push(const uint8_t* data, std::size_t size, RxMillis rx_ms, OnPose&& on_pose) {
        bytes_ += size;
        scanner_.feed(data, size, [&](std::string_view line) {
            const std::optional<TrackedPose> pose =
                tracker_.offer(protocol::decodeLine(line), rx_ms);
            if (pose) {
                on_pose(*pose);
            }
        });
    }

    // The source reconnected: a partial line from before is not continued.
    void resetTransport() { scanner_.reset(); }

    // A replayed stream rewound.
    void forgetOrdering() { tracker_.forgetOrdering(); }

    FeedCounts              counts() const;
    std::optional<RxMillis> lastRx() const { return tracker_.lastRx(); }

  private:
    protocol::LineScanner scanner_;
    PoseTracker           tracker_;
    uint64_t              bytes_ = 0;
};

} // namespace spectagatr::receiver::feed
