// feed_status.h
// The browser's status message as data: link state, staleness and receiver-local pose age. The
// age is how long ago this receiver accepted the newest pose (its own monotonic clock). It is not
// the end-to-end latency from the Brain, which nothing here measures. Pure.

#pragma once
#include <cstdint>
#include <optional>
#include <string>

#include "receiver/feed/pose_feed.h"

namespace spectagatr::receiver::feed
{

enum class LinkState : uint8_t { kOpen, kSearching, kError };

struct FeedStatus {
    RxMillis               rx_ms  = 0;
    std::string            source = "none"; // controller, brain, sim, file, none
    std::string            port;            // empty when none
    LinkState              link = LinkState::kSearching;
    std::string            detail;
    bool                   stale = true;
    std::optional<int64_t> pose_age_ms; // none before the first pose
    FeedCounts             counts;
};

// Whole ms since the newest accepted pose, never negative; none before the first.
std::optional<int64_t> poseAgeMs(std::optional<RxMillis> last_rx, RxMillis now);

// No pose yet, the newest older than stale_ms, or the link not open.
bool isStale(std::optional<RxMillis> last_rx, RxMillis now, double stale_ms, LinkState link);

// Worth sending at once: source, port, link, detail, stale or a drop count changed. The clock, the
// age and the poses and bytes counts move all the time and go with the periodic status.
bool statusChanged(const FeedStatus& sent, const FeedStatus& now);

const char* linkStateName(LinkState link); // "open", "searching", "error"

} // namespace spectagatr::receiver::feed
