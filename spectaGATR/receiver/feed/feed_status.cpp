// feed_status.cpp

#include "receiver/feed/feed_status.h"

#include <cmath>

namespace spectagatr::receiver::feed
{

namespace
{

bool dropsChanged(const FeedCounts& a, const FeedCounts& b) {
    return a.bad_crc != b.bad_crc || a.malformed != b.malformed ||
           a.out_of_range != b.out_of_range || a.repeated != b.repeated ||
           a.out_of_order != b.out_of_order || a.truncated != b.truncated ||
           a.too_long != b.too_long || a.garbage != b.garbage || a.sessions != b.sessions;
}

} // namespace

std::optional<int64_t> poseAgeMs(std::optional<RxMillis> last_rx, RxMillis now) {
    if (!last_rx) {
        return std::nullopt;
    }
    const double age = now - *last_rx;
    return age <= 0.0 ? 0 : static_cast<int64_t>(std::floor(age));
}

bool isStale(std::optional<RxMillis> last_rx, RxMillis now, double stale_ms, LinkState link) {
    if (!last_rx || link != LinkState::kOpen) {
        return true;
    }
    return now - *last_rx > stale_ms;
}

bool statusChanged(const FeedStatus& sent, const FeedStatus& now) {
    return sent.source != now.source || sent.port != now.port || sent.link != now.link ||
           sent.detail != now.detail || sent.stale != now.stale ||
           dropsChanged(sent.counts, now.counts);
}

const char* linkStateName(LinkState link) {
    switch (link) {
    case LinkState::kOpen:
        return "open";
    case LinkState::kSearching:
        return "searching";
    case LinkState::kError:
        return "error";
    }
    return "error";
}

} // namespace spectagatr::receiver::feed
