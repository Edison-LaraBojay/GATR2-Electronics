// pose_feed.cpp

#include "receiver/feed/pose_feed.h"

namespace spectagatr::receiver::feed
{

bool operator==(const FeedCounts& a, const FeedCounts& b) {
    return a.poses == b.poses && a.bad_crc == b.bad_crc && a.malformed == b.malformed &&
           a.out_of_range == b.out_of_range && a.repeated == b.repeated &&
           a.out_of_order == b.out_of_order && a.truncated == b.truncated &&
           a.too_long == b.too_long && a.garbage == b.garbage && a.sessions == b.sessions &&
           a.bytes == b.bytes;
}

FeedCounts PoseFeed::counts() const {
    const TrackerCounts&                tracked = tracker_.counts();
    const protocol::LineScanner::Stats& scanned = scanner_.stats();
    FeedCounts                          counts;
    counts.poses        = tracked.poses;
    counts.bad_crc      = tracked.bad_crc;
    counts.malformed    = tracked.malformed;
    counts.out_of_range = tracked.out_of_range;
    counts.repeated     = tracked.repeated;
    counts.out_of_order = tracked.out_of_order;
    counts.truncated    = scanned.truncated;
    counts.too_long     = scanned.too_long;
    counts.garbage      = scanned.garbage;
    counts.sessions     = tracked.sessions;
    counts.bytes        = bytes_;
    return counts;
}

} // namespace spectagatr::receiver::feed
