// feed_status_gtest.cpp
// Staleness and receiver-local age with an injected clock, change detection for status sends,
// and the live hub's periodic status.

#include "receiver/feed/feed_status.h"

#include <gtest/gtest.h>

#include "receiver/app/live_feed.h"
#include "receiver/sources/fakes/brain_stdio.h"

namespace spectagatr::receiver::feed
{
namespace
{

constexpr double kStaleMs = 750.0;

TEST(FeedStatus, NoPoseIsStaleWithNoAge) {
    EXPECT_TRUE(isStale(std::nullopt, 1000.0, kStaleMs, LinkState::kOpen));
    EXPECT_FALSE(poseAgeMs(std::nullopt, 1000.0));
}

TEST(FeedStatus, StaleOnlyPastTheThreshold) {
    const RxMillis last = 10000.0;
    EXPECT_FALSE(isStale(last, last, kStaleMs, LinkState::kOpen));
    EXPECT_FALSE(isStale(last, last + 749.9, kStaleMs, LinkState::kOpen));
    EXPECT_FALSE(isStale(last, last + 750.0, kStaleMs, LinkState::kOpen));
    EXPECT_TRUE(isStale(last, last + 750.1, kStaleMs, LinkState::kOpen));
    EXPECT_EQ(*poseAgeMs(last, last + 35.7), 35);
    EXPECT_EQ(*poseAgeMs(last, last - 5.0), 0); // never negative
}

TEST(FeedStatus, ALinkThatIsNotOpenIsStaleAtOnce) {
    const RxMillis last = 10000.0;
    EXPECT_TRUE(isStale(last, last + 1.0, kStaleMs, LinkState::kSearching));
    EXPECT_TRUE(isStale(last, last + 1.0, kStaleMs, LinkState::kError));
}

TEST(FeedStatus, ChangesWorthSendingAtOnce) {
    FeedStatus a;
    a.link        = LinkState::kOpen;
    a.source      = "sim";
    a.stale       = false;
    FeedStatus b  = a;
    b.rx_ms       = 500;
    b.pose_age_ms = 20;
    b.counts.poses += 10;
    b.counts.bytes += 1000;
    EXPECT_FALSE(statusChanged(a, b)); // clock, age, poses and bytes go with the period
    b.stale = true;
    EXPECT_TRUE(statusChanged(a, b));
    b                = a;
    b.counts.bad_crc = 1;
    EXPECT_TRUE(statusChanged(a, b));
    b        = a;
    b.detail = "controller: no Brain linked (flags 0x00)";
    EXPECT_TRUE(statusChanged(a, b));
}

TEST(FeedStatus, LinkNames) {
    EXPECT_STREQ(linkStateName(LinkState::kOpen), "open");
    EXPECT_STREQ(linkStateName(LinkState::kSearching), "searching");
    EXPECT_STREQ(linkStateName(LinkState::kError), "error");
}

// The hub with an injected receiver clock: fresh, then stale 750 ms after the last pose.
TEST(LiveFeedStatus, StaleTimingFollowsTheReceiverClock) {
    server::HttpServer server(server::HttpServerConfig{}, nullptr);
    app::LiveFeed      live(server, app::FeedSettings{});

    sources::SourceStatus open;
    open.src    = sources::SourceKind::kSim;
    open.state  = sources::PortState::kOpen;
    open.detail = "sim: figure8";
    live.onStatus(open);
    EXPECT_TRUE(live.status(0).stale); // open, but no pose yet

    sources::fakes::FakeBrainStdio brain(1);
    sources::SourceChunk           chunk;
    chunk.src   = sources::SourceKind::kSim;
    chunk.rx_us = 2000000; // 2000 ms
    chunk.bytes = brain.pose(0, 0, 0, 0);
    live.onChunk(chunk);

    EXPECT_FALSE(live.status(2000.0).stale);
    EXPECT_EQ(*live.status(2100.0).pose_age_ms, 100);
    EXPECT_FALSE(live.status(2750.0).stale);
    const FeedStatus late = live.status(2750.5);
    EXPECT_TRUE(late.stale);
    EXPECT_EQ(*late.pose_age_ms, 750);
    EXPECT_EQ(late.source, "sim");
    EXPECT_EQ(late.counts.poses, 1u);

    sources::SourceStatus lost = open;
    lost.state                 = sources::PortState::kClosed;
    lost.detail                = "COM9: device gone";
    live.onStatus(lost);
    const FeedStatus gone = live.status(2001.0);
    EXPECT_TRUE(gone.stale); // the link is not open
    EXPECT_EQ(gone.link, LinkState::kError);
    EXPECT_EQ(gone.detail, "lost: COM9: device gone");
}

TEST(LiveFeedStatus, ControllerDetailShowsTheRadio) {
    sources::SourceStatus status;
    status.src         = sources::SourceKind::kController;
    status.state       = sources::PortState::kOpen;
    status.detail      = "controller: Brain linked by radio";
    status.stats.radio = sources::RadioStatus{4, 87, -50, 55, 1};
    EXPECT_EQ(app::detailOf(status), "controller: Brain linked by radio, radio quality 87");
    status.state  = sources::PortState::kNoBrainLinked;
    status.detail = "controller: no Brain linked (flags 0x00)";
    EXPECT_EQ(app::detailOf(status), "controller: no Brain linked (flags 0x00)");
    EXPECT_EQ(app::linkStateOf(status.state), LinkState::kSearching);
}

} // namespace
} // namespace spectagatr::receiver::feed
