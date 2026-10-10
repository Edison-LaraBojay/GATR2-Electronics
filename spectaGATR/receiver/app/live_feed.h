// live_feed.h
// The running receiver's hub. Source chunks go through the PoseFeed; each accepted pose goes to
// every browser at once, latest-wins (a restart pose goes in order, so its trail break is never
// replaced away). Status goes on a change and at least every 250 ms. A new browser gets hello
// first, then the current status and newest pose. Thread-safe: the source thread, the server
// thread and the main loop all call in.

#pragma once
#include <mutex>
#include <optional>
#include <set>
#include <string>

#include "receiver/feed/feed_json.h"
#include "receiver/feed/feed_status.h"
#include "receiver/feed/pose_feed.h"
#include "receiver/server/http_server.h"
#include "receiver/sources/source.h"

namespace spectagatr::receiver::app
{

using ClientId = server::HttpServer::ClientId;

constexpr double      kStatusPeriodMs = 150.0; // ticks of 50 to 65 ms: never over 250 ms apart
constexpr const char* kPoseChannel    = "pose";
constexpr const char* kStatusChannel  = "status";

struct FeedSettings {
    double          stale_ms = 750.0;
    feed::HelloInfo hello;
};

class LiveFeed {
  public:
    LiveFeed(server::HttpServer& server, FeedSettings settings);

    void onChunk(const sources::SourceChunk& chunk);    // source thread
    void onStatus(const sources::SourceStatus& status); // source thread
    void onOpen(ClientId client);                       // server thread
    void onClose(ClientId client);                      // server thread

    // Main loop, every few tens of ms: sends status when due. The status as of now.
    feed::FeedStatus tick(feed::RxMillis now);

    feed::FeedStatus status(feed::RxMillis now) const;

  private:
    feed::FeedStatus statusLocked(feed::RxMillis now) const;
    void             sendStatusLocked(const feed::FeedStatus& status);

    server::HttpServer&             server_;
    FeedSettings                    settings_;
    mutable std::mutex              mutex_;
    feed::PoseFeed                  feed_;
    std::set<ClientId>              clients_; // hello sent
    std::string                     source_ = "none";
    std::string                     port_;
    std::string                     detail_ = "starting";
    feed::LinkState                 link_   = feed::LinkState::kSearching;
    std::optional<std::string>      last_pose_;
    std::optional<feed::FeedStatus> sent_;
    feed::RxMillis                  sent_at_ = 0;
};

// The browser's link state for a source's port state.
feed::LinkState linkStateOf(sources::PortState state);

// status.detail: the source's text, plus the radio quality through a linked controller.
std::string detailOf(const sources::SourceStatus& status);

} // namespace spectagatr::receiver::app
