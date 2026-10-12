// main.cpp
// spectagatr: poses from a V5 controller, a Brain USB port, the sim or a replay, to the browser
// viewer over HTTP and WebSocket on this machine.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "receiver/app/asset_dirs.h"
#include "receiver/app/command_line.h"
#include "receiver/app/live_feed.h"
#include "receiver/app/run_source.h"
#include "receiver/feed/delivery_meter.h"
#include "receiver/feed/pose_feed.h"
#include "receiver/platform/browser.h"
#include "receiver/platform/process.h"
#include "receiver/platform/stop_signal.h"
#include "receiver/server/http_server.h"
#include "receiver/server/static_site.h"

#ifndef SPECTAGATR_SOURCE_WEB_DIR
#define SPECTAGATR_SOURCE_WEB_DIR ""
#endif
#ifndef SPECTAGATR_SOURCE_FIELDS_DIR
#define SPECTAGATR_SOURCE_FIELDS_DIR ""
#endif

namespace
{

using namespace spectagatr::receiver;
using SteadyClock = std::chrono::steady_clock;

constexpr auto kTick        = std::chrono::milliseconds(50);
constexpr auto kJoinTimeout = std::chrono::milliseconds(2000);

const SteadyClock::time_point kStart = SteadyClock::now();

sources::ReceiverUs receiverUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - kStart)
        .count();
}

feed::RxMillis receiverMs() { return static_cast<double>(receiverUs()) / 1000.0; }

bool contains(const feed::FieldNames& names, const std::string& name) {
    for (const std::string& candidate : names) {
        if (candidate == name) {
            return true;
        }
    }
    return false;
}

// One line when the link, its detail or staleness changes.
void printChanges(const feed::FeedStatus& now, std::optional<feed::FeedStatus>& printed) {
    const bool link_changed = !printed || printed->source != now.source ||
                              printed->port != now.port || printed->link != now.link ||
                              printed->detail != now.detail;
    if (link_changed) {
        std::printf("spectagatr: %s%s%s %s: %s\n", now.source.c_str(), now.port.empty() ? "" : " ",
                    now.port.c_str(), feed::linkStateName(now.link), now.detail.c_str());
    }
    if (printed && printed->stale != now.stale) {
        std::printf("spectagatr: poses %s\n", now.stale ? "stale" : "live");
    }
    std::fflush(stdout);
    printed = now;
}

void printLinkStats(const sources::TransportStats& stats) {
    char rtt[32] = "--";
    char channel[8] = "--";
    if (stats.rtt_us) {
        std::snprintf(rtt, sizeof(rtt), "%.1f", static_cast<double>(*stats.rtt_us) / 1000.0);
    }
    if (stats.radio) {
        std::snprintf(channel, sizeof(channel), "%u", static_cast<unsigned>(stats.radio->channel));
    }
    std::fprintf(stderr,
                 "spectagatr: controller polls=%lu empty_polls=%lu timeouts=%lu "
                 "bad_replies=%lu late_replies=%lu stray_bytes=%lu padding_bytes=%lu "
                 "data_bytes=%lu channel_switches=%lu rtt_ms=%s radio_channel=%s\n",
                 static_cast<unsigned long>(stats.polls),
                 static_cast<unsigned long>(stats.empty_polls),
                 static_cast<unsigned long>(stats.timeouts),
                 static_cast<unsigned long>(stats.bad_replies),
                 static_cast<unsigned long>(stats.late_replies),
                 static_cast<unsigned long>(stats.stray_bytes),
                 static_cast<unsigned long>(stats.padding_bytes),
                 static_cast<unsigned long>(stats.data_bytes),
                 static_cast<unsigned long>(stats.channel_switches), rtt, channel);
    std::fflush(stderr);
}

// corrupt: lines dropped by the parser this window (bad CRC, truncated, malformed, too long).
void printDelivery(const feed::DeliveryWindow& w, uint64_t corrupt) {
    const double per_reply = w.chunks > 0 ? static_cast<double>(w.poses) / w.chunks : 0.0;
    std::fprintf(stderr,
                 "spectagatr: delivery poses=%lu rate_hz=%.1f max_gap_ms=%.0f seq_lost=%lu "
                 "corrupt=%llu stale_gaps=%lu lag_growth_ms=%.0f bytes_per_s=%.0f "
                 "replies_with_data=%lu poses_per_reply=%.1f\n",
                 static_cast<unsigned long>(w.poses), w.rate_hz, w.max_gap_ms,
                 static_cast<unsigned long>(w.seq_lost), static_cast<unsigned long long>(corrupt),
                 static_cast<unsigned long>(w.stale_gaps), w.max_lag_ms, w.bytes_per_s,
                 static_cast<unsigned long>(w.chunks), per_reply);
    std::fflush(stderr);
}

uint64_t corruptLines(const feed::FeedCounts& c) {
    return c.bad_crc + c.truncated + c.malformed + c.too_long;
}

int run(const app::Options& options) {
    if (options.list_ports) {
        std::printf("%s", app::describePorts().c_str());
        return 0;
    }
    platform::installInterruptHandler();

    app::AssetSearch search;
    search.web_flag           = options.web_dir;
    search.fields_flag        = options.fields_dir;
    search.executable_dir     = platform::executablePath().parent_path();
    search.source_web         = SPECTAGATR_SOURCE_WEB_DIR;
    search.source_fields      = SPECTAGATR_SOURCE_FIELDS_DIR;
    const app::AssetDirs dirs = app::findAssetDirs(search);

    server::SiteConfig site;
    site.web_dir                  = dirs.web;
    site.fields_dir               = dirs.fields;
    site.default_field            = options.field;
    const feed::FieldNames fields = server::listFields(dirs.fields);
    if (!contains(fields, options.field)) {
        std::fprintf(stderr, "spectagatr: warning: no field '%s' in %s\n", options.field.c_str(),
                     dirs.fields.string().c_str());
    }
    std::error_code error;
    if (!std::filesystem::is_regular_file(dirs.web / "index.html", error)) {
        std::fprintf(stderr, "spectagatr: warning: no index.html in %s\n",
                     dirs.web.string().c_str());
    }

    server::HttpServerConfig config;
    config.bind = options.bind;
    config.port = options.http_port;
    server::HttpServer server(config, [site](const server::HttpRequest& request) {
        return server::serveSite(site, request);
    });

    app::FeedSettings settings;
    settings.stale_ms       = options.stale_ms;
    settings.hello.stale_ms = static_cast<int64_t>(options.stale_ms);
    settings.hello.field    = options.field;
    settings.hello.fields   = fields;
    settings.hello.robot    = options.robot;
    app::LiveFeed live(server, settings);
    server.setWebSocketHandlers({[&live](app::ClientId client) { live.onOpen(client); },
                                 [&live](app::ClientId client) { live.onClose(client); }});

    std::string err;
    if (!server.start(err)) {
        std::fprintf(stderr, "spectagatr: %s\n", err.c_str());
        return 1;
    }
    const std::string host = options.bind == "0.0.0.0" ? "127.0.0.1" : options.bind;
    const std::string url  = "http://" + host + ":" + std::to_string(server.port()) + "/";
    std::printf("spectagatr: viewer at %s\n", url.c_str());
    std::printf("spectagatr: web %s, fields %s\n", dirs.web.string().c_str(),
                dirs.fields.string().c_str());
    std::printf("spectagatr: source %s; Ctrl+C stops\n", app::sourceChoiceName(options.source));
    std::fflush(stdout);
    if (options.open_browser && !platform::openBrowser(url, err)) {
        std::fprintf(stderr, "spectagatr: %s; open %s yourself\n", err.c_str(), url.c_str());
    }

    platform::StopSignal stop;
    sources::SourceHooks hooks;
    SteadyClock::time_point next_link_debug{}; // source thread only
    hooks.clock     = receiverUs;
    // --debug-link: a second parse of the same bytes measures what the viewer receives.
    feed::PoseFeed      meter_feed;              // source thread only
    feed::DeliveryMeter meter(options.stale_ms); // source thread only
    uint64_t            corrupt_printed = 0;     // source thread only
    hooks.on_chunk = [&](const sources::SourceChunk& chunk) {
        live.onChunk(chunk);
        if (options.debug_link) {
            meter.onBytes(chunk.bytes.size());
            meter_feed.push(chunk.bytes.data(), chunk.bytes.size(),
                            static_cast<double>(chunk.rx_us) / 1000.0,
                            [&](const feed::TrackedPose& pose) { meter.onPose(pose); });
        }
    };
    hooks.on_status = [&](const sources::SourceStatus& status) {
        live.onStatus(status);
        if (!options.debug_link || status.src != sources::SourceKind::kController) {
            return;
        }
        const SteadyClock::time_point now = SteadyClock::now();
        if (now >= next_link_debug) {
            printLinkStats(status.stats);
            const uint64_t corrupt = corruptLines(meter_feed.counts());
            printDelivery(meter.take(receiverMs()), corrupt - corrupt_printed);
            corrupt_printed = corrupt;
            next_link_debug = now + std::chrono::seconds(1);
        }
    };
    std::atomic<bool> source_done{false};
    std::thread       source_thread([&] {
        app::runSource(options, hooks, stop);
        source_done = true;
    });

    std::optional<feed::FeedStatus> printed;
    while (!platform::interruptRequested()) {
        printChanges(live.tick(receiverMs()), printed);
        std::this_thread::sleep_for(kTick);
    }

    std::printf("spectagatr: stopping\n");
    stop.request();
    const SteadyClock::time_point deadline = SteadyClock::now() + kJoinTimeout;
    while (!source_done && SteadyClock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    server.stop();
    std::printf("spectagatr: stopped\n");
    std::fflush(stdout);
    if (!source_done) {
        // Blocked reading stdin: end the process here, before anything it uses is destroyed.
        std::_Exit(0);
    }
    source_thread.join();
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    const app::ParsedCommandLine   parsed = app::parseCommandLine(args);
    if (!parsed.options) {
        std::fprintf(stderr, "spectagatr: %s\n\n%s", parsed.error.c_str(),
                     app::usageText().c_str());
        return 2;
    }
    if (parsed.options->help) {
        std::printf("%s", app::usageText().c_str());
        return 0;
    }
    return run(*parsed.options);
}
