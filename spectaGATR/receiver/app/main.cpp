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
    hooks.clock     = receiverUs;
    hooks.on_chunk  = [&live](const sources::SourceChunk& chunk) { live.onChunk(chunk); };
    hooks.on_status = [&live](const sources::SourceStatus& status) { live.onStatus(status); };
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
