// cdp_bench.cpp
// Drives a headless Chrome/Edge over the DevTools protocol against a
// running viewer and reports frame timing, long tasks, message handling
// cost, receive-to-frame delay, DOM churn and heap, idle and during a
// scripted orbit drag, optionally with CPU or network throttling. The page
// side is perf_probe.js, injected before the viewer loads, so any viewer
// version can be measured without hooks in the app.
//
//   <browser> --headless=new --remote-debugging-port=9333 --user-data-dir=<tmp> about:blank
//   navigatr_cdp_bench --cdp 127.0.0.1:9333 --url http://127.0.0.1:18765/ --out r.json
//                      [--scenarios idle,orbit,idle_cpu4,orbit_cpu4] [--duration 15]
//
// Scenario names: idle, orbit, each optionally suffixed _cpuN (CPU throttle
// rate N) and/or _netK (network throttle to K kB/s, 20 ms latency). CDP
// network emulation did not slow the WebSocket in Chrome 153, so _netK is
// not in the defaults; when used, the page's received rate is compared
// with K and a throttle that did not apply is flagged.
//
// --expect-session and --expect-browser-ws make sure the measured server
// and browser are the ones the caller started, not something already on
// the port (exit 3 otherwise).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "perf_json.h"
#include "perf_net.h"
#include "perf_sys.h"

using namespace navigatr::perf;

namespace
{

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

constexpr double kPi = 3.14159265358979323846;

struct Options {
    std::string cdp_host = "127.0.0.1";
    int         cdp_port = 9333;
    std::string url;
    std::string out;
    std::string scenarios = "idle,orbit,idle_cpu4,orbit_cpu4";
    std::string probe     = NAVIGATR_PERF_PROBE_JS;
    double      duration_s = 15.0;
    double      warmup_s   = 8.0;
    double      settle_s   = 5.0;   // between scenarios, so a throttled one does not leak backlog
    int         width      = 1600;
    int         height     = 900;
    std::string label;
    std::string server_host;   // optional: inspection server counters around each scenario
    int         server_port = 0;
    bool        close_browser = false;
    std::string expect_session;      // /api/hello session.id of the server we started
    std::string expect_browser_ws;   // the "DevTools listening on" URL of the browser we started
};

void usage() {
    std::printf(
        "navigatr_cdp_bench: headless browser measurements of the viewer over CDP\n"
        "  --cdp <host:port>     browser remote debugging endpoint (127.0.0.1:9333)\n"
        "  --url <url>           viewer page\n"
        "  --scenarios <list>    idle, orbit, with optional _cpuN / _netK suffixes\n"
        "  --duration <s>        per scenario (15)\n"
        "  --warmup <s>          after the viewer is live (8)\n"
        "  --settle <s>          pause between scenarios (5)\n"
        "  --width/--height      viewport (1600x900)\n"
        "  --probe <file>        page probe script (the perf_probe.js next to the source)\n"
        "  --label <text>        recorded in the output\n"
        "  --server <host:port>  inspection server: hello and counters around each scenario\n"
        "  --expect-session <id> exit 3 unless the server's /api/hello session.id is this\n"
        "  --expect-browser-ws <url>  exit 3 unless the DevTools browser URL is this\n"
        "  --close-browser 1     close the browser at the end\n"
        "  --out <file>          JSON results\n");
}

bool parseOptions(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            usage();
            std::exit(0);
        }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value for %s\n", a.c_str());
            return false;
        }
        const std::string v = argv[++i];
        if (a == "--cdp") {
            const std::size_t colon = v.rfind(':');
            if (colon == std::string::npos) {
                return false;
            }
            o.cdp_host = v.substr(0, colon);
            o.cdp_port = std::atoi(v.c_str() + colon + 1);
        } else if (a == "--url") {
            o.url = v;
        } else if (a == "--out") {
            o.out = v;
        } else if (a == "--scenarios") {
            o.scenarios = v;
        } else if (a == "--probe") {
            o.probe = v;
        } else if (a == "--duration") {
            o.duration_s = std::atof(v.c_str());
        } else if (a == "--settle") {
            o.settle_s = std::atof(v.c_str());
        } else if (a == "--warmup") {
            o.warmup_s = std::atof(v.c_str());
        } else if (a == "--width") {
            o.width = std::atoi(v.c_str());
        } else if (a == "--height") {
            o.height = std::atoi(v.c_str());
        } else if (a == "--label") {
            o.label = v;
        } else if (a == "--close-browser") {
            o.close_browser = v == "1" || v == "true";
        } else if (a == "--server") {
            const std::size_t colon = v.rfind(':');
            if (colon == std::string::npos) {
                return false;
            }
            o.server_host = v.substr(0, colon);
            o.server_port = std::atoi(v.c_str() + colon + 1);
        } else if (a == "--expect-session") {
            o.expect_session = v;
        } else if (a == "--expect-browser-ws") {
            o.expect_browser_ws = v;
        } else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return false;
        }
    }
    if (o.url.empty()) {
        std::fprintf(stderr, "--url is required\n");
        return false;
    }
    if (!o.expect_session.empty() && o.server_port == 0) {
        std::fprintf(stderr, "--expect-session needs --server\n");
        return false;
    }
    return true;
}

std::string num(double v) {
    if (!std::isfinite(v)) {
        return "null";
    }
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

// ---- CDP session ---------------------------------------------------------------------------

class Cdp
{
public:
    bool open(const std::string& host, int port, const std::string& ws_path, std::string& err) {
        return ws_.connect(host, port, ws_path, 0, err);
    }

    // One command; waits for its reply, dropping events. False on error reply.
    bool call(const std::string& method, const std::string& params, Json& result,
              std::string& err, double timeout_s = 30.0) {
        const uint64_t id = ++next_id_;
        const std::string msg = "{\"id\":" + std::to_string(id) + ",\"method\":\"" + method +
                                "\",\"params\":" + (params.empty() ? "{}" : params) + "}";
        if (!ws_.sendText(msg)) {
            err = method + ": send failed";
            return false;
        }
        const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                                 std::chrono::duration<double>(timeout_s));
        for (;;) {
            WsMessage m;
            while (ws_.pop(m)) {
                if (m.opcode != 0x1) {
                    continue;
                }
                Json v;
                if (!parseJson(m.payload, v)) {
                    continue;
                }
                const Json* mid = v.get("id");
                if (mid == nullptr) {
                    ++events_;
                    continue;
                }
                if (static_cast<uint64_t>(mid->number) != id) {
                    continue;
                }
                if (const Json* e = v.get("error")) {
                    err = method + ": " + toJson(*e);
                    return false;
                }
                const Json* r = v.get("result");
                result        = r != nullptr ? *r : Json{};
                return true;
            }
            if (ws_.closed() || Clock::now() >= deadline) {
                err = method + ": no reply";
                return false;
            }
            ws_.readSome(20);
        }
    }

    // Keeps the socket drained while waiting.
    void pump(double seconds) {
        const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                            std::chrono::duration<double>(seconds));
        while (Clock::now() < end) {
            ws_.readSome(20);
            WsMessage m;
            while (ws_.pop(m)) {
                ++events_;
            }
        }
    }

    // Runtime.evaluate returning a string value.
    bool evalString(const std::string& expr, std::string& out, std::string& err) {
        Json r;
        if (!call("Runtime.evaluate",
                  "{\"expression\":" + jsonQuote(expr) + ",\"returnByValue\":true}", r, err)) {
            return false;
        }
        const Json* v = r.at({"result", "value"});
        if (const Json* ex = r.get("exceptionDetails")) {
            err = "exception: " + toJson(*ex).substr(0, 400);
            return false;
        }
        if (v == nullptr) {
            out.clear();
            return true;
        }
        out = v->type == Json::Type::kString ? v->str : toJson(*v);
        return true;
    }

    uint64_t events() const { return events_; }

private:
    WsClient ws_;
    uint64_t next_id_ = 0;
    uint64_t events_  = 0;
};

// Performance.getMetrics as name -> value.
bool metrics(Cdp& cdp, Json& out, std::string& err) {
    Json r;
    if (!cdp.call("Performance.getMetrics", "{}", r, err)) {
        return false;
    }
    out.type = Json::Type::kObject;
    out.fields.clear();
    const Json* list = r.get("metrics");
    if (list == nullptr) {
        return true;
    }
    for (const Json& m : list->items) {
        const Json* n = m.get("name");
        const Json* v = m.get("value");
        if (n != nullptr && v != nullptr) {
            out.fields.emplace_back(n->str, *v);
        }
    }
    return true;
}

double metric(const Json& m, const char* name) {
    const Json* v = m.get(name);
    return v != nullptr ? v->number : NAN;
}

// workers and any top-level inspection object of /api/snapshot, or null
std::string serverCounters(const Options& o) {
    if (o.server_port == 0) {
        return "null";
    }
    HttpResult r;
    Json       doc;
    if (!httpRequest(o.server_host, o.server_port, "GET", "/api/snapshot", r, 5000) ||
        r.status != 200 || !parseJson(r.body, doc)) {
        return "null";
    }
    const Json* w    = doc.get("workers");
    const Json* insp = doc.get("inspection");
    return std::string("{\"workers\":") + (w != nullptr ? toJson(*w) : "null") +
           ",\"inspection\":" + (insp != nullptr ? toJson(*insp) : "null") + "}";
}

// The measured server's /api/hello identity: contract, session and
// configuration. Null when --server is not given or it does not answer.
bool serverHello(const Options& o, Json& hello) {
    hello = Json{};
    if (o.server_port == 0) {
        return false;
    }
    HttpResult r;
    return httpRequest(o.server_host, o.server_port, "GET", "/api/hello", r, 5000) &&
           r.status == 200 && parseJson(r.body, hello);
}

std::string helloIdentity(const Json& hello) {
    if (hello.type != Json::Type::kObject) {
        return "null";
    }
    const auto sub = [&hello](const char* k) {
        const Json* v = hello.get(k);
        return v != nullptr ? toJson(*v) : std::string("null");
    };
    return "{\"contract\":" + sub("contract") + ",\"session\":" + sub("session") +
           ",\"configuration\":" + sub("configuration") + ",\"inspection\":" + sub("inspection") +
           "}";
}

std::string helloSession(const Json& hello) {
    const Json* id = hello.at({"session", "id"});
    return id != nullptr ? id->stringOr("") : std::string();
}

struct Scenario {
    std::string name;
    bool        orbit    = false;
    double      cpu_rate = 1.0;
    double      net_kbps = 0.0;
};

Scenario parseScenario(const std::string& name) {
    Scenario s;
    s.name = name;
    std::stringstream ss(name);
    std::string       part;
    bool              first = true;
    while (std::getline(ss, part, '_')) {
        if (first) {
            s.orbit = part == "orbit";
            first   = false;
        } else if (part.compare(0, 3, "cpu") == 0) {
            s.cpu_rate = std::atof(part.c_str() + 3);
        } else if (part.compare(0, 3, "net") == 0) {
            s.net_kbps = std::atof(part.c_str() + 3);
        }
    }
    return s;
}

std::string mouse(const char* type, double x, double y, bool pressed) {
    return std::string("{\"type\":\"") + type + "\",\"x\":" + num(x) + ",\"y\":" + num(y) +
           ",\"button\":\"left\",\"buttons\":" + (pressed ? "1" : "0") + ",\"clickCount\":1}";
}

std::string runScenario(Cdp& cdp, const Options& o, const Scenario& sc) {
    std::string err;
    Json        ignore;
    if (sc.cpu_rate > 1.0 &&
        !cdp.call("Emulation.setCPUThrottlingRate", "{\"rate\":" + num(sc.cpu_rate) + "}", ignore,
                  err)) {
        return "{\"error\":" + jsonQuote(err) + "}";
    }
    std::string net_note = "off";
    if (sc.net_kbps > 0.0) {
        const double bps = sc.net_kbps * 1000.0;
        if (cdp.call("Network.enable", "{}", ignore, err) &&
            cdp.call("Network.emulateNetworkConditions",
                     "{\"offline\":false,\"latency\":20,\"downloadThroughput\":" + num(bps) +
                         ",\"uploadThroughput\":" + num(bps) + "}",
                     ignore, err)) {
            net_note = "requested " + num(sc.net_kbps) + " kB/s, 20 ms";
        } else {
            net_note = "failed: " + err;
        }
    }
    const std::string server_before = serverCounters(o);
    Json              before;
    metrics(cdp, before, err);
    const CpuSample cpu0 = sampleCpu();
    std::string     tmp;
    cdp.evalString("window.__perf.reset()", tmp, err);

    std::vector<double> ack_ms;
    const auto          t_start = Clock::now();
    const auto          t_end   = t_start + std::chrono::duration_cast<Clock::duration>(
                                           std::chrono::duration<double>(o.duration_s));
    if (sc.orbit) {
        std::string rect;
        double      cx = o.width * 0.4, cy = o.height * 0.5;
        if (cdp.evalString("(() => { const cs = [...document.querySelectorAll('canvas')]"
                           ".filter(c => c.offsetParent !== null)"
                           ".sort((a, b) => b.clientWidth * b.clientHeight - a.clientWidth * "
                           "a.clientHeight); if (!cs.length) return ''; const r = "
                           "cs[0].getBoundingClientRect(); return JSON.stringify([r.left + "
                           "r.width / 2, r.top + r.height / 2]); })()",
                           rect, err) &&
            !rect.empty()) {
            Json v;
            if (parseJson(rect, v) && v.items.size() == 2) {
                cx = v.items[0].number;
                cy = v.items[1].number;
            }
        }
        Json r;
        cdp.call("Input.dispatchMouseEvent", mouse("mouseMoved", cx, cy, false), r, err);
        cdp.call("Input.dispatchMouseEvent", mouse("mousePressed", cx, cy, true), r, err);
        // ~60 Hz moves on a slow ellipse, one revolution every 4 s
        auto next = Clock::now();
        while (Clock::now() < t_end) {
            const double t  = std::chrono::duration<double>(Clock::now() - t_start).count();
            const double x  = cx + 180.0 * std::sin(2.0 * kPi * t / 4.0);
            const double y  = cy + 50.0 * std::sin(4.0 * kPi * t / 4.0);
            const auto   a0 = Clock::now();
            cdp.call("Input.dispatchMouseEvent", mouse("mouseMoved", x, y, true), r, err);
            ack_ms.push_back(std::chrono::duration<double, std::milli>(Clock::now() - a0).count());
            next += Ms(16);
            if (next > Clock::now()) {
                std::this_thread::sleep_until(next);
            } else {
                next = Clock::now();
            }
        }
        cdp.call("Input.dispatchMouseEvent", mouse("mouseReleased", cx, cy, false), r, err);
    } else {
        cdp.pump(o.duration_s);
    }

    const CpuSample cpu1 = sampleCpu();
    std::string     report;
    const bool      ok_report = cdp.evalString("window.__perf.report()", report, err);
    Json        after;
    metrics(cdp, after, err);
    if (sc.cpu_rate > 1.0) {
        cdp.call("Emulation.setCPUThrottlingRate", "{\"rate\":1}", ignore, err);
    }
    if (sc.net_kbps > 0.0) {
        cdp.call("Network.emulateNetworkConditions",
                 "{\"offline\":false,\"latency\":0,\"downloadThroughput\":-1,"
                 "\"uploadThroughput\":-1}",
                 ignore, err);
        cdp.call("Network.disable", "{}", ignore, err);
    }

    const std::string server_after = serverCounters(o);
    const double wall_s = std::chrono::duration<double>(Clock::now() - t_start).count();
    const auto   d      = [&](const char* k) { return metric(after, k) - metric(before, k); };

    // what the page actually received over its WebSockets, all message kinds
    double ws_bytes_s = NAN;
    {
        Json pj;
        if (ok_report && parseJson(report, pj)) {
            const Json* msgs = pj.get("messages");
            if (msgs != nullptr) {
                ws_bytes_s = 0.0;
                for (const auto& kv : msgs->fields) {
                    const Json* b = kv.second.get("bytes_s");
                    ws_bytes_s += b != nullptr ? b->numberOr(0.0) : 0.0;
                }
            }
        }
    }
    // CDP network emulation has not slowed WebSockets in the Chrome tested
    std::string throttle = "off";
    if (sc.net_kbps > 0.0) {
        if (std::isnan(ws_bytes_s)) {
            throttle = "unknown (no probe report)";
        } else if (ws_bytes_s > sc.net_kbps * 1000.0 * 1.1) {
            throttle = "NOT applied to the WebSocket (received more than requested)";
            std::printf("  warning: %s asked for %.0f kB/s but the page received %.0f kB/s: the "
                        "throttle did not apply to the WebSocket\n",
                        sc.name.c_str(), sc.net_kbps, ws_bytes_s / 1000.0);
        } else {
            throttle = "received at or under the requested rate";
        }
    }

    std::ostringstream j;
    j << "{\"scenario\":" << jsonQuote(sc.name) << ",\"orbit\":" << (sc.orbit ? "true" : "false")
      << ",\"cpu_throttle\":" << num(sc.cpu_rate) << ",\"network\":" << jsonQuote(net_note)
      << ",\"network_throttle\":" << jsonQuote(throttle)
      << ",\"ws_received_bytes_s\":" << num(ws_bytes_s)
      << ",\"wall_s\":" << num(wall_s)
      << ",\"cdp_metrics\":{\"task_s_per_s\":" << num(d("TaskDuration") / wall_s)
      << ",\"script_s_per_s\":" << num(d("ScriptDuration") / wall_s)
      << ",\"layout_s_per_s\":" << num(d("LayoutDuration") / wall_s)
      << ",\"recalc_style_s_per_s\":" << num(d("RecalcStyleDuration") / wall_s)
      << ",\"layouts_per_s\":" << num(d("LayoutCount") / wall_s)
      << ",\"style_recalcs_per_s\":" << num(d("RecalcStyleCount") / wall_s)
      << ",\"js_heap_used_start\":" << num(metric(before, "JSHeapUsedSize"))
      << ",\"js_heap_used_end\":" << num(metric(after, "JSHeapUsedSize"))
      << ",\"js_heap_total_end\":" << num(metric(after, "JSHeapTotalSize"))
      << ",\"dom_nodes_end\":" << num(metric(after, "Nodes"))
      << ",\"js_event_listeners_end\":" << num(metric(after, "JSEventListeners")) << "}"
      << ",\"input_ack_ms\":" << summaryJson(summarize(ack_ms))
      << ",\"host_cpu\":{\"machine_busy_pct\":" << num(machineBusyPercent(cpu0, cpu1)) << "}"
      << ",\"server_before\":" << server_before << ",\"server_after\":" << server_after
      << ",\"probe\":" << (ok_report && !report.empty() ? report : "null");
    if (!ok_report) {
        j << ",\"probe_error\":" << jsonQuote(err);
    }
    j << "}";
    return j.str();
}

void printScenario(const std::string& json) {
    Json v;
    if (!parseJson(json, v)) {
        std::printf("  (unparsed)\n");
        return;
    }
    const auto s = [&](std::initializer_list<const char*> path) -> std::string {
        const Json* x = v.at(path);
        if (x == nullptr || x->get("n") == nullptr || x->get("n")->number == 0) {
            return "n/a";
        }
        char buf[160];
        std::snprintf(buf, sizeof(buf), "p50 %.2f p95 %.2f max %.2f (n %.0f)",
                      x->get("p50")->number, x->get("p95")->number, x->get("max")->number,
                      x->get("n")->number);
        return buf;
    };
    const Json* fps = v.at({"probe", "frames", "fps"});
    const Json* lt  = v.at({"probe", "long_tasks", "count"});
    const Json* mh  = v.at({"probe", "message_handler_total_ms_per_s"});
    const Json* ts  = v.at({"cdp_metrics", "task_s_per_s"});
    std::printf("%s: fps %.1f, long tasks %.0f, onmessage %.1f ms/s, main-thread busy %.0f%%\n",
                v.get("scenario")->str.c_str(), fps != nullptr ? fps->number : NAN,
                lt != nullptr ? lt->number : NAN, mh != nullptr ? mh->number : NAN,
                ts != nullptr ? ts->number * 100.0 : NAN);
    const Json* hc = v.at({"host_cpu", "machine_busy_pct"});
    std::printf("  host cpu            machine %.0f%% of all cores\n",
                hc != nullptr ? hc->number : NAN);
    std::printf("  frame interval ms   %s\n", s({"probe", "frames", "interval_ms"}).c_str());
    std::printf("  rAF callbacks ms    %s\n", s({"probe", "raf_callback_ms"}).c_str());
    std::printf("  long task ms        %s\n", s({"probe", "long_tasks", "ms"}).c_str());
    std::printf("  recv->frame ms      %s\n", s({"probe", "receive_to_next_frame_ms"}).c_str());
    std::printf("  relative lag ms     %s\n", s({"probe", "relative_lag_ms"}).c_str());
    std::printf("  input ack ms        %s\n", s({"input_ack_ms"}).c_str());
    const Json* sa = v.at({"server_after", "workers", "inspection"});
    const Json* sb = v.at({"server_before", "workers", "inspection"});
    if (sa != nullptr && sb != nullptr) {
        const auto dd = [&](const char* k) {
            const Json* x = sa->get(k);
            const Json* y = sb->get(k);
            return x != nullptr && y != nullptr ? x->number - y->number : NAN;
        };
        std::printf("  server: snapshots sent +%.0f skipped +%.0f, frames skipped +%.0f, "
                    "disconnects +%.0f\n",
                    dd("snapshots_sent"), dd("snapshots_skipped"), dd("frames_skipped"),
                    dd("client_disconnects"));
    }
    const Json* msgs = v.at({"probe", "messages"});
    if (msgs != nullptr) {
        for (const auto& kv : msgs->fields) {
            const Json* h = kv.second.get("handler_ms");
            std::printf("  msg %-14s %.1f/s  %.0f B  handler p50 %.2f p95 %.2f max %.2f ms\n",
                        kv.first.c_str(), kv.second.get("per_s")->number,
                        kv.second.get("mean_bytes")->number,
                        h != nullptr && h->get("p50") ? h->get("p50")->number : NAN,
                        h != nullptr && h->get("p95") ? h->get("p95")->number : NAN,
                        h != nullptr && h->get("max") ? h->get("max")->number : NAN);
        }
    }
}

bool readFile(const std::string& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

} // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parseOptions(argc, argv, o)) {
        usage();
        return 2;
    }
    initSockets();
    std::string probe;
    if (!readFile(o.probe, probe)) {
        std::fprintf(stderr, "cannot read probe %s\n", o.probe.c_str());
        return 1;
    }

    // the browser may still be starting
    HttpResult version;
    const auto deadline = Clock::now() + std::chrono::seconds(20);
    while (!httpRequest(o.cdp_host, o.cdp_port, "GET", "/json/version", version, 2000) ||
           version.status != 200) {
        if (Clock::now() >= deadline) {
            std::fprintf(stderr, "no DevTools endpoint at %s:%d\n", o.cdp_host.c_str(),
                         o.cdp_port);
            return 1;
        }
        std::this_thread::sleep_for(Ms(250));
    }
    Json ver;
    parseJson(version.body, ver);
    const std::string browser_ws =
        ver.get("webSocketDebuggerUrl") != nullptr ? ver.get("webSocketDebuggerUrl")->str : "";
    if (!o.expect_browser_ws.empty() && browser_ws != o.expect_browser_ws) {
        std::fprintf(stderr,
                     "the DevTools endpoint %s:%d belongs to another browser (%s, expected %s)\n",
                     o.cdp_host.c_str(), o.cdp_port, browser_ws.c_str(),
                     o.expect_browser_ws.c_str());
        return 3;
    }
    // the server we are about to measure
    Json hello_start;
    serverHello(o, hello_start);
    if (!o.expect_session.empty() && helloSession(hello_start) != o.expect_session) {
        std::fprintf(stderr, "the server on %s:%d is not the one started (session '%s', expected "
                             "'%s')\n",
                     o.server_host.c_str(), o.server_port, helloSession(hello_start).c_str(),
                     o.expect_session.c_str());
        return 3;
    }
    if (o.server_port != 0) {
        const Json* c = hello_start.get("contract");
        std::printf("server %s:%d: contract %s, session %s\n", o.server_host.c_str(),
                    o.server_port, c != nullptr ? c->stringOr("?").c_str() : "?",
                    helloSession(hello_start).c_str());
    }
    HttpResult target;
    Json       tj;
    if (!httpRequest(o.cdp_host, o.cdp_port, "PUT", "/json/new?about:blank", target) ||
        !parseJson(target.body, tj) || tj.get("webSocketDebuggerUrl") == nullptr) {
        std::fprintf(stderr, "cannot open a page target: %s\n", target.body.c_str());
        return 1;
    }
    const std::string ws_url = tj.get("webSocketDebuggerUrl")->str;
    const std::string target_id = tj.get("id") != nullptr ? tj.get("id")->str : "";
    const std::size_t path_at   = ws_url.find('/', std::string("ws://").size());
    const std::string ws_path   = path_at == std::string::npos ? "/" : ws_url.substr(path_at);

    Cdp         cdp;
    std::string err;
    if (!cdp.open(o.cdp_host, o.cdp_port, ws_path, err)) {
        std::fprintf(stderr, "cdp connect: %s\n", err.c_str());
        return 1;
    }
    Json r;
    const bool setup =
        cdp.call("Page.enable", "{}", r, err) &&
        cdp.call("Performance.enable", "{\"timeDomain\":\"timeTicks\"}", r, err) &&
        cdp.call("Emulation.setDeviceMetricsOverride",
                 "{\"width\":" + std::to_string(o.width) + ",\"height\":" +
                     std::to_string(o.height) + ",\"deviceScaleFactor\":1,\"mobile\":false}",
                 r, err) &&
        cdp.call("Page.addScriptToEvaluateOnNewDocument", "{\"source\":" + jsonQuote(probe) + "}",
                 r, err);
    if (!setup) {
        std::fprintf(stderr, "setup: %s\n", err.c_str());
        return 1;
    }

    // live: the viewer's smoke-test hook says a feed message arrived. A page
    // that never gets there is loaded again, up to 3 times, and each failed
    // load's resource statuses are kept (the inspect/1 server can answer a
    // module fetch 503 when the browser opens more connections than max_clients).
    bool        live         = false;
    int         attempts     = 0;
    std::string failed_loads = "[";
    while (!live && attempts < 3) {
        ++attempts;
        if (!cdp.call("Page.navigate", "{\"url\":" + jsonQuote(o.url) + "}", r, err)) {
            std::fprintf(stderr, "navigate: %s\n", err.c_str());
            return 1;
        }
        const auto live_by = Clock::now() + std::chrono::seconds(15);
        while (Clock::now() < live_by) {
            std::string v;
            if (cdp.evalString("(() => { const s = document.querySelector('#status'); return "
                               "!!(window.__perf && s && s.dataset.liveSeen && "
                               "s.dataset.liveSeen !== '0' && s.dataset.liveSeen !== 'false'); "
                               "})()",
                               v, err) &&
                v == "true") {
                live = true;
                break;
            }
            cdp.pump(0.25);
        }
        if (!live) {
            std::string resources;
            cdp.evalString("JSON.stringify(performance.getEntriesByType('resource').map(e => "
                           "[e.name, e.responseStatus, Math.round(e.duration)]))",
                           resources, err);
            std::printf("load %d not live; page resources: %s\n", attempts, resources.c_str());
            failed_loads += (failed_loads.size() > 1 ? "," : "") +
                            (resources.empty() ? std::string("null") : resources);
        }
    }
    failed_loads += "]";
    std::printf("browser %s, viewer %s after %d load(s)\n",
                ver.get("Browser") != nullptr ? ver.get("Browser")->str.c_str() : "?",
                live ? "live" : "NOT live (continuing, results may be empty)", attempts);
    std::printf("warmup %.1f s\n", o.warmup_s);
    cdp.pump(o.warmup_s);

    std::vector<std::string> names;
    {
        std::stringstream ss(o.scenarios);
        std::string       item;
        while (std::getline(ss, item, ',')) {
            if (!item.empty()) {
                names.push_back(item);
            }
        }
    }
    std::ostringstream results;
    results << "[";
    for (std::size_t i = 0; i < names.size(); ++i) {
        const std::string sj = runScenario(cdp, o, parseScenario(names[i]));
        printScenario(sj);
        results << (i ? "," : "") << sj;
        cdp.pump(o.settle_s);
    }
    results << "]";
    Json hello_end;
    serverHello(o, hello_end);
    const bool server_changed = o.server_port != 0 &&
                                helloSession(hello_end) != helloSession(hello_start);
    if (server_changed) {
        std::fprintf(stderr, "the server's session changed during the run ('%s' to '%s'): the "
                             "results mix two servers\n",
                     helloSession(hello_start).c_str(), helloSession(hello_end).c_str());
    }
    if (!target_id.empty()) {
        HttpResult closed;
        httpRequest(o.cdp_host, o.cdp_port, "GET", "/json/close/" + target_id, closed, 2000);
    }
    if (o.close_browser && ver.get("webSocketDebuggerUrl") != nullptr) {
        const std::string bu = ver.get("webSocketDebuggerUrl")->str;
        const std::size_t at = bu.find('/', std::string("ws://").size());
        Cdp               browser;
        std::string       berr;
        Json              ignored;
        if (at != std::string::npos && browser.open(o.cdp_host, o.cdp_port, bu.substr(at), berr)) {
            browser.call("Browser.close", "{}", ignored, berr, 5.0);
        }
    }

    char       when[64] = "";
    std::time_t t       = std::time(nullptr);
    std::strftime(when, sizeof(when), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    std::ostringstream all;
    all << "{\"tool\":\"navigatr_cdp_bench\",\"conditions\":{\"when\":\"" << when
        << "\",\"label\":" << jsonQuote(o.label) << ",\"url\":" << jsonQuote(o.url)
        << ",\"browser\":" << (ver.get("Browser") ? jsonQuote(ver.get("Browser")->str) : "null")
        << ",\"user_agent\":"
        << (ver.get("User-Agent") ? jsonQuote(ver.get("User-Agent")->str) : "null")
        << ",\"viewport\":[" << o.width << "," << o.height << "],\"duration_s\":"
        << num(o.duration_s) << ",\"warmup_s\":" << num(o.warmup_s)
        << ",\"viewer_live\":" << (live ? "true" : "false")
        << ",\"load_attempts\":" << attempts << ",\"failed_loads\":" << failed_loads
        << ",\"cdp_events_dropped\":" << cdp.events()
        << ",\"browser_ws\":" << jsonQuote(browser_ws)
        << ",\"expect_session\":" << jsonQuote(o.expect_session)
        << ",\"server_hello_start\":" << helloIdentity(hello_start)
        << ",\"server_hello_end\":" << helloIdentity(hello_end)
        << ",\"server_changed\":" << (server_changed ? "true" : "false")
        << "},\"scenarios\":" << results.str() << "}\n";
    if (!o.out.empty()) {
        std::ofstream f(o.out, std::ios::binary);
        f << all.str();
        std::printf("wrote %s\n", o.out.c_str());
    } else {
        std::printf("%s", all.str().c_str());
    }
    return server_changed ? 3 : 0;
}
