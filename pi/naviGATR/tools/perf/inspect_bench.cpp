// inspect_bench.cpp
// Inspection feed measurements. In-process mode builds a System from a
// configuration and runs the InspectionService next to real loopback
// WebSocket clients, so every age is taken on the one Pi host clock and is
// exact. Connect mode measures a server in another process (the Pi, or a
// running navigatr) and reports only RTT and ping-offset estimates.
// Speaks navigatr.inspect/1 (snapshot only) and navigatr.inspect/2.
//
//   navigatr_inspect_bench --config <xml> [--scenario all|build|fast|slow|stall|burst]
//                          [--duration 20] [--warmup 6] [--out result.json] ...
//   navigatr_inspect_bench --connect 127.0.0.1:8765 --scenario fast,slow
//
// Run with --help for every option. Nothing here changes the runtime: the
// harness only calls public read paths and connects as ordinary clients.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "core/host_clock.h"
#include "inspection/inspection_document.h"
#include "inspection/inspection_service.h"
#include "perf_analysis.h"
#include "perf_json.h"
#include "perf_net.h"
#include "perf_sys.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;
using namespace navigatr::perf;

namespace
{

using Ms = std::chrono::milliseconds;

// ---- options -----------------------------------------------------------------------------

struct Options {
    std::string config;
    std::string connect_host;
    int         connect_port = 0;
    std::string scenarios    = "all";
    double      duration_s   = 20.0;
    double      warmup_s     = 6.0;
    std::string out;
    int         ping_ms = 250;

    // slow client: token-bucket reads with a small receive buffer
    double slow_bytes_s   = 100000.0;
    int    slow_rcvbuf    = 8192;
    // stall client: stops reading, then drains
    double stall_at_s     = 3.0;
    double stall_ms       = 3000.0;
    // burst: raise one client's preview budget for a while
    double burst_at_s     = 3.0;
    double burst_ms       = 5000.0;
    double burst_hz       = 30.0;
    long   burst_width    = 1280;
    long   burst_quality  = 90;

    double fresh_ms = 100.0;   // "stale" threshold for publish-to-receive
    std::string subscribe;     // inspect/2 subscribe JSON sent after hello
    // ping-offset estimates wider than this (+-RTT/2) are counted, not used
    double offset_max_half_ms = 10.0;
    // text messages are kept raw and parsed after the run, so parse cost
    // never delays the next socket read; past this many MB per client they
    // are parsed inline (counted in the output)
    double defer_mb = 256.0;

    // in-process overrides (negative: keep the configuration)
    double snapshot_hz      = -1;
    long   client_buffer_kb = -1;
    long   stall_close_ms   = -1;
    long   max_clients      = -1;
    double state_hz         = -1;   // inspect/2 fields, applied when they exist
    double diag_hz          = -1;
    long   reliable_kb      = -1;
    long   send_buffer_kb   = -1;
    long   port             = 0;    // 0: ephemeral
};

void usage() {
    std::printf(
        "navigatr_inspect_bench: inspection feed measurements (inspect/1 and inspect/2)\n"
        "  --config <xml>            in-process: build this System and serve it (exact ages)\n"
        "  --connect <host:port>     measure a server in another process (estimates only)\n"
        "  --scenario <list>         all, or a comma list of build,fast,slow,stall,burst\n"
        "  --duration <s>            per client scenario (20)\n"
        "  --warmup <s>              run before measuring, fills the trail (6)\n"
        "  --out <file>              JSON results (stdout gets a summary)\n"
        "  --ping-ms <ms>            ping period per client (250)\n"
        "  --slow-bytes-s <n>        slow client read budget (100000)\n"
        "  --slow-rcvbuf <n>         slow client SO_RCVBUF (8192)\n"
        "  --stall-at <s> --stall-ms <ms>          stall scenario (3, 3000)\n"
        "  --burst-at <s> --burst-ms <ms> --burst-hz <hz> --burst-width <px>\n"
        "  --burst-quality <q>       preview burst (3, 5000, 30, 1280, 90)\n"
        "  --fresh-ms <ms>           publish-to-receive threshold called stale (100)\n"
        "  --subscribe <json>        inspect/2 subscribe message sent after hello\n"
        "  --offset-max-half-ms <ms> ping-offset estimates wider than +-this are not used (10)\n"
        "  --defer-mb <mb>           raw text kept per client for after-run parsing (256)\n"
        "  --snapshot-hz --client-buffer-kb --stall-close-ms --max-clients --port\n"
        "  --state-hz --diag-hz --reliable-kb --send-buffer-kb   in-process overrides\n");
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
        if (a == "--config") {
            o.config = v;
        } else if (a == "--connect") {
            const std::size_t colon = v.rfind(':');
            if (colon == std::string::npos) {
                std::fprintf(stderr, "--connect wants host:port\n");
                return false;
            }
            o.connect_host = v.substr(0, colon);
            o.connect_port = std::atoi(v.c_str() + colon + 1);
        } else if (a == "--scenario") {
            o.scenarios = v;
        } else if (a == "--duration") {
            o.duration_s = std::atof(v.c_str());
        } else if (a == "--warmup") {
            o.warmup_s = std::atof(v.c_str());
        } else if (a == "--out") {
            o.out = v;
        } else if (a == "--ping-ms") {
            o.ping_ms = std::atoi(v.c_str());
        } else if (a == "--slow-bytes-s") {
            o.slow_bytes_s = std::atof(v.c_str());
        } else if (a == "--slow-rcvbuf") {
            o.slow_rcvbuf = std::atoi(v.c_str());
        } else if (a == "--stall-at") {
            o.stall_at_s = std::atof(v.c_str());
        } else if (a == "--stall-ms") {
            o.stall_ms = std::atof(v.c_str());
        } else if (a == "--burst-at") {
            o.burst_at_s = std::atof(v.c_str());
        } else if (a == "--burst-ms") {
            o.burst_ms = std::atof(v.c_str());
        } else if (a == "--burst-hz") {
            o.burst_hz = std::atof(v.c_str());
        } else if (a == "--burst-width") {
            o.burst_width = std::atol(v.c_str());
        } else if (a == "--burst-quality") {
            o.burst_quality = std::atol(v.c_str());
        } else if (a == "--fresh-ms") {
            o.fresh_ms = std::atof(v.c_str());
        } else if (a == "--subscribe") {
            o.subscribe = v;
        } else if (a == "--offset-max-half-ms") {
            o.offset_max_half_ms = std::atof(v.c_str());
        } else if (a == "--defer-mb") {
            o.defer_mb = std::atof(v.c_str());
        } else if (a == "--send-buffer-kb") {
            o.send_buffer_kb = std::atol(v.c_str());
        } else if (a == "--snapshot-hz") {
            o.snapshot_hz = std::atof(v.c_str());
        } else if (a == "--client-buffer-kb") {
            o.client_buffer_kb = std::atol(v.c_str());
        } else if (a == "--stall-close-ms") {
            o.stall_close_ms = std::atol(v.c_str());
        } else if (a == "--max-clients") {
            o.max_clients = std::atol(v.c_str());
        } else if (a == "--state-hz") {
            o.state_hz = std::atof(v.c_str());
        } else if (a == "--diag-hz") {
            o.diag_hz = std::atof(v.c_str());
        } else if (a == "--reliable-kb") {
            o.reliable_kb = std::atol(v.c_str());
        } else if (a == "--port") {
            o.port = std::atol(v.c_str());
        } else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return false;
        }
    }
    if (o.config.empty() == o.connect_host.empty()) {
        std::fprintf(stderr, "give exactly one of --config or --connect\n");
        return false;
    }
    return true;
}

// inspect/2 adds InspectionConfig fields; set and read them only when this
// tree has them (get_ returns NaN when absent)
#define PERF_OPTIONAL_FIELD(name)                                                            \
    template <class C, class V>                                                              \
    auto set_##name(C& c, V v, int)->decltype(c.name = v, bool()) {                          \
        c.name = v;                                                                          \
        return true;                                                                         \
    }                                                                                        \
    template <class C, class V> bool set_##name(C&, V, long) { return false; }              \
    template <class C>                                                                       \
    auto get_##name(const C& c, int)->decltype(static_cast<double>(c.name)) {                \
        return static_cast<double>(c.name);                                                  \
    }                                                                                        \
    template <class C> double get_##name(const C&, long) { return NAN; }
PERF_OPTIONAL_FIELD(state_hz)
PERF_OPTIONAL_FIELD(diag_hz)
PERF_OPTIONAL_FIELD(reliable_kb)
PERF_OPTIONAL_FIELD(send_buffer_kb)
#undef PERF_OPTIONAL_FIELD

// inspect/2 document builders, timed when this tree has them (empty: absent)
template <class S>
auto buildState(const S& sys, int) -> decltype(stateDocument(sys, uint64_t{1}, HostClock::now(),
                                                             int64_t{0}, nullptr),
                                               std::string()) {
    return stateDocument(sys, uint64_t{1}, HostClock::now(), HostClock::nowUs(), nullptr);
}
template <class S> std::string buildState(const S&, long) { return std::string(); }

template <class S>
auto buildHistory(const S& sys, int)
    -> decltype(historyDocument(sys, uint64_t{1}, HostClock::now(), std::size_t{300}),
                std::string()) {
    return historyDocument(sys, uint64_t{1}, HostClock::now(), std::size_t{300});
}
template <class S> std::string buildHistory(const S&, long) { return std::string(); }

// ---- clients -------------------------------------------------------------------------------

enum class Behavior { kFast, kSlow, kStall, kBurst };

const char* behaviorName(Behavior b) {
    switch (b) {
    case Behavior::kFast: return "fast";
    case Behavior::kSlow: return "slow";
    case Behavior::kStall: return "stall";
    case Behavior::kBurst: return "preview_burst";
    }
    return "?";
}

struct StateSample {
    int64_t recv_us      = 0;
    double  host_ms      = NAN;
    double  host_us      = NAN;   // inspect/2 only
    double  measured_ms  = NAN;   // robot.measured_at_host_ms
    double  publication  = NAN;
    std::size_t bytes    = 0;
};

struct PongSample {
    int64_t send_us = 0;
    int64_t recv_us = 0;
    double  host_us = NAN;   // inspect/2 pong
};

// one inspect/2 diag as received: the server's view of every client
struct DiagSample {
    int64_t     recv_us = 0;
    double      host_ms = NAN;
    double      seq     = NAN;
    Json        clients;        // diag.inspection.clients
    std::string clients_json;
};

struct ClientResult {
    std::string name;
    Behavior    behavior = Behavior::kFast;
    std::string contract;
    uint64_t    client_id = 0;   // inspect/2 hello.client_id, 0 when not sent
    bool        connected = false;
    std::string error;
    bool        closed_by_server = false;
    double      closed_at_s      = NAN;
    int64_t     start_us = 0, end_us = 0;
    int64_t     stall_begin_us = 0, stall_end_us = 0;
    int64_t     burst_begin_us = 0, burst_end_us = 0;
    uint64_t    bytes_read = 0;
    std::map<std::string, std::pair<uint64_t, uint64_t>> by_type;   // count, bytes
    std::vector<StateSample> states;
    std::vector<PongSample>  pongs;
    std::vector<double>      parse_ms;   // harness parse cost per text message
    std::size_t              parsed_after_run = 0;
    std::size_t              parsed_inline    = 0;   // over --defer-mb, in the receive loop
    std::vector<DiagSample>  diags;
    std::string              last_diag_inspection;   // inspect/2 diag.inspection
    std::vector<std::string> diag_clients_timeline;  // inspect/2, one per second
};

struct ClientSpec {
    std::string name;
    Behavior    behavior = Behavior::kFast;
};

class ClientRunner
{
public:
    ClientRunner(const Options& o, ClientSpec spec, std::string host, int port, ClockUs clock)
        : o_(o), spec_(std::move(spec)), host_(std::move(host)), port_(port), clock_(clock),
          ws_(clock) {
        result_.name     = spec_.name;
        result_.behavior = spec_.behavior;
    }

    void run(const std::atomic<bool>& stop) {
        std::string err;
        const int   rcvbuf = spec_.behavior == Behavior::kSlow ? o_.slow_rcvbuf : 0;
        result_.start_us   = clock_();
        if (!ws_.connect(host_, port_, "/ws", rcvbuf, err)) {
            result_.error = err;
            return;
        }
        result_.connected = true;
        int64_t next_ping = clock_() + 200000;
        int64_t last_tick = clock_();
        double  allowance = 0.0;
        const int64_t t0  = result_.start_us;
        if (spec_.behavior == Behavior::kStall) {
            result_.stall_begin_us = t0 + static_cast<int64_t>(o_.stall_at_s * 1e6);
            result_.stall_end_us   = result_.stall_begin_us + static_cast<int64_t>(o_.stall_ms * 1e3);
        }
        if (spec_.behavior == Behavior::kBurst) {
            result_.burst_begin_us = t0 + static_cast<int64_t>(o_.burst_at_s * 1e6);
            result_.burst_end_us   = result_.burst_begin_us + static_cast<int64_t>(o_.burst_ms * 1e3);
        }
        bool burst_on = false, burst_done = false;

        while (!stop.load() && !ws_.closed()) {
            const int64_t now = clock_();
            if (!contract_.empty() && now >= next_ping) {
                sendPing(now);
                next_ping = now + static_cast<int64_t>(o_.ping_ms) * 1000;
            }
            if (spec_.behavior == Behavior::kBurst && !contract_.empty()) {
                if (!burst_on && !burst_done && now >= result_.burst_begin_us) {
                    ws_.sendText("{\"type\":\"preview\",\"hz\":" + std::to_string(o_.burst_hz) +
                                 ",\"quality\":" + std::to_string(o_.burst_quality) +
                                 ",\"max_width\":" + std::to_string(o_.burst_width) + "}");
                    burst_on = true;
                } else if (burst_on && now >= result_.burst_end_us) {
                    ws_.sendText("{\"type\":\"preview\",\"hz\":" + std::to_string(preview_hz_) +
                                 ",\"quality\":" + std::to_string(preview_quality_) +
                                 ",\"max_width\":" + std::to_string(preview_width_) + "}");
                    burst_on   = false;
                    burst_done = true;
                }
            }
            long n = 0;
            if (spec_.behavior == Behavior::kSlow) {
                allowance += o_.slow_bytes_s * static_cast<double>(now - last_tick) / 1e6;
                allowance = std::min(allowance, o_.slow_bytes_s * 0.25);   // no hoarding
                last_tick = now;
                if (allowance >= 512.0) {
                    n = ws_.readSome(5, static_cast<std::size_t>(allowance));
                    if (n > 0) {
                        allowance -= static_cast<double>(n);
                    }
                } else {
                    std::this_thread::sleep_for(Ms(5));
                }
            } else if (spec_.behavior == Behavior::kStall && now >= result_.stall_begin_us &&
                       now < result_.stall_end_us) {
                std::this_thread::sleep_for(Ms(5));
            } else {
                n = ws_.readSome(5);
            }
            WsMessage m;
            while (ws_.pop(m)) {
                take(m);
            }
            if (n < 0 && ws_.closed()) {
                break;
            }
            if (!o_.subscribe.empty() && contract_ == "navigatr.inspect/2" && !subscribed_) {
                ws_.sendText(o_.subscribe);
                subscribed_ = true;
            }
        }
        result_.end_us     = clock_();
        result_.bytes_read = ws_.bytesRead();
        if (ws_.closed() && !stop.load()) {
            result_.closed_by_server = true;
            result_.closed_at_s = static_cast<double>(result_.end_us - result_.start_us) / 1e6;
        }
        ws_.close();
        finish();
    }

    const ClientResult& result() const { return result_; }

    void setPreviewDefaults(double hz, long quality, long width) {
        preview_hz_      = hz;
        preview_quality_ = quality;
        preview_width_   = width;
    }

private:
    void sendPing(int64_t now) {
        ++ping_id_;
        pending_[ping_id_] = now;
        if (contract_ == "navigatr.inspect/1") {
            ws_.sendPing(std::to_string(ping_id_));   // control ping; the pong shares the FIFO
        } else {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "{\"type\":\"ping\",\"id\":%llu,\"client_ms\":%.3f}",
                          static_cast<unsigned long long>(ping_id_),
                          static_cast<double>(now) / 1000.0);
            ws_.sendText(buf);
        }
    }

    void count(const std::string& type, std::size_t bytes) {
        auto& e = result_.by_type[type];
        ++e.first;
        e.second += bytes;
    }

    // Parsing one 51 KB inspect/1 snapshot cost this harness 12 to 28 ms
    // (p50, busy dev host). Done between reads it delays every later
    // message of a burst, so only the hello (the loop needs the contract)
    // is parsed now; the rest is kept raw with its read time and parsed by
    // finish().
    void take(WsMessage& m) {
        if (m.opcode == 0x2) {
            count("binary_preview", m.payload.size());
            return;
        }
        if (contract_.empty() && m.opcode == 0x1) {
            handle(m);
            return;
        }
        if (static_cast<double>(deferred_bytes_ + m.payload.size()) > o_.defer_mb * 1e6) {
            handle(m);
            ++result_.parsed_inline;
            return;
        }
        deferred_bytes_ += m.payload.size();
        raw_.push_back(std::move(m));
        m = WsMessage{};
    }

    void finish() {
        for (const WsMessage& m : raw_) {
            handle(m);
            ++result_.parsed_after_run;
        }
        raw_.clear();
        raw_.shrink_to_fit();
        // inline parses (over the cap) ran before the deferred ones
        const auto by_recv = [](const auto& a, const auto& b) { return a.recv_us < b.recv_us; };
        std::stable_sort(result_.states.begin(), result_.states.end(), by_recv);
        std::stable_sort(result_.pongs.begin(), result_.pongs.end(), by_recv);
        std::stable_sort(result_.diags.begin(), result_.diags.end(), by_recv);
        // the latest diag clients list received by each second of the run
        std::size_t di = 0;
        std::string last;
        for (int64_t t = result_.start_us + 1000000; t <= result_.end_us; t += 1000000) {
            while (di < result_.diags.size() && result_.diags[di].recv_us <= t) {
                last = result_.diags[di].clients_json;
                ++di;
            }
            if (!last.empty()) {
                result_.diag_clients_timeline.push_back(last);
            }
        }
    }

    void handle(const WsMessage& m) {
        const int64_t t0 = clock_();
        if (m.opcode == 0xA) {
            count("ws_pong", m.payload.size());
            notePong(std::strtoull(m.payload.c_str(), nullptr, 10), m.recv_us, NAN);
            return;
        }
        if (m.opcode == 0x2) {
            count("binary_preview", m.payload.size());
            return;
        }
        if (m.opcode == 0x8) {
            count("ws_close", m.payload.size());
            return;
        }
        if (m.opcode != 0x1) {
            count("ws_other", m.payload.size());
            return;
        }
        Json        doc;
        std::string err;
        if (!parseJson(m.payload, doc, &err)) {
            count("unparsed", m.payload.size());
            return;
        }
        const std::string type = doc.get("type") != nullptr ? doc.get("type")->stringOr("?") : "?";
        count(type, m.payload.size());
        if (type == "hello") {
            const Json* c = doc.get("contract");
            if (c != nullptr) {
                contract_         = c->stringOr("");
                result_.contract  = contract_;
            }
            const Json* id = doc.get("client_id");
            if (id != nullptr && id->isNumber()) {
                result_.client_id = static_cast<uint64_t>(id->number);
            }
        } else if (type == "snapshot" || type == "state") {
            StateSample s;
            s.recv_us = m.recv_us;
            s.bytes   = m.payload.size();
            s.host_ms = doc.get("host_ms") != nullptr ? doc.get("host_ms")->numberOr(NAN) : NAN;
            s.host_us = doc.get("host_us") != nullptr ? doc.get("host_us")->numberOr(NAN) : NAN;
            const Json* meas = doc.at({"robot", "measured_at_host_ms"});
            s.measured_ms    = meas != nullptr ? meas->numberOr(NAN) : NAN;
            const Json* pub  = type == "snapshot" ? doc.at({"localization", "publication"})
                                                  : doc.get("publication");
            s.publication = pub != nullptr ? pub->numberOr(NAN) : NAN;
            result_.states.push_back(s);
        } else if (type == "pong") {
            const Json* id = doc.get("id");
            const Json* hu = doc.get("host_us");
            const Json* hm = doc.get("host_ms");
            double      host_us = hu != nullptr ? hu->numberOr(NAN) : NAN;
            if (std::isnan(host_us) && hm != nullptr) {
                host_us = hm->numberOr(NAN) * 1000.0;
            }
            if (id != nullptr) {
                notePong(static_cast<uint64_t>(id->numberOr(0)), m.recv_us, host_us);
            }
        } else if (type == "diag") {
            // top level or under workers, as the snapshot has it
            const Json* insp = doc.get("inspection");
            if (insp == nullptr) {
                insp = doc.at({"workers", "inspection"});
            }
            if (insp != nullptr) {
                const Json* clients = insp->get("clients");
                if (clients != nullptr) {
                    DiagSample d;
                    d.recv_us      = m.recv_us;
                    d.host_ms      = doc.get("host_ms") != nullptr ? doc.get("host_ms")->numberOr(NAN)
                                                                   : NAN;
                    d.seq          = doc.get("seq") != nullptr ? doc.get("seq")->numberOr(NAN) : NAN;
                    d.clients      = *clients;
                    d.clients_json = toJson(*clients);
                    result_.diags.push_back(std::move(d));
                }
                result_.last_diag_inspection = toJson(*insp);
            }
        }
        result_.parse_ms.push_back(static_cast<double>(clock_() - t0) / 1000.0);
    }

    void notePong(uint64_t id, int64_t recv_us, double host_us) {
        const auto it = pending_.find(id);
        if (it == pending_.end()) {
            return;
        }
        PongSample p;
        p.send_us = it->second;
        p.recv_us = recv_us;
        p.host_us = host_us;
        result_.pongs.push_back(p);
        pending_.erase(it);
    }

    const Options&               o_;
    ClientSpec                   spec_;
    std::string                  host_;
    int                          port_;
    ClockUs                      clock_;
    WsClient                     ws_;
    ClientResult                 result_;
    std::string                  contract_;
    bool                         subscribed_ = false;
    uint64_t                     ping_id_    = 0;
    std::map<uint64_t, int64_t>  pending_;
    std::vector<WsMessage>       raw_;
    std::size_t                  deferred_bytes_  = 0;
    double                       preview_hz_      = 5.0;
    long                         preview_quality_ = 70;
    long                         preview_width_   = 640;
};

// ---- reporting ------------------------------------------------------------------------------

std::string num(double v) {
    if (!std::isfinite(v)) {
        return "null";
    }
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

// The server's own queue view of one client (inspect/2
// diag.inspection.clients[id]) over every diag any client of the scenario
// received: queue samples once per diag seq, and the channel counters of
// the newest one. "null" on inspect/1 or when the client never appeared.
std::string serverView(const std::vector<const ClientResult*>& all, uint64_t id) {
    if (id == 0) {
        return "null";
    }
    std::vector<const DiagSample*> ds;
    for (const ClientResult* c : all) {
        for (const DiagSample& d : c->diags) {
            ds.push_back(&d);
        }
    }
    std::stable_sort(ds.begin(), ds.end(), [](const DiagSample* a, const DiagSample* b) {
        return a->host_ms < b->host_ms;
    });
    std::vector<double> seen_seq;
    std::vector<double> queued, in_flight, backlog;
    const Json*         last      = nullptr;
    double              last_host = NAN;
    for (const DiagSample* d : ds) {
        if (!std::isnan(d->seq)) {
            if (std::find(seen_seq.begin(), seen_seq.end(), d->seq) != seen_seq.end()) {
                continue;   // the same diag, received by another client
            }
            seen_seq.push_back(d->seq);
        }
        for (const Json& e : d->clients.items) {
            const Json* eid = e.get("id");
            if (eid == nullptr || static_cast<uint64_t>(eid->numberOr(0)) != id) {
                continue;
            }
            const auto val = [&e](const char* k) {
                const Json* v = e.get(k);
                return v != nullptr ? v->numberOr(NAN) : NAN;
            };
            queued.push_back(val("queued_bytes"));
            in_flight.push_back(val("in_flight_bytes"));
            backlog.push_back(val("reliable_backlog"));
            last      = &e;
            last_host = d->host_ms;
        }
    }
    if (last == nullptr) {
        return "null";
    }
    // per channel as the server counts it, plus preview:* and reliable totals
    struct Tot {
        double queued = 0, sent = 0, replaced = 0, refused = 0, dropped = 0, bytes = 0;
        bool   any    = false;
    };
    Tot         preview, reliable;
    std::string channels = "{";
    const Json* chs      = last->get("channels");
    if (chs != nullptr) {
        for (std::size_t i = 0; i < chs->fields.size(); ++i) {
            const std::string& name = chs->fields[i].first;
            const Json&        ch   = chs->fields[i].second;
            channels += (i ? "," : "") + jsonQuote(name) + ":" + toJson(ch);
            const auto   val   = [&ch](const char* k) {
                const Json* v = ch.get(k);
                return v != nullptr ? v->numberOr(0) : 0.0;
            };
            const Json*  rep   = ch.get("replaceable");
            Tot*         t     = nullptr;
            if (name.compare(0, 8, "preview:") == 0) {
                t = &preview;
            } else if (rep != nullptr && rep->type == Json::Type::kBool && !rep->b) {
                t = &reliable;
            }
            if (t != nullptr) {
                t->any = true;
                t->queued += val("queued");
                t->sent += val("sent");
                t->replaced += val("replaced");
                t->refused += val("refused");
                t->dropped += val("dropped");
                t->bytes += val("bytes");
            }
        }
    }
    channels += "}";
    const auto tot = [](const Tot& t) -> std::string {
        if (!t.any) {
            return "null";
        }
        return "{\"queued\":" + num(t.queued) + ",\"sent\":" + num(t.sent) +
               ",\"replaced\":" + num(t.replaced) + ",\"refused\":" + num(t.refused) +
               ",\"dropped\":" + num(t.dropped) + ",\"bytes\":" + num(t.bytes) + "}";
    };
    std::ostringstream j;
    j << "{\"source\":\"diag.inspection.clients, newest diag received in the scenario\""
      << ",\"newest_diag_host_ms\":" << num(last_host) << ",\"samples\":" << queued.size()
      << ",\"queued_bytes\":" << summaryJson(summarize(queued))
      << ",\"in_flight_bytes\":" << summaryJson(summarize(in_flight))
      << ",\"reliable_backlog\":" << summaryJson(summarize(backlog))
      << ",\"channels\":" << channels << ",\"preview_total\":" << tot(preview)
      << ",\"reliable_total\":" << tot(reliable) << "}";
    return j.str();
}

// exact: both ends on the same host clock (in-process)
std::string clientJson(const ClientResult& r, bool same_clock, const Options& o,
                       const std::string& server_view) {
    std::ostringstream j;
    const double dur_s = std::max(1e-6, static_cast<double>(r.end_us - r.start_us) / 1e6);
    j << "{\"name\":" << jsonQuote(r.name) << ",\"behavior\":\"" << behaviorName(r.behavior)
      << "\",\"contract\":" << jsonQuote(r.contract) << ",\"connected\":"
      << (r.connected ? "true" : "false") << ",\"error\":" << jsonQuote(r.error)
      << ",\"duration_s\":" << num(dur_s) << ",\"closed_by_server\":"
      << (r.closed_by_server ? "true" : "false") << ",\"closed_at_s\":" << num(r.closed_at_s)
      << ",\"bytes_read\":" << r.bytes_read
      << ",\"read_bytes_s\":" << num(static_cast<double>(r.bytes_read) / dur_s);

    j << ",\"by_type\":{";
    bool first = true;
    for (const auto& kv : r.by_type) {
        j << (first ? "" : ",") << jsonQuote(kv.first) << ":{\"count\":" << kv.second.first
          << ",\"bytes\":" << kv.second.second
          << ",\"per_s\":" << num(static_cast<double>(kv.second.first) / dur_s)
          << ",\"bytes_s\":" << num(static_cast<double>(kv.second.second) / dur_s)
          << ",\"mean_bytes\":"
          << num(kv.second.first > 0 ? static_cast<double>(kv.second.second) /
                                           static_cast<double>(kv.second.first)
                                     : NAN)
          << "}";
        first = false;
    }
    j << "}";

    // state or snapshot messages
    std::vector<double> interval, pub_to_recv, source_age, age_at_recv, sizes;
    std::vector<double> rel_lag;
    double              min_raw = INFINITY;
    for (const StateSample& s : r.states) {
        const double recv_ms = static_cast<double>(s.recv_us) / 1000.0;
        const double host_ms = !std::isnan(s.host_us) ? s.host_us / 1000.0 : s.host_ms;
        min_raw              = std::min(min_raw, recv_ms - host_ms);
    }
    std::size_t stale = 0;
    for (std::size_t i = 0; i < r.states.size(); ++i) {
        const StateSample& s       = r.states[i];
        const double       recv_ms = static_cast<double>(s.recv_us) / 1000.0;
        const double host_ms = !std::isnan(s.host_us) ? s.host_us / 1000.0 : s.host_ms;
        sizes.push_back(static_cast<double>(s.bytes));
        if (i > 0) {
            interval.push_back(static_cast<double>(s.recv_us - r.states[i - 1].recv_us) / 1000.0);
        }
        if (!std::isnan(s.measured_ms) && !std::isnan(s.host_ms)) {
            source_age.push_back(s.host_ms - s.measured_ms);
        }
        rel_lag.push_back((recv_ms - host_ms) - min_raw);
        if (same_clock) {
            pub_to_recv.push_back(recv_ms - host_ms);
            if (recv_ms - host_ms > o.fresh_ms) {
                ++stale;
            }
            if (!std::isnan(s.measured_ms)) {
                age_at_recv.push_back(recv_ms - s.measured_ms);
            }
        }
    }
    double pub_first = NAN, pub_last = NAN;
    std::size_t distinct = 0;
    double      prev_pub = NAN;
    for (const StateSample& s : r.states) {
        if (std::isnan(s.publication)) {
            continue;
        }
        if (std::isnan(pub_first)) {
            pub_first = s.publication;
        }
        pub_last = s.publication;
        if (s.publication != prev_pub) {
            ++distinct;
        }
        prev_pub = s.publication;
    }
    double publication_rate = NAN;
    if (r.states.size() >= 2 && !std::isnan(pub_first)) {
        // over the documents' own build times, so a lagging reader still reads the true rate
        const double span_s = (r.states.back().host_ms - r.states.front().host_ms) / 1000.0;
        publication_rate    = span_s > 0 ? (pub_last - pub_first) / span_s : NAN;
    }
    j << ",\"state\":{\"count\":" << r.states.size()
      << ",\"per_s\":" << num(static_cast<double>(r.states.size()) / dur_s)
      << ",\"distinct_publications\":" << distinct
      << ",\"publication_rate_hz\":" << num(publication_rate)
      << ",\"size_bytes\":" << summaryJson(summarize(sizes))
      << ",\"interval_ms\":" << summaryJson(summarize(interval))
      << ",\"source_age_at_publish_ms\":" << summaryJson(summarize(source_age));
    if (same_clock) {
        j << ",\"publish_to_receive_ms\":" << summaryJson(summarize(pub_to_recv))
          << ",\"age_at_receipt_ms\":" << summaryJson(summarize(age_at_recv))
          << ",\"stale_over_fresh_ms\":" << stale;
    } else {
        j << ",\"relative_lag_ms\":" << summaryJson(summarize(rel_lag));
    }
    j << "}";

    // RTT and, for inspect/2 pongs, the section 3.4 offset estimate
    std::vector<double> rtt;
    for (const PongSample& p : r.pongs) {
        rtt.push_back(static_cast<double>(p.recv_us - p.send_us) / 1000.0);
    }
    j << ",\"rtt_ms\":" << summaryJson(summarize(rtt));
    {
        // spec 3.4, rolling over the last 20 pongs; each estimate is good
        // to +-half_width (the RTT/2 of the pong it used). Estimates wider
        // than offset_max_half_ms (pongs queued behind a backlog) are
        // counted but left out of publish_to_receive_ms.
        std::vector<Receipt> receipts;
        for (const StateSample& s : r.states) {
            receipts.push_back(Receipt{s.recv_us, s.host_us});
        }
        std::vector<PongTimes> pongs;
        for (const PongSample& p : r.pongs) {
            pongs.push_back(PongTimes{p.send_us, p.recv_us, p.host_us});
        }
        const std::vector<OffsetEstimate> est = pingOffsetEstimates(receipts, pongs, 20);
        // same clock only: the true offset is 0, so offset_ms is the error
        std::vector<double> used, used_half, all, all_half, err_all, err_used;
        for (const OffsetEstimate& e : est) {
            all.push_back(e.publish_to_receive_ms);
            all_half.push_back(e.half_width_ms);
            err_all.push_back(e.offset_ms);
            if (e.half_width_ms <= o.offset_max_half_ms) {
                used.push_back(e.publish_to_receive_ms);
                used_half.push_back(e.half_width_ms);
                err_used.push_back(e.offset_ms);
            }
        }
        if (est.empty()) {
            // inspect/1 pongs carry no host stamp: nothing to estimate from
            j << ",\"ping_offset_estimate\":null";
        } else {
            j << ",\"ping_offset_estimate\":{\"window_pongs\":20,\"max_half_width_ms\":"
              << num(o.offset_max_half_ms) << ",\"estimates\":" << est.size()
              << ",\"too_wide\":" << (est.size() - used.size())
              << ",\"publish_to_receive_ms\":" << summaryJson(summarize(used))
              << ",\"half_width_ms\":" << summaryJson(summarize(used_half))
              << ",\"publish_to_receive_all_ms\":" << summaryJson(summarize(all))
              << ",\"half_width_all_ms\":" << summaryJson(summarize(all_half));
            if (same_clock) {
                j << ",\"offset_error_ms\":" << summaryJson(summarize(err_used))
                  << ",\"offset_error_all_ms\":" << summaryJson(summarize(err_all));
            }
            j << "}";
        }
    }

    // per-second timeline from the client's start
    const int buckets = static_cast<int>(std::ceil(dur_s));
    std::vector<int>    tl_n(static_cast<std::size_t>(std::max(0, buckets)), 0);
    std::vector<double> tl_max(tl_n.size(), NAN);
    for (const StateSample& s : r.states) {
        const int b = static_cast<int>((s.recv_us - r.start_us) / 1000000);
        if (b < 0 || b >= buckets) {
            continue;
        }
        ++tl_n[static_cast<std::size_t>(b)];
        const double recv_ms = static_cast<double>(s.recv_us) / 1000.0;
        const double host_ms = !std::isnan(s.host_us) ? s.host_us / 1000.0 : s.host_ms;
        const double lag     = same_clock ? recv_ms - host_ms : (recv_ms - host_ms) - min_raw;
        double&      mx      = tl_max[static_cast<std::size_t>(b)];
        mx                   = std::isnan(mx) ? lag : std::max(mx, lag);
    }
    j << ",\"timeline\":{\"lag_kind\":\""
      << (same_clock ? "publish_to_receive_ms" : "relative_lag_ms") << "\",\"states\":[";
    for (std::size_t i = 0; i < tl_n.size(); ++i) {
        j << (i ? "," : "") << tl_n[i];
    }
    j << "],\"max_lag_ms\":[";
    for (std::size_t i = 0; i < tl_max.size(); ++i) {
        j << (i ? "," : "") << num(tl_max[i]);
    }
    j << "]}";

    // stall: what arrived after reading resumed
    if (r.behavior == Behavior::kStall && same_clock) {
        std::vector<double> pre;
        double              max_after = NAN, first_after = NAN, time_to_fresh = NAN;
        std::size_t         stale_after = 0;
        for (const StateSample& s : r.states) {
            const double lag = static_cast<double>(s.recv_us) / 1000.0 -
                               (!std::isnan(s.host_us) ? s.host_us / 1000.0 : s.host_ms);
            if (s.recv_us < r.stall_begin_us) {
                pre.push_back(lag);
                continue;
            }
            if (s.recv_us < r.stall_end_us) {
                continue;
            }
            if (std::isnan(first_after)) {
                first_after = lag;
            }
            max_after = std::isnan(max_after) ? lag : std::max(max_after, lag);
            if (lag > o.fresh_ms) {
                ++stale_after;
            } else if (std::isnan(time_to_fresh)) {
                time_to_fresh = static_cast<double>(s.recv_us - r.stall_end_us) / 1000.0;
            }
        }
        j << ",\"stall\":{\"stall_ms\":" << num(o.stall_ms) << ",\"pre_publish_to_receive_ms\":"
          << summaryJson(summarize(pre)) << ",\"first_after_resume_lag_ms\":" << num(first_after)
          << ",\"max_after_resume_lag_ms\":" << num(max_after)
          << ",\"stale_messages_after_resume\":" << stale_after
          << ",\"time_to_fresh_after_resume_ms\":" << num(time_to_fresh) << "}";
    }
    if (r.behavior == Behavior::kBurst && same_clock) {
        std::vector<double> before, during, after;
        for (const StateSample& s : r.states) {
            const double lag = static_cast<double>(s.recv_us) / 1000.0 -
                               (!std::isnan(s.host_us) ? s.host_us / 1000.0 : s.host_ms);
            if (s.recv_us < r.burst_begin_us) {
                before.push_back(lag);
            } else if (s.recv_us < r.burst_end_us) {
                during.push_back(lag);
            } else {
                after.push_back(lag);
            }
        }
        j << ",\"burst\":{\"burst_ms\":" << num(o.burst_ms) << ",\"hz\":" << num(o.burst_hz)
          << ",\"max_width\":" << o.burst_width << ",\"quality\":" << o.burst_quality
          << ",\"before_publish_to_receive_ms\":" << summaryJson(summarize(before))
          << ",\"during_publish_to_receive_ms\":" << summaryJson(summarize(during))
          << ",\"after_publish_to_receive_ms\":" << summaryJson(summarize(after)) << "}";
    }
    j << ",\"harness_parse\":{\"parsed\":\"after the run; the receive loop only reads\""
      << ",\"after_run\":" << r.parsed_after_run << ",\"inline_over_defer_cap\":" << r.parsed_inline
      << ",\"parse_ms\":" << summaryJson(summarize(r.parse_ms)) << "}";
    j << ",\"client_id\":" << r.client_id << ",\"server_view\":" << server_view;
    if (!r.last_diag_inspection.empty()) {
        j << ",\"last_diag_inspection\":" << r.last_diag_inspection;
    }
    if (!r.diag_clients_timeline.empty()) {
        j << ",\"diag_clients_timeline\":[";
        for (std::size_t i = 0; i < r.diag_clients_timeline.size(); ++i) {
            j << (i ? "," : "") << r.diag_clients_timeline[i];
        }
        j << "]";
    }
    j << "}";
    return j.str();
}

void printClient(const std::string& json) {
    Json v;
    if (!parseJson(json, v)) {
        return;
    }
    const auto sum = [&](const char* a, const char* b) -> std::string {
        const Json* s = b != nullptr ? v.at({a, b}) : v.get(a);
        if (s == nullptr || s->get("n") == nullptr || s->get("n")->number == 0) {
            return "n/a";
        }
        char buf[160];
        std::snprintf(buf, sizeof(buf), "p50 %.2f p95 %.2f max %.2f (n %.0f)",
                      s->get("p50")->number, s->get("p95")->number, s->get("max")->number,
                      s->get("n")->number);
        return buf;
    };
    std::printf("  client %-10s %-14s states %.1f/s  read %.0f B/s  closed %s\n",
                v.get("name")->str.c_str(), v.get("behavior")->str.c_str(),
                v.at({"state", "per_s"})->number, v.get("read_bytes_s")->number,
                v.get("closed_by_server")->b ? "yes" : "no");
    std::printf("    size bytes          %s\n", sum("state", "size_bytes").c_str());
    std::printf("    interval ms         %s\n", sum("state", "interval_ms").c_str());
    std::printf("    source age @pub ms  %s\n", sum("state", "source_age_at_publish_ms").c_str());
    if (v.at({"state", "publish_to_receive_ms"}) != nullptr) {
        // host_ms/host_us are stamped before the document is built
        std::printf("    publish->recv ms    %s  (build + queue + socket + read)\n",
                    sum("state", "publish_to_receive_ms").c_str());
        std::printf("    age at receipt ms   %s\n", sum("state", "age_at_receipt_ms").c_str());
    } else {
        std::printf("    relative lag ms     %s\n", sum("state", "relative_lag_ms").c_str());
    }
    const Json* est = v.get("ping_offset_estimate");
    if (est != nullptr && est->get("estimates") != nullptr && est->get("estimates")->number > 0) {
        const Json*  hw    = est->at({"half_width_ms", "p50"});
        const Json*  hn    = est->at({"half_width_ms", "n"});
        const Json*  wide  = est->get("too_wide");
        const double total = est->get("estimates")->number;
        const double limit = est->get("max_half_width_ms") != nullptr
                                 ? est->get("max_half_width_ms")->number
                                 : NAN;
        if (hw != nullptr && hn != nullptr && hn->number > 0) {
            std::printf("    ping-offset p->r ms %s  +-%.2f ms (half-width p50); %.0f of %.0f "
                        "estimates wider than +-%.0f ms not used\n",
                        sum("ping_offset_estimate", "publish_to_receive_ms").c_str(), hw->number,
                        wide != nullptr ? wide->number : NAN, total, limit);
        } else {
            std::printf("    ping-offset p->r ms n/a: all %.0f estimates wider than +-%.0f ms "
                        "(pongs waited behind queued data)\n",
                        total, limit);
        }
    }
    std::printf("    rtt ms              %s\n", sum("rtt_ms", nullptr).c_str());
    const Json* sv = v.get("server_view");
    if (sv != nullptr && sv->type == Json::Type::kObject) {
        const auto ch = [sv](const char* name) -> std::string {
            const Json* c = sv->at({"channels", name});
            if (c == nullptr) {
                return "-";
            }
            char buf[96];
            std::snprintf(buf, sizeof(buf), "%.0f/%.0f/%.0f",
                          c->get("sent") ? c->get("sent")->number : NAN,
                          c->get("replaced") ? c->get("replaced")->number : NAN,
                          c->get("refused") ? c->get("refused")->number : NAN);
            return buf;
        };
        const Json* qmax = sv->at({"queued_bytes", "max"});
        std::printf("    server view         state %s, diag %s (sent/replaced/refused); queued "
                    "bytes max %.0f\n",
                    ch("state").c_str(), ch("diag").c_str(),
                    qmax != nullptr ? qmax->number : NAN);
    }
}

// ---- server-side documents -------------------------------------------------------------------

std::string fetchSnapshotWorkers(const std::string& host, int port) {
    HttpResult r;
    if (!httpRequest(host, port, "GET", "/api/snapshot", r) || r.status != 200) {
        return "null";
    }
    Json doc;
    if (!parseJson(r.body, doc)) {
        return "null";
    }
    const Json* w = doc.get("workers");
    return w != nullptr ? toJson(*w) : "null";
}

struct Harness {
    Options                            o;
    FunctionRegistry                   functions;
    std::unique_ptr<System>            system;
    std::unique_ptr<InspectionService> service;
    std::string                        host = "127.0.0.1";
    int                                port = 0;
    ClockUs                            clock = &steadyUs;
    bool                               same_clock = false;
    double                             preview_hz = 5.0;
    long                               preview_quality = 70;
    long                               preview_width = 640;
    std::string                        conditions_extra;

    bool start(std::string& err) {
        if (o.config.empty()) {
            host  = o.connect_host;
            port  = o.connect_port;
            clock = &steadyUs;
            return true;
        }
        registerAll(functions);
        system = System::buildFromFile(o.config, functions, err);
        if (system == nullptr) {
            return false;
        }
        InspectionConfig cfg = system->inspection();
        cfg.enabled          = true;
        cfg.bind             = "127.0.0.1";
        cfg.port             = o.port;
        if (o.snapshot_hz > 0) {
            cfg.snapshot_hz = o.snapshot_hz;
        }
        if (o.client_buffer_kb > 0) {
            cfg.client_buffer_kb = o.client_buffer_kb;
        }
        if (o.stall_close_ms > 0) {
            cfg.stall_close_ms = o.stall_close_ms;
        }
        if (o.max_clients > 0) {
            cfg.max_clients = o.max_clients;
        }
        std::string extra;
        if (o.state_hz > 0) {
            extra += set_state_hz(cfg, o.state_hz, 0) ? ",\"state_hz_override\":" + num(o.state_hz)
                                                      : ",\"state_hz_override\":\"unsupported\"";
        }
        if (o.diag_hz > 0) {
            extra += set_diag_hz(cfg, o.diag_hz, 0) ? ",\"diag_hz_override\":" + num(o.diag_hz)
                                                    : ",\"diag_hz_override\":\"unsupported\"";
        }
        if (o.reliable_kb > 0) {
            extra += set_reliable_kb(cfg, o.reliable_kb, 0)
                         ? ",\"reliable_kb_override\":" + std::to_string(o.reliable_kb)
                         : ",\"reliable_kb_override\":\"unsupported\"";
        }
        if (o.send_buffer_kb >= 0) {
            extra += set_send_buffer_kb(cfg, o.send_buffer_kb, 0)
                         ? ",\"send_buffer_kb_override\":" + std::to_string(o.send_buffer_kb)
                         : ",\"send_buffer_kb_override\":\"unsupported\"";
        }
        preview_hz      = cfg.preview_hz;
        preview_quality = cfg.preview_quality;
        preview_width   = cfg.preview_max_width;
        // the settings in effect, whatever the tree: null = not in this tree.
        // snapshot_hz only drives inspect/1; inspect/2 runs on state_hz/diag_hz.
        conditions_extra =
            ",\"state_hz\":" + num(get_state_hz(cfg, 0)) +
            ",\"diag_hz\":" + num(get_diag_hz(cfg, 0)) +
            ",\"reliable_kb\":" + num(get_reliable_kb(cfg, 0)) +
            ",\"send_buffer_kb\":" + num(get_send_buffer_kb(cfg, 0)) +
            ",\"snapshot_hz_applies_to\":\"inspect/1 only\"" +
            ",\"snapshot_hz\":" + num(cfg.snapshot_hz) + ",\"preview_hz\":" + num(cfg.preview_hz) +
            ",\"preview_max_width\":" + std::to_string(cfg.preview_max_width) +
            ",\"preview_quality\":" + std::to_string(cfg.preview_quality) +
            ",\"client_buffer_kb\":" + std::to_string(cfg.client_buffer_kb) +
            ",\"stall_close_ms\":" + std::to_string(cfg.stall_close_ms) +
            ",\"max_clients\":" + std::to_string(cfg.max_clients) +
            ",\"loop_rate_hz\":" + num(system->loopRateHz()) + extra;
        service = InspectionService::create(*system, cfg, err);
        if (service == nullptr) {
            return false;
        }
        if (!system->start(err)) {
            return false;
        }
        if (!service->start(err)) {
            return false;
        }
        port       = service->port();
        clock      = &HostClock::nowUs;
        same_clock = true;
        return true;
    }

    // keeps the Brain profile boundary serviced, as main() does
    void idle(double seconds) {
        const auto end = std::chrono::steady_clock::now() +
                         std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(seconds));
        while (std::chrono::steady_clock::now() < end) {
            if (system != nullptr) {
                system->applyPendingProfile();
            }
            std::this_thread::sleep_for(Ms(20));
        }
    }

    void stop() {
        if (service != nullptr) {
            service->stop();
        }
        if (system != nullptr) {
            system->stop();
        }
    }
};

// Top-level key sizes of one snapshot, re-serialized compactly.
std::string sectionSizes(const std::string& doc_text) {
    Json doc;
    if (!parseJson(doc_text, doc) || doc.type != Json::Type::kObject) {
        return "null";
    }
    std::string out = "{";
    for (std::size_t i = 0; i < doc.fields.size(); ++i) {
        out += (i ? "," : "") + jsonQuote(doc.fields[i].first) + ":" +
               std::to_string(toJson(doc.fields[i].second).size());
    }
    const Json* trail = doc.get("trail");
    out += std::string(doc.fields.empty() ? "" : ",") + "\"trail_entries\":" +
           std::to_string(trail != nullptr ? trail->items.size() : 0) + "}";
    return out;
}

std::string scenarioBuild(Harness& h) {
    if (h.system == nullptr) {
        return "{\"skipped\":\"in-process only\"}";
    }
    std::vector<double> full_us, full_bytes, notrail_us, notrail_bytes;
    std::vector<double> state_us, state_bytes, history_us, history_bytes;
    std::string         last;
    const auto          end = std::chrono::steady_clock::now() +
                     std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(h.o.duration_s));
    int iter = 0;
    while (std::chrono::steady_clock::now() < end) {
        const InspectionServiceStats stats = h.service->stats();
        int64_t                      t0    = HostClock::nowUs();
        std::string                  doc   = snapshotDocument(*h.system, stats, HostClock::now());
        int64_t                      t1    = HostClock::nowUs();
        full_us.push_back(static_cast<double>(t1 - t0));
        full_bytes.push_back(static_cast<double>(doc.size()));
        if (iter % 4 == 0) {
            t0                    = HostClock::nowUs();
            const std::string nt  = snapshotDocument(*h.system, stats, HostClock::now(), 0);
            t1                    = HostClock::nowUs();
            notrail_us.push_back(static_cast<double>(t1 - t0));
            notrail_bytes.push_back(static_cast<double>(nt.size()));
        }
        t0                     = HostClock::nowUs();
        const std::string st   = buildState(*h.system, 0);
        t1                     = HostClock::nowUs();
        if (!st.empty()) {
            state_us.push_back(static_cast<double>(t1 - t0));
            state_bytes.push_back(static_cast<double>(st.size()));
        }
        if (iter % 4 == 0) {
            t0                    = HostClock::nowUs();
            const std::string hi  = buildHistory(*h.system, 0);
            t1                    = HostClock::nowUs();
            if (!hi.empty()) {
                history_us.push_back(static_cast<double>(t1 - t0));
                history_bytes.push_back(static_cast<double>(hi.size()));
            }
        }
        last = std::move(doc);
        ++iter;
        h.system->applyPendingProfile();
        std::this_thread::sleep_for(Ms(50));   // the default 20 Hz cadence
    }
    const int64_t     h0    = HostClock::nowUs();
    const std::string hello = helloDocument(*h.system, HostClock::now());
    const int64_t     h1    = HostClock::nowUs();
    std::ostringstream j;
    j << "{\"snapshot_build_us\":" << summaryJson(summarize(full_us))
      << ",\"snapshot_bytes\":" << summaryJson(summarize(full_bytes))
      << ",\"snapshot_no_trail_build_us\":" << summaryJson(summarize(notrail_us))
      << ",\"snapshot_no_trail_bytes\":" << summaryJson(summarize(notrail_bytes))
      << ",\"state_build_us\":" << summaryJson(summarize(state_us))
      << ",\"state_bytes\":" << summaryJson(summarize(state_bytes))
      << ",\"history_build_us\":" << summaryJson(summarize(history_us))
      << ",\"history_bytes\":" << summaryJson(summarize(history_bytes))
      << ",\"hello_build_us\":" << (h1 - h0) << ",\"hello_bytes\":" << hello.size()
      << ",\"last_snapshot_section_bytes\":" << sectionSizes(last) << "}";
    std::printf("build: snapshot %.0f B (p50), build p50 %.0f us p95 %.0f us max %.0f us; "
                "no-trail %.0f B, p50 %.0f us; hello %zu B %lld us\n",
                summarize(full_bytes).p50, summarize(full_us).p50, summarize(full_us).p95,
                summarize(full_us).max, summarize(notrail_bytes).p50, summarize(notrail_us).p50,
                hello.size(), static_cast<long long>(h1 - h0));
    if (!state_us.empty()) {
        std::printf("build: state %.0f B (p50), build p50 %.0f us p95 %.0f us; history %.0f B, "
                    "p50 %.0f us\n",
                    summarize(state_bytes).p50, summarize(state_us).p50, summarize(state_us).p95,
                    summarize(history_bytes).p50, summarize(history_us).p50);
    }
    return j.str();
}

std::string runClients(Harness& h, const std::vector<ClientSpec>& specs, const char* title) {
    const std::string workers_before = fetchSnapshotWorkers(h.host, h.port);
    std::vector<std::unique_ptr<ClientRunner>> runners;
    for (const ClientSpec& s : specs) {
        runners.push_back(std::make_unique<ClientRunner>(h.o, s, h.host, h.port, h.clock));
        runners.back()->setPreviewDefaults(h.preview_hz, h.preview_quality, h.preview_width);
    }
    std::atomic<bool>        stop{false};
    std::vector<std::thread> threads;
    for (auto& r : runners) {
        ClientRunner* p = r.get();
        threads.emplace_back([p, &stop] { p->run(stop); });
    }
    h.idle(h.o.duration_s);
    stop = true;
    for (std::thread& t : threads) {
        t.join();
    }
    // let the server notice the closes before sampling its counters
    h.idle(0.3);
    const std::string workers_after = fetchSnapshotWorkers(h.host, h.port);
    std::ostringstream j;
    j << "{\"clients\":[";
    std::printf("%s\n", title);
    std::vector<const ClientResult*> all;
    for (const auto& r : runners) {
        all.push_back(&r->result());
    }
    for (std::size_t i = 0; i < runners.size(); ++i) {
        const ClientResult& cr = runners[i]->result();
        const std::string   cj = clientJson(cr, h.same_clock, h.o, serverView(all, cr.client_id));
        j << (i ? "," : "") << cj;
        printClient(cj);
    }
    j << "],\"server_workers_before\":" << workers_before
      << ",\"server_workers_after\":" << workers_after << "}";
    // aggregate counters, from the documents so this compiles against any tree
    Json a, b;
    if (parseJson(workers_after, a) && parseJson(workers_before, b)) {
        const auto delta = [&](const char* k) {
            const Json* x = a.at({"inspection", k});
            const Json* y = b.at({"inspection", k});
            return x != nullptr && y != nullptr ? x->number - y->number : NAN;
        };
        // snapshots_skipped: inspect/1 refusals; inspect/2 states replaced
        // unsent or refused (latest-wins working, mostly)
        std::printf("  server: state msgs queued +%.0f not delivered +%.0f, previews queued +%.0f "
                    "not delivered +%.0f, disconnects +%.0f\n",
                    delta("snapshots_sent"), delta("snapshots_skipped"), delta("frames_sent"),
                    delta("frames_skipped"), delta("client_disconnects"));
        if (a.at({"inspection", "messages_refused"}) != nullptr) {
            std::printf("  server (inspect/2): replaced unsent +%.0f, refused +%.0f, closed "
                        "stalled +%.0f, closed reliable overflow +%.0f\n",
                        delta("messages_replaced"), delta("messages_refused"),
                        delta("closed_stalled"), delta("closed_reliable_overflow"));
        }
    }
    return j.str();
}

} // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parseOptions(argc, argv, o)) {
        usage();
        return 2;
    }
    initSockets();
    Harness h;
    h.o = o;
    std::string err;
    if (!h.start(err)) {
        std::fprintf(stderr, "start failed: %s\n", err.c_str());
        return 1;
    }

    std::vector<std::string> list;
    {
        std::string s = o.scenarios == "all" ? "build,fast,slow,stall,burst" : o.scenarios;
        std::stringstream ss(s);
        std::string       item;
        while (std::getline(ss, item, ',')) {
            if (!item.empty()) {
                list.push_back(item);
            }
        }
    }

    std::printf("warmup %.1f s\n", o.warmup_s);
    h.idle(o.warmup_s);

    // hello first: the contract, and which server this is (connect mode
    // has only the server's word for its settings)
    std::string contract = "?";
    std::string server_hello = ",\"server_session\":null,\"server_configuration\":null,"
                               "\"server_inspection\":null";
    {
        HttpResult r;
        Json       hello;
        if (httpRequest(h.host, h.port, "GET", "/api/hello", r) && parseJson(r.body, hello)) {
            const Json* c = hello.get("contract");
            if (c != nullptr) {
                contract = c->str;
            }
            const auto sub = [&hello](const char* k) {
                const Json* v = hello.get(k);
                return v != nullptr ? toJson(*v) : std::string("null");
            };
            server_hello = ",\"server_session\":" + sub("session") +
                           ",\"server_configuration\":" + sub("configuration") +
                           ",\"server_inspection\":" + sub("inspection");
        }
    }
    std::printf("contract %s, %s\n", contract.c_str(),
                h.same_clock ? "in-process (exact host-clock ages)"
                             : "connected (RTT and ping-offset estimates only)");

    std::ostringstream results;
    results << "{";
    bool first = true;
    for (const std::string& sc : list) {
        std::string     r;
        const CpuSample cpu0 = sampleCpu();
        if (sc == "build") {
            r = scenarioBuild(h);
        } else if (sc == "fast") {
            r = runClients(h, {{"fast", Behavior::kFast}}, "fast: one client reading at once");
        } else if (sc == "slow") {
            r = runClients(h, {{"fast", Behavior::kFast}, {"slow", Behavior::kSlow}},
                           "slow: one fast client plus one throttled reader");
        } else if (sc == "stall") {
            r = runClients(h, {{"observer", Behavior::kFast}, {"stall", Behavior::kStall}},
                           "stall: one client stops reading, then drains");
        } else if (sc == "burst") {
            r = runClients(h, {{"observer", Behavior::kFast}, {"burst", Behavior::kBurst}},
                           "burst: one client raises its preview budget for a while");
        } else {
            std::fprintf(stderr, "unknown scenario %s\n", sc.c_str());
            continue;
        }
        const CpuSample cpu1    = sampleCpu();
        const double    machine = machineBusyPercent(cpu0, cpu1);
        const double    self    = processPercent(cpu0, cpu1);
        std::printf("  host cpu: machine %.0f%% of all cores, this process %.1f%%\n", machine,
                    self);
        if (!r.empty() && r.back() == '}') {
            r.insert(r.size() - 1, ",\"host_cpu\":{\"machine_busy_pct\":" + num(machine) +
                                       ",\"process_pct\":" + num(self) +
                                       ",\"other_pct\":" + num(machine - self) + "}");
        }
        results << (first ? "" : ",") << jsonQuote(sc) << ":" << r;
        first = false;
    }
    results << "}";
    h.stop();

    char       when[64] = "";
    std::time_t t       = std::time(nullptr);
    std::strftime(when, sizeof(when), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    std::ostringstream all;
    all << "{\"tool\":\"navigatr_inspect_bench\",\"conditions\":{\"when\":\"" << when
        << "\",\"mode\":\"" << (h.same_clock ? "in_process" : "connect")
        << "\",\"config\":" << jsonQuote(o.config) << ",\"target\":"
        << jsonQuote(h.host + ":" + std::to_string(h.port)) << ",\"contract\":"
        << jsonQuote(contract) << ",\"duration_s\":" << num(o.duration_s)
        << ",\"warmup_s\":" << num(o.warmup_s) << ",\"ping_ms\":" << o.ping_ms
        << ",\"slow_bytes_s\":" << num(o.slow_bytes_s) << ",\"slow_rcvbuf\":" << o.slow_rcvbuf
        << ",\"stall_at_s\":" << num(o.stall_at_s) << ",\"stall_ms\":" << num(o.stall_ms)
        << ",\"burst_at_s\":" << num(o.burst_at_s) << ",\"burst_ms\":" << num(o.burst_ms)
        << ",\"fresh_ms\":" << num(o.fresh_ms) << ",\"subscribe\":" << jsonQuote(o.subscribe)
        << ",\"offset_max_half_ms\":" << num(o.offset_max_half_ms)
        << ",\"harness_parse\":\"after each client's run\"" << server_hello
        << ",\"hardware_threads\":" << std::thread::hardware_concurrency()
#ifdef NDEBUG
        << ",\"build\":\"optimized (NDEBUG)\""
#else
        << ",\"build\":\"debug (no NDEBUG)\""
#endif
        << ",\"compiler\":" << jsonQuote(__VERSION__) << h.conditions_extra
        << "},\"scenarios\":" << results.str() << "}\n";
    if (!o.out.empty()) {
        std::ofstream f(o.out, std::ios::binary);
        f << all.str();
        std::printf("wrote %s\n", o.out.c_str());
    } else {
        std::printf("%s", all.str().c_str());
    }
    return 0;
}
