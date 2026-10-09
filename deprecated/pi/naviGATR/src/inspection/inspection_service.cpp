// inspection_service.cpp
// The service thread builds the inspect/2 feed from the System's published
// snapshots and hands complete frames to the HTTP server, which owns the
// sockets and the per-client queues. Documents are built once per tick and
// shared by every client due for them. HTTP requests and the WebSocket
// callbacks (open, text, close) run on the server thread; the client table
// and the counters sit under one mutex, the preview cache under another.
// The service never holds its mutex while it builds a document or calls
// into the server, and nothing here touches a worker's mutable maps.
//
// Ordering per client: hello, history and capture status are queued on the
// server thread when the client opens, before the client is registered
// here, so they precede everything the service thread sends. A session
// reset clears the client's unsent replaceable messages and queues a new
// hello and history before the next state.

#include "inspection/inspection_service.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "capture/capture_http.h"
#include "capture/capture_service.h"
#include "core/host_clock.h"
#include "diagnostics/hub.h"
#include "diagnostics/instrumentation.h"
#include "inspection/http_server.h"
#include "inspection/inspection_document.h"
#include "inspection/jpeg_encoder.h"
#include "inspection/json_writer.h"
#include "inspection/viewer_assets.h"

namespace navigatr
{

namespace
{

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

constexpr std::size_t kCacheEntries = 16;

// Channel priorities: lower goes first when the socket frees up.
const WsChannel kStateChannel{"state", 0};
const WsChannel kTelemetryChannel{"telemetry", 1};
const WsChannel kDiagChannel{"diag", 2};
const WsChannel kInstrumentationChannel{"instrumentation", 3};
constexpr int   kPreviewPriority = 4;

constexpr Ms     kStateKeepalive{1000};    // a state goes at least this often
// A diag whose content did not change (only clocks, ages, counters and the
// robot, which state carries) still goes this often, so those stay current.
constexpr Ms     kDiagRefresh{1000};
constexpr Ms     kPublicationPoll{5};      // waiting for the feed to advance
constexpr Ms     kEventPoll{50};
constexpr Ms     kTelemetryPeriod{50};     // 20 Hz at most
constexpr Ms     kInstrumentationPeriod{250};   // 4 Hz at most
constexpr Ms     kIdleWait{250};

Clock::duration periodOf(double hz) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz));
}

// A flat scan for one key in a small JSON object: enough for the client
// messages, which a browser writes and nothing else parses. quoted tells
// whether the value was a string.
bool jsonToken(const std::string& json, const char* key, std::string& token,
               bool* quoted = nullptr) {
    const std::string needle = std::string("\"") + key + "\"";
    std::size_t       pos    = json.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    pos += needle.size();
    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
        ++pos;
    }
    if (pos >= json.size() || json[pos] != ':') {
        return false;
    }
    ++pos;
    while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
        ++pos;
    }
    if (pos >= json.size()) {
        return false;
    }
    if (json[pos] == '"') {
        const std::size_t end = json.find('"', pos + 1);
        if (end == std::string::npos) {
            return false;
        }
        token = json.substr(pos + 1, end - pos - 1);
        if (quoted != nullptr) {
            *quoted = true;
        }
        return true;
    }
    std::size_t end = pos;
    while (end < json.size() && json[end] != ',' && json[end] != '}' && json[end] != ']' &&
           !std::isspace(static_cast<unsigned char>(json[end]))) {
        ++end;
    }
    token = json.substr(pos, end - pos);
    if (quoted != nullptr) {
        *quoted = false;
    }
    return !token.empty();
}

bool jsonNumber(const std::string& json, const char* key, double& out) {
    std::string token;
    bool        quoted = false;
    if (!jsonToken(json, key, token, &quoted) || quoted) {
        return false;
    }
    char*        end = nullptr;
    const double v   = std::strtod(token.c_str(), &end);
    if (end == token.c_str() || *end != '\0') {
        return false;
    }
    out = v;
    return true;
}

bool jsonBool(const std::string& json, const char* key, bool& out) {
    std::string token;
    bool        quoted = false;
    if (!jsonToken(json, key, token, &quoted) || quoted) {
        return false;
    }
    if (token == "true") {
        out = true;
        return true;
    }
    if (token == "false") {
        out = false;
        return true;
    }
    return false;
}

// A value to echo verbatim in a pong: a plain number token, else a string
// rewritten through the JSON writer, else empty (null).
std::string echoToken(const std::string& json, const char* key) {
    std::string token;
    bool        quoted = false;
    if (!jsonToken(json, key, token, &quoted) || token.size() > 40) {
        return {};
    }
    if (quoted) {
        JsonWriter w;
        w.value(token);
        return w.take();
    }
    char*        end = nullptr;
    const double v   = std::strtod(token.c_str(), &end);
    (void)v;
    if (end == token.c_str() || *end != '\0' ||
        token.find_first_not_of("0123456789+-.eE") != std::string::npos) {
        return {};
    }
    return token;
}

std::string contentTypeFor(const std::string& path) {
    const std::size_t dot = path.rfind('.');
    std::string       ext = dot == std::string::npos ? std::string() : path.substr(dot);
    for (char& c : ext) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (ext == ".html") {
        return "text/html; charset=utf-8";
    }
    if (ext == ".js") {
        return "text/javascript; charset=utf-8";
    }
    if (ext == ".css") {
        return "text/css; charset=utf-8";
    }
    if (ext == ".json") {
        return "application/json";
    }
    if (ext == ".svg") {
        return "image/svg+xml";
    }
    if (ext == ".png") {
        return "image/png";
    }
    if (ext == ".ico") {
        return "image/x-icon";
    }
    return "application/octet-stream";
}

// A relative path a development directory may serve: no traversal, no
// backslashes, no absolute component.
bool safeRelativePath(const std::string& rel) {
    if (rel.empty() || rel.front() == '/' || rel.find("..") != std::string::npos ||
        rel.find('\\') != std::string::npos || rel.find(':') != std::string::npos) {
        return false;
    }
    for (const char c : rel) {
        if (static_cast<unsigned char>(c) < 0x20) {
            return false;
        }
    }
    return true;
}

HttpResponse jsonResponse(int status, std::string body) {
    HttpResponse r;
    r.status                   = status;
    r.content_type             = "application/json";
    r.body                     = std::move(body);
    r.headers["Cache-Control"] = "no-store";
    return r;
}

HttpResponse notFound() {
    HttpResponse r;
    r.status = 404;
    r.body   = "not found\n";
    return r;
}

HttpResponse methodNotAllowed() {
    HttpResponse r;
    r.status           = 405;
    r.body             = "method not allowed\n";
    r.headers["Allow"] = "GET";
    return r;
}

double usSince(int64_t start_us) { return static_cast<double>(HostClock::nowUs() - start_us); }

} // namespace

struct InspectionService::Impl {
    struct FrameIdentity {
        uint64_t epoch    = 0;
        uint32_t sequence = 0;
        bool     set      = false;

        bool operator==(const FrameIdentity& o) const {
            return set && o.set && epoch == o.epoch && sequence == o.sequence;
        }
    };

    struct ClientState {
        // preview budget
        long              quality   = 70;
        long              max_width = 640;
        Clock::time_point next_preview;
        std::map<std::string, FrameIdentity> received;   // per camera, last queued

        FeedSubscription sub;

        // delivery bookkeeping, service thread
        uint64_t          hello_reset = 0;   // reset count the newest hello described
        bool              have_state  = false;
        uint64_t          state_publication = 0;
        Clock::time_point last_state;
        Clock::time_point next_state;
        bool              diag_now  = true;   // next diag goes regardless of identity
        bool              have_diag = false;
        uint64_t          diag_hash = 0;
        Clock::time_point last_diag;
        uint64_t          diag_skipped  = 0;
        uint64_t          telemetry_seq = 0;
        Clock::time_point next_telemetry;
        bool              instrumentation_now = false;   // just subscribed: one now
        uint64_t          capture_version     = 0;
        bool              history_requested = false;
    };

    struct CachedPreview {
        std::string camera;
        uint64_t    epoch     = 0;
        uint32_t    sequence  = 0;
        long        max_width = 0;
        long        quality   = 0;
        int         width_px  = 0;
        int         height_px = 0;
        HttpServer::Frame message;   // WebSocket frame of [u32 LE len][header][jpeg]
        std::shared_ptr<const std::vector<uint8_t>> jpeg;
    };

    struct Asset {
        std::string          content_type;
        const unsigned char* data = nullptr;
        std::size_t          size = 0;
    };

    System*          system = nullptr;
    InspectionConfig config;

    std::map<std::string, Asset> embedded;   // served path -> bytes
    std::unique_ptr<HttpServer>  server;

    std::thread             thread;
    std::mutex              wake_mutex;
    std::condition_variable wake_cv;
    bool                    stop_requested = false;
    bool                    wake_pending   = false;
    std::atomic<bool>       running{false};

    mutable std::mutex                          mutex;   // clients, stats, sequence numbers
    std::map<HttpServer::ClientId, ClientState> clients;
    InspectionServiceStats                      stats;
    InspectionFeedStats                         feed;
    uint64_t                                    seq_state   = 0;
    uint64_t                                    seq_diag    = 0;
    uint64_t                                    seq_history = 0;
    uint64_t                                    seq_capture = 0;
    uint64_t                                    seq_telemetry       = 0;
    uint64_t                                    seq_instrumentation = 0;

    std::mutex                 cache_mutex;
    std::vector<CachedPreview> cache;

    // service thread only
    uint64_t          event_cursor = 0;   // newest event sequence pushed
    Clock::time_point next_event_check;
    bool              links_raw     = false;   // what this service switched on
    bool              links_decoded = false;
    // one schedule for every client, so a document is built at most once
    // per period however many clients subscribe
    Clock::time_point next_diag;
    Clock::time_point next_instrumentation;
    std::atomic<bool> telemetry_seen{false};   // the hub has had a TELEMETRY record
    Clock::time_point window_start;
    uint64_t          window_states = 0;
    uint64_t          window_frames = 0;
    uint64_t          window_diags  = 0;

    InspectionServiceStats composedStats() const;
    InspectionFeedStats    composedFeed() const;
    HttpResponse           handle(const HttpRequest& req);
    HttpResponse           serveStatic(const std::string& path);
    void                   onOpen(HttpServer::ClientId id);
    void                   onText(HttpServer::ClientId id, const std::string& text);
    void                   onClose(HttpServer::ClientId id);
    void                   wake();
    bool preview(const DetectionFrameSnapshot& frame, long max_width, long quality,
                 CachedPreview& out, std::string& err);
    void run();
    Clock::time_point tick(Clock::time_point now);
    void rehello(Clock::time_point now);
    void pushEvents(Clock::time_point now);
    void pushCapture();
    void pushHistories();
    void pushStates(Clock::time_point now);
    void pushTelemetry(Clock::time_point now);
    void pushDiags(Clock::time_point now);
    void pushInstrumentation(Clock::time_point now);
    void publishPreviews(Clock::time_point now);
    void applyLinkMonitors(bool raw, bool decoded);
    void countEnqueue(HttpServer::Enqueue r, uint64_t& queued, uint64_t* replaced);
    Clock::time_point nextDeadline(Clock::time_point now) const;
};

InspectionServiceStats InspectionService::Impl::composedStats() const {
    InspectionServiceStats s;
    {
        std::lock_guard<std::mutex> lock(mutex);
        s = stats;
    }
    const HttpServerStats hs = server->stats();
    s.running                  = running.load();
    s.port                     = server->port();
    s.clients                  = hs.websocket_clients;
    s.bytes_sent               = hs.bytes_sent;
    s.messages_replaced        = hs.messages_replaced;
    s.messages_refused         = hs.messages_refused;
    s.closed_stalled           = hs.clients_closed_stalled;
    s.closed_reliable_overflow = hs.clients_closed_reliable_overflow;
    s.closed_protocol          = hs.clients_closed_protocol;
    s.closed_peer              = hs.clients_closed_peer;
    s.flow_control_clients     = hs.flow_control_clients;
    if (!s.running) {
        s.snapshot_rate_hz = 0.0;
        s.frame_rate_hz    = 0.0;
        s.diag_rate_hz     = 0.0;
    }
    return s;
}

InspectionFeedStats InspectionService::Impl::composedFeed() const {
    const std::vector<WsClientStats> queues = server->clientStats();
    const HttpServerStats            hs     = server->stats();
    std::lock_guard<std::mutex>      lock(mutex);
    InspectionFeedStats              f = feed;
    f.state_hz                         = config.state_hz;
    f.diag_hz                          = config.diag_hz;
    f.recent_closes                    = hs.recent_closes;
    f.clients.clear();
    for (const WsClientStats& q : queues) {
        FeedClientStats c;
        c.queue        = q;
        const auto it  = clients.find(q.id);
        if (it != clients.end()) {
            c.subscription = it->second.sub;
            c.diag_skipped = it->second.diag_skipped;
        }
        f.clients.push_back(std::move(c));
    }
    return f;
}

void InspectionService::Impl::wake() {
    {
        std::lock_guard<std::mutex> lock(wake_mutex);
        wake_pending = true;
    }
    wake_cv.notify_all();
}

void InspectionService::Impl::countEnqueue(HttpServer::Enqueue r, uint64_t& queued,
                                           uint64_t* replaced) {
    // caller holds mutex
    if (r == HttpServer::Enqueue::kQueued || r == HttpServer::Enqueue::kReplaced) {
        ++queued;
    }
    if (replaced != nullptr &&
        (r == HttpServer::Enqueue::kReplaced || r == HttpServer::Enqueue::kRefused)) {
        ++*replaced;
    }
}

// Previews are encoded once per (frame identity, width, quality) and shared
// by every client asking for that budget; only the newest identity per
// camera is ever wanted, so older entries for the camera go when a new one
// is encoded.
bool InspectionService::Impl::preview(const DetectionFrameSnapshot& frame, long max_width,
                                      long quality, CachedPreview& out, std::string& err) {
    if (frame.y8 == nullptr || frame.y8->empty()) {
        err = "frame has no image";
        return false;
    }
    std::lock_guard<std::mutex> lock(cache_mutex);
    for (const CachedPreview& c : cache) {
        if (c.camera == frame.camera.value && c.epoch == frame.frame_epoch &&
            c.sequence == frame.frame_sequence && c.max_width == max_width &&
            c.quality == quality) {
            out = c;
            return true;
        }
    }
    cache.erase(std::remove_if(cache.begin(), cache.end(),
                               [&](const CachedPreview& c) {
                                   return c.camera == frame.camera.value &&
                                          (c.epoch != frame.frame_epoch ||
                                           c.sequence != frame.frame_sequence);
                               }),
                cache.end());
    if (cache.size() >= kCacheEntries) {
        cache.erase(cache.begin());
    }

    JpegPreview jpeg;
    if (!encodeY8Jpeg(frame.y8->data(), frame.width_px, frame.height_px,
                      static_cast<int>(max_width), static_cast<int>(quality), jpeg, err)) {
        return false;
    }
    const std::string header = frameHeaderDocument(frame, jpeg.width_px, jpeg.height_px, quality,
                                                   jpeg.encode_ms, HostClock::now(), system);
    std::vector<uint8_t> message;
    message.reserve(4 + header.size() + jpeg.bytes.size());
    const uint32_t len = static_cast<uint32_t>(header.size());
    message.push_back(static_cast<uint8_t>(len & 0xff));
    message.push_back(static_cast<uint8_t>((len >> 8) & 0xff));
    message.push_back(static_cast<uint8_t>((len >> 16) & 0xff));
    message.push_back(static_cast<uint8_t>((len >> 24) & 0xff));
    message.insert(message.end(), header.begin(), header.end());
    message.insert(message.end(), jpeg.bytes.begin(), jpeg.bytes.end());

    CachedPreview entry;
    entry.camera    = frame.camera.value;
    entry.epoch     = frame.frame_epoch;
    entry.sequence  = frame.frame_sequence;
    entry.max_width = max_width;
    entry.quality   = quality;
    entry.width_px  = jpeg.width_px;
    entry.height_px = jpeg.height_px;
    entry.message   = HttpServer::binaryFrame(message.data(), message.size());
    entry.jpeg      = std::make_shared<const std::vector<uint8_t>>(std::move(jpeg.bytes));
    cache.push_back(entry);
    out = entry;

    std::lock_guard<std::mutex> stats_lock(mutex);
    ++stats.encodes;
    stats.last_encode_ms = jpeg.encode_ms;
    stats.mean_encode_ms = stats.encodes == 1
                               ? jpeg.encode_ms
                               : 0.9 * stats.mean_encode_ms + 0.1 * jpeg.encode_ms;
    return true;
}

HttpResponse InspectionService::Impl::serveStatic(const std::string& path) {
    const std::string rel = path == "/" ? std::string("index.html") : path.substr(1);
    if (!config.static_root.empty() && safeRelativePath(rel)) {
        const std::filesystem::path full = std::filesystem::path(config.static_root) / rel;
        std::error_code             ec;
        if (std::filesystem::is_regular_file(full, ec)) {
            std::ifstream in(full, std::ios::binary);
            if (in) {
                HttpResponse r;
                r.body.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
                r.content_type             = contentTypeFor(rel);
                r.headers["Cache-Control"] = "no-store";   // development: always fresh
                return r;
            }
        }
    }
    const auto it = embedded.find(rel);
    if (it == embedded.end()) {
        return notFound();
    }
    HttpResponse r;
    r.content_type = it->second.content_type;
    r.body.assign(reinterpret_cast<const char*>(it->second.data), it->second.size);
    return r;
}

HttpResponse InspectionService::Impl::handle(const HttpRequest& req) {
    const std::string& path = req.path;
    // capture control and download: the only routes that take POST
    {
        HttpResponse r;
        if (captureRoute(*system, req, r)) {
            return r;
        }
    }
    if (req.method != "GET") {
        return methodNotAllowed();
    }
    if (path.rfind("/api/", 0) != 0) {
        return serveStatic(path);
    }
    if (path == "/api/hello") {
        return jsonResponse(200, helloDocument(*system, HostClock::now()));
    }
    if (path == "/api/snapshot") {
        return jsonResponse(200, snapshotDocument(*system, composedStats(), HostClock::now()));
    }
    if (path == "/api/health") {
        const HttpServerStats hs = server->stats();
        std::string           body = "{\"ok\":true,\"running\":";
        body += running.load() ? "true" : "false";
        body += ",\"clients\":" + std::to_string(hs.websocket_clients);
        body += ",\"port\":" + std::to_string(server->port());
        body += ",\"contract\":\"" + std::string(kInspectionContract) + "\"}";
        return jsonResponse(200, body);
    }
    if (path == "/api/frame.jpg") {
        const std::string camera = req.param("camera");
        const auto        frames = system->detectionFrames();
        std::shared_ptr<const DetectionFrameSnapshot> frame;
        for (const auto& kv : frames) {
            if (camera.empty() || kv.first.value == camera) {
                frame = kv.second;
                break;
            }
        }
        if (frame == nullptr) {
            return jsonResponse(404, "{\"error\":\"no frame\"}");
        }
        CachedPreview p;
        std::string   err;
        if (!preview(*frame, config.preview_max_width, config.preview_quality, p, err)) {
            return jsonResponse(404, "{\"error\":\"no image\"}");
        }
        HttpResponse r;
        r.content_type = "image/jpeg";
        r.body.assign(reinterpret_cast<const char*>(p.jpeg->data()), p.jpeg->size());
        r.headers["Cache-Control"] = "no-store";
        return r;
    }
    return jsonResponse(404, "{\"error\":\"not found\"}");
}

// Server thread. Hello, history and capture status are queued before the
// client is registered, so the service thread cannot send it anything
// ahead of its hello.
void InspectionService::Impl::onOpen(HttpServer::ClientId id) {
    const uint64_t      reset = system->resetCount();
    const MonotonicTime now   = HostClock::now();
    uint64_t            h_seq = 0, c_seq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex);
        h_seq = ++seq_history;
    }
    const auto          r_hello = server->sendReliable(
        id, HttpServer::textFrame(helloDocument(*system, now, id)), "hello");
    const int64_t       t0      = HostClock::nowUs();
    const std::string   history = historyDocument(*system, h_seq, now);
    const double        h_us    = usSince(t0);
    const auto          r_hist  = server->sendReliable(id, HttpServer::textFrame(history), "history");
    uint64_t            capture_version = 0;
    auto                r_cap           = HttpServer::Enqueue::kUnknownClient;
    if (const CaptureService* capture = system->capture()) {
        capture_version = capture->version();
        {
            std::lock_guard<std::mutex> lock(mutex);
            c_seq = ++seq_capture;
        }
        r_cap = server->sendReliable(
            id, HttpServer::textFrame(captureDocument(*capture, c_seq, now)), "capture");
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        ClientState&                c = clients[id];
        c.sub.state_hz                = config.state_hz;
        c.sub.preview_hz              = config.preview_hz;
        c.quality                     = config.preview_quality;
        c.max_width                   = config.preview_max_width;
        c.next_preview                = Clock::now();
        c.hello_reset                 = reset;
        c.capture_version             = capture_version;
        ++stats.clients_total;
        feed.history.add(h_us, history.size());
        countEnqueue(r_hello, stats.hellos_queued, nullptr);
        countEnqueue(r_hist, stats.histories_queued, nullptr);
        countEnqueue(r_cap, stats.captures_queued, nullptr);
    }
    wake();
}

void InspectionService::Impl::onText(HttpServer::ClientId id, const std::string& text) {
    std::string type;
    if (!jsonToken(text, "type", type)) {
        return;
    }
    if (type == "ping") {
        // answered at once, on this thread: the RTT excludes the service tick
        const std::string pong = pongDocument(echoToken(text, "id"), echoToken(text, "client_ms"),
                                              HostClock::now(), HostClock::nowUs());
        const auto r = server->sendReliable(id, HttpServer::textFrame(pong), "pong");
        std::lock_guard<std::mutex> lock(mutex);
        countEnqueue(r, stats.pongs_queued, nullptr);
        return;
    }
    double v = 0.0;
    bool   b = false;
    if (type == "preview") {
        std::lock_guard<std::mutex> lock(mutex);
        const auto                  it = clients.find(id);
        if (it == clients.end()) {
            return;
        }
        ClientState& c = it->second;
        if (jsonNumber(text, "hz", v)) {
            c.sub.preview_hz = std::min(30.0, std::max(0.0, v));
        }
        if (jsonNumber(text, "quality", v)) {
            c.quality = static_cast<long>(std::min(100.0, std::max(1.0, v)));
        }
        if (jsonNumber(text, "max_width", v)) {
            c.max_width = static_cast<long>(std::min(1920.0, std::max(64.0, v)));
        }
        c.next_preview = Clock::now();
    } else if (type == "subscribe") {
        std::vector<std::string> clear;
        {
            std::lock_guard<std::mutex> lock(mutex);
            const auto                  it = clients.find(id);
            if (it == clients.end()) {
                return;
            }
            ClientState& c = it->second;
            if (jsonNumber(text, "state_hz", v)) {
                c.sub.state_hz = std::min(kMaxStateHz, std::max(0.0, v));
                c.next_state   = Clock::now();
                if (c.sub.state_hz <= 0.0) {
                    clear.push_back(kStateChannel.name);
                }
            }
            if (jsonBool(text, "diag", b)) {
                if (b && !c.sub.diag) {
                    c.diag_now = true;   // back from hidden: send one now
                }
                c.sub.diag = b;
                if (!b) {
                    clear.push_back(kDiagChannel.name);
                }
            }
            if (jsonBool(text, "instrumentation", b)) {
                c.sub.instrumentation = b;
                c.instrumentation_now  = b;
                if (!b) {
                    clear.push_back(kInstrumentationChannel.name);
                }
            }
            if (jsonBool(text, "raw", b)) {
                c.sub.raw = b;
            }
            if (jsonBool(text, "decoded", b)) {
                c.sub.decoded = b;
            }
            if (jsonBool(text, "telemetry", b)) {
                c.sub.telemetry = b;
                if (!b) {
                    clear.push_back(kTelemetryChannel.name);
                }
            }
        }
        for (const std::string& name : clear) {
            server->clearLatest(id, name);
        }
    } else if (type == "history") {
        std::lock_guard<std::mutex> lock(mutex);
        const auto                  it = clients.find(id);
        if (it == clients.end()) {
            return;
        }
        it->second.history_requested = true;
    } else {
        return;   // unknown types are ignored
    }
    wake();
}

void InspectionService::Impl::onClose(HttpServer::ClientId id) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        clients.erase(id);
        ++stats.client_disconnects;
    }
    wake();
}

void InspectionService::Impl::applyLinkMonitors(bool raw, bool decoded) {
    // edge-triggered, so a capture or another tool switching them is not
    // fought every tick
    LinkMonitorRegistry& links = system->diagHub().links();
    if (raw != links_raw) {
        links.setRaw(raw);
        links_raw = raw;
    }
    if (decoded != links_decoded) {
        links.setDecoded(decoded);
        links_decoded = decoded;
    }
}

void InspectionService::Impl::rehello(Clock::time_point now) {
    const uint64_t                    reset = system->resetCount();
    std::vector<HttpServer::ClientId> ids;
    uint64_t                          h_seq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            ClientState& c = kv.second;
            if (c.hello_reset == reset) {
                continue;
            }
            ids.push_back(kv.first);
            c.hello_reset = reset;
            c.have_state  = false;
            c.next_state  = now;
            c.diag_now    = true;
            c.received.clear();   // previews of the old session are not resent, new ones go
        }
        if (ids.empty()) {
            return;
        }
        h_seq = ++seq_history;
    }
    const MonotonicTime host_now = HostClock::now();
    const int64_t       t0       = HostClock::nowUs();
    const std::string   history  = historyDocument(*system, h_seq, host_now);
    const double        h_us     = usSince(t0);
    const auto          hframe   = HttpServer::textFrame(history);
    std::vector<std::pair<HttpServer::Enqueue, HttpServer::Enqueue>> results;
    for (HttpServer::ClientId id : ids) {
        // unsent state of the old session must not follow the new hello
        server->clearLatest(id, "");
        // two statements: argument evaluation order is unspecified
        const auto r_hello = server->sendReliable(
            id, HttpServer::textFrame(helloDocument(*system, host_now, id)), "hello");
        const auto r_hist  = server->sendReliable(id, hframe, "history");
        results.emplace_back(r_hello, r_hist);
    }
    std::lock_guard<std::mutex> lock(mutex);
    feed.history.add(h_us, history.size());
    for (const auto& r : results) {
        countEnqueue(r.first, stats.hellos_queued, nullptr);
        countEnqueue(r.second, stats.histories_queued, nullptr);
    }
}

void InspectionService::Impl::pushEvents(Clock::time_point now) {
    if (now < next_event_check) {
        return;
    }
    next_event_check = now + kEventPoll;
    const std::vector<RuntimeEvent> events = system->events();
    std::vector<HttpServer::Frame>  frames;
    for (const RuntimeEvent& e : events) {
        if (e.sequence > event_cursor) {
            frames.push_back(HttpServer::textFrame(eventDocument(e)));
            event_cursor = e.sequence;
        }
    }
    if (frames.empty()) {
        return;
    }
    const std::vector<HttpServer::ClientId> ids = server->clients();
    uint64_t                                queued = 0;
    for (HttpServer::ClientId id : ids) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (clients.count(id) == 0) {
                continue;   // not registered yet: its first diag carries the events
            }
        }
        for (const auto& f : frames) {
            countEnqueue(server->sendReliable(id, f, "event"), queued, nullptr);
        }
    }
    std::lock_guard<std::mutex> lock(mutex);
    stats.events_queued += queued;
}

void InspectionService::Impl::pushCapture() {
    const CaptureService* capture = system->capture();
    if (capture == nullptr) {
        return;
    }
    const uint64_t                    version = capture->version();
    std::vector<HttpServer::ClientId> ids;
    uint64_t                          seq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            if (kv.second.capture_version != version) {
                kv.second.capture_version = version;
                ids.push_back(kv.first);
            }
        }
        if (ids.empty()) {
            return;
        }
        seq = ++seq_capture;
    }
    const auto frame = HttpServer::textFrame(captureDocument(*capture, seq, HostClock::now()));
    uint64_t   queued = 0;
    for (HttpServer::ClientId id : ids) {
        countEnqueue(server->sendReliable(id, frame, "capture"), queued, nullptr);
    }
    std::lock_guard<std::mutex> lock(mutex);
    stats.captures_queued += queued;
}

void InspectionService::Impl::pushHistories() {
    std::vector<HttpServer::ClientId> ids;
    uint64_t                          seq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            if (kv.second.history_requested) {
                kv.second.history_requested = false;
                ids.push_back(kv.first);
            }
        }
        if (ids.empty()) {
            return;
        }
        seq = ++seq_history;
    }
    const int64_t     t0      = HostClock::nowUs();
    const std::string history = historyDocument(*system, seq, HostClock::now());
    const double      us      = usSince(t0);
    const auto        frame   = HttpServer::textFrame(history);
    uint64_t          queued  = 0;
    for (HttpServer::ClientId id : ids) {
        countEnqueue(server->sendReliable(id, frame, "history"), queued, nullptr);
    }
    std::lock_guard<std::mutex> lock(mutex);
    feed.history.add(us, history.size());
    stats.histories_queued += queued;
}

void InspectionService::Impl::pushStates(Clock::time_point now) {
    const uint64_t                    publication = system->robotFeed()->publication();
    std::vector<HttpServer::ClientId> due;
    uint64_t                          seq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            const ClientState& c = kv.second;
            if (c.sub.state_hz <= 0.0 || now < c.next_state) {
                continue;
            }
            if (!c.have_state || c.state_publication != publication ||
                now - c.last_state >= kStateKeepalive) {
                due.push_back(kv.first);
            }
        }
        if (due.empty()) {
            return;
        }
        seq = ++seq_state;
    }
    uint64_t          doc_publication = 0;
    const int64_t     t0              = HostClock::nowUs();
    const std::string doc = stateDocument(*system, seq, HostClock::now(), t0, &doc_publication);
    const double      us  = usSince(t0);
    if (doc.empty()) {
        return;   // resets kept landing: the clients stay due
    }
    const auto        frame = HttpServer::textFrame(doc);
    std::vector<std::pair<HttpServer::ClientId, HttpServer::Enqueue>> results;
    for (HttpServer::ClientId id : due) {
        results.emplace_back(id, server->sendLatest(id, kStateChannel, frame));
    }
    std::lock_guard<std::mutex> lock(mutex);
    feed.state.add(us, doc.size());
    for (const auto& r : results) {
        countEnqueue(r.second, stats.states_queued, &stats.states_replaced);
        const auto it = clients.find(r.first);
        if (it == clients.end()) {
            continue;
        }
        ClientState&                  c      = it->second;
        const Clock::duration         period = periodOf(c.sub.state_hz);
        c.have_state                         = true;
        c.state_publication                  = doc_publication;
        c.last_state                         = now;
        // keep the average rate without bursting after a pause
        c.next_state = std::max(c.next_state + period, now + period / 2);
        ++window_states;
    }
    stats.snapshots_sent    = stats.states_queued;
    stats.snapshots_skipped = stats.states_replaced;
}

void InspectionService::Impl::pushTelemetry(Clock::time_point now) {
    DiagRecord record;
    uint64_t   hub_seq = 0;
    if (!system->diagHub().latest(DiagKind::kBrainTelemetry, record, &hub_seq)) {
        return;
    }
    telemetry_seen = true;
    std::vector<HttpServer::ClientId> due;
    uint64_t                          seq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            ClientState& c = kv.second;
            if (!c.sub.telemetry || c.telemetry_seq == hub_seq || now < c.next_telemetry) {
                continue;
            }
            c.telemetry_seq  = hub_seq;
            c.next_telemetry = now + kTelemetryPeriod;
            due.push_back(kv.first);
        }
        if (due.empty()) {
            return;
        }
        seq = ++seq_telemetry;
    }
    const auto* body = std::get_if<DiagBrainTelemetry>(&record.payload);
    translagatr::BrainTelemetry t;
    if (body == nullptr || !decodeTelemetryRecord(*body, t)) {
        return;   // not a body this codec reads: nothing to show
    }
    const auto frame =
        HttpServer::textFrame(telemetryDocument(seq, HostClock::now(), record, *body, t));
    uint64_t queued = 0;
    for (HttpServer::ClientId id : due) {
        countEnqueue(server->sendLatest(id, kTelemetryChannel, frame), queued, nullptr);
    }
    std::lock_guard<std::mutex> lock(mutex);
    stats.telemetry_queued += queued;
}

void InspectionService::Impl::pushDiags(Clock::time_point now) {
    const bool scheduled = now >= next_diag;
    if (scheduled) {
        const Clock::duration period = periodOf(config.diag_hz);
        next_diag = std::max(next_diag + period, now + period / 2);
    }
    std::vector<HttpServer::ClientId> due;
    uint64_t                          seq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& kv : clients) {
            const ClientState& c = kv.second;
            if (c.sub.diag && (c.diag_now || scheduled)) {
                due.push_back(kv.first);
            }
        }
        if (due.empty()) {
            return;
        }
        seq = ++seq_diag;
    }
    const InspectionServiceStats service = composedStats();
    const InspectionFeedStats    f       = composedFeed();
    uint64_t          hash  = 0;
    const int64_t     t0    = HostClock::nowUs();
    const std::string doc   = diagDocument(*system, service, f, seq, HostClock::now(), &hash);
    const double      us    = usSince(t0);
    if (doc.empty()) {
        return;   // resets kept landing; the next scheduled diag goes
    }
    HttpServer::Frame frame;
    std::vector<std::pair<HttpServer::ClientId, HttpServer::Enqueue>> results;
    std::vector<HttpServer::ClientId>                                 send;
    {
        std::lock_guard<std::mutex> lock(mutex);
        feed.diag.add(us, doc.size());
        for (HttpServer::ClientId id : due) {
            const auto it = clients.find(id);
            if (it == clients.end()) {
                continue;
            }
            ClientState& c = it->second;
            if (!c.diag_now && c.have_diag && c.diag_hash == hash &&
                now - c.last_diag < kDiagRefresh) {
                ++c.diag_skipped;
                ++stats.diags_skipped;
                continue;
            }
            c.diag_now  = false;
            c.have_diag = true;
            c.diag_hash = hash;
            c.last_diag = now;
            send.push_back(id);
        }
    }
    if (send.empty()) {
        return;
    }
    frame = HttpServer::textFrame(doc);
    for (HttpServer::ClientId id : send) {
        results.emplace_back(id, server->sendLatest(id, kDiagChannel, frame));
    }
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto& r : results) {
        countEnqueue(r.second, stats.diags_queued, &stats.diags_replaced);
    }
    ++window_diags;
}

void InspectionService::Impl::pushInstrumentation(Clock::time_point now) {
    const bool scheduled = now >= next_instrumentation;
    if (scheduled) {
        next_instrumentation = std::max(next_instrumentation + kInstrumentationPeriod,
                                        now + kInstrumentationPeriod / 2);
    }
    bool                                  any_raw = false, any_decoded = false;
    std::vector<std::pair<HttpServer::ClientId, bool>> due;   // id, raw
    uint64_t                              seq = 0;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            ClientState& c = kv.second;
            if (!c.sub.instrumentation) {
                continue;
            }
            any_raw     = any_raw || c.sub.raw;
            any_decoded = any_decoded || c.sub.decoded;
            if (scheduled || c.instrumentation_now) {
                c.instrumentation_now = false;
                due.emplace_back(kv.first, c.sub.raw);
            }
        }
        if (!due.empty()) {
            seq = ++seq_instrumentation;
        }
    }
    applyLinkMonitors(any_raw, any_decoded);
    if (due.empty()) {
        return;
    }
    // at most two variants: with and without raw bytes
    HttpServer::Frame frames[2];
    uint64_t          queued = 0;
    for (const auto& d : due) {
        HttpServer::Frame& f = frames[d.second ? 1 : 0];
        if (f == nullptr) {
            InstrumentationOptions options;
            options.raw           = d.second;
            const int64_t     t0  = HostClock::nowUs();
            const std::string doc =
                instrumentationDocument(*system, seq, HostClock::now(), options);
            const double us = usSince(t0);
            f               = HttpServer::textFrame(doc);
            std::lock_guard<std::mutex> lock(mutex);
            feed.instrumentation.add(us, doc.size());
        }
        countEnqueue(server->sendLatest(d.first, kInstrumentationChannel, f), queued, nullptr);
    }
    std::lock_guard<std::mutex> lock(mutex);
    stats.instrumentation_queued += queued;
}

void InspectionService::Impl::publishPreviews(Clock::time_point now) {
    struct Due {
        HttpServer::ClientId                 id;
        long                                 quality;
        long                                 max_width;
        std::map<std::string, FrameIdentity> received;
    };
    std::vector<Due> due;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            ClientState& c = kv.second;
            if (c.sub.preview_hz <= 0.0 || now < c.next_preview) {
                continue;
            }
            c.next_preview = now + periodOf(c.sub.preview_hz);
            due.push_back(Due{kv.first, c.quality, c.max_width, c.received});
        }
    }
    if (due.empty()) {
        return;
    }
    const auto frames = system->detectionFrames();
    if (frames.empty()) {
        return;
    }
    for (const Due& d : due) {
        for (const auto& kv : frames) {
            const DetectionFrameSnapshot& frame = *kv.second;
            if (frame.y8 == nullptr || frame.y8->empty()) {
                continue;
            }
            FrameIdentity ident;
            ident.epoch    = frame.frame_epoch;
            ident.sequence = frame.frame_sequence;
            ident.set      = true;
            const auto seen = d.received.find(frame.camera.value);
            if (seen != d.received.end() && seen->second == ident) {
                continue;
            }
            CachedPreview p;
            std::string   err;
            if (!preview(frame, d.max_width, d.quality, p, err)) {
                continue;
            }
            const WsChannel channel{"preview:" + frame.camera.value, kPreviewPriority};
            const HttpServer::Enqueue r = server->sendLatest(d.id, channel, p.message);
            std::lock_guard<std::mutex> lock(mutex);
            if (r == HttpServer::Enqueue::kRefused || r == HttpServer::Enqueue::kUnknownClient) {
                ++stats.frames_skipped;   // retried at the next due tick
                continue;
            }
            if (r == HttpServer::Enqueue::kReplaced) {
                ++stats.frames_skipped;   // the older one never went
            }
            ++stats.frames_sent;
            ++window_frames;
            const auto it = clients.find(d.id);
            if (it != clients.end()) {
                it->second.received[frame.camera.value] = ident;
            }
        }
    }
}

// The earliest time something may be due, so the thread sleeps between.
Clock::time_point InspectionService::Impl::nextDeadline(Clock::time_point now) const {
    std::lock_guard<std::mutex> lock(mutex);
    if (clients.empty()) {
        return now + kIdleWait;
    }
    Clock::time_point next = std::min(next_event_check, now + kIdleWait);
    for (const auto& kv : clients) {
        const ClientState& c = kv.second;
        if (c.sub.state_hz > 0.0) {
            // once allowed, poll for the next publication
            next = std::min(next, std::max(c.next_state, now + kPublicationPoll));
        }
        if (c.sub.diag) {
            next = std::min(next, c.diag_now ? now : next_diag);
        }
        if (c.sub.telemetry && telemetry_seen.load()) {
            next = std::min(next, std::max(c.next_telemetry, now + kTelemetryPeriod / 2));
        }
        if (c.sub.instrumentation) {
            next = std::min(next, c.instrumentation_now ? now : next_instrumentation);
        }
        if (c.sub.preview_hz > 0.0) {
            next = std::min(next, c.next_preview);
        }
        if (c.history_requested) {
            next = now;
        }
    }
    return next;
}

Clock::time_point InspectionService::Impl::tick(Clock::time_point now) {
    rehello(now);
    pushEvents(now);
    pushCapture();
    pushHistories();
    pushStates(now);
    pushTelemetry(now);
    pushDiags(now);
    pushInstrumentation(now);
    publishPreviews(now);

    const double window_s = std::chrono::duration<double>(now - window_start).count();
    if (window_s >= 1.0) {
        std::lock_guard<std::mutex> lock(mutex);
        stats.snapshot_rate_hz = window_states / window_s;
        stats.frame_rate_hz    = window_frames / window_s;
        stats.diag_rate_hz     = window_diags / window_s;
        window_start           = now;
        window_states          = 0;
        window_frames          = 0;
        window_diags           = 0;
    }
    return nextDeadline(Clock::now());
}

void InspectionService::Impl::run() {
    window_start     = Clock::now();
    next_event_check = window_start;
    window_states = window_frames = window_diags = 0;
    // events before the service started reach clients in diag.events
    for (const RuntimeEvent& e : system->events()) {
        event_cursor = std::max(event_cursor, e.sequence);
    }
    Clock::time_point deadline = window_start;
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(wake_mutex);
            wake_cv.wait_until(lock, deadline, [&] { return stop_requested || wake_pending; });
            if (stop_requested) {
                break;
            }
            wake_pending = false;
        }
        deadline = tick(Clock::now());
    }
    applyLinkMonitors(false, false);
    std::lock_guard<std::mutex> lock(mutex);
    stats.snapshot_rate_hz = 0.0;
    stats.frame_rate_hz    = 0.0;
    stats.diag_rate_hz     = 0.0;
}

// ---- public surface -------------------------------------------------------------

InspectionService::~InspectionService() {
    if (impl_ != nullptr) {
        stop();
    }
}

std::unique_ptr<InspectionService> InspectionService::create(System& system,
                                                             const InspectionConfig& config,
                                                             std::string& err) {
    if (!config.static_root.empty()) {
        std::error_code ec;
        if (!std::filesystem::is_directory(config.static_root, ec)) {
            err = "inspection static_root is not a readable directory: " + config.static_root;
            return nullptr;
        }
    }
    std::unique_ptr<InspectionService> s(new InspectionService());
    s->impl_.reset(new Impl());
    Impl& impl  = *s->impl_;
    impl.system = &system;
    impl.config = config;

    std::size_t          count  = 0;
    const EmbeddedAsset* assets = embeddedViewerAssets(count);
    for (std::size_t i = 0; i < count; ++i) {
        impl.embedded[assets[i].path] = Impl::Asset{assets[i].content_type, assets[i].data,
                                                    assets[i].size};
    }

    HttpServerConfig hc;
    hc.bind                   = config.bind;
    hc.port                   = static_cast<int>(config.port);
    hc.max_clients            = static_cast<int>(config.max_clients);
    hc.client_buffer_bytes    = static_cast<std::size_t>(config.client_buffer_kb) * 1024;
    hc.reliable_bytes         = static_cast<std::size_t>(config.reliable_kb) * 1024;
    hc.close_after_stalled_ms = static_cast<int>(config.stall_close_ms);
    hc.send_buffer_bytes      = static_cast<int>(config.send_buffer_kb * 1024);
    hc.ack_window_bytes       = static_cast<std::size_t>(config.ack_window_kb) * 1024;
    hc.websocket_path         = "/ws";
    Impl* raw                 = &impl;
    impl.server.reset(new HttpServer(hc, [raw](const HttpRequest& r) { return raw->handle(r); }));
    HttpServer::WebSocketHandlers ws;
    ws.onOpen  = [raw](HttpServer::ClientId id) { raw->onOpen(id); };
    ws.onText  = [raw](HttpServer::ClientId id, const std::string& t) { raw->onText(id, t); };
    ws.onClose = [raw](HttpServer::ClientId id) { raw->onClose(id); };
    impl.server->setWebSocketHandlers(std::move(ws));
    return s;
}

bool InspectionService::start(std::string& err) {
    if (impl_->running.load()) {
        return true;
    }
    if (!impl_->server->start(err)) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(impl_->wake_mutex);
        impl_->stop_requested = false;
    }
    impl_->running = true;
    impl_->thread  = std::thread([this] { impl_->run(); });
    return true;
}

void InspectionService::stop() {
    {
        std::lock_guard<std::mutex> lock(impl_->wake_mutex);
        impl_->stop_requested = true;
    }
    impl_->wake_cv.notify_all();
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
    impl_->running = false;
    impl_->server->stop();   // closes clients; their onClose still lands on the server thread
}

bool InspectionService::running() const { return impl_->running.load(); }
int  InspectionService::port() const { return impl_->server->port(); }

std::string InspectionService::url() const {
    return "http://" + impl_->config.bind + ":" + std::to_string(port()) + "/";
}

InspectionServiceStats InspectionService::stats() const { return impl_->composedStats(); }

} // namespace navigatr
