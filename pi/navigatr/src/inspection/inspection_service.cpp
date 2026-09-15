// inspection_service.cpp
// The service thread ticks at snapshot_hz and, per client, at that
// client's preview rate: it builds documents from the System's published
// snapshots, encodes previews once per (frame identity, budget) and hands
// them to the HTTP server, which owns the sockets. HTTP requests and the
// WebSocket callbacks run on the server thread; the client table and the
// counters are the only shared state and sit under one mutex, the encode
// cache under another. Nothing here touches a worker's mutable maps.

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

#include "core/host_clock.h"
#include "inspection/http_server.h"
#include "inspection/inspection_document.h"
#include "inspection/jpeg_encoder.h"
#include "inspection/viewer_assets.h"

namespace navigatr
{

namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::size_t kCacheEntries = 16;

// A flat scan for one key in a small JSON object: enough for the preview
// message, which a browser writes and nothing else parses.
bool jsonToken(const std::string& json, const char* key, std::string& token) {
    const std::string quoted = std::string("\"") + key + "\"";
    std::size_t       pos    = json.find(quoted);
    if (pos == std::string::npos) {
        return false;
    }
    pos += quoted.size();
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
        return true;
    }
    std::size_t end = pos;
    while (end < json.size() && json[end] != ',' && json[end] != '}' && json[end] != ']' &&
           !std::isspace(static_cast<unsigned char>(json[end]))) {
        ++end;
    }
    token = json.substr(pos, end - pos);
    return !token.empty();
}

bool jsonNumber(const std::string& json, const char* key, double& out) {
    std::string token;
    if (!jsonToken(json, key, token)) {
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
        double            hz        = 0.0;
        long              quality   = 70;
        long              max_width = 640;
        Clock::time_point next_preview;
        std::map<std::string, FrameIdentity> received;   // per camera
    };

    struct CachedPreview {
        std::string camera;
        uint64_t    epoch     = 0;
        uint32_t    sequence  = 0;
        long        max_width = 0;
        long        quality   = 0;
        int         width_px  = 0;
        int         height_px = 0;
        std::shared_ptr<const std::vector<uint8_t>> message;   // [u32 LE len][header][jpeg]
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
    std::atomic<bool>       running{false};

    mutable std::mutex                          mutex;   // clients and stats
    std::map<HttpServer::ClientId, ClientState> clients;
    InspectionServiceStats                      stats;

    std::mutex                 cache_mutex;
    std::vector<CachedPreview> cache;

    // rate windows, service thread only
    Clock::time_point window_start;
    uint64_t          window_snapshots = 0;
    uint64_t          window_frames    = 0;

    InspectionServiceStats composedStats() const;
    HttpResponse           handle(const HttpRequest& req);
    HttpResponse           serveStatic(const std::string& path);
    void                   onOpen(HttpServer::ClientId id);
    void                   onText(HttpServer::ClientId id, const std::string& text);
    void                   onClose(HttpServer::ClientId id);
    bool preview(const DetectionFrameSnapshot& frame, long max_width, long quality,
                 CachedPreview& out, std::string& err);
    void run();
    void publishSnapshot();
    void publishPreviews(Clock::time_point now);
    Clock::time_point earliestPreviewDeadline(Clock::time_point fallback) const;
};

InspectionServiceStats InspectionService::Impl::composedStats() const {
    InspectionServiceStats s;
    {
        std::lock_guard<std::mutex> lock(mutex);
        s = stats;
    }
    const HttpServerStats hs = server->stats();
    s.running    = running.load();
    s.port       = server->port();
    s.clients    = hs.websocket_clients;
    s.bytes_sent = hs.bytes_sent;
    if (!s.running) {
        s.snapshot_rate_hz = 0.0;
        s.frame_rate_hz    = 0.0;
    }
    return s;
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
                                                   jpeg.encode_ms, HostClock::now());
    auto message = std::make_shared<std::vector<uint8_t>>();
    message->reserve(4 + header.size() + jpeg.bytes.size());
    const uint32_t len = static_cast<uint32_t>(header.size());
    message->push_back(static_cast<uint8_t>(len & 0xff));
    message->push_back(static_cast<uint8_t>((len >> 8) & 0xff));
    message->push_back(static_cast<uint8_t>((len >> 16) & 0xff));
    message->push_back(static_cast<uint8_t>((len >> 24) & 0xff));
    message->insert(message->end(), header.begin(), header.end());
    message->insert(message->end(), jpeg.bytes.begin(), jpeg.bytes.end());

    CachedPreview entry;
    entry.camera    = frame.camera.value;
    entry.epoch     = frame.frame_epoch;
    entry.sequence  = frame.frame_sequence;
    entry.max_width = max_width;
    entry.quality   = quality;
    entry.width_px  = jpeg.width_px;
    entry.height_px = jpeg.height_px;
    entry.message   = std::move(message);
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
        body += ",\"port\":" + std::to_string(server->port()) + "}";
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

void InspectionService::Impl::onOpen(HttpServer::ClientId id) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        ClientState&                c = clients[id];
        c.hz                          = config.preview_hz;
        c.quality                     = config.preview_quality;
        c.max_width                   = config.preview_max_width;
        c.next_preview                = Clock::now();
        ++stats.clients_total;
    }
    server->sendText(id, helloDocument(*system, HostClock::now()));
}

void InspectionService::Impl::onText(HttpServer::ClientId id, const std::string& text) {
    std::string type;
    if (!jsonToken(text, "type", type) || type != "preview") {
        return;
    }
    double                      v = 0.0;
    std::lock_guard<std::mutex> lock(mutex);
    const auto                  it = clients.find(id);
    if (it == clients.end()) {
        return;
    }
    ClientState& c = it->second;
    if (jsonNumber(text, "hz", v)) {
        c.hz = std::min(30.0, std::max(0.0, v));
    }
    if (jsonNumber(text, "quality", v)) {
        c.quality = static_cast<long>(std::min(100.0, std::max(1.0, v)));
    }
    if (jsonNumber(text, "max_width", v)) {
        c.max_width = static_cast<long>(std::min(1920.0, std::max(64.0, v)));
    }
    c.next_preview = Clock::now();
}

void InspectionService::Impl::onClose(HttpServer::ClientId id) {
    std::lock_guard<std::mutex> lock(mutex);
    clients.erase(id);
    ++stats.client_disconnects;
}

void InspectionService::Impl::publishSnapshot() {
    if (server->clients().empty()) {
        return;   // nothing to serialize for
    }
    const std::string doc = snapshotDocument(*system, composedStats(), HostClock::now());
    std::size_t       refused  = 0;
    const std::size_t accepted = server->broadcastText(doc, refused);
    ++window_snapshots;
    std::lock_guard<std::mutex> lock(mutex);
    stats.snapshots_sent += accepted;
    stats.snapshots_skipped += refused;
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
            if (c.hz <= 0.0 || now < c.next_preview) {
                continue;
            }
            c.next_preview = now + std::chrono::duration_cast<Clock::duration>(
                                       std::chrono::duration<double>(1.0 / c.hz));
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
            const bool                  sent = server->sendBinary(d.id, *p.message);
            std::lock_guard<std::mutex> lock(mutex);
            if (!sent) {
                ++stats.frames_skipped;   // retried at the next due tick
                continue;
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

Clock::time_point InspectionService::Impl::earliestPreviewDeadline(
    Clock::time_point fallback) const {
    std::lock_guard<std::mutex> lock(mutex);
    Clock::time_point           earliest = fallback;
    for (const auto& kv : clients) {
        if (kv.second.hz > 0.0 && kv.second.next_preview < earliest) {
            earliest = kv.second.next_preview;
        }
    }
    return earliest;
}

void InspectionService::Impl::run() {
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.snapshot_hz));
    Clock::time_point next_snapshot = Clock::now();
    window_start                    = next_snapshot;
    window_snapshots                = 0;
    window_frames                   = 0;
    for (;;) {
        {
            const Clock::time_point      deadline = earliestPreviewDeadline(next_snapshot);
            std::unique_lock<std::mutex> lock(wake_mutex);
            wake_cv.wait_until(lock, deadline, [&] { return stop_requested; });
            if (stop_requested) {
                break;
            }
        }
        const Clock::time_point now = Clock::now();
        if (now >= next_snapshot) {
            publishSnapshot();
            next_snapshot += period;
            if (next_snapshot < now) {
                next_snapshot = now + period;   // fell behind: no catch-up burst
            }
        }
        publishPreviews(now);

        const double window_s = std::chrono::duration<double>(now - window_start).count();
        if (window_s >= 1.0) {
            std::lock_guard<std::mutex> lock(mutex);
            stats.snapshot_rate_hz = window_snapshots / window_s;
            stats.frame_rate_hz    = window_frames / window_s;
            window_start           = now;
            window_snapshots       = 0;
            window_frames          = 0;
        }
    }
    std::lock_guard<std::mutex> lock(mutex);
    stats.snapshot_rate_hz = 0.0;
    stats.frame_rate_hz    = 0.0;
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
    hc.close_after_stalled_ms = static_cast<int>(config.stall_close_ms);
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
