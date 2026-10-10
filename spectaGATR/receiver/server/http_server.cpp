// http_server.cpp
// One service thread owns every socket: it accepts, reads, parses and writes under a short
// select() timeout so stop() is prompt. Handlers run on that thread but outside the client lock,
// because a handler may call back into the send functions; the lock guards the client table, the
// queues and the counters. Sockets are non-blocking: an enqueue tries at most a bounded
// non-blocking write, and what the kernel refuses waits in the bounded per-client queue.
// Provenance and the SHA-1/base64 note: http_server.h.

#include "receiver/server/http_server.h"

#if defined(_WIN32)
#ifndef FD_SETSIZE
#define FD_SETSIZE 128 // listen socket plus every client
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <climits>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>

namespace spectagatr::receiver::server
{

namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::size_t kMaxRequestHead     = 16 * 1024;
constexpr std::size_t kMaxClientMessage   = 4 * 1024; // the feed reads nothing from browsers
constexpr int         kSelectTimeoutMs    = 20;
constexpr std::size_t kSendChunk          = 64 * 1024;
constexpr std::size_t kFlushBudget        = 256 * 1024; // bytes per flush call, bounds lock time
constexpr int         kCloseGraceMs       = 1000;       // a close frame gets this long to leave
constexpr std::size_t kMaxHttpConnections = 32; // plain requests at once, besides feed clients

// ---- platform shim -------------------------------------------------------

#if defined(_WIN32)
using SocketHandle                    = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
constexpr int          kSendFlags     = 0;

void closeSocket(SocketHandle s) { closesocket(s); }
void shutdownSend(SocketHandle s) { shutdown(s, SD_SEND); }
bool wouldBlock() {
    const int e = WSAGetLastError();
    return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS;
}
bool prepareSocket(SocketHandle s) {
    u_long on = 1;
    return ioctlsocket(s, FIONBIO, &on) == 0;
}
bool        selectable(SocketHandle) { return true; }
std::string socketError() { return "winsock error " + std::to_string(WSAGetLastError()); }

std::mutex g_wsa_mutex;
int        g_wsa_refs = 0;

bool wsaAcquire(std::string& err) {
    const std::lock_guard<std::mutex> lock(g_wsa_mutex);
    if (g_wsa_refs == 0) {
        WSADATA   data;
        const int r = WSAStartup(MAKEWORD(2, 2), &data);
        if (r != 0) {
            err = "WSAStartup failed: " + std::to_string(r);
            return false;
        }
    }
    ++g_wsa_refs;
    return true;
}
void wsaRelease() {
    const std::lock_guard<std::mutex> lock(g_wsa_mutex);
    if (g_wsa_refs > 0 && --g_wsa_refs == 0) {
        WSACleanup();
    }
}
#else
using SocketHandle                    = int;
constexpr SocketHandle kInvalidSocket = -1;
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

void closeSocket(SocketHandle s) { ::close(s); }
void shutdownSend(SocketHandle s) { ::shutdown(s, SHUT_WR); }
bool wouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR; }
// Non-blocking, not inherited by a browser the receiver starts, no SIGPIPE on macOS.
bool prepareSocket(SocketHandle s) {
    const int flags = fcntl(s, F_GETFL, 0);
    if (flags < 0 || fcntl(s, F_SETFL, flags | O_NONBLOCK) != 0) {
        return false;
    }
    fcntl(s, F_SETFD, FD_CLOEXEC);
#ifdef SO_NOSIGPIPE
    const int one = 1;
    setsockopt(s, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
    return true;
}
bool        selectable(SocketHandle s) { return s < FD_SETSIZE; }
std::string socketError() { return std::strerror(errno); }
bool        wsaAcquire(std::string&) { return true; }
void        wsaRelease() {}
#endif

// ---- SHA-1 (FIPS 180-4) and base64 (RFC 4648), for Sec-WebSocket-Accept ----

uint32_t rotl(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }

void sha1(const std::string& in, uint8_t out[20]) {
    uint32_t             h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    std::vector<uint8_t> msg(in.begin(), in.end());
    msg.push_back(0x80);
    while (msg.size() % 64 != 56) {
        msg.push_back(0);
    }
    const uint64_t bits = static_cast<uint64_t>(in.size()) * 8;
    for (int i = 7; i >= 0; --i) {
        msg.push_back(static_cast<uint8_t>(bits >> (i * 8)));
    }
    for (std::size_t off = 0; off < msg.size(); off += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i) {
            const uint8_t* p = &msg[off + 4 * static_cast<std::size_t>(i)];
            w[i] = (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
                   (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
        }
        for (int i = 16; i < 80; ++i) {
            w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; ++i) {
            uint32_t f = 0, k = 0;
            if (i < 20) {
                f = (b & c) | (~b & d);
                k = 0x5A827999u;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1u;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDCu;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6u;
            }
            const uint32_t t = rotl(a, 5) + f + e + k + w[i];
            e                = d;
            d                = c;
            c                = rotl(b, 30);
            b                = a;
            a                = t;
        }
        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
    }
    for (std::size_t i = 0; i < 5; ++i) {
        out[4 * i]     = static_cast<uint8_t>(h[i] >> 24);
        out[4 * i + 1] = static_cast<uint8_t>(h[i] >> 16);
        out[4 * i + 2] = static_cast<uint8_t>(h[i] >> 8);
        out[4 * i + 3] = static_cast<uint8_t>(h[i]);
    }
}

std::string base64(const uint8_t* data, std::size_t len) {
    static const char* kAlphabet =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((len + 2) / 3 * 4);
    for (std::size_t i = 0; i < len; i += 3) {
        const uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
                           (i + 1 < len ? static_cast<uint32_t>(data[i + 1]) << 8 : 0) |
                           (i + 2 < len ? static_cast<uint32_t>(data[i + 2]) : 0);
        out.push_back(kAlphabet[(n >> 18) & 63]);
        out.push_back(kAlphabet[(n >> 12) & 63]);
        out.push_back(i + 1 < len ? kAlphabet[(n >> 6) & 63] : '=');
        out.push_back(i + 2 < len ? kAlphabet[n & 63] : '=');
    }
    return out;
}

// ---- text helpers ----------------------------------------------------------

std::string lower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

std::string trim(const std::string& s) {
    std::size_t b = 0, e = s.size();
    while (b < e && (s[b] == ' ' || s[b] == '\t')) {
        ++b;
    }
    while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r')) {
        --e;
    }
    return s.substr(b, e - b);
}

const char* reasonPhrase(int status) {
    switch (status) {
    case 101:
        return "Switching Protocols";
    case 200:
        return "OK";
    case 400:
        return "Bad Request";
    case 403:
        return "Forbidden";
    case 404:
        return "Not Found";
    case 405:
        return "Method Not Allowed";
    case 500:
        return "Internal Server Error";
    case 503:
        return "Service Unavailable";
    default:
        return "Status";
    }
}

std::string serializeResponse(const HttpResponse& r) {
    std::string s = "HTTP/1.1 " + std::to_string(r.status) + " " + reasonPhrase(r.status) + "\r\n";
    s += "Content-Type: " + r.content_type + "\r\n";
    s += "Content-Length: " + std::to_string(r.body.size()) + "\r\n";
    s += "Connection: close\r\n";
    for (const auto& kv : r.headers) {
        s += kv.first + ": " + kv.second + "\r\n";
    }
    s += "\r\n";
    return s + r.body;
}

HttpResponse plainResponse(int status, const std::string& body) {
    HttpResponse r;
    r.status = status;
    r.body   = body;
    return r;
}

// Server frames are never masked and never fragmented.
std::string websocketFrame(uint8_t opcode, const char* data, std::size_t len) {
    std::string f;
    f.reserve(len + 10);
    f.push_back(static_cast<char>(0x80 | opcode));
    if (len < 126) {
        f.push_back(static_cast<char>(len));
    } else if (len < 65536) {
        f.push_back(static_cast<char>(126));
        f.push_back(static_cast<char>(len >> 8));
        f.push_back(static_cast<char>(len & 0xff));
    } else {
        f.push_back(static_cast<char>(127));
        for (int i = 7; i >= 0; --i) {
            f.push_back(static_cast<char>((static_cast<uint64_t>(len) >> (i * 8)) & 0xff));
        }
    }
    f.append(data, len);
    return f;
}

std::string closeFrame(uint16_t code, const char* reason) {
    std::string payload;
    payload.push_back(static_cast<char>(code >> 8));
    payload.push_back(static_cast<char>(code & 0xff));
    payload += reason;
    return websocketFrame(0x8, payload.data(), payload.size());
}

// True when a full request head was parsed; consumed is its length. An empty method means the
// request line was bad.
bool parseRequestHead(const std::string& in, HttpRequest& req, std::size_t& consumed) {
    const std::size_t end = in.find("\r\n\r\n");
    if (end == std::string::npos) {
        return false;
    }
    consumed                   = end + 4;
    const std::size_t line_end = in.find("\r\n");
    const std::string line     = in.substr(0, line_end);
    const std::size_t sp1      = line.find(' ');
    const std::size_t sp2 = sp1 == std::string::npos ? std::string::npos : line.find(' ', sp1 + 1);
    if (sp1 == std::string::npos || sp2 == std::string::npos) {
        req.method.clear();
        return true;
    }
    req.method               = line.substr(0, sp1);
    const std::string target = line.substr(sp1 + 1, sp2 - sp1 - 1);
    const std::size_t q      = target.find('?');
    req.path                 = urlDecode(q == std::string::npos ? target : target.substr(0, q));
    req.query                = q == std::string::npos ? std::string() : target.substr(q + 1);
    std::size_t pos          = line_end + 2;
    while (pos < end) {
        const std::size_t next  = in.find("\r\n", pos);
        const std::string hline = in.substr(pos, next - pos);
        const std::size_t colon = hline.find(':');
        if (colon != std::string::npos) {
            req.headers[lower(trim(hline.substr(0, colon)))] = trim(hline.substr(colon + 1));
        }
        pos = next + 2;
    }
    return true;
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

} // namespace

std::string urlDecode(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '%' && i + 2 < text.size() && hexValue(text[i + 1]) >= 0 &&
            hexValue(text[i + 2]) >= 0) {
            out.push_back(static_cast<char>(hexValue(text[i + 1]) * 16 + hexValue(text[i + 2])));
            i += 2;
        } else {
            out.push_back(text[i]);
        }
    }
    return out;
}

std::string websocketAccept(const std::string& key) {
    uint8_t digest[20];
    sha1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11", digest);
    return base64(digest, sizeof(digest));
}

// ---- the server ------------------------------------------------------------

struct HttpServer::Impl {
    enum class Mode { kHttp, kWebSocket };

    struct Slot {
        Frame frame; // null when empty
        int   priority = 0;
    };

    struct Client {
        SocketHandle sock = kInvalidSocket;
        ClientId     id   = 0;
        Mode         mode = Mode::kHttp;
        std::string  in;

        Frame       inflight; // the frame being written: never dropped once started
        std::size_t inflight_off = 0;

        std::deque<Frame>           reliable;
        std::size_t                 reliable_bytes = 0;
        std::map<std::string, Slot> slots; // by channel name
        std::size_t                 slot_bytes = 0;

        bool              awaiting_handler   = false; // request dispatched, response pending
        bool              close_after_flush  = false;
        bool              half_closed        = false; // our side done, waiting for the peer's EOF
        bool              dead               = false;
        bool              refused            = false; // over the limit: answers 503 and goes
        bool              has_close_deadline = false;
        Clock::time_point close_deadline;
        Clock::time_point last_progress;

        std::size_t inflightRest() const { return inflight ? inflight->size() - inflight_off : 0; }
        std::size_t queued() const { return inflightRest() + reliable_bytes + slot_bytes; }
        bool        webSocketOpen() const {
            return mode == Mode::kWebSocket && !dead && !close_after_flush;
        }
    };

    struct Event {
        enum Kind { kHttp, kOpen, kClose } kind;
        ClientId    id;
        HttpRequest request;
    };

    HttpServerConfig  config;
    HttpHandler       handler;
    WebSocketHandlers ws;

    SocketHandle      listen_sock = kInvalidSocket;
    std::thread       thread;
    std::atomic<bool> stop_flag{false};
    std::atomic<bool> running{false};
    std::atomic<int>  port{0};
    bool              wsa_held = false;

    mutable std::mutex         mutex; // clients, queues and stats
    std::map<ClientId, Client> clients;
    ClientId                   next_id = 1;
    HttpServerStats            stats;

    bool        listen(std::string& err);
    void        run();
    void        acceptPending(Clock::time_point now);
    void        readClient(Client& c, std::vector<Event>& events, Clock::time_point now);
    void        parseHttp(Client& c, std::vector<Event>& events, Clock::time_point now);
    void        parseWebSocket(Client& c, Clock::time_point now);
    void        kill(Client& c, const char* reason);
    bool        promote(Client& c);
    bool        flush(Client& c, Clock::time_point now);
    void        push(Client& c, Frame frame, Clock::time_point now);
    Enqueue     reliable(Client& c, const Frame& frame, Clock::time_point now);
    void        dropSlot(Client& c, Slot& slot);
    void        closeWith(Client& c, const std::string& frame, Clock::time_point now);
    void        queueHttp(Client& c, const HttpResponse& r, Clock::time_point now);
    void        reapDead(std::vector<Event>& events);
    void        dispatch(std::vector<Event>& events);
    std::size_t admitted() const;
    std::size_t feedClients() const;
    Client*     openWebSocket(ClientId id);
};

std::size_t HttpServer::Impl::admitted() const {
    std::size_t n = 0;
    for (const auto& kv : clients) {
        n += !kv.second.refused && !kv.second.dead ? 1 : 0;
    }
    return n;
}

std::size_t HttpServer::Impl::feedClients() const {
    std::size_t n = 0;
    for (const auto& kv : clients) {
        n += kv.second.mode == Mode::kWebSocket && !kv.second.dead ? 1 : 0;
    }
    return n;
}

HttpServer::Impl::Client* HttpServer::Impl::openWebSocket(ClientId id) {
    const auto it = clients.find(id);
    if (it == clients.end() || !it->second.webSocketOpen()) {
        return nullptr;
    }
    return &it->second;
}

bool HttpServer::Impl::listen(std::string& err) {
    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(config.port));
    if (inet_pton(AF_INET, config.bind.c_str(), &addr.sin_addr) != 1) {
        err = "bad bind address " + config.bind;
        return false;
    }
    listen_sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock == kInvalidSocket) {
        err = "socket: " + socketError();
        return false;
    }
    const int one = 1;
#if defined(_WIN32)
    setsockopt(listen_sock, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&one),
               sizeof(one));
#else
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one),
               sizeof(one));
#endif
    if (::bind(listen_sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        err = "bind " + config.bind + ":" + std::to_string(config.port) + ": " + socketError() +
              " (another receiver running? try --http-port)";
        return false;
    }
    if (::listen(listen_sock, 16) != 0) {
        err = "listen: " + socketError();
        return false;
    }
    if (!prepareSocket(listen_sock)) {
        err = "non-blocking listen socket: " + socketError();
        return false;
    }
    sockaddr_in bound;
    std::memset(&bound, 0, sizeof(bound));
#if defined(_WIN32)
    int len = sizeof(bound);
#else
    socklen_t len = sizeof(bound);
#endif
    if (getsockname(listen_sock, reinterpret_cast<sockaddr*>(&bound), &len) != 0) {
        err = "getsockname: " + socketError();
        return false;
    }
    port = ntohs(bound.sin_port);
    return true;
}

void HttpServer::Impl::acceptPending(Clock::time_point now) {
    while (true) {
        const SocketHandle s = ::accept(listen_sock, nullptr, nullptr);
        if (s == kInvalidSocket) {
            return; // nothing pending, or a transient error: retry next loop
        }
        if (!selectable(s) || !prepareSocket(s)) {
            closeSocket(s);
            continue;
        }
        const int one = 1;
        setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));

        const std::lock_guard<std::mutex> lock(mutex);
        Client                            c;
        c.sock          = s;
        c.id            = next_id++;
        c.last_progress = now;
        // Over the limit the request is still read, so the 503 reaches the client instead of a
        // reset, and the connection goes right after.
        c.refused = admitted() >= feedClients() + kMaxHttpConnections;
        if (c.refused) {
            ++stats.refused_connections;
        } else {
            ++stats.connections_total;
        }
        clients.emplace(c.id, std::move(c));
    }
}

// Caller holds mutex. The first reason wins.
void HttpServer::Impl::kill(Client& c, const char* reason) {
    if (c.dead) {
        return;
    }
    c.dead              = true;
    const std::string r = reason;
    if (r == "stalled") {
        ++stats.clients_closed_stalled;
    }
    if (c.mode != Mode::kWebSocket) {
        return;
    }
    if (r == "protocol") {
        ++stats.clients_closed_protocol;
    } else if (r == "peer") {
        ++stats.clients_closed_peer;
    }
}

// Next frame: the reliable head, else the full slot with the lowest priority number.
bool HttpServer::Impl::promote(Client& c) {
    if (c.inflight) {
        return true;
    }
    if (!c.reliable.empty()) {
        c.inflight = std::move(c.reliable.front());
        c.reliable.pop_front();
        c.reliable_bytes -= c.inflight->size();
        c.inflight_off = 0;
        return true;
    }
    Slot* best = nullptr;
    for (auto& kv : c.slots) {
        if (kv.second.frame && (best == nullptr || kv.second.priority < best->priority)) {
            best = &kv.second;
        }
    }
    if (best == nullptr) {
        return false;
    }
    c.inflight = std::move(best->frame);
    best->frame.reset();
    c.slot_bytes -= c.inflight->size();
    c.inflight_off = 0;
    return true;
}

// Writes what the socket takes, frame after frame, up to kFlushBudget. False on a socket error.
bool HttpServer::Impl::flush(Client& c, Clock::time_point now) {
    std::size_t budget = kFlushBudget;
    while (budget > 0 && promote(c)) {
        const std::string& f     = *c.inflight;
        const std::size_t  rest  = f.size() - c.inflight_off;
        const int          chunk = static_cast<int>(std::min({rest, kSendChunk, budget}));
        const auto         n     = ::send(c.sock, f.data() + c.inflight_off, chunk, kSendFlags);
        if (n > 0) {
            const auto sent = static_cast<std::size_t>(n);
            c.inflight_off += sent;
            stats.bytes_sent += sent;
            c.last_progress = now;
            budget -= std::min(budget, sent);
            if (c.inflight_off == f.size()) {
                c.inflight.reset();
                c.inflight_off = 0;
            }
            continue;
        }
        if (n < 0 && wouldBlock()) {
            break;
        }
        if (n == 0) {
            break;
        }
        return false;
    }
    return true;
}

// Caller holds mutex. Unbounded append: HTTP responses and control frames.
void HttpServer::Impl::push(Client& c, Frame frame, Clock::time_point now) {
    if (c.dead || frame == nullptr || frame->empty()) {
        return;
    }
    if (c.queued() == 0) {
        c.last_progress = now; // the stall clock starts when data waits
    }
    c.reliable_bytes += frame->size();
    c.reliable.push_back(std::move(frame));
    if (!flush(c, now)) {
        kill(c, "peer");
    }
}

void HttpServer::Impl::dropSlot(Client& c, Slot& slot) {
    if (slot.frame) {
        c.slot_bytes -= slot.frame->size();
        slot.frame.reset();
    }
}

// Unsent messages go, the close frame follows the in-flight one, and the socket goes after a
// short grace whether or not the client read it.
void HttpServer::Impl::closeWith(Client& c, const std::string& frame, Clock::time_point now) {
    c.reliable.clear();
    c.reliable_bytes = 0;
    for (auto& kv : c.slots) {
        dropSlot(c, kv.second);
    }
    c.close_after_flush = true;
    push(c, std::make_shared<const std::string>(frame), now);
    c.has_close_deadline = true;
    c.close_deadline     = now + std::chrono::milliseconds(kCloseGraceMs);
}

HttpServer::Enqueue HttpServer::Impl::reliable(Client& c, const Frame& frame,
                                               Clock::time_point now) {
    if (frame == nullptr) {
        return Enqueue::kRefused;
    }
    // The budget bounds the backlog, not one message.
    if (!c.reliable.empty() && c.reliable_bytes + frame->size() > config.reliable_bytes) {
        ++stats.clients_closed_overflow;
        closeWith(c, closeFrame(1013, "reliable backlog overflow"), now);
        return Enqueue::kClosed;
    }
    push(c, frame, now);
    return c.dead ? Enqueue::kClosed : Enqueue::kQueued;
}

void HttpServer::Impl::queueHttp(Client& c, const HttpResponse& r, Clock::time_point now) {
    push(c, std::make_shared<const std::string>(serializeResponse(r)), now);
    c.close_after_flush = true;
}

void HttpServer::Impl::parseHttp(Client& c, std::vector<Event>& events, Clock::time_point now) {
    if (c.awaiting_handler || c.close_after_flush) {
        return; // one request per connection
    }
    HttpRequest req;
    std::size_t consumed = 0;
    if (!parseRequestHead(c.in, req, consumed)) {
        if (c.in.size() > kMaxRequestHead) {
            kill(c, "protocol");
        }
        return;
    }
    c.in.erase(0, consumed);
    ++stats.requests;
    if (c.refused) {
        queueHttp(c, plainResponse(503, "too many clients\n"), now);
        return;
    }
    if (req.method.empty()) {
        queueHttp(c, plainResponse(400, "bad request\n"), now);
        return;
    }
    const auto upgrade = req.headers.find("upgrade");
    const auto key     = req.headers.find("sec-websocket-key");
    if (req.method == "GET" && req.path == config.websocket_path && upgrade != req.headers.end() &&
        lower(upgrade->second).find("websocket") != std::string::npos && key != req.headers.end()) {
        if (feedClients() >= static_cast<std::size_t>(std::max(1, config.max_clients))) {
            ++stats.refused_connections;
            queueHttp(c, plainResponse(503, "too many clients\n"), now);
            return;
        }
        std::string r = "HTTP/1.1 101 Switching Protocols\r\n"
                        "Upgrade: websocket\r\n"
                        "Connection: Upgrade\r\n"
                        "Sec-WebSocket-Accept: " +
                        websocketAccept(trim(key->second)) + "\r\n\r\n";
        c.mode        = Mode::kWebSocket;
        if (config.send_buffer_bytes > 0) {
            setsockopt(c.sock, SOL_SOCKET, SO_SNDBUF,
                       reinterpret_cast<const char*>(&config.send_buffer_bytes),
                       sizeof(config.send_buffer_bytes));
        }
        ++stats.websocket_total;
        ++stats.websocket_clients;
        push(c, std::make_shared<const std::string>(std::move(r)), now);
        events.push_back(Event{Event::kOpen, c.id, {}});
        return;
    }
    if (req.method != "GET") {
        HttpResponse r     = plainResponse(405, "method not allowed\n");
        r.headers["Allow"] = "GET";
        queueHttp(c, r, now);
        return;
    }
    c.awaiting_handler = true;
    events.push_back(Event{Event::kHttp, c.id, std::move(req)});
}

void HttpServer::Impl::parseWebSocket(Client& c, Clock::time_point now) {
    while (true) {
        if (c.dead || c.close_after_flush || c.in.size() < 2) {
            return;
        }
        const auto    b0     = static_cast<uint8_t>(c.in[0]);
        const auto    b1     = static_cast<uint8_t>(c.in[1]);
        const bool    fin    = (b0 & 0x80) != 0;
        const uint8_t opcode = b0 & 0x0f;
        const bool    masked = (b1 & 0x80) != 0;
        uint64_t      len    = b1 & 0x7f;
        std::size_t   pos    = 2;
        if (len == 126) {
            if (c.in.size() < 4) {
                return;
            }
            len = (static_cast<uint64_t>(static_cast<uint8_t>(c.in[2])) << 8) |
                  static_cast<uint8_t>(c.in[3]);
            pos = 4;
        } else if (len == 127) {
            if (c.in.size() < 10) {
                return;
            }
            len = 0;
            for (std::size_t i = 0; i < 8; ++i) {
                len = (len << 8) | static_cast<uint8_t>(c.in[2 + i]);
            }
            pos = 10;
        }
        // Unmasked, fragmented, reserved-bit or oversized client frames end the connection.
        if (!masked || !fin || opcode == 0 || (b0 & 0x70) != 0 || len > kMaxClientMessage) {
            kill(c, "protocol");
            return;
        }
        const std::size_t total = pos + 4 + static_cast<std::size_t>(len);
        if (c.in.size() < total) {
            return;
        }
        uint8_t mask[4];
        for (std::size_t i = 0; i < 4; ++i) {
            mask[i] = static_cast<uint8_t>(c.in[pos + i]);
        }
        std::string payload(c.in.begin() + static_cast<std::ptrdiff_t>(pos + 4),
                            c.in.begin() + static_cast<std::ptrdiff_t>(total));
        for (std::size_t i = 0; i < payload.size(); ++i) {
            payload[i] = static_cast<char>(static_cast<uint8_t>(payload[i]) ^ mask[i & 3]);
        }
        c.in.erase(0, total);
        switch (opcode) {
        case 0x1: // text: the feed is receiver to browser only
        case 0x2:
        case 0xA: // pong
            break;
        case 0x8: // close: echo it after the in-flight frame; nothing else follows
            ++stats.clients_closed_peer;
            closeWith(c,
                      websocketFrame(0x8, payload.data(), std::min<std::size_t>(payload.size(), 2)),
                      now);
            return;
        case 0x9:
            reliable(c,
                     std::make_shared<const std::string>(
                         websocketFrame(0xA, payload.data(), payload.size())),
                     now);
            break;
        default:
            kill(c, "protocol");
            return;
        }
    }
}

void HttpServer::Impl::readClient(Client& c, std::vector<Event>& events, Clock::time_point now) {
    char buf[16 * 1024];
    while (true) {
        const auto n = ::recv(c.sock, buf, sizeof(buf), 0);
        if (n > 0) {
            if (c.close_after_flush) {
                continue; // closing: only the peer's EOF matters now
            }
            c.in.append(buf, static_cast<std::size_t>(n));
            if (c.in.size() > kMaxRequestHead + kMaxClientMessage) {
                kill(c, "protocol");
                return;
            }
            continue;
        }
        if (n < 0 && wouldBlock()) {
            break;
        }
        kill(c, "peer"); // closed or errored
        return;
    }
    if (c.mode == Mode::kHttp) {
        parseHttp(c, events, now);
    } else {
        parseWebSocket(c, now);
    }
}

void HttpServer::Impl::reapDead(std::vector<Event>& events) {
    for (auto it = clients.begin(); it != clients.end();) {
        Client& c = it->second;
        if (!c.dead) {
            ++it;
            continue;
        }
        closeSocket(c.sock);
        if (c.mode == Mode::kWebSocket) {
            --stats.websocket_clients;
            events.push_back(Event{Event::kClose, c.id, {}});
        }
        it = clients.erase(it);
    }
}

void HttpServer::Impl::dispatch(std::vector<Event>& events) {
    for (Event& e : events) {
        switch (e.kind) {
        case Event::kHttp: {
            const HttpResponse r = handler ? handler(e.request) : plainResponse(404, "not found\n");
            const std::lock_guard<std::mutex> lock(mutex);
            const auto                        it = clients.find(e.id);
            if (it != clients.end()) {
                it->second.awaiting_handler = false;
                queueHttp(it->second, r, Clock::now());
            }
            break;
        }
        case Event::kOpen:
            if (ws.on_open) {
                ws.on_open(e.id);
            }
            break;
        case Event::kClose:
            if (ws.on_close) {
                ws.on_close(e.id);
            }
            break;
        }
    }
    events.clear();
}

void HttpServer::Impl::run() {
    std::vector<Event> events;
    while (!stop_flag.load()) {
        fd_set rd, wr;
        FD_ZERO(&rd);
        FD_ZERO(&wr);
        FD_SET(listen_sock, &rd);
#if !defined(_WIN32)
        int maxfd = listen_sock;
#endif
        {
            const std::lock_guard<std::mutex> lock(mutex);
            for (const auto& kv : clients) {
                const Client& c = kv.second;
                FD_SET(c.sock, &rd);
                if (c.queued() > 0) {
                    FD_SET(c.sock, &wr);
                }
#if !defined(_WIN32)
                maxfd = std::max(maxfd, c.sock);
#endif
            }
        }
        timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = kSelectTimeoutMs * 1000;
#if defined(_WIN32)
        const int n = select(0, &rd, &wr, nullptr, &tv);
#else
        const int n = select(maxfd + 1, &rd, &wr, nullptr, &tv);
#endif
        const Clock::time_point now = Clock::now();
        if (n < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kSelectTimeoutMs));
            continue;
        }
        if (n > 0 && FD_ISSET(listen_sock, &rd)) {
            acceptPending(now);
        }
        {
            const std::lock_guard<std::mutex> lock(mutex);
            for (auto& kv : clients) {
                Client& c = kv.second;
                if (c.dead) {
                    continue;
                }
                if (n > 0 && FD_ISSET(c.sock, &rd)) {
                    readClient(c, events, now);
                }
                if (c.dead) {
                    continue;
                }
                // also catches frames queued since select() started
                if (c.queued() > 0 && !flush(c, now)) {
                    kill(c, "peer");
                    continue;
                }
                if (c.queued() == 0 && c.close_after_flush && !c.half_closed) {
                    // Lingering close: an immediate close with unread peer bytes would be a
                    // reset, which can discard what was sent last. Wait for EOF, bounded.
                    shutdownSend(c.sock);
                    c.half_closed = true;
                    if (!c.has_close_deadline) {
                        c.has_close_deadline = true;
                        c.close_deadline     = now + std::chrono::milliseconds(kCloseGraceMs);
                    }
                }
                if (c.has_close_deadline && now >= c.close_deadline) {
                    kill(c, "closed");
                    continue;
                }
                const bool stalled = now - c.last_progress >=
                                     std::chrono::milliseconds(config.close_after_stalled_ms);
                if (c.queued() > 0 && stalled) {
                    kill(c, "stalled");
                } else if (c.mode == Mode::kHttp && !c.awaiting_handler && stalled) {
                    kill(c, "idle"); // connected, never asked for anything
                }
            }
            reapDead(events);
        }
        dispatch(events);
    }
    // Stopping: every client goes, and their close handlers still run here.
    {
        const std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            kill(kv.second, "server_stop");
        }
        reapDead(events);
    }
    dispatch(events);
}

// ---- public surface ----------------------------------------------------------

HttpServer::HttpServer(HttpServerConfig config, HttpHandler handler) : impl_(new Impl()) {
    impl_->config  = std::move(config);
    impl_->handler = std::move(handler);
}

HttpServer::~HttpServer() { stop(); }

void HttpServer::setWebSocketHandlers(WebSocketHandlers handlers) {
    impl_->ws = std::move(handlers); // before start(): the thread reads it unlocked
}

bool HttpServer::start(std::string& err) {
    if (impl_->running.load()) {
        return true;
    }
    if (!wsaAcquire(err)) {
        return false;
    }
    impl_->wsa_held = true;
    if (!impl_->listen(err)) {
        if (impl_->listen_sock != kInvalidSocket) {
            closeSocket(impl_->listen_sock);
            impl_->listen_sock = kInvalidSocket;
        }
        wsaRelease();
        impl_->wsa_held = false;
        return false;
    }
    impl_->stop_flag = false;
    impl_->running   = true;
    {
        const std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->stats.running = true;
        impl_->stats.port    = impl_->port.load();
    }
    impl_->thread = std::thread([this] { impl_->run(); });
    return true;
}

void HttpServer::stop() {
    impl_->stop_flag = true;
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
    if (impl_->listen_sock != kInvalidSocket) {
        closeSocket(impl_->listen_sock);
        impl_->listen_sock = kInvalidSocket;
    }
    impl_->running = false;
    {
        const std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->stats.running = false;
    }
    if (impl_->wsa_held) {
        wsaRelease();
        impl_->wsa_held = false;
    }
}

bool HttpServer::running() const { return impl_->running.load(); }
int  HttpServer::port() const { return impl_->port.load(); }

HttpServer::Frame HttpServer::textFrame(std::string_view text) {
    return std::make_shared<const std::string>(websocketFrame(0x1, text.data(), text.size()));
}

HttpServer::Enqueue HttpServer::sendReliable(ClientId client, const Frame& frame) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl::Client*                     c = impl_->openWebSocket(client);
    if (c == nullptr) {
        return Enqueue::kUnknownClient;
    }
    return impl_->reliable(*c, frame, Clock::now());
}

HttpServer::Enqueue HttpServer::sendLatest(ClientId client, const std::string& channel,
                                           int priority, const Frame& frame) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl::Client*                     c = impl_->openWebSocket(client);
    if (c == nullptr) {
        return Enqueue::kUnknownClient;
    }
    if (frame == nullptr) {
        return Enqueue::kRefused;
    }
    const Clock::time_point now  = Clock::now();
    Impl::Slot&             slot = c->slots[channel];
    slot.priority                = priority;
    const std::size_t old        = slot.frame ? slot.frame->size() : 0;
    if (c->queued() - old + frame->size() > impl_->config.client_buffer_bytes) {
        impl_->dropSlot(*c, slot); // the older one is no more useful than this one
        ++impl_->stats.messages_refused;
        return Enqueue::kRefused;
    }
    if (c->queued() == 0) {
        c->last_progress = now;
    }
    Enqueue result = Enqueue::kQueued;
    if (slot.frame) {
        impl_->dropSlot(*c, slot);
        ++impl_->stats.messages_replaced;
        result = Enqueue::kReplaced;
    }
    slot.frame = frame;
    c->slot_bytes += frame->size();
    if (!impl_->flush(*c, now)) {
        impl_->kill(*c, "peer");
    }
    return result;
}

void HttpServer::clearLatest(ClientId client, const std::string& channel) {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto                        it = impl_->clients.find(client);
    if (it == impl_->clients.end()) {
        return;
    }
    const auto slot = it->second.slots.find(channel);
    if (slot != it->second.slots.end()) {
        impl_->dropSlot(it->second, slot->second);
    }
}

std::size_t HttpServer::queued(ClientId client) const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto                        it = impl_->clients.find(client);
    return it == impl_->clients.end() ? 0 : it->second.queued();
}

std::vector<HttpServer::ClientId> HttpServer::clients() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<ClientId>             out;
    for (const auto& kv : impl_->clients) {
        if (kv.second.webSocketOpen()) {
            out.push_back(kv.first);
        }
    }
    return out;
}

HttpServerStats HttpServer::stats() const {
    const std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

} // namespace spectagatr::receiver::server
