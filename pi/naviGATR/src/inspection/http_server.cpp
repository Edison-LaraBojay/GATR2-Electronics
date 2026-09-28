// http_server.cpp
// One service thread owns every socket: it accepts, reads, parses and
// writes under a short select() timeout so stop() is prompt. Handlers run
// on that thread but outside the client lock, because a handler (or the
// inspection thread) may call back into the send functions; the lock
// guards the client table, the queues and the counters. Nothing here ever
// blocks on a socket: sockets are non-blocking, an enqueue tries at most a
// bounded non-blocking write, and what the kernel refuses waits in the
// bounded per-client queue (see http_server.h) for the server thread.
//
// SHA-1 and base64 are implemented here for the WebSocket accept key; the
// runtime takes no dependency for two short functions.

#include "inspection/http_server.h"

#ifdef _WIN32
#ifndef FD_SETSIZE
#define FD_SETSIZE 128   // listen socket plus up to 64 clients
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
#include <errno.h>
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
#include <cstdlib>
#include <cstring>
#include <deque>
#include <thread>

#include "core/host_clock.h"

namespace navigatr
{

namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::size_t kMaxRequestHead   = 16 * 1024;
constexpr std::size_t kMaxRequestBody   = 64 * 1024;
constexpr std::size_t kMaxClientMessage = 64 * 1024;
constexpr int         kSelectTimeoutMs  = 20;
constexpr std::size_t kSendChunk        = 64 * 1024;
constexpr std::size_t kFlushBudget      = 256 * 1024;  // bytes per flush call, bounds lock time
constexpr int         kCloseGraceMs     = 1000;        // close frame gets this long to leave
constexpr std::size_t kRecentCloses     = 8;
constexpr std::size_t kMaxHttpConnections = 32;  // concurrent plain requests, besides feed clients

// Flow control, see http_server.h.
constexpr std::size_t kMaxPings          = 64;          // outstanding per client
constexpr std::size_t kLatestBacklog     = 16 * 1024;   // unconfirmed bytes a replaceable frame may follow
constexpr std::size_t kUnackedPerChannel = 2;           // unconfirmed frames of one replaceable channel
constexpr std::size_t kPacedFrameBytes   = 4 * 1024;    // replaceable frames this large are paced
constexpr std::size_t kPacedBacklog      = 8 * 1024;    // unconfirmed bytes a paced frame may follow
constexpr double      kPaceFactor        = 3.0;         // gap = factor x the last delivery time
constexpr double      kMaxPaceGapMs      = 4000.0;      // and never near the stall close
constexpr std::size_t kMaxUnackedRecords = 64;          // delivery records per channel

// ---- platform shim -------------------------------------------------------

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
constexpr int          kSendFlags     = 0;

void closeSocket(SocketHandle s) { closesocket(s); }
void shutdownSend(SocketHandle s) { shutdown(s, SD_SEND); }
bool wouldBlock() {
    const int e = WSAGetLastError();
    return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS;
}
bool setNonBlocking(SocketHandle s) {
    u_long on = 1;
    return ioctlsocket(s, FIONBIO, &on) == 0;
}
std::string socketError() { return "winsock error " + std::to_string(WSAGetLastError()); }

std::mutex g_wsa_mutex;
int        g_wsa_refs = 0;

bool wsaAcquire(std::string& err) {
    std::lock_guard<std::mutex> lock(g_wsa_mutex);
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
    std::lock_guard<std::mutex> lock(g_wsa_mutex);
    if (g_wsa_refs > 0 && --g_wsa_refs == 0) {
        WSACleanup();
    }
}
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

void closeSocket(SocketHandle s) { ::close(s); }
void shutdownSend(SocketHandle s) { ::shutdown(s, SHUT_WR); }
bool wouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK; }
bool setNonBlocking(SocketHandle s) {
    const int flags = fcntl(s, F_GETFL, 0);
    return flags >= 0 && fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
}
std::string socketError() { return std::strerror(errno); }
bool        wsaAcquire(std::string&) { return true; }
void        wsaRelease() {}
#endif

// ---- SHA-1 and base64, for Sec-WebSocket-Accept ---------------------------

uint32_t rotl(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }

void sha1(const std::string& in, uint8_t out[20]) {
    uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
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
            const uint8_t* p = &msg[off + 4 * i];
            w[i] = (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
                   (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
        }
        for (int i = 16; i < 80; ++i) {
            w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
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
    for (int i = 0; i < 5; ++i) {
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

std::string websocketAccept(const std::string& key) {
    uint8_t digest[20];
    sha1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11", digest);
    return base64(digest, sizeof(digest));
}

// ---- text helpers ------------------------------------------------------------

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
    case 101: return "Switching Protocols";
    case 200: return "OK";
    case 400: return "Bad Request";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 409: return "Conflict";
    case 413: return "Payload Too Large";
    case 500: return "Internal Server Error";
    case 503: return "Service Unavailable";
    }
    return "Status";
}

std::string serializeHead(const HttpResponse& r, std::size_t body_len) {
    std::string s = "HTTP/1.1 " + std::to_string(r.status) + " " + reasonPhrase(r.status) + "\r\n";
    s += "Content-Type: " + r.content_type + "\r\n";
    s += "Content-Length: " + std::to_string(body_len) + "\r\n";
    s += "Connection: close\r\n";
    for (const auto& kv : r.headers) {
        s += kv.first + ": " + kv.second + "\r\n";
    }
    s += "\r\n";
    return s;
}

std::string serializeResponse(const HttpResponse& r) {
    return serializeHead(r, r.body.size()) + r.body;
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

// True when a full request head was parsed; consumed is its length.
bool parseRequestHead(const std::string& in, HttpRequest& req, std::size_t& consumed) {
    const std::size_t end = in.find("\r\n\r\n");
    if (end == std::string::npos) {
        return false;
    }
    consumed = end + 4;
    std::size_t line_end = in.find("\r\n");
    const std::string line = in.substr(0, line_end);
    const std::size_t sp1 = line.find(' ');
    const std::size_t sp2 = sp1 == std::string::npos ? std::string::npos : line.find(' ', sp1 + 1);
    if (sp1 == std::string::npos || sp2 == std::string::npos) {
        req.method = "";
        return true;   // caller answers 400
    }
    req.method = line.substr(0, sp1);
    const std::string target = line.substr(sp1 + 1, sp2 - sp1 - 1);
    const std::size_t q      = target.find('?');
    req.path  = urlDecode(q == std::string::npos ? target : target.substr(0, q));
    req.query = q == std::string::npos ? std::string() : target.substr(q + 1);
    std::size_t pos = line_end + 2;
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

double msBetween(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace

// ---- helpers declared in the header ------------------------------------------

std::string urlDecode(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '+') {
            out.push_back(' ');
        } else if (s[i] == '%' && i + 2 < s.size() && std::isxdigit(static_cast<unsigned char>(s[i + 1])) &&
                   std::isxdigit(static_cast<unsigned char>(s[i + 2]))) {
            out.push_back(static_cast<char>(std::stoi(s.substr(i + 1, 2), nullptr, 16)));
            i += 2;
        } else {
            out.push_back(s[i]);
        }
    }
    return out;
}

std::map<std::string, std::string> parseQuery(const std::string& query) {
    std::map<std::string, std::string> out;
    std::size_t                        pos = 0;
    while (pos <= query.size()) {
        std::size_t amp = query.find('&', pos);
        if (amp == std::string::npos) {
            amp = query.size();
        }
        const std::string pair = query.substr(pos, amp - pos);
        if (!pair.empty()) {
            const std::size_t eq = pair.find('=');
            const std::string k  = urlDecode(eq == std::string::npos ? pair : pair.substr(0, eq));
            if (!k.empty()) {
                out[k] = eq == std::string::npos ? std::string() : urlDecode(pair.substr(eq + 1));
            }
        }
        pos = amp + 1;
    }
    return out;
}

std::string HttpRequest::param(const std::string& name, const std::string& def) const {
    const auto q  = parseQuery(query);
    const auto it = q.find(name);
    return it == q.end() ? def : it->second;
}

// ---- the server ------------------------------------------------------------------

struct HttpServer::Impl {
    enum class Mode { kHttp, kWebSocket };

    struct Pending {
        Frame             frame;
        std::string       channel;
        Clock::time_point enqueued;
    };

    struct Slot {
        Pending msg;
        int     priority = 0;
        bool    full     = false;
    };

    // A frame written but not yet confirmed: its end position.
    struct Unacked {
        uint64_t          end = 0;
        Clock::time_point enqueued;
    };

    struct Counters {
        bool     replaceable     = false;
        uint64_t queued          = 0;
        uint64_t sent            = 0;
        uint64_t replaced        = 0;
        uint64_t refused         = 0;
        uint64_t dropped         = 0;
        uint64_t bytes           = 0;
        double   last_latency_ms = -1;
        double   last_acked_ms   = -1;
        std::deque<Unacked> unacked;   // flow control only, oldest first
    };

    struct Ping {
        uint64_t          pos = 0;   // bytes written before it
        Clock::time_point sent;
    };

    struct Client {
        SocketHandle sock = kInvalidSocket;
        ClientId     id   = 0;
        Mode         mode = Mode::kHttp;
        std::string  in;

        // the frame being written: never modified, never dropped once started
        bool        has_inflight = false;
        Pending     inflight;
        std::size_t inflight_off = 0;

        std::deque<Pending>         control;   // pings: between frames, before the rest
        std::size_t                 control_bytes = 0;
        std::deque<Pending>         reliable;
        std::size_t                 reliable_bytes = 0;
        std::map<std::string, Slot> slots;   // by channel name
        std::size_t                 slot_bytes    = 0;
        std::size_t                 slots_pending = 0;
        std::map<int, std::string>  rr_last;   // priority -> channel served last
        std::map<std::string, Counters> counters;

        // flow control: positions count bytes written since the upgrade
        uint64_t          written    = 0;
        uint64_t          data_end   = 0;       // position after the newest frame not a ping
        uint64_t          acked      = 0;       // highest position a pong confirmed
        bool              acking     = false;   // answered a ping: paced by pongs
        bool              probe_sent = false;
        bool              ping_owed  = false;   // a frame ended while kMaxPings were out
        std::deque<Ping>  pings;                // outstanding, oldest first
        double            ack_rtt_ms     = -1;
        double            ack_rtt_min_ms = -1;
        // large replaceable frames: one at a time, then a gap
        bool              paced_pending = false;
        uint64_t          paced_end     = 0;
        Clock::time_point paced_started;
        Clock::time_point paced_next;
        double            pace_gap_ms = 0;

        bool        awaiting_handler  = false;  // request dispatched, response pending
        bool        close_after_flush = false;
        bool        half_closed       = false;   // our side is done, waiting for the peer's EOF
        bool        dead              = false;
        bool        refused           = false;  // over max_clients: answers 503 and goes
        std::string close_reason;
        bool        has_close_deadline = false;
        Clock::time_point close_deadline;
        Clock::time_point last_progress;
        Clock::time_point opened;

        std::size_t inflightRest() const {
            return has_inflight ? inflight.frame->size() - inflight_off : 0;
        }
        std::size_t queued() const {
            return inflightRest() + control_bytes + reliable_bytes + slot_bytes;
        }
        uint64_t unacked() const { return acking ? written - acked : 0; }
        bool        webSocketOpen() const {
            return mode == Mode::kWebSocket && !dead && !close_after_flush;
        }
    };

    struct Event {
        enum Kind { kHttp, kOpen, kText, kClose } kind;
        ClientId    id;
        HttpRequest request;
        std::string text;
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

    mutable std::mutex         mutex;   // clients, queues and stats
    std::map<ClientId, Client> clients;
    ClientId                   next_id = 1;
    HttpServerStats            stats;

    bool listen(std::string& err);
    void run();
    void acceptPending(Clock::time_point now);
    void readClient(Client& c, std::vector<Event>& events, Clock::time_point now);
    void parseHttp(Client& c, std::vector<Event>& events, Clock::time_point now);
    void parseWebSocket(Client& c, std::vector<Event>& events, Clock::time_point now);
    void kill(Client& c, const char* reason);
    bool eligible(const Client& c, const std::string& name, const Slot& slot,
                  Clock::time_point now) const;
    bool choose(const Client& c, Clock::time_point now, int& kind, std::string& slot) const;
    bool promote(Client& c, Clock::time_point now);
    bool wantsWrite(const Client& c, Clock::time_point now) const;
    void complete(Client& c, Clock::time_point now);
    void queuePing(Client& c, Clock::time_point now);
    void onPong(Client& c, const std::string& payload, Clock::time_point now);
    void endPaced(Client& c, Clock::time_point now);
    bool flush(Client& c, Clock::time_point now);
    void push(Client& c, Frame frame, const std::string& channel, Clock::time_point now);
    Enqueue reliable(Client& c, const Frame& frame, const std::string& channel,
                     Clock::time_point now);
    void dropSlots(Client& c, const std::string& prefix);
    void closeOverflowing(Client& c, Clock::time_point now);
    void queueHttp(Client& c, const HttpResponse& r, Clock::time_point now);
    void boundSendBuffer(SocketHandle s) const;
    void reapDead(std::vector<Event>& events);
    void dispatch(std::vector<Event>& events);
    std::size_t admitted() const;
    std::size_t feedClients() const;
    Client* openWebSocket(ClientId id);
};

std::size_t HttpServer::Impl::admitted() const {
    std::size_t n = 0;
    for (const auto& kv : clients) {
        if (!kv.second.refused && !kv.second.dead) {
            ++n;
        }
    }
    return n;
}

std::size_t HttpServer::Impl::feedClients() const {
    std::size_t n = 0;
    for (const auto& kv : clients) {
        if (kv.second.mode == Mode::kWebSocket && !kv.second.dead) {
            ++n;
        }
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
#ifdef _WIN32
    setsockopt(listen_sock, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&one),
               sizeof(one));
#else
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one),
               sizeof(one));
#endif
    if (::bind(listen_sock, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        err = "bind " + config.bind + ":" + std::to_string(config.port) + ": " + socketError();
        return false;
    }
    if (::listen(listen_sock, 16) != 0) {
        err = "listen: " + socketError();
        return false;
    }
    if (!setNonBlocking(listen_sock)) {
        err = "non-blocking listen socket: " + socketError();
        return false;
    }
    sockaddr_in bound;
    std::memset(&bound, 0, sizeof(bound));
#ifdef _WIN32
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

// A small kernel send buffer bounds what already-written frames can wait
// in the kernel; TCP_NOTSENT_LOWAT (Linux) bounds the unsent part without
// capping the congestion window. Feed sockets only: downloads keep the
// OS default.
void HttpServer::Impl::boundSendBuffer(SocketHandle s) const {
    if (config.send_buffer_bytes <= 0) {
        return;
    }
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, reinterpret_cast<const char*>(&config.send_buffer_bytes),
               sizeof(config.send_buffer_bytes));
#ifdef TCP_NOTSENT_LOWAT
    setsockopt(s, IPPROTO_TCP, TCP_NOTSENT_LOWAT,
               reinterpret_cast<const char*>(&config.send_buffer_bytes),
               sizeof(config.send_buffer_bytes));
#endif
}

void HttpServer::Impl::acceptPending(Clock::time_point now) {
    for (;;) {
        const SocketHandle s = ::accept(listen_sock, nullptr, nullptr);
        if (s == kInvalidSocket) {
            return;   // nothing pending, or a transient error: retry next loop
        }
        setNonBlocking(s);
        const int one = 1;
        setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));

        std::lock_guard<std::mutex> lock(mutex);
        Client                      c;
        c.sock          = s;
        c.id            = next_id++;
        c.last_progress = now;
        c.opened        = now;
        // over the limit: the request is still read so the 503 reaches the
        // client instead of a reset, and the connection goes right after.
        // Short requests have their own bound: a page load opens several at
        // once, and max_clients counts feed clients only.
        c.refused = admitted() >= feedClients() + kMaxHttpConnections;
        if (c.refused) {
            ++stats.refused_connections;
        } else {
            ++stats.connections_total;
        }
        clients.emplace(c.id, std::move(c));
        stats.connections_open = admitted();
    }
}

// Caller holds mutex. The first reason wins; counters are per WebSocket
// client except stalls, which the HTTP side has always counted too.
void HttpServer::Impl::kill(Client& c, const char* reason) {
    if (c.dead) {
        return;
    }
    c.dead = true;
    if (!c.close_reason.empty()) {
        return;
    }
    c.close_reason = reason;
    const std::string r = reason;
    if (r == "stalled") {
        ++stats.clients_closed_stalled;
    }
    if (c.mode != Mode::kWebSocket) {
        return;
    }
    if (r == "protocol") {
        ++stats.clients_closed_protocol;
    } else if (r == "peer" || r == "closed") {
        ++stats.clients_closed_peer;
    }
}

// A replaceable slot flow control lets go now. Without pongs only the
// pacing of large frames applies (by write completion).
bool HttpServer::Impl::eligible(const Client& c, const std::string& name, const Slot& slot,
                                Clock::time_point now) const {
    const bool paced = slot.msg.frame->size() >= kPacedFrameBytes;
    if (paced && (c.paced_pending || now < c.paced_next)) {
        return false;
    }
    if (!c.acking) {
        return true;
    }
    if (c.unacked() > (paced ? kPacedBacklog : kLatestBacklog)) {
        return false;
    }
    const auto k = c.counters.find(name);
    return k == c.counters.end() || k->second.unacked.size() < kUnackedPerChannel;
}

enum ChoiceKind { kChooseControl, kChooseReliable, kChooseSlot };

// Next frame to write: a ping, else the reliable head, else the eligible
// replaceable slot with the lowest priority number, round robin among equal
// priorities. False when nothing may go now.
bool HttpServer::Impl::choose(const Client& c, Clock::time_point now, int& kind,
                              std::string& slot) const {
    if (!c.control.empty()) {
        kind = kChooseControl;
        return true;
    }
    const bool window_open =
        !c.acking || config.ack_window_bytes == 0 || c.unacked() < config.ack_window_bytes;
    if (!c.reliable.empty()) {
        // strict order: nothing overtakes the reliable head. A close frame
        // goes whatever the window; nothing follows it.
        if (window_open || c.reliable.front().channel == "close") {
            kind = kChooseReliable;
            return true;
        }
        return false;
    }
    if (c.slots_pending == 0 || !window_open) {
        return false;
    }
    int best = INT_MAX;
    for (const auto& kv : c.slots) {
        if (kv.second.full && kv.second.priority < best && eligible(c, kv.first, kv.second, now)) {
            best = kv.second.priority;
        }
    }
    if (best == INT_MAX) {
        return false;
    }
    const auto         rr    = c.rr_last.find(best);
    const std::string  last  = rr == c.rr_last.end() ? std::string() : rr->second;
    const std::string* first = nullptr;
    const std::string* after = nullptr;
    for (const auto& kv : c.slots) {
        if (!kv.second.full || kv.second.priority != best ||
            !eligible(c, kv.first, kv.second, now)) {
            continue;
        }
        if (first == nullptr) {
            first = &kv.first;
        }
        if (after == nullptr && kv.first > last) {
            after = &kv.first;
        }
    }
    kind = kChooseSlot;
    slot = after != nullptr ? *after : *first;
    return true;
}

bool HttpServer::Impl::promote(Client& c, Clock::time_point now) {
    if (c.has_inflight) {
        return true;
    }
    int         kind = 0;
    std::string name;
    if (!choose(c, now, kind, name)) {
        return false;
    }
    if (kind == kChooseControl) {
        c.inflight = std::move(c.control.front());
        c.control.pop_front();
        c.control_bytes -= c.inflight.frame->size();
    } else if (kind == kChooseReliable) {
        c.inflight = std::move(c.reliable.front());
        c.reliable.pop_front();
        c.reliable_bytes -= c.inflight.frame->size();
    } else {
        Slot& pick = c.slots[name];
        c.inflight = std::move(pick.msg);
        pick.full  = false;
        c.slot_bytes -= c.inflight.frame->size();
        --c.slots_pending;
        c.rr_last[pick.priority] = name;
        if (c.inflight.frame->size() >= kPacedFrameBytes) {
            c.paced_pending = true;
            c.paced_started = now;
            c.paced_end     = c.written + c.inflight.frame->size();
        }
    }
    c.has_inflight = true;
    c.inflight_off = 0;
    return true;
}

// Worth a writability wait: something may be written now.
bool HttpServer::Impl::wantsWrite(const Client& c, Clock::time_point now) const {
    if (c.has_inflight) {
        return true;
    }
    int         kind = 0;
    std::string name;
    return c.queued() > 0 && choose(c, now, kind, name);
}

void HttpServer::Impl::endPaced(Client& c, Clock::time_point now) {
    const double delivery = msBetween(c.paced_started, now);
    c.pace_gap_ms =
        std::min({kPaceFactor * delivery, kMaxPaceGapMs, config.close_after_stalled_ms / 2.0});
    c.paced_next =
        now + std::chrono::microseconds(static_cast<int64_t>(c.pace_gap_ms * 1000.0));
    c.paced_pending = false;
}

// Caller holds mutex. The ping carries the position of everything written
// before it; queued right after a frame ends, it goes next.
void HttpServer::Impl::queuePing(Client& c, Clock::time_point now) {
    if (!c.pings.empty() && c.pings.back().pos == c.written) {
        return;   // nothing new since the last one
    }
    if (c.pings.size() >= kMaxPings) {
        c.ping_owed = true;
        return;
    }
    char payload[8];
    for (int i = 0; i < 8; ++i) {
        payload[i] = static_cast<char>((c.written >> (56 - 8 * i)) & 0xff);
    }
    Frame f = std::make_shared<const std::string>(websocketFrame(0x9, payload, sizeof(payload)));
    c.control_bytes += f->size();
    c.control.push_back(Pending{std::move(f), "ws_ping", now});
    c.pings.push_back(Ping{c.written, now});
}

// Caller holds mutex. Only a pong echoing a ping this server sent counts:
// unsolicited pongs are legal and mean nothing.
void HttpServer::Impl::onPong(Client& c, const std::string& payload, Clock::time_point now) {
    if (payload.size() != 8) {
        return;
    }
    uint64_t pos = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        pos = (pos << 8) | static_cast<uint8_t>(payload[i]);
    }
    const auto it = std::find_if(c.pings.begin(), c.pings.end(),
                                 [&](const Ping& p) { return p.pos == pos; });
    if (it == c.pings.end()) {
        return;
    }
    c.ack_rtt_ms = msBetween(it->sent, now);
    c.ack_rtt_min_ms =
        c.ack_rtt_min_ms < 0.0 ? c.ack_rtt_ms : std::min(c.ack_rtt_min_ms, c.ack_rtt_ms);
    c.pings.erase(c.pings.begin(), it + 1);   // older ones are covered too
    if (!c.acking && !c.has_inflight && c.data_end > pos) {
        // the probe's answer: what went out since it has no ping behind it
        queuePing(c, now);
    }
    c.acking        = true;
    c.acked         = std::max(c.acked, pos);
    c.last_progress = now;   // the client is reading
    for (auto& kv : c.counters) {
        std::deque<Unacked>& u = kv.second.unacked;
        while (!u.empty() && u.front().end <= c.acked) {
            kv.second.last_acked_ms = msBetween(u.front().enqueued, now);
            u.pop_front();
        }
    }
    if (c.paced_pending && c.acked >= c.paced_end) {
        endPaced(c, now);
    }
    if (c.ping_owed && c.pings.size() < kMaxPings) {
        c.ping_owed = false;
        if (!c.has_inflight && c.data_end > c.acked) {
            queuePing(c, now);   // else the frame's end pings
        }
    }
}

void HttpServer::Impl::complete(Client& c, Clock::time_point) {
    const Clock::time_point now = Clock::now();   // the caller's can equal the enqueue time
    Counters&               k   = c.counters[c.inflight.channel];
    ++k.sent;
    k.bytes += c.inflight.frame->size();
    k.last_latency_ms          = msBetween(c.inflight.enqueued, now);
    const std::string& channel = c.inflight.channel;
    if (channel != "ws_ping") {
        c.data_end = c.written;
    }
    if (c.mode == Mode::kWebSocket && !c.close_after_flush && channel != "ws_ping" &&
        channel != "close" && config.ack_window_bytes > 0) {
        if (c.acking) {
            k.unacked.push_back(Unacked{c.written, c.inflight.enqueued});
            if (k.unacked.size() > kMaxUnackedRecords) {
                k.unacked.pop_front();
            }
            queuePing(c, now);
        } else if (!c.probe_sent) {
            c.probe_sent = true;   // after the handshake: does this client answer?
            queuePing(c, now);
        }
    }
    if (c.paced_pending && !c.acking && c.written >= c.paced_end) {
        endPaced(c, now);   // no pongs: the kernel taking it is all there is
    }
    c.inflight     = Pending{};
    c.has_inflight = false;
    c.inflight_off = 0;
}

// Writes what the socket takes, frame after frame, up to kFlushBudget.
// False on a socket error.
bool HttpServer::Impl::flush(Client& c, Clock::time_point now) {
    std::size_t budget = kFlushBudget;
    while (budget > 0) {
        if (!promote(c, now)) {
            break;
        }
        const std::string& f     = *c.inflight.frame;
        const std::size_t  rest  = f.size() - c.inflight_off;
        const int          chunk = static_cast<int>(std::min({rest, kSendChunk, budget}));
        const auto n = ::send(c.sock, f.data() + c.inflight_off, chunk, kSendFlags);
        if (n > 0) {
            c.inflight_off += static_cast<std::size_t>(n);
            c.written += static_cast<uint64_t>(n);
            stats.bytes_sent += static_cast<uint64_t>(n);
            c.last_progress = now;
            budget -= std::min(budget, static_cast<std::size_t>(n));
            if (c.inflight_off == f.size()) {
                complete(c, now);
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

// Caller holds mutex. Unbounded append: HTTP responses and control frames
// that must follow a clear.
void HttpServer::Impl::push(Client& c, Frame frame, const std::string& channel,
                            Clock::time_point now) {
    if (c.dead || frame == nullptr || frame->empty()) {
        return;
    }
    if (c.queued() == 0) {
        c.last_progress = now;   // the stall clock starts when data waits
    }
    c.reliable_bytes += frame->size();
    c.reliable.push_back(Pending{std::move(frame), channel, now});
    ++c.counters[channel].queued;
    if (!flush(c, now)) {
        kill(c, "peer");
    }
}

void HttpServer::Impl::dropSlots(Client& c, const std::string& prefix) {
    for (auto& kv : c.slots) {
        if (!kv.second.full || kv.first.compare(0, prefix.size(), prefix) != 0) {
            continue;
        }
        c.slot_bytes -= kv.second.msg.frame->size();
        --c.slots_pending;
        kv.second.full = false;
        kv.second.msg  = Pending{};
        ++c.counters[kv.first].dropped;
    }
}

// A client whose reliable backlog overflowed: unsent messages go, a close
// frame follows the in-flight one, and the socket goes after a short grace
// whether or not the client read it.
void HttpServer::Impl::closeOverflowing(Client& c, Clock::time_point now) {
    c.control.clear();
    c.control_bytes = 0;
    c.reliable.clear();
    c.reliable_bytes = 0;
    dropSlots(c, "");
    c.close_reason = "reliable_overflow";
    ++stats.clients_closed_reliable_overflow;
    c.close_after_flush = true;   // before the push: no ping follows the close
    const std::string f = closeFrame(1013, "reliable backlog overflow");
    push(c, std::make_shared<const std::string>(f), "close", now);
    c.has_close_deadline = true;
    c.close_deadline     = now + std::chrono::milliseconds(kCloseGraceMs);
}

HttpServer::Enqueue HttpServer::Impl::reliable(Client& c, const Frame& frame,
                                               const std::string& channel, Clock::time_point now) {
    if (frame == nullptr) {
        return Enqueue::kRefused;
    }
    // the budget bounds the backlog, not one message: a hello or history
    // larger than it still goes when nothing waits
    if (!c.reliable.empty() && c.reliable_bytes + frame->size() > config.reliable_bytes) {
        closeOverflowing(c, now);
        return Enqueue::kClosed;
    }
    push(c, frame, channel, now);
    return c.dead ? Enqueue::kClosed : Enqueue::kQueued;
}

void HttpServer::Impl::queueHttp(Client& c, const HttpResponse& r, Clock::time_point now) {
    if (r.shared_body != nullptr) {
        push(c, std::make_shared<const std::string>(serializeHead(r, r.shared_body->size())),
             "http", now);
        push(c, r.shared_body, "http", now);
    } else {
        push(c, std::make_shared<const std::string>(serializeResponse(r)), "http", now);
    }
    c.close_after_flush = true;
}

void HttpServer::Impl::parseHttp(Client& c, std::vector<Event>& events, Clock::time_point now) {
    if (c.awaiting_handler || c.close_after_flush) {
        return;   // one request per connection; anything more is ignored
    }
    HttpRequest req;
    std::size_t consumed = 0;
    if (!parseRequestHead(c.in, req, consumed)) {
        if (c.in.size() > kMaxRequestHead) {
            kill(c, "protocol");
        }
        return;
    }
    if (c.refused) {
        c.in.clear();
        ++stats.requests;
        queueHttp(c, plainResponse(503, "too many clients\n"), now);
        return;
    }
    if (req.method.empty()) {
        c.in.clear();
        ++stats.requests;
        queueHttp(c, plainResponse(400, "bad request\n"), now);
        return;
    }
    // a POST body arrives after the head; wait for all of it, bounded
    std::size_t body_len = 0;
    const auto  cl       = req.headers.find("content-length");
    if (cl != req.headers.end()) {
        char*                    end = nullptr;
        const unsigned long long v   = std::strtoull(cl->second.c_str(), &end, 10);
        if (end == cl->second.c_str() || v > kMaxRequestBody) {
            c.in.clear();
            ++stats.requests;
            queueHttp(c, plainResponse(413, "request body too large\n"), now);
            return;
        }
        body_len = static_cast<std::size_t>(v);
    }
    if (c.in.size() < consumed + body_len) {
        return;
    }
    req.body = c.in.substr(consumed, body_len);
    c.in.erase(0, consumed + body_len);
    ++stats.requests;

    const auto upgrade = req.headers.find("upgrade");
    const auto key     = req.headers.find("sec-websocket-key");
    if (req.method == "GET" && req.path == config.websocket_path && upgrade != req.headers.end() &&
        lower(upgrade->second).find("websocket") != std::string::npos &&
        key != req.headers.end()) {
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
        c.mode   = Mode::kWebSocket;
        c.opened = now;
        boundSendBuffer(c.sock);
        ++stats.websocket_total;
        ++stats.websocket_clients;
        push(c, std::make_shared<const std::string>(std::move(r)), "handshake", now);
        events.push_back(Event{Event::kOpen, c.id, {}, {}});
        return;
    }
    if (req.method != "GET" && req.method != "POST") {
        HttpResponse r      = plainResponse(405, "method not allowed\n");
        r.headers["Allow"] = "GET, POST";
        queueHttp(c, r, now);
        return;
    }
    c.awaiting_handler = true;
    events.push_back(Event{Event::kHttp, c.id, std::move(req), {}});
}

void HttpServer::Impl::parseWebSocket(Client& c, std::vector<Event>& events,
                                      Clock::time_point now) {
    for (;;) {
        if (c.dead || c.close_after_flush || c.in.size() < 2) {
            return;
        }
        const uint8_t b0     = static_cast<uint8_t>(c.in[0]);
        const uint8_t b1     = static_cast<uint8_t>(c.in[1]);
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
            for (int i = 0; i < 8; ++i) {
                len = (len << 8) | static_cast<uint8_t>(c.in[2 + i]);
            }
            pos = 10;
        }
        // unmasked, fragmented, reserved-bit or oversized client frames end
        // the connection: the feed has no use for any of them
        if (!masked || !fin || opcode == 0 || (b0 & 0x70) != 0 || len > kMaxClientMessage) {
            kill(c, "protocol");
            return;
        }
        const std::size_t total = pos + 4 + static_cast<std::size_t>(len);
        if (c.in.size() < total) {
            return;
        }
        uint8_t mask[4];
        for (int i = 0; i < 4; ++i) {
            mask[i] = static_cast<uint8_t>(c.in[pos + i]);
        }
        std::string payload(c.in.begin() + static_cast<std::ptrdiff_t>(pos + 4),
                            c.in.begin() + static_cast<std::ptrdiff_t>(total));
        for (std::size_t i = 0; i < payload.size(); ++i) {
            payload[i] = static_cast<char>(static_cast<uint8_t>(payload[i]) ^ mask[i & 3]);
        }
        c.in.erase(0, total);
        switch (opcode) {
        case 0x1:
            events.push_back(Event{Event::kText, c.id, {}, std::move(payload)});
            break;
        case 0x2:
            break;   // binary from a browser carries nothing the feed reads
        case 0x8: {
            // echo the close after the in-flight frame; nothing else follows
            c.control.clear();
            c.control_bytes = 0;
            c.reliable.clear();
            c.reliable_bytes = 0;
            dropSlots(c, "");
            c.close_reason = "closed";
            ++stats.clients_closed_peer;
            c.close_after_flush = true;
            const std::string f =
                websocketFrame(0x8, payload.data(), std::min<std::size_t>(payload.size(), 2));
            push(c, std::make_shared<const std::string>(f), "close", now);
            c.has_close_deadline = true;
            c.close_deadline     = now + std::chrono::milliseconds(kCloseGraceMs);
            return;
        }
        case 0x9:
            reliable(c, std::make_shared<const std::string>(
                            websocketFrame(0xA, payload.data(), payload.size())),
                     "ws_pong", now);
            break;
        case 0xA:
            onPong(c, payload, now);
            break;
        default:
            kill(c, "protocol");
            return;
        }
    }
}

void HttpServer::Impl::readClient(Client& c, std::vector<Event>& events, Clock::time_point now) {
    char buf[16 * 1024];
    for (;;) {
        const auto n = ::recv(c.sock, buf, sizeof(buf), 0);
        if (n > 0) {
            if (c.close_after_flush) {
                continue;   // closing: only the peer's EOF matters now
            }
            c.in.append(buf, static_cast<std::size_t>(n));
            if (c.in.size() > kMaxRequestHead + kMaxRequestBody + kMaxClientMessage) {
                kill(c, "protocol");   // nobody legitimate sends this much to a feed
                return;
            }
            continue;
        }
        if (n < 0 && wouldBlock()) {
            break;
        }
        kill(c, "peer");   // closed or errored
        return;
    }
    if (c.mode == Mode::kHttp) {
        parseHttp(c, events, now);
    } else {
        parseWebSocket(c, events, now);
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
            stats.recent_closes.push_back(
                WsCloseRecord{c.id, c.close_reason.empty() ? "closed" : c.close_reason,
                              static_cast<int64_t>(HostClock::now().ms)});
            if (stats.recent_closes.size() > kRecentCloses) {
                stats.recent_closes.erase(stats.recent_closes.begin());
            }
            events.push_back(Event{Event::kClose, c.id, {}, {}});
        }
        it = clients.erase(it);
    }
    stats.connections_open = admitted();
}

void HttpServer::Impl::dispatch(std::vector<Event>& events) {
    for (Event& e : events) {
        switch (e.kind) {
        case Event::kHttp: {
            HttpResponse r;
            if (handler) {
                r = handler(e.request);
            } else {
                r = plainResponse(404, "not found\n");
            }
            std::lock_guard<std::mutex> lock(mutex);
            const auto                  it = clients.find(e.id);
            if (it != clients.end()) {
                it->second.awaiting_handler = false;
                queueHttp(it->second, r, Clock::now());
            }
            break;
        }
        case Event::kOpen:
            if (ws.onOpen) {
                ws.onOpen(e.id);
            }
            break;
        case Event::kText:
            if (ws.onText) {
                ws.onText(e.id, e.text);
            }
            break;
        case Event::kClose:
            if (ws.onClose) {
                ws.onClose(e.id);
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
#ifndef _WIN32
        int maxfd = listen_sock;
#endif
        {
            const Clock::time_point     select_start = Clock::now();
            std::lock_guard<std::mutex> lock(mutex);
            for (const auto& kv : clients) {
                const Client& c = kv.second;
                FD_SET(c.sock, &rd);
                // flow control can hold queued data back: no busy wait then
                if (wantsWrite(c, select_start)) {
                    FD_SET(c.sock, &wr);
                }
#ifndef _WIN32
                maxfd = std::max(maxfd, c.sock);
#endif
            }
        }
        timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = kSelectTimeoutMs * 1000;
#ifdef _WIN32
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
            std::lock_guard<std::mutex> lock(mutex);
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
                    // lingering close: bytes the peer still sends (a pong
                    // already on its way) would make an immediate close a
                    // reset, and a reset can discard what was sent last.
                    // Wait for its EOF, bounded by the grace.
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
                    kill(c, "idle");   // connected, never asked for anything
                }
            }
            reapDead(events);
        }
        dispatch(events);
    }
    // stopping: every client goes, and their close handlers still run here
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& kv : clients) {
            kill(kv.second, "server_stop");
        }
        reapDead(events);
    }
    dispatch(events);
}

// ---- public surface -------------------------------------------------------------

HttpServer::HttpServer(HttpServerConfig config, HttpHandler handler) : impl_(new Impl()) {
    impl_->config  = std::move(config);
    impl_->handler = std::move(handler);
}

HttpServer::~HttpServer() { stop(); }

void HttpServer::setWebSocketHandlers(WebSocketHandlers handlers) {
    impl_->ws = std::move(handlers);   // before start(): the thread reads it unlocked
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
        std::lock_guard<std::mutex> lock(impl_->mutex);
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
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->stats.running = false;
    }
    if (impl_->wsa_held) {
        wsaRelease();
        impl_->wsa_held = false;
    }
}

bool HttpServer::running() const { return impl_->running.load(); }
int  HttpServer::port() const { return impl_->port.load(); }

HttpServer::Frame HttpServer::textFrame(const std::string& text) {
    return std::make_shared<const std::string>(websocketFrame(0x1, text.data(), text.size()));
}

HttpServer::Frame HttpServer::binaryFrame(const uint8_t* data, std::size_t len) {
    return std::make_shared<const std::string>(
        websocketFrame(0x2, reinterpret_cast<const char*>(data), len));
}

HttpServer::Enqueue HttpServer::sendReliable(ClientId client, const Frame& frame,
                                             const std::string& channel) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl::Client*               c = impl_->openWebSocket(client);
    if (c == nullptr) {
        return Enqueue::kUnknownClient;
    }
    return impl_->reliable(*c, frame, channel, Clock::now());
}

HttpServer::Enqueue HttpServer::sendLatest(ClientId client, const WsChannel& channel,
                                           const Frame& frame) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    Impl::Client*               c = impl_->openWebSocket(client);
    if (c == nullptr) {
        return Enqueue::kUnknownClient;
    }
    if (frame == nullptr) {
        return Enqueue::kRefused;
    }
    const Clock::time_point now  = Clock::now();
    Impl::Slot&             slot = c->slots[channel.name];
    Impl::Counters&         k    = c->counters[channel.name];
    k.replaceable                = true;
    slot.priority                = channel.priority;
    const std::size_t old        = slot.full ? slot.msg.frame->size() : 0;
    if (c->queued() - old + frame->size() > impl_->config.client_buffer_bytes) {
        // the older unsent one is no more useful than this one: it goes too
        if (slot.full) {
            c->slot_bytes -= old;
            --c->slots_pending;
            slot.full = false;
            slot.msg  = Impl::Pending{};
            ++k.dropped;
        }
        ++k.refused;
        ++impl_->stats.messages_refused;
        return Enqueue::kRefused;
    }
    if (c->queued() == 0) {
        c->last_progress = now;
    }
    Enqueue r = Enqueue::kQueued;
    if (slot.full) {
        c->slot_bytes -= old;
        ++k.replaced;
        ++impl_->stats.messages_replaced;
        r = Enqueue::kReplaced;
    } else {
        ++c->slots_pending;
    }
    slot.msg  = Impl::Pending{frame, channel.name, now};
    slot.full = true;
    c->slot_bytes += frame->size();
    ++k.queued;
    if (!impl_->flush(*c, now)) {
        impl_->kill(*c, "peer");
    }
    return r;
}

void HttpServer::clearLatest(ClientId client, const std::string& prefix) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto                  it = impl_->clients.find(client);
    if (it != impl_->clients.end()) {
        impl_->dropSlots(it->second, prefix);
    }
}

bool HttpServer::sendText(ClientId client, const std::string& text) {
    return sendReliable(client, textFrame(text), "text") == Enqueue::kQueued;
}

bool HttpServer::sendBinary(ClientId client, const std::vector<uint8_t>& bytes) {
    return sendReliable(client, binaryFrame(bytes.data(), bytes.size()), "binary") ==
           Enqueue::kQueued;
}

std::size_t HttpServer::queued(ClientId client) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto                  it = impl_->clients.find(client);
    return it == impl_->clients.end() ? 0 : it->second.queued();
}

std::vector<HttpServer::ClientId> HttpServer::clients() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<ClientId>       out;
    for (const auto& kv : impl_->clients) {
        if (kv.second.webSocketOpen()) {
            out.push_back(kv.first);
        }
    }
    return out;
}

std::vector<WsClientStats> HttpServer::clientStats() const {
    const Clock::time_point     now = Clock::now();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<WsClientStats>  out;
    for (const auto& kv : impl_->clients) {
        const Impl::Client& c = kv.second;
        if (c.mode != Impl::Mode::kWebSocket || c.dead) {
            continue;
        }
        WsClientStats s;
        s.id               = c.id;
        s.connected_ms     = static_cast<int64_t>(msBetween(c.opened, now));
        s.queued_bytes     = c.queued();
        s.in_flight_bytes  = c.inflightRest();
        s.reliable_backlog = c.reliable.size();
        s.reliable_bytes   = c.reliable_bytes;
        s.slots_pending    = c.slots_pending;
        s.flow_control     = c.acking;
        s.unacked_bytes    = c.unacked();
        s.ack_rtt_ms       = c.ack_rtt_ms;
        s.ack_rtt_min_ms   = c.ack_rtt_min_ms;
        s.pace_gap_ms      = c.pace_gap_ms;
        for (const auto& ch : c.counters) {
            if (ch.first == "handshake") {
                continue;
            }
            WsChannelStats cs;
            cs.name            = ch.first;
            cs.replaceable     = ch.second.replaceable;
            cs.queued          = ch.second.queued;
            cs.sent            = ch.second.sent;
            cs.replaced        = ch.second.replaced;
            cs.refused         = ch.second.refused;
            cs.dropped         = ch.second.dropped;
            cs.bytes           = ch.second.bytes;
            cs.last_latency_ms = ch.second.last_latency_ms;
            cs.last_acked_ms   = ch.second.last_acked_ms;
            s.channels.push_back(std::move(cs));
        }
        out.push_back(std::move(s));
    }
    return out;
}

HttpServerStats HttpServer::stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    HttpServerStats             s = impl_->stats;
    s.flow_control_clients        = 0;
    for (const auto& kv : impl_->clients) {
        if (kv.second.webSocketOpen() && kv.second.acking) {
            ++s.flow_control_clients;
        }
    }
    return s;
}

} // namespace navigatr
