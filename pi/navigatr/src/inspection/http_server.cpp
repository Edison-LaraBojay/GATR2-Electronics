// http_server.cpp
// One service thread owns every socket: it accepts, reads, parses and
// writes under a short select() timeout so stop() is prompt. Handlers run
// on that thread but outside the client lock, because a handler (or the
// inspection thread) may call back into sendText; the lock only guards the
// client table and the counters. Nothing here ever blocks on a socket:
// what the kernel refuses stays queued, bounded by client_buffer_bytes,
// and a client that drains nothing for close_after_stalled_ms is dropped.
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
#include <cstring>
#include <thread>

namespace navigatr
{

namespace
{

using Clock = std::chrono::steady_clock;

constexpr std::size_t kMaxRequestHead   = 16 * 1024;
constexpr std::size_t kMaxClientMessage = 64 * 1024;
constexpr int         kSelectTimeoutMs  = 20;

// ---- platform shim -------------------------------------------------------

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
constexpr int          kSendFlags     = 0;

void closeSocket(SocketHandle s) { closesocket(s); }
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
    case 500: return "Internal Server Error";
    case 503: return "Service Unavailable";
    }
    return "Status";
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
    s += r.body;
    return s;
}

HttpResponse plainResponse(int status, const std::string& body) {
    HttpResponse r;
    r.status = status;
    r.body   = body;
    return r;
}

// Server frames are never masked.
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

    struct Client {
        SocketHandle sock = kInvalidSocket;
        ClientId     id   = 0;
        Mode         mode = Mode::kHttp;
        std::string  in;
        std::string  out;
        std::size_t  out_offset        = 0;      // bytes of out already written
        bool         awaiting_handler  = false;  // request dispatched, response pending
        bool         close_after_flush = false;
        bool         dead              = false;
        bool         refused           = false;  // over max_clients: answers 503 and goes
        Clock::time_point last_progress;

        std::size_t queued() const { return out.size() - out_offset; }
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

    mutable std::mutex         mutex;   // clients and stats
    std::map<ClientId, Client> clients;
    ClientId                   next_id = 1;
    HttpServerStats            stats;

    bool listen(std::string& err);
    void run();
    void acceptPending(Clock::time_point now);
    void readClient(Client& c, std::vector<Event>& events, Clock::time_point now);
    void parseHttp(Client& c, std::vector<Event>& events, Clock::time_point now);
    void parseWebSocket(Client& c, std::vector<Event>& events, Clock::time_point now);
    bool flush(Client& c, Clock::time_point now);
    void queueBytes(Client& c, const std::string& bytes, Clock::time_point now);
    void reapDead(std::vector<Event>& events);
    void dispatch(std::vector<Event>& events);
    bool sendFrame(ClientId id, uint8_t opcode, const char* data, std::size_t len);
    std::size_t admitted() const;
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
        // over the limit: the request is still read so the 503 reaches the
        // client instead of a reset, and the connection goes right after
        c.refused = admitted() >= static_cast<std::size_t>(std::max(1, config.max_clients));
        if (c.refused) {
            ++stats.refused_connections;
        } else {
            ++stats.connections_total;
        }
        clients.emplace(c.id, std::move(c));
        stats.connections_open = admitted();
    }
}

bool HttpServer::Impl::flush(Client& c, Clock::time_point now) {
    while (c.out_offset < c.out.size()) {
        const std::size_t remaining = c.out.size() - c.out_offset;
        const int chunk = static_cast<int>(std::min<std::size_t>(remaining, 64 * 1024));
        const auto n = ::send(c.sock, c.out.data() + c.out_offset, chunk, kSendFlags);
        if (n > 0) {
            c.out_offset += static_cast<std::size_t>(n);
            stats.bytes_sent += static_cast<uint64_t>(n);
            c.last_progress = now;
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
    if (c.out_offset == c.out.size()) {
        c.out.clear();
        c.out_offset = 0;
    } else if (c.out_offset > 64 * 1024 && c.out_offset > c.out.size() / 2) {
        c.out.erase(0, c.out_offset);
        c.out_offset = 0;
    }
    return true;
}

void HttpServer::Impl::queueBytes(Client& c, const std::string& bytes, Clock::time_point now) {
    if (c.dead) {
        return;
    }
    if (c.queued() == 0) {
        c.last_progress = now;   // the stall clock starts when data waits
    }
    c.out += bytes;
    if (!flush(c, now)) {
        c.dead = true;
    }
}

void HttpServer::Impl::parseHttp(Client& c, std::vector<Event>& events, Clock::time_point now) {
    if (c.awaiting_handler || c.close_after_flush) {
        return;   // one request per connection; anything more is ignored
    }
    HttpRequest req;
    std::size_t consumed = 0;
    if (!parseRequestHead(c.in, req, consumed)) {
        if (c.in.size() > kMaxRequestHead) {
            c.dead = true;
        }
        return;
    }
    c.in.erase(0, consumed);
    ++stats.requests;
    if (c.refused) {
        queueBytes(c, serializeResponse(plainResponse(503, "too many clients\n")), now);
        c.close_after_flush = true;
        return;
    }
    if (req.method.empty()) {
        queueBytes(c, serializeResponse(plainResponse(400, "bad request\n")), now);
        c.close_after_flush = true;
        return;
    }
    const auto upgrade = req.headers.find("upgrade");
    const auto key     = req.headers.find("sec-websocket-key");
    if (req.method == "GET" && req.path == config.websocket_path && upgrade != req.headers.end() &&
        lower(upgrade->second).find("websocket") != std::string::npos &&
        key != req.headers.end()) {
        std::string r = "HTTP/1.1 101 Switching Protocols\r\n"
                        "Upgrade: websocket\r\n"
                        "Connection: Upgrade\r\n"
                        "Sec-WebSocket-Accept: " +
                        websocketAccept(trim(key->second)) + "\r\n\r\n";
        c.mode = Mode::kWebSocket;
        ++stats.websocket_total;
        ++stats.websocket_clients;
        queueBytes(c, r, now);
        events.push_back(Event{Event::kOpen, c.id, {}, {}});
        return;
    }
    if (req.method != "GET") {
        HttpResponse r      = plainResponse(405, "method not allowed\n");
        r.headers["Allow"] = "GET";
        queueBytes(c, serializeResponse(r), now);
        c.close_after_flush = true;
        return;
    }
    c.awaiting_handler = true;
    events.push_back(Event{Event::kHttp, c.id, std::move(req), {}});
}

void HttpServer::Impl::parseWebSocket(Client& c, std::vector<Event>& events,
                                      Clock::time_point now) {
    for (;;) {
        if (c.in.size() < 2) {
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
            c.dead = true;
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
        case 0x8:
            queueBytes(c, websocketFrame(0x8, payload.data(), std::min<std::size_t>(payload.size(), 2)),
                       now);
            c.close_after_flush = true;
            return;
        case 0x9:
            queueBytes(c, websocketFrame(0xA, payload.data(), payload.size()), now);
            break;
        case 0xA:
            break;
        default:
            c.dead = true;
            return;
        }
    }
}

void HttpServer::Impl::readClient(Client& c, std::vector<Event>& events, Clock::time_point now) {
    char buf[16 * 1024];
    for (;;) {
        const auto n = ::recv(c.sock, buf, sizeof(buf), 0);
        if (n > 0) {
            c.in.append(buf, static_cast<std::size_t>(n));
            if (c.in.size() > kMaxRequestHead + kMaxClientMessage) {
                c.dead = true;   // nobody legitimate sends this much to a feed
                return;
            }
            continue;
        }
        if (n < 0 && wouldBlock()) {
            break;
        }
        c.dead = true;   // closed or errored
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
            const std::string           bytes = serializeResponse(r);
            std::lock_guard<std::mutex> lock(mutex);
            const auto                  it = clients.find(e.id);
            if (it != clients.end()) {
                it->second.awaiting_handler  = false;
                it->second.close_after_flush = true;
                queueBytes(it->second, bytes, Clock::now());
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
            std::lock_guard<std::mutex> lock(mutex);
            for (const auto& kv : clients) {
                const Client& c = kv.second;
                FD_SET(c.sock, &rd);
                if (c.queued() > 0) {
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
                if (c.queued() > 0 && n > 0 && FD_ISSET(c.sock, &wr) && !flush(c, now)) {
                    c.dead = true;
                    continue;
                }
                if (c.queued() == 0 && c.close_after_flush) {
                    c.dead = true;
                    continue;
                }
                const bool stalled = now - c.last_progress >=
                                     std::chrono::milliseconds(config.close_after_stalled_ms);
                if (c.queued() > 0 && stalled) {
                    c.dead = true;
                    ++stats.clients_closed_stalled;
                } else if (c.mode == Mode::kHttp && !c.awaiting_handler && stalled) {
                    c.dead = true;   // connected, never asked for anything
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
            kv.second.dead = true;
        }
        reapDead(events);
    }
    dispatch(events);
}

bool HttpServer::Impl::sendFrame(ClientId id, uint8_t opcode, const char* data, std::size_t len) {
    std::lock_guard<std::mutex> lock(mutex);
    const auto                  it = clients.find(id);
    if (it == clients.end() || it->second.mode != Mode::kWebSocket || it->second.dead ||
        it->second.close_after_flush) {
        return false;
    }
    Client& c = it->second;
    if (c.queued() + len + 10 > config.client_buffer_bytes) {
        ++stats.messages_refused;
        return false;
    }
    queueBytes(c, websocketFrame(opcode, data, len), Clock::now());
    return !c.dead;
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

bool HttpServer::sendText(ClientId client, const std::string& text) {
    return impl_->sendFrame(client, 0x1, text.data(), text.size());
}

bool HttpServer::sendBinary(ClientId client, const std::vector<uint8_t>& bytes) {
    return impl_->sendFrame(client, 0x2, reinterpret_cast<const char*>(bytes.data()),
                            bytes.size());
}

std::size_t HttpServer::broadcastText(const std::string& text, std::size_t& refused) {
    refused = 0;
    std::size_t accepted = 0;
    for (ClientId id : clients()) {
        if (sendText(id, text)) {
            ++accepted;
        } else {
            ++refused;
        }
    }
    return accepted;
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
        if (kv.second.mode == Impl::Mode::kWebSocket && !kv.second.dead &&
            !kv.second.close_after_flush) {
            out.push_back(kv.first);
        }
    }
    return out;
}

HttpServerStats HttpServer::stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

} // namespace navigatr
