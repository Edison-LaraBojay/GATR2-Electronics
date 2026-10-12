// inspection_test_client.h
// Test-side HTTP and WebSocket client for the inspection suites: blocking
// sockets with timeouts, a minimal RFC 6455 client (masked frames out,
// unmasked unfragmented frames in) and flat JSON pickers. Deliberately not
// a library: enough to drive the server from a test.

#pragma once

#ifdef _WIN32
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
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include <gtest/gtest.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

namespace inspection_client
{

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

#ifdef _WIN32
using Sock = SOCKET;
const Sock kBadSock = INVALID_SOCKET;
inline void closeSock(Sock s) { closesocket(s); }
struct SocketsInit {
    SocketsInit() {
        WSADATA d;
        WSAStartup(MAKEWORD(2, 2), &d);
    }
    ~SocketsInit() { WSACleanup(); }
};
#else
using Sock = int;
const Sock  kBadSock = -1;
inline void closeSock(Sock s) { ::close(s); }
struct SocketsInit {};
#endif

inline void socketsInit() {
    static SocketsInit init;
    (void)init;
}

class TestConnection
{
public:
    ~TestConnection() { close(); }

    // A small receive buffer makes the kernel stop absorbing quickly, so a
    // client that never reads stalls the server within the test budget.
    bool connect(int port, int recv_buffer_bytes = 0) {
        socketsInit();
        sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == kBadSock) {
            return false;
        }
        if (recv_buffer_bytes > 0) {
            setsockopt(sock_, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<const char*>(&recv_buffer_bytes),
                       sizeof(recv_buffer_bytes));
        }
        sockaddr_in a;
        std::memset(&a, 0, sizeof(a));
        a.sin_family = AF_INET;
        a.sin_port   = htons(static_cast<uint16_t>(port));
        inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
        if (::connect(sock_, reinterpret_cast<const sockaddr*>(&a), sizeof(a)) != 0) {
            close();
            return false;
        }
        const int one = 1;
        setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one),
                   sizeof(one));
        return true;
    }

    bool sendAll(const std::string& bytes) {
        std::size_t off = 0;
        while (off < bytes.size()) {
            const auto n = ::send(sock_, bytes.data() + off,
                                  static_cast<int>(bytes.size() - off), 0);
            if (n <= 0) {
                return false;
            }
            off += static_cast<std::size_t>(n);
        }
        return true;
    }

    // Appends what arrives within timeout, at most max_bytes. False on
    // timeout or close.
    bool readSome(Ms timeout, std::size_t max_bytes = 64 * 1024) {
        if (sock_ == kBadSock || closed) {
            return false;
        }
        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(sock_, &rd);
        timeval tv;
        tv.tv_sec  = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
#ifdef _WIN32
        const int r = select(0, &rd, nullptr, nullptr, &tv);
#else
        const int r = select(sock_ + 1, &rd, nullptr, nullptr, &tv);
#endif
        if (r <= 0) {
            return false;
        }
        char       buf[64 * 1024];
        const int  want = static_cast<int>(std::min<std::size_t>(sizeof(buf), max_bytes));
        const auto n    = ::recv(sock_, buf, want, 0);
        if (n <= 0) {
            closed = true;
            return false;
        }
        buffer.append(buf, static_cast<std::size_t>(n));
        return true;
    }

    void close() {
        if (sock_ != kBadSock) {
            closeSock(sock_);
            sock_ = kBadSock;
        }
    }

    std::string buffer;
    bool        closed = false;

private:
    Sock sock_ = kBadSock;
};

struct HttpReply {
    bool                               ok     = false;
    int                                status = 0;
    std::map<std::string, std::string> headers;   // lower-case names
    std::string                        body;
};

inline HttpReply httpRequest(int port, const std::string& method, const std::string& path,
                             const std::string& body = std::string()) {
    HttpReply      reply;
    TestConnection c;
    if (!c.connect(port)) {
        return reply;
    }
    std::string req = method + " " + path + " HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n";
    if (!body.empty() || method == "POST") {
        req += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    }
    req += "\r\n" + body;
    c.sendAll(req);
    const auto deadline = Clock::now() + Ms(10000);
    while (!c.closed && Clock::now() < deadline) {
        c.readSome(Ms(200));
    }
    const std::size_t head_end = c.buffer.find("\r\n\r\n");
    if (head_end == std::string::npos) {
        return reply;
    }
    const std::string head = c.buffer.substr(0, head_end);
    reply.body             = c.buffer.substr(head_end + 4);
    std::size_t pos        = head.find("\r\n");
    std::string line       = head.substr(0, pos);
    if (line.size() > 12) {
        reply.status = std::atoi(line.substr(9, 3).c_str());
    }
    while (pos != std::string::npos) {
        const std::size_t next  = head.find("\r\n", pos + 2);
        const std::string hline = head.substr(pos + 2, next == std::string::npos
                                                          ? std::string::npos
                                                          : next - pos - 2);
        const std::size_t colon = hline.find(':');
        if (colon != std::string::npos) {
            std::string name = hline.substr(0, colon);
            for (char& ch : name) {
                ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            }
            std::size_t v = colon + 1;
            while (v < hline.size() && hline[v] == ' ') {
                ++v;
            }
            reply.headers[name] = hline.substr(v);
        }
        pos = next;
    }
    reply.ok = reply.status != 0;
    return reply;
}

struct WsMessage {
    int         opcode = -1;
    std::string payload;
};

class WsClient
{
public:
    bool connect(int port, int recv_buffer_bytes = 0) {
        if (!conn_.connect(port, recv_buffer_bytes)) {
            return false;
        }
        // the RFC 6455 sample key: the accept value is known
        conn_.sendAll("GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n"
                      "Connection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                      "Sec-WebSocket-Version: 13\r\n\r\n");
        const auto deadline = Clock::now() + Ms(5000);
        while (conn_.buffer.find("\r\n\r\n") == std::string::npos && !conn_.closed &&
               Clock::now() < deadline) {
            conn_.readSome(Ms(200));
        }
        const std::size_t end = conn_.buffer.find("\r\n\r\n");
        if (end == std::string::npos) {
            return false;
        }
        const std::string head = conn_.buffer.substr(0, end);
        conn_.buffer.erase(0, end + 4);
        if (head.find("HTTP/1.1 101") != 0) {
            return false;
        }
        std::string lower = head;
        for (char& ch : lower) {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        const std::size_t accept = lower.find("sec-websocket-accept:");
        if (accept == std::string::npos) {
            return false;
        }
        return head.find("s3pPLMBiTxaQ9kYGzzhZRbK+xOo=", accept) != std::string::npos;
    }

    // The next complete message within timeout. read_chunk bounds each
    // recv, to emulate a slow reader. Server pings are answered here, when
    // the reader reaches them, as a browser's network stack does, and are
    // not returned unless answer_pings is off.
    bool next(WsMessage& out, Ms timeout, std::size_t read_chunk = 64 * 1024) {
        const auto deadline = Clock::now() + timeout;
        for (;;) {
            if (parseFrame(out)) {
                if (out.opcode == 0x9 && answer_pings) {
                    sendFrame(0xA, out.payload, true);
                    ++pings_answered;
                    continue;
                }
                return true;
            }
            if (conn_.closed) {
                return false;
            }
            const auto now = Clock::now();
            if (now >= deadline) {
                return false;
            }
            const auto remaining = std::chrono::duration_cast<Ms>(deadline - now);
            conn_.readSome(std::min(remaining, Ms(100)), read_chunk);
        }
    }

    void sendText(const std::string& text) { sendFrame(0x1, text, true); }
    void sendClose() { sendFrame(0x8, std::string("\x03\xe8", 2), true); }
    // A protocol violation: client frames must be masked.
    void sendUnmaskedText(const std::string& text) { sendFrame(0x1, text, false); }
    void close() { conn_.close(); }
    bool closed() const { return conn_.closed; }
    std::size_t buffered() const { return conn_.buffer.size(); }

    // A frame that broke the rules: set by parseFrame, checked by tests.
    bool bad_frame = false;
    // Off: pings reach the caller unanswered, a client without flow control.
    bool answer_pings   = true;
    int  pings_answered = 0;

private:
    void sendFrame(uint8_t opcode, const std::string& payload, bool masked) {
        std::string f;
        f.push_back(static_cast<char>(0x80 | opcode));
        const std::size_t len      = payload.size();
        const uint8_t     mask_bit = masked ? 0x80 : 0x00;
        if (len < 126) {
            f.push_back(static_cast<char>(mask_bit | len));
        } else {
            f.push_back(static_cast<char>(mask_bit | 126));
            f.push_back(static_cast<char>(len >> 8));
            f.push_back(static_cast<char>(len & 0xff));
        }
        if (!masked) {
            f += payload;
            conn_.sendAll(f);
            return;
        }
        const uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
        f.append(reinterpret_cast<const char*>(mask), 4);
        for (std::size_t i = 0; i < len; ++i) {
            f.push_back(static_cast<char>(static_cast<uint8_t>(payload[i]) ^ mask[i & 3]));
        }
        conn_.sendAll(f);
    }

    bool parseFrame(WsMessage& out) {
        std::string& b = conn_.buffer;
        if (b.size() < 2) {
            return false;
        }
        const uint8_t b0  = static_cast<uint8_t>(b[0]);
        const uint8_t b1  = static_cast<uint8_t>(b[1]);
        uint64_t      len = b1 & 0x7f;
        std::size_t   pos = 2;
        if (len == 126) {
            if (b.size() < 4) {
                return false;
            }
            len = (static_cast<uint64_t>(static_cast<uint8_t>(b[2])) << 8) |
                  static_cast<uint8_t>(b[3]);
            pos = 4;
        } else if (len == 127) {
            if (b.size() < 10) {
                return false;
            }
            len = 0;
            for (int i = 0; i < 8; ++i) {
                len = (len << 8) | static_cast<uint8_t>(b[2 + i]);
            }
            pos = 10;
        }
        const int opcode = b0 & 0x0f;
        if ((b1 & 0x80) != 0 || (b0 & 0x80) == 0 || (b0 & 0x70) != 0 ||
            (opcode != 0x1 && opcode != 0x2 && opcode != 0x8 && opcode != 0x9 &&
             opcode != 0xA)) {
            bad_frame = true;
        }
        EXPECT_EQ(b1 & 0x80, 0) << "server frames must not be masked";
        EXPECT_NE(b0 & 0x80, 0) << "server frames must not be fragmented";
        const std::size_t total = pos + static_cast<std::size_t>(len);
        if (b.size() < total) {
            return false;
        }
        out.opcode  = opcode;
        out.payload = b.substr(pos, static_cast<std::size_t>(len));
        b.erase(0, total);
        return true;
    }

    TestConnection conn_;
};

// ---- json picking -------------------------------------------------------------

// Value of the first "key":"..." at or after from; empty when absent.
inline std::string jsonString(const std::string& json, const std::string& key,
                              std::size_t from = 0) {
    const std::string needle = "\"" + key + "\":\"";
    const std::size_t pos    = json.find(needle, from);
    if (pos == std::string::npos) {
        return {};
    }
    const std::size_t start = pos + needle.size();
    const std::size_t end   = json.find('"', start);
    return end == std::string::npos ? std::string() : json.substr(start, end - start);
}

inline bool jsonNumber(const std::string& json, const std::string& key, long long& out,
                       std::size_t from = 0) {
    const std::string needle = "\"" + key + "\":";
    const std::size_t pos    = json.find(needle, from);
    if (pos == std::string::npos) {
        return false;
    }
    const char* p = json.c_str() + pos + needle.size();
    if (*p != '-' && (*p < '0' || *p > '9')) {
        return false;
    }
    out = std::atoll(p);
    return true;
}

inline bool jsonDouble(const std::string& json, const std::string& key, double& out,
                       std::size_t from = 0) {
    const std::string needle = "\"" + key + "\":";
    const std::size_t pos    = json.find(needle, from);
    if (pos == std::string::npos) {
        return false;
    }
    const char* p   = json.c_str() + pos + needle.size();
    char*       end = nullptr;
    out             = std::strtod(p, &end);
    return end != p;
}

inline bool jsonBool(const std::string& json, const std::string& key, std::size_t from = 0) {
    return json.find("\"" + key + "\":true", from) != std::string::npos;
}

inline std::string messageType(const std::string& json) { return jsonString(json, "type"); }

} // namespace inspection_client
