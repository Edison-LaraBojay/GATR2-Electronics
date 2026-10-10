// test_client.h
// Test-side HTTP and WebSocket client over loopback: blocking sockets with select timeouts, a
// minimal RFC 6455 client (masked frames out, unmasked unfragmented frames in). Enough to drive
// the server from a test; not a library.

#pragma once

#if defined(_WIN32)
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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>

namespace spectagatr::receiver::test_client
{

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

#if defined(_WIN32)
using Sock                = SOCKET;
inline const Sock kNoSock = INVALID_SOCKET;
inline void       closeSock(Sock s) { closesocket(s); }
struct SocketsInit {
    SocketsInit() {
        WSADATA data;
        WSAStartup(MAKEWORD(2, 2), &data);
    }
    ~SocketsInit() { WSACleanup(); }
};
#else
using Sock                = int;
inline const Sock kNoSock = -1;
inline void       closeSock(Sock s) { ::close(s); }
struct SocketsInit {};
#endif

inline void socketsInit() {
    static SocketsInit init;
    (void)init;
}

class Connection {
  public:
    Connection()                             = default;
    Connection(const Connection&)            = delete;
    Connection& operator=(const Connection&) = delete;
    ~Connection() { close(); }

    // A small receive buffer makes the kernel stop absorbing quickly (a client that never reads).
    bool connect(int port, int recv_buffer_bytes = 0) {
        socketsInit();
        sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ == kNoSock) {
            return false;
        }
        if (recv_buffer_bytes > 0) {
            setsockopt(sock_, SOL_SOCKET, SO_RCVBUF,
                       reinterpret_cast<const char*>(&recv_buffer_bytes),
                       sizeof(recv_buffer_bytes));
        }
        sockaddr_in address;
        std::memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port   = htons(static_cast<uint16_t>(port));
        inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
        if (::connect(sock_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
            close();
            return false;
        }
        return true;
    }

    bool sendAll(const std::string& bytes) {
        std::size_t off = 0;
        while (off < bytes.size()) {
            const auto n =
                ::send(sock_, bytes.data() + off, static_cast<int>(bytes.size() - off), 0);
            if (n <= 0) {
                return false;
            }
            off += static_cast<std::size_t>(n);
        }
        return true;
    }

    // Appends what arrives within timeout. False on timeout or close.
    bool readSome(Ms timeout) {
        if (sock_ == kNoSock || closed) {
            return false;
        }
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(sock_, &readable);
        timeval tv;
        tv.tv_sec  = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
#if defined(_WIN32)
        const int ready = select(0, &readable, nullptr, nullptr, &tv);
#else
        const int ready = select(sock_ + 1, &readable, nullptr, nullptr, &tv);
#endif
        if (ready <= 0) {
            return false;
        }
        char       chunk[16 * 1024];
        const auto n = ::recv(sock_, chunk, sizeof(chunk), 0);
        if (n <= 0) {
            closed = true;
            return false;
        }
        buffer.append(chunk, static_cast<std::size_t>(n));
        return true;
    }

    void close() {
        if (sock_ != kNoSock) {
            closeSock(sock_);
            sock_ = kNoSock;
        }
    }

    std::string buffer;
    bool        closed = false;

  private:
    Sock sock_ = kNoSock;
};

struct HttpReply {
    int         status = 0;
    std::string head;
    std::string body;
};

inline HttpReply httpGet(int port, const std::string& path, const std::string& method = "GET") {
    HttpReply  reply;
    Connection c;
    if (!c.connect(port)) {
        return reply;
    }
    c.sendAll(method + " " + path + " HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n");
    const auto deadline = Clock::now() + Ms(5000);
    while (!c.closed && Clock::now() < deadline) {
        c.readSome(Ms(100));
    }
    const std::size_t end = c.buffer.find("\r\n\r\n");
    if (end == std::string::npos || c.buffer.size() < 12) {
        return reply;
    }
    reply.head   = c.buffer.substr(0, end);
    reply.body   = c.buffer.substr(end + 4);
    reply.status = std::atoi(c.buffer.substr(9, 3).c_str());
    return reply;
}

struct WsMessage {
    int         opcode = -1;
    std::string payload;
};

class WsClient {
  public:
    // The RFC 6455 sample key, so the accept value is known.
    bool connect(int port, int recv_buffer_bytes = 0) {
        if (!conn_.connect(port, recv_buffer_bytes)) {
            return false;
        }
        conn_.sendAll("GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n"
                      "Connection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                      "Sec-WebSocket-Version: 13\r\n\r\n");
        const auto deadline = Clock::now() + Ms(5000);
        while (conn_.buffer.find("\r\n\r\n") == std::string::npos && !conn_.closed &&
               Clock::now() < deadline) {
            conn_.readSome(Ms(100));
        }
        const std::size_t end = conn_.buffer.find("\r\n\r\n");
        if (end == std::string::npos) {
            return false;
        }
        head = conn_.buffer.substr(0, end);
        conn_.buffer.erase(0, end + 4);
        return head.rfind("HTTP/1.1 101", 0) == 0;
    }

    // The next complete frame within timeout.
    std::optional<WsMessage> next(Ms timeout) {
        const auto deadline = Clock::now() + timeout;
        while (true) {
            if (std::optional<WsMessage> message = parse()) {
                return message;
            }
            const auto left = std::chrono::duration_cast<Ms>(deadline - Clock::now());
            if (left.count() <= 0 || conn_.closed) {
                return std::nullopt;
            }
            conn_.readSome(std::min(left, Ms(100)));
        }
    }

    // The next text frame whose payload contains needle, skipping others.
    std::optional<std::string> nextContaining(const std::string& needle, Ms timeout) {
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline) {
            const auto left = std::chrono::duration_cast<Ms>(deadline - Clock::now());
            const std::optional<WsMessage> message = next(left);
            if (!message) {
                return std::nullopt;
            }
            if (message->opcode == 1 && message->payload.find(needle) != std::string::npos) {
                return message->payload;
            }
        }
        return std::nullopt;
    }

    bool closed() const { return conn_.closed; }

    std::string head;

  private:
    std::optional<WsMessage> parse() {
        const std::string& b = conn_.buffer;
        if (b.size() < 2) {
            return std::nullopt;
        }
        const auto  b1  = static_cast<uint8_t>(b[1]);
        uint64_t    len = b1 & 0x7F;
        std::size_t pos = 2;
        if (len == 126) {
            if (b.size() < 4) {
                return std::nullopt;
            }
            len = (static_cast<uint64_t>(static_cast<uint8_t>(b[2])) << 8) |
                  static_cast<uint8_t>(b[3]);
            pos = 4;
        } else if (len == 127) {
            if (b.size() < 10) {
                return std::nullopt;
            }
            len = 0;
            for (std::size_t i = 0; i < 8; ++i) {
                len = (len << 8) | static_cast<uint8_t>(b[2 + i]);
            }
            pos = 10;
        }
        if (b.size() < pos + len) {
            return std::nullopt;
        }
        WsMessage message;
        message.opcode  = static_cast<uint8_t>(b[0]) & 0x0F;
        message.payload = b.substr(pos, static_cast<std::size_t>(len));
        conn_.buffer.erase(0, pos + static_cast<std::size_t>(len));
        return message;
    }

    Connection conn_;
};

} // namespace spectagatr::receiver::test_client
