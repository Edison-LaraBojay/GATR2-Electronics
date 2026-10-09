// perf_net.cpp

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
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#include "perf_net.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace navigatr
{
namespace perf
{

namespace
{

#ifdef _WIN32
using Sock          = SOCKET;
const Sock kBadSock = INVALID_SOCKET;
void       closeSock(Sock s) { closesocket(s); }
#else
using Sock          = int;
const Sock kBadSock = -1;
void       closeSock(Sock s) { ::close(s); }
#endif

Sock toSock(intptr_t v) { return static_cast<Sock>(v); }

Sock connectTo(const std::string& host, int port, int recv_buffer_bytes, std::string& err) {
    addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res     = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 ||
        res == nullptr) {
        err = "cannot resolve " + host;
        return kBadSock;
    }
    Sock s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s == kBadSock) {
        freeaddrinfo(res);
        err = "socket failed";
        return kBadSock;
    }
    if (recv_buffer_bytes > 0) {
        setsockopt(s, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&recv_buffer_bytes),
                   sizeof(recv_buffer_bytes));
    }
    const int rc = ::connect(s, res->ai_addr, static_cast<int>(res->ai_addrlen));
    freeaddrinfo(res);
    if (rc != 0) {
        closeSock(s);
        err = "connect " + host + ":" + std::to_string(port) + " failed";
        return kBadSock;
    }
    const int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));
    return s;
}

// 1 readable, 0 timeout, -1 error
int waitReadable(Sock s, int timeout_ms) {
    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(s, &rd);
    timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
#ifdef _WIN32
    const int r = select(0, &rd, nullptr, nullptr, &tv);
#else
    const int r = select(s + 1, &rd, nullptr, nullptr, &tv);
#endif
    return r < 0 ? -1 : (r == 0 ? 0 : 1);
}

bool sendAllSock(Sock s, const std::string& bytes) {
    std::size_t off = 0;
    while (off < bytes.size()) {
        const auto n = ::send(s, bytes.data() + off, static_cast<int>(bytes.size() - off), 0);
        if (n <= 0) {
            return false;
        }
        off += static_cast<std::size_t>(n);
    }
    return true;
}

} // namespace

int64_t steadyUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void initSockets() {
#ifdef _WIN32
    static bool done = false;
    if (!done) {
        WSADATA d;
        WSAStartup(MAKEWORD(2, 2), &d);
        done = true;
    }
#endif
}

bool httpRequest(const std::string& host, int port, const std::string& method,
                 const std::string& path, HttpResult& out, int timeout_ms) {
    out = HttpResult{};
    std::string err;
    const Sock  s = connectTo(host, port, 0, err);
    if (s == kBadSock) {
        return false;
    }
    const std::string req = method + " " + path + " HTTP/1.1\r\nHost: " + host + ":" +
                            std::to_string(port) +
                            "\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    if (!sendAllSock(s, req)) {
        closeSock(s);
        return false;
    }
    std::string buf;
    const auto  deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::vector<char> chunk(64 * 1024);
    while (std::chrono::steady_clock::now() < deadline) {
        if (waitReadable(s, 100) <= 0) {
            continue;
        }
        const auto n = ::recv(s, chunk.data(), static_cast<int>(chunk.size()), 0);
        if (n <= 0) {
            break;
        }
        buf.append(chunk.data(), static_cast<std::size_t>(n));
    }
    closeSock(s);
    const std::size_t head_end = buf.find("\r\n\r\n");
    if (head_end == std::string::npos || buf.size() < 12) {
        return false;
    }
    out.status = std::atoi(buf.substr(9, 3).c_str());
    out.body   = buf.substr(head_end + 4);
    // chunked bodies are not produced by the servers measured here
    return out.status != 0;
}

WsClient::~WsClient() { close(); }

bool WsClient::connect(const std::string& host, int port, const std::string& path,
                       int recv_buffer_bytes, std::string& err) {
    const Sock s = connectTo(host, port, recv_buffer_bytes, err);
    if (s == kBadSock) {
        return false;
    }
    sock_ = static_cast<intptr_t>(s);
    // the RFC 6455 sample key; the accept value is not checked
    const std::string req = "GET " + path + " HTTP/1.1\r\nHost: " + host + ":" +
                            std::to_string(port) +
                            "\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                            "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                            "Sec-WebSocket-Version: 13\r\n\r\n";
    if (!sendAll(req)) {
        err = "handshake send failed";
        close();
        return false;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    std::size_t end     = std::string::npos;
    while (std::chrono::steady_clock::now() < deadline) {
        end = buffer_.find("\r\n\r\n");
        if (end != std::string::npos || closed_) {
            break;
        }
        readSome(100, 4096);
    }
    if (end == std::string::npos) {
        err = "no handshake reply";
        close();
        return false;
    }
    const std::string head = buffer_.substr(0, end);
    buffer_.erase(0, end + 4);
    if (head.compare(0, 12, "HTTP/1.1 101") != 0) {
        err = "handshake refused: " + head.substr(0, head.find("\r\n"));
        close();
        return false;
    }
    return true;
}

long WsClient::readSome(int timeout_ms, std::size_t max_bytes) {
    if (sock_ < 0 || closed_) {
        return -1;
    }
    const int w = waitReadable(toSock(sock_), timeout_ms);
    if (w <= 0) {
        return w < 0 ? -1 : 0;
    }
    scratch_.resize(std::max<std::size_t>(max_bytes, 1));
    const auto n = ::recv(toSock(sock_), scratch_.data(), static_cast<int>(scratch_.size()), 0);
    if (n <= 0) {
        closed_ = true;
        return -1;
    }
    last_read_us_ = clock_();
    if (offset_ > 0 && offset_ > buffer_.size() / 2) {
        buffer_.erase(0, offset_);
        offset_ = 0;
    }
    buffer_.append(scratch_.data(), static_cast<std::size_t>(n));
    bytes_read_ += static_cast<uint64_t>(n);
    return static_cast<long>(n);
}

bool WsClient::pop(WsMessage& out) {
    const std::size_t avail = buffer_.size() - offset_;
    if (avail < 2) {
        return false;
    }
    const uint8_t* b   = reinterpret_cast<const uint8_t*>(buffer_.data() + offset_);
    uint64_t       len = b[1] & 0x7f;
    std::size_t    pos = 2;
    if (len == 126) {
        if (avail < 4) {
            return false;
        }
        len = (static_cast<uint64_t>(b[2]) << 8) | b[3];
        pos = 4;
    } else if (len == 127) {
        if (avail < 10) {
            return false;
        }
        len = 0;
        for (int i = 0; i < 8; ++i) {
            len = (len << 8) | b[2 + i];
        }
        pos = 10;
    }
    if ((b[1] & 0x80) != 0) {
        pos += 4;   // servers do not mask; tolerate it anyway
    }
    if (avail < pos + len) {
        return false;
    }
    out.opcode = b[0] & 0x0f;
    out.payload.assign(buffer_.data() + offset_ + pos, static_cast<std::size_t>(len));
    out.recv_us = last_read_us_;
    offset_ += pos + static_cast<std::size_t>(len);
    if (offset_ == buffer_.size()) {
        buffer_.clear();
        offset_ = 0;
    }
    return true;
}

bool WsClient::sendText(const std::string& text) { return sendFrame(0x1, text); }

bool WsClient::sendPing(const std::string& payload) { return sendFrame(0x9, payload); }

void WsClient::close() {
    if (sock_ >= 0) {
        closeSock(toSock(sock_));
        sock_ = -1;
    }
    closed_ = true;
}

bool WsClient::sendFrame(uint8_t opcode, const std::string& payload) {
    std::string f;
    f.push_back(static_cast<char>(0x80 | opcode));
    const std::size_t len = payload.size();
    if (len < 126) {
        f.push_back(static_cast<char>(0x80 | len));
    } else if (len < 65536) {
        f.push_back(static_cast<char>(0x80 | 126));
        f.push_back(static_cast<char>(len >> 8));
        f.push_back(static_cast<char>(len & 0xff));
    } else {
        f.push_back(static_cast<char>(0x80 | 127));
        for (int i = 7; i >= 0; --i) {
            f.push_back(static_cast<char>((static_cast<uint64_t>(len) >> (8 * i)) & 0xff));
        }
    }
    mask_seed_ = mask_seed_ * 1664525u + 1013904223u;
    const uint8_t mask[4] = {static_cast<uint8_t>(mask_seed_), static_cast<uint8_t>(mask_seed_ >> 8),
                             static_cast<uint8_t>(mask_seed_ >> 16),
                             static_cast<uint8_t>(mask_seed_ >> 24)};
    f.append(reinterpret_cast<const char*>(mask), 4);
    for (std::size_t i = 0; i < len; ++i) {
        f.push_back(static_cast<char>(static_cast<uint8_t>(payload[i]) ^ mask[i & 3]));
    }
    return sendAll(f);
}

bool WsClient::sendAll(const std::string& bytes) {
    if (sock_ < 0) {
        return false;
    }
    return sendAllSock(toSock(sock_), bytes);
}

} // namespace perf
} // namespace navigatr
