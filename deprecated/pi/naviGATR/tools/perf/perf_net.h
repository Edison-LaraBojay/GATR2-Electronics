// perf_net.h
// Loopback client sockets for the perf tools: one-shot HTTP requests and a
// WebSocket client that can read at a throttled rate. Blocking connect,
// select-bounded reads, no TLS.

#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace navigatr
{
namespace perf
{

// steady_clock microseconds; callers that need the Pi host clock pass their own.
using ClockUs = int64_t (*)();
int64_t steadyUs();

// Winsock setup on Windows, nothing elsewhere. Call once before any socket.
void initSockets();

struct HttpResult {
    int         status = 0;
    std::string body;
};

// method/path on host:port with Connection: close. False on no reply.
bool httpRequest(const std::string& host, int port, const std::string& method,
                 const std::string& path, HttpResult& out, int timeout_ms = 10000);

struct WsMessage {
    int         opcode = -1;   // 1 text, 2 binary, 8 close, 9 ping, 10 pong
    std::string payload;
    int64_t     recv_us = 0;   // clock time of the read that completed it
};

class WsClient
{
public:
    explicit WsClient(ClockUs clock = &steadyUs) : clock_(clock) {}
    ~WsClient();
    WsClient(const WsClient&)            = delete;
    WsClient& operator=(const WsClient&) = delete;

    // recv_buffer_bytes > 0 shrinks SO_RCVBUF so a slow reader backs up the
    // server instead of the kernel.
    bool connect(const std::string& host, int port, const std::string& path,
                 int recv_buffer_bytes, std::string& err);

    // One read of at most max_bytes, waiting up to timeout_ms. Bytes read,
    // 0 on timeout, -1 once closed.
    long readSome(int timeout_ms, std::size_t max_bytes = 256 * 1024);

    // The next complete frame already buffered.
    bool pop(WsMessage& out);

    bool sendText(const std::string& text);
    bool sendPing(const std::string& payload);
    void close();

    bool        closed() const { return closed_; }
    std::size_t buffered() const { return buffer_.size() - offset_; }
    uint64_t    bytesRead() const { return bytes_read_; }

private:
    bool sendFrame(uint8_t opcode, const std::string& payload);
    bool sendAll(const std::string& bytes);

    ClockUs     clock_;
    intptr_t    sock_ = -1;
    bool        closed_ = false;
    std::string       buffer_;
    std::size_t       offset_ = 0;
    std::vector<char> scratch_;
    int64_t     last_read_us_ = 0;
    uint64_t    bytes_read_ = 0;
    uint32_t    mask_seed_ = 0x9e3779b9u;
};

} // namespace perf
} // namespace navigatr
