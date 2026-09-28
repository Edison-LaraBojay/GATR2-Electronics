// http_server.h
// A small single-threaded HTTP/1.1 and WebSocket server for the inspection
// service: static files and JSON over GET and POST, a text/binary WebSocket
// for the live feed. One service thread multiplexes every client with
// select(); no request handler or send ever blocks estimation, because
// nothing here runs on a worker thread.
//
// Per WebSocket client the outgoing side is:
//   - one in-flight frame, written in pieces as the socket accepts them and
//     never modified or dropped once started;
//   - a reliable FIFO (hello, history, events, capture status, pongs),
//     bounded by reliable_bytes; overflowing it closes that client, which
//     reconnects and recovers full state;
//   - one replaceable slot per channel (state, diag, preview per camera...):
//     a newer message replaces an unsent older one, so a slow client gets
//     the newest state instead of a backlog of old ones.
// When nothing is in flight the next frame is a pending ping, else the
// reliable head, else the replaceable slot with the lowest priority number
// (round robin among equal priorities) that flow control lets go.
//
// Flow control. A frame written to a socket is out of reach: kernels, an
// SSH tunnel or a proxy can hold seconds of it, and a replaceable message
// written too early is a stale message delivered late. So the server pings
// after each WebSocket frame, with the byte position as payload; a
// compliant client (every browser) answers in its network stack, and the
// pong proves everything before that position was read. Once a client has
// answered a ping:
//   - nothing new starts while ack_window_bytes are unconfirmed;
//   - a replaceable frame starts only while little is unconfirmed, and at
//     most two frames of one channel are unconfirmed, so the slot keeps
//     coalescing instead of the network queueing old states;
//   - a large replaceable frame (diag, preview) starts only after a gap of
//     three times the last one's delivery time (write start to its pong),
//     so large frames take at most about a quarter of a slow link and a
//     state waits behind at most one of them.
// A client that never answers keeps the kernel bound only (send buffer),
// with large frames paced by write completion instead of pongs.
//
// Enqueueing never waits for a socket: it may try one non-blocking send
// with a bounded byte budget, the rest goes from the server thread. A
// client with queued data and no progress (bytes written, or a pong) for
// close_after_stalled_ms is closed. Loopback bind is the default.
//
// Windows builds use Winsock so the host suite and the desktop browser
// smoke test exercise the same code as the Pi.

#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace navigatr
{

struct HttpRequest {
    std::string                        method;
    std::string                        path;    // without query
    std::string                        query;   // after '?', may be empty
    std::map<std::string, std::string> headers; // lower-case names
    std::string                        body;    // POST body, bounded

    // One query parameter, or def.
    std::string param(const std::string& name, const std::string& def = "") const;
};

struct HttpResponse {
    int                                status       = 200;
    std::string                        content_type = "text/plain; charset=utf-8";
    std::string                        body;
    // When set, sent instead of body without copying (large bundles).
    std::shared_ptr<const std::string> shared_body;
    std::map<std::string, std::string> headers;   // extra headers
};

using HttpHandler = std::function<HttpResponse(const HttpRequest&)>;

struct HttpServerConfig {
    std::string bind                   = "127.0.0.1";
    int         port                   = 8765;   // 0 = ephemeral
    int         max_clients            = 4;   // WebSocket feed clients; plain requests: 32 more at once
    std::size_t client_buffer_bytes    = 1024 * 1024; // all unsent bytes of one client
    std::size_t reliable_bytes         = 256 * 1024;  // reliable FIFO of one WebSocket client
    int         close_after_stalled_ms = 10000;
    std::string websocket_path         = "/ws";
    // WebSocket clients: SO_SNDBUF, and TCP_NOTSENT_LOWAT where the OS has
    // it (Linux); 0 = OS default. Plain HTTP (downloads) keeps the default.
    int         send_buffer_bytes      = 0;
    std::size_t ack_window_bytes       = 64 * 1024;   // flow control, 0 = off (no pings)
};

// A replaceable WebSocket channel: at most one unsent message per name.
struct WsChannel {
    std::string name;          // stats key, e.g. "state" or "preview:front_camera"
    int         priority = 0;  // lower goes first
};

struct WsChannelStats {
    std::string name;
    bool        replaceable     = false;
    uint64_t    queued          = 0;  // accepted into the queue or a slot
    uint64_t    sent            = 0;  // frames fully written to the socket
    uint64_t    replaced        = 0;  // unsent, replaced by a newer one
    uint64_t    refused         = 0;  // did not fit client_buffer_bytes
    uint64_t    dropped         = 0;  // unsent slot cleared (unsubscribe, session reset)
    uint64_t    bytes           = 0;  // frame bytes fully written
    double      last_latency_ms = -1; // enqueue to last byte written, newest frame
    double      last_acked_ms   = -1; // enqueue to the pong covering it (flow control only)
};

struct WsClientStats {
    uint64_t    id               = 0;
    int64_t     connected_ms     = 0; // since the upgrade
    std::size_t queued_bytes     = 0; // unsent: in-flight rest, reliable, slots
    std::size_t in_flight_bytes  = 0; // rest of the frame being written
    std::size_t reliable_backlog = 0; // messages waiting in the FIFO
    std::size_t reliable_bytes   = 0;
    std::size_t slots_pending    = 0; // replaceable messages waiting
    bool        flow_control     = false; // answered a ping: paced by its pongs
    uint64_t    unacked_bytes    = 0;     // written, not yet confirmed by a pong
    double      ack_rtt_ms       = -1;    // newest ping to its pong, data ahead included
    double      ack_rtt_min_ms   = -1;
    double      pace_gap_ms      = 0;     // current gap between large replaceable frames
    std::vector<WsChannelStats> channels;
};

struct WsCloseRecord {
    uint64_t    id      = 0;
    std::string reason; // stalled, reliable_overflow, protocol, peer, closed, server_stop
    int64_t     host_ms = 0;
};

struct HttpServerStats {
    bool     running            = false;
    int      port               = 0;
    uint64_t connections_total  = 0;
    uint64_t connections_open   = 0;
    uint64_t websocket_clients  = 0;
    uint64_t websocket_total    = 0;
    uint64_t requests           = 0;
    uint64_t bytes_sent         = 0;
    uint64_t messages_refused   = 0;   // replaceable message larger than the client budget
    uint64_t messages_replaced  = 0;   // replaceable message replaced unsent
    uint64_t clients_closed_stalled           = 0;
    uint64_t clients_closed_reliable_overflow = 0;
    uint64_t clients_closed_protocol          = 0;   // bad or oversized client frames
    uint64_t clients_closed_peer              = 0;   // the client closed or reset
    uint64_t refused_connections = 0;  // feed upgrades over max_clients, plain requests over 32
    uint64_t flow_control_clients = 0; // open WebSocket clients that answer pings
    std::vector<WsCloseRecord> recent_closes;   // WebSocket clients, newest last, bounded
};

class HttpServer
{
public:
    using ClientId = uint64_t;
    using Frame    = std::shared_ptr<const std::string>;   // one complete WebSocket frame

    enum class Enqueue {
        kQueued,        // waiting or already written
        kReplaced,      // queued, and an unsent older message of the channel went
        kRefused,       // replaceable message over the client budget, nothing queued
        kClosed,        // reliable overflow: the client is being closed
        kUnknownClient, // not an open WebSocket client
    };

    struct WebSocketHandlers {
        std::function<void(ClientId)>                     onOpen;
        std::function<void(ClientId, const std::string&)> onText;
        std::function<void(ClientId)>                     onClose;
    };

    HttpServer(HttpServerConfig config, HttpHandler handler);
    ~HttpServer();

    HttpServer(const HttpServer&)            = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    void setWebSocketHandlers(WebSocketHandlers handlers);

    // Binds, listens, starts the service thread. False with err.
    bool start(std::string& err);

    // Closes every client, stops and joins the thread. Idempotent.
    void stop();

    bool running() const;
    int  port() const;   // bound port, valid after start()

    // One unmasked, unfragmented server frame; share it between clients.
    static Frame textFrame(const std::string& text);
    static Frame binaryFrame(const uint8_t* data, std::size_t len);

    // Reliable, in order. Overflowing reliable_bytes closes the client; one
    // message larger than the budget still goes when nothing waits.
    Enqueue sendReliable(ClientId client, const Frame& frame, const std::string& channel);
    // Replaceable: replaces the unsent message of the same channel name.
    Enqueue sendLatest(ClientId client, const WsChannel& channel, const Frame& frame);
    // Drops unsent replaceable messages whose channel name starts with
    // prefix ("" = all). The in-flight frame is never touched.
    void clearLatest(ClientId client, const std::string& prefix);

    // Reliable convenience wrappers; false unless queued.
    bool sendText(ClientId client, const std::string& text);
    bool sendBinary(ClientId client, const std::vector<uint8_t>& bytes);

    // Unsent bytes of a client, or 0 for an unknown client.
    std::size_t queued(ClientId client) const;

    std::vector<ClientId>      clients() const;
    std::vector<WsClientStats> clientStats() const;

    HttpServerStats stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Percent-decoding and query parsing helpers, exposed for tests.
std::string                        urlDecode(const std::string& s);
std::map<std::string, std::string> parseQuery(const std::string& query);

} // namespace navigatr
