// http_server.h
// A small single-threaded HTTP/1.1 and WebSocket server for the browser feed: GET for static
// files and JSON, and a server-to-browser WebSocket. One service thread multiplexes every client
// with select(); a sender never waits for a socket.
//
// Per WebSocket client the outgoing side is:
//   - one in-flight frame, written as the socket accepts it, never dropped once started;
//   - a reliable FIFO (hello, a restart pose), bounded by reliable_bytes; overflowing it closes
//     the client, which reconnects and starts again from hello;
//   - one replaceable slot per channel (pose, status): a newer message replaces the unsent older
//     one, so a slow client gets the newest message, never a backlog.
// The next frame is the reliable head, else the full slot with the lowest priority number. A
// feed socket's kernel send buffer is kept small (send_buffer_bytes) so little waits there
// either. A client with unsent data and no write progress for close_after_stalled_ms is closed.
//
// Provenance: adapted from pi/naviGATR/src/inspection/http_server.{h,cpp} (commit 10c568e), this
// repository's own code; the ping-based flow control and pacing were left out. SHA-1 and base64
// for the WebSocket accept key are written here from FIPS 180-4 and RFC 4648: no third-party
// code, no license notice applies.

#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace spectagatr::receiver::server
{

struct HttpRequest {
    std::string                        method;
    std::string                        path;    // percent-decoded, without the query
    std::string                        query;   // after '?', may be empty
    std::map<std::string, std::string> headers; // lower-case names
};

struct HttpResponse {
    int                                status       = 200;
    std::string                        content_type = "text/plain; charset=utf-8";
    std::string                        body;
    std::map<std::string, std::string> headers; // extra headers
};

using HttpHandler = std::function<HttpResponse(const HttpRequest&)>;

struct HttpServerConfig {
    std::string bind                   = "127.0.0.1";
    int         port                   = 8790;       // 0 = ephemeral
    int         max_clients            = 8;          // WebSocket clients; plain requests: 32 more
    std::size_t client_buffer_bytes    = 256 * 1024; // every unsent byte of one feed client
    std::size_t reliable_bytes         = 64 * 1024;  // reliable FIFO of one feed client
    int         close_after_stalled_ms = 5000;
    std::string websocket_path         = "/ws";
    int         send_buffer_bytes      = 16 * 1024; // feed sockets' SO_SNDBUF; 0 = OS default
};

struct HttpServerStats {
    bool     running                 = false;
    int      port                    = 0;
    uint64_t connections_total       = 0;
    uint64_t websocket_clients       = 0; // open now
    uint64_t websocket_total         = 0;
    uint64_t requests                = 0;
    uint64_t bytes_sent              = 0;
    uint64_t messages_replaced       = 0; // replaceable message replaced unsent
    uint64_t messages_refused        = 0; // over client_buffer_bytes
    uint64_t clients_closed_stalled  = 0;
    uint64_t clients_closed_overflow = 0; // reliable FIFO overflow
    uint64_t clients_closed_protocol = 0;
    uint64_t clients_closed_peer     = 0;
    uint64_t refused_connections     = 0;
};

class HttpServer {
  public:
    using ClientId = uint64_t;
    using Frame    = std::shared_ptr<const std::string>; // one complete WebSocket frame

    enum class Enqueue {
        kQueued,        // waiting or already written
        kReplaced,      // queued, and the channel's unsent older message went
        kRefused,       // over the client budget, nothing queued
        kClosed,        // reliable overflow: the client is being closed
        kUnknownClient, // not an open WebSocket client
    };

    // Run on the server thread, outside its lock: they may send.
    struct WebSocketHandlers {
        std::function<void(ClientId)> on_open;
        std::function<void(ClientId)> on_close;
    };

    HttpServer(HttpServerConfig config, HttpHandler handler);
    ~HttpServer();

    HttpServer(const HttpServer&)            = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    // Before start().
    void setWebSocketHandlers(WebSocketHandlers handlers);

    // Binds, listens, starts the service thread. False with err.
    bool start(std::string& err);

    // Closes every client, stops and joins the thread. Idempotent.
    void stop();

    bool running() const;
    int  port() const; // bound port, valid after start()

    // One unmasked, unfragmented text frame; share it between clients.
    static Frame textFrame(std::string_view text);

    // In order. Overflowing reliable_bytes closes the client; one message larger than the budget
    // still goes when nothing waits.
    Enqueue sendReliable(ClientId client, const Frame& frame);

    // Replaces the unsent message of the same channel.
    Enqueue sendLatest(ClientId client, const std::string& channel, int priority,
                       const Frame& frame);

    // Drops the channel's unsent message. The in-flight frame is never touched.
    void clearLatest(ClientId client, const std::string& channel);

    // Unsent bytes of a client, 0 for an unknown one.
    std::size_t queued(ClientId client) const;

    std::vector<ClientId> clients() const;
    HttpServerStats       stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Percent-decoding, exposed for tests. '+' stays '+' in a path.
std::string urlDecode(std::string_view text);

// The Sec-WebSocket-Accept value for a key (RFC 6455 4.2.2).
std::string websocketAccept(const std::string& key);

} // namespace spectagatr::receiver::server
