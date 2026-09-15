// http_server.h
// A small single-threaded HTTP/1.1 and WebSocket server for the inspection
// service: static files and JSON over GET, a text/binary WebSocket for the
// live feed. One service thread multiplexes every client with poll(); no
// request handler or send ever blocks estimation, because nothing here runs
// on a worker thread.
//
// Bounded by construction: at most max_clients connections, at most
// client_buffer_bytes queued per client. A client that cannot drain its
// queue has newer messages refused (the caller counts the drop) and is
// closed after close_after_stalled_ms without progress; the runtime never
// waits for a browser. Loopback bind is the default; anything else is an
// explicit configuration choice.
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
    std::string                        body;

    // One query parameter, or def.
    std::string param(const std::string& name, const std::string& def = "") const;
};

struct HttpResponse {
    int                                status       = 200;
    std::string                        content_type = "text/plain; charset=utf-8";
    std::string                        body;
    std::map<std::string, std::string> headers;   // extra headers
};

using HttpHandler = std::function<HttpResponse(const HttpRequest&)>;

struct HttpServerConfig {
    std::string bind                    = "127.0.0.1";
    int         port                    = 8765;   // 0 = ephemeral
    int         max_clients             = 4;
    std::size_t client_buffer_bytes     = 1024 * 1024;
    int         close_after_stalled_ms  = 10000;
    std::string websocket_path          = "/ws";
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
    uint64_t messages_refused   = 0;   // sendText/sendBinary refused for a full buffer
    uint64_t clients_closed_stalled = 0;
    uint64_t refused_connections = 0;  // over max_clients
};

class HttpServer
{
public:
    using ClientId = uint64_t;

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

    // Queue one message for one WebSocket client. False when the client is
    // unknown or its buffer cannot take the message (nothing is queued).
    bool sendText(ClientId client, const std::string& text);
    bool sendBinary(ClientId client, const std::vector<uint8_t>& bytes);

    // Queue for every open WebSocket client; returns how many accepted it
    // and counts the rest in refused.
    std::size_t broadcastText(const std::string& text, std::size_t& refused);

    // Bytes waiting in a client's outgoing buffer, or 0 for an unknown client.
    std::size_t queued(ClientId client) const;

    std::vector<ClientId> clients() const;

    HttpServerStats stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Percent-decoding and query parsing helpers, exposed for tests.
std::string                        urlDecode(const std::string& s);
std::map<std::string, std::string> parseQuery(const std::string& query);

} // namespace navigatr
