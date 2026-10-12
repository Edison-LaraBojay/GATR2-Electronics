// inspection_service.h
// The optional read-only inspection service: one thread that serves the
// bundled viewer over HTTP on the configured (loopback) address and pushes
// the navigatr.inspect/2 live feed over a WebSocket: small frequent state
// messages, slower diagnostics, the trail history, events, capture status,
// Brain telemetry, link instrumentation, and JPEG previews of the detection
// frames the field worker published, each preceded by a header naming the
// exact frame identity.
//
// It reads System snapshots only (shared immutable pointers and
// synchronized lookups), encodes and serializes on its own thread, and
// bounds every client: replaceable messages keep only the newest unsent
// one per channel, reliable ones share a bounded FIFO whose overflow closes
// the client, and disconnecting a browser changes nothing in estimation.
// Stopping the service before the System keeps the shutdown order
// explicit; the service holds a reference, not ownership.
//
// Client messages (JSON text), docs/inspection.md:
//   {"type":"preview","hz":5,"quality":70,"max_width":640}
//   {"type":"subscribe","state_hz":30,"diag":true,"instrumentation":false,
//    "raw":false,"decoded":false,"telemetry":true}     missing fields keep
//   {"type":"ping","id":1,"client_ms":123.4}            answered with pong
//   {"type":"history"}                                  answered with history
//
// HTTP routes:
//   GET  /                          viewer index
//   GET  /app.js /style.css         viewer files
//   GET  /vendor/<file>             pinned three.js module and OrbitControls
//   GET  /api/hello                 hello document
//   GET  /api/snapshot              full snapshot (inspect/1 shape, with trail)
//   GET  /api/frame.jpg[?camera=id] newest preview for one camera
//   GET  /api/health                {"ok":true,...}
//   POST /api/capture/start|cancel  capture control (capture/capture_http.h)
//   GET  /api/capture/status        capture status
//   GET  /api/capture/<id>.zip      finished capture bundle
//   GET  /ws                        WebSocket upgrade

#pragma once
#include <memory>
#include <string>

#include "inspection/inspection_stats.h"
#include "runtime/inspection_config.h"
#include "runtime/system.h"

namespace navigatr
{

class InspectionService
{
public:
    // Builds the route table (embedded viewer, or static_root when set).
    // False with err when static_root is set and unreadable.
    static std::unique_ptr<InspectionService> create(System& system,
                                                     const InspectionConfig& config,
                                                     std::string& err);
    ~InspectionService();

    InspectionService(const InspectionService&)            = delete;
    InspectionService& operator=(const InspectionService&) = delete;

    // Binds and starts the service thread. False with err (port in use,
    // bad bind address).
    bool start(std::string& err);

    // Closes clients, stops and joins the thread. Idempotent.
    void stop();

    bool running() const;
    int  port() const;   // bound port
    std::string url() const;

    InspectionServiceStats stats() const;

private:
    InspectionService() = default;

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace navigatr
