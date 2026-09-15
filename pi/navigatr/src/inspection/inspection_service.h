// inspection_service.h
// The optional read-only inspection service: one thread that serves the
// bundled viewer over HTTP on the configured (loopback) address, pushes
// navigatr.inspect/1 snapshot documents over a WebSocket at snapshot_hz,
// and streams JPEG previews of the detection frames the field worker
// published, each preceded by a header naming the exact frame identity.
//
// It reads System snapshots only (shared immutable pointers and
// synchronized lookups), encodes and serializes on its own thread, and
// bounds every client: a browser that cannot keep up has frames and
// snapshots skipped, never queued without limit, and disconnecting it
// changes nothing in estimation. Stopping the service before the System
// keeps the shutdown order explicit; the service holds a reference, not
// ownership.
//
// Client messages (JSON text):
//   {"type":"preview","hz":5,"quality":70,"max_width":640}
//       per-client preview budget, clamped to sane bounds
//
// HTTP routes:
//   GET /                          viewer index
//   GET /app.js /style.css         viewer files
//   GET /vendor/<file>             pinned three.js module and OrbitControls
//   GET /api/hello                 hello document
//   GET /api/snapshot              snapshot document
//   GET /api/frame.jpg[?camera=id] newest preview for one camera
//   GET /api/health                {"ok":true,...}
//   GET /ws                        WebSocket upgrade

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
