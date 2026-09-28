// capture_service.h
// Bounded diagnostic capture on the Pi. The service drains the
// DiagnosticsHub on its own thread, keeps a short rolling pre-trigger
// window, and on a trigger (a viewer or HTTP request, or an optional fault
// trigger) keeps recording for a bounded time, then builds a ZIP bundle of
// per-stream CSVs plus metadata. It never changes sensor rates, localization
// or control, and never blocks a producer.
//
// Owned by System, built after the pipeline, stopped before it. The
// inspection service only routes requests to it.

#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "diagnostics/records.h"

namespace navigatr
{

class JsonWriter;
class System;

struct CaptureRequest {
    double      pre_s  = 10.0; // kept from before the trigger, clamped to the configured bound
    double      post_s = 15.0; // recorded after the trigger, clamped to the configured bound
    uint32_t    kinds  = kDiagAllKinds & ~diagBit(DiagKind::kBytes); // streams to keep
    std::string reason = "manual";  // "manual" or an automatic trigger name
    std::string requester;          // who asked, e.g. "viewer 3" or "http"
};

class CaptureService
{
public:
    virtual ~CaptureService() = default;

    // Starts a capture. False with err when one is already recording, the
    // request is out of range, or capture is unavailable.
    virtual bool start(const CaptureRequest& request, std::string& id, std::string& err) = 0;
    virtual bool cancel(const std::string& id, std::string& err) = 0;

    // One JSON object: {"available":bool, "state":..., "active":..., "captures":[...],
    // "limits":{...}, "auto":{...}}. The exact schema is in docs/inspection.md.
    virtual void writeStatus(JsonWriter& w) const = 0;

    // Changes whenever writeStatus would say something new.
    virtual uint64_t version() const = 0;

    // The finished ZIP bundle of a capture, or null while it is not ready.
    virtual std::shared_ptr<const std::string> bundle(const std::string& id) const = 0;

    // Stops the capture thread. Idempotent; called by System before it stops.
    virtual void stop() = 0;
};

// Builds the service for a System from its <Capture> configuration.
std::unique_ptr<CaptureService> makeCaptureService(System& system, std::string& err);

} // namespace navigatr
