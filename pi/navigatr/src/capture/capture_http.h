// capture_http.h
// HTTP handlers for capture control and download (docs/capture.md). The
// inspection service routes /api/capture/... here from its request handler;
// nothing in this file touches the robot or any worker state.
//
//   POST /api/capture/start?pre_s=&post_s=&streams=&requester=
//        streams: comma list of kind names, "default" or "all"
//        -> {"ok":true,"id":"..."} or {"ok":false,"error":"..."}
//   POST /api/capture/cancel?id=
//        -> {"ok":true} or {"ok":false,"error":"..."}
//   GET  /api/capture/status       -> CaptureService::writeStatus
//   GET  /api/capture/<id>.zip     -> application/zip, 404 until ready

#pragma once
#include <string>

#include "inspection/http_server.h"

namespace navigatr
{

class CaptureService;
class System;

// True when path belongs to the capture API (a prefix check only).
bool isCaptureRoute(const std::string& path);

// Fills response and returns true for a capture route; returns false and
// leaves response untouched otherwise. A null service answers every capture
// route as unavailable (503).
bool captureRoute(CaptureService* capture, const HttpRequest& request, HttpResponse& response);
bool captureRoute(System& system, const HttpRequest& request, HttpResponse& response);

} // namespace navigatr
