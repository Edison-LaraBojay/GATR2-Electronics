// capture_http.cpp

#include "capture/capture_http.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>

#include "capture/capture_bundle.h"
#include "capture/capture_recorder.h"
#include "inspection/json_writer.h"
#include "runtime/system.h"

namespace navigatr
{
namespace
{

constexpr const char* kPrefix = "/api/capture/";

HttpResponse json(int status, std::string body) {
    HttpResponse r;
    r.status       = status;
    r.content_type = "application/json";
    r.body         = std::move(body);
    r.headers["Cache-Control"] = "no-store";
    return r;
}

HttpResponse failure(int status, const std::string& error) {
    JsonWriter w;
    w.beginObject();
    w.field("ok", false);
    w.field("error", error);
    w.endObject();
    return json(status, w.take());
}

// Strict: the whole text is one finite number.
bool parseSeconds(const std::string& text, double& out) {
    if (text.empty()) {
        return false;
    }
    errno        = 0;
    char*        end = nullptr;
    const double v   = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(v)) {
        return false;
    }
    out = v;
    return true;
}

// Printable ASCII, bounded: it only labels the capture.
std::string cleanRequester(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (out.size() >= 48) {
            break;
        }
        if (c >= 0x20 && c < 0x7F) {
            out += c;
        }
    }
    return out.empty() ? std::string("http") : out;
}

bool validId(const std::string& id) {
    if (id.empty() || id.size() > 64) {
        return false;
    }
    for (char c : id) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                        (c >= 'A' && c <= 'Z') || c == '-' || c == '_';
        if (!ok) {
            return false;
        }
    }
    return true;
}

HttpResponse start(CaptureService& capture, const HttpRequest& req) {
    if (req.method != "POST") {
        return failure(405, "use POST");
    }
    CaptureRequest request;
    // the configured defaults unless given
    if (const auto* recorder = dynamic_cast<const CaptureRecorder*>(&capture)) {
        request.pre_s  = recorder->config().default_pre_s;
        request.post_s = recorder->config().default_post_s;
    }
    const std::string pre  = req.param("pre_s");
    const std::string post = req.param("post_s");
    if (!pre.empty() && !parseSeconds(pre, request.pre_s)) {
        return failure(400, "pre_s is not a number");
    }
    if (!post.empty() && !parseSeconds(post, request.post_s)) {
        return failure(400, "post_s is not a number");
    }
    std::string bad;
    if (!parseCaptureStreams(req.param("streams"), request.kinds, bad)) {
        return failure(400, "unknown stream '" + bad + "'");
    }
    request.reason    = "manual";
    request.requester = cleanRequester(req.param("requester"));
    std::string id, err;
    if (!capture.start(request, id, err)) {
        return failure(err.rfind("busy", 0) == 0 ? 409 : 400, err);
    }
    JsonWriter w;
    w.beginObject();
    w.field("ok", true);
    w.field("id", id);
    w.endObject();
    return json(200, w.take());
}

HttpResponse cancel(CaptureService& capture, const HttpRequest& req) {
    if (req.method != "POST") {
        return failure(405, "use POST");
    }
    std::string err;
    if (!capture.cancel(req.param("id"), err)) {
        return failure(404, err);
    }
    return json(200, "{\"ok\":true}");
}

} // namespace

bool isCaptureRoute(const std::string& path) {
    return path.rfind(kPrefix, 0) == 0 || path == "/api/capture";
}

bool captureRoute(CaptureService* capture, const HttpRequest& req, HttpResponse& response) {
    if (!isCaptureRoute(req.path)) {
        return false;
    }
    if (capture == nullptr) {
        response = failure(503, "capture is not available");
        return true;
    }
    const std::string rest = req.path.size() > std::string(kPrefix).size()
                                 ? req.path.substr(std::string(kPrefix).size())
                                 : std::string();
    if (rest == "start") {
        response = start(*capture, req);
    } else if (rest == "cancel") {
        response = cancel(*capture, req);
    } else if (rest == "status" || rest.empty()) {
        JsonWriter w;
        capture->writeStatus(w);
        response = json(200, w.take());
    } else if (rest.size() > 4 && rest.compare(rest.size() - 4, 4, ".zip") == 0) {
        const std::string id = rest.substr(0, rest.size() - 4);
        if (!validId(id)) {
            response = failure(400, "bad capture id");
            return true;
        }
        const std::shared_ptr<const std::string> zip = capture->bundle(id);
        if (zip == nullptr) {
            response = failure(404, "capture " + id + " is not ready or no longer kept");
            return true;
        }
        HttpResponse ok;
        ok.content_type = "application/zip";
        ok.shared_body  = zip; // sent as is, never copied per request
        ok.headers["Content-Disposition"] = "attachment; filename=\"gatr2_capture_" + id + ".zip\"";
        ok.headers["Cache-Control"]       = "no-store";
        response                          = std::move(ok);
    } else {
        response = failure(404, "unknown capture route");
    }
    return true;
}

bool captureRoute(System& system, const HttpRequest& req, HttpResponse& response) {
    return captureRoute(system.capture(), req, response);
}

} // namespace navigatr
