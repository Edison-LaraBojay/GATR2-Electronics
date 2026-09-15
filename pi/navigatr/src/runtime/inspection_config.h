// inspection_config.h
// The optional inspection service, as configured on the System root:
//
//   <Inspection enabled="true" bind="127.0.0.1" port="8765"
//               snapshot_hz="20" preview_hz="5" preview_quality="70"
//               preview_max_width="640" max_clients="4"
//               client_buffer_kb="1024" stall_close_ms="10000" static_root="">
//       <RobotBody length_m="0.45" width_m="0.45" height_m="0.30"
//                  origin_x_m="0" origin_y_m="0"/>          display only
//   </Inspection>
//
// Everything here is read-only instrumentation policy: rates and quality
// budgets for browser clients, the loopback bind, and the cosmetic robot
// body drawn by the viewer. Nothing in it changes an estimate. The default
// bind is loopback so the service is reached through an SSH port forward.
// static_root serves the viewer from a directory during development; when
// empty the files compiled into the binary are served.

#pragma once
#include <string>

#include "config/config_node.h"

namespace navigatr
{

struct RobotBodyVisual {
    double length_m   = 0.45;   // along +x
    double width_m    = 0.45;   // along +y
    double height_m   = 0.30;
    double origin_x_m = 0.0;    // body center relative to the robot origin
    double origin_y_m = 0.0;
};

struct InspectionConfig {
    bool        enabled = false;
    std::string bind    = "127.0.0.1";
    long        port    = 8765;

    double snapshot_hz       = 20.0;   // state snapshots pushed per second
    double preview_hz        = 5.0;    // default camera preview rate per client
    long   preview_quality   = 70;     // JPEG quality 1..100
    long   preview_max_width = 640;    // previews are downscaled to fit
    long   max_clients       = 4;
    long   client_buffer_kb  = 1024;   // per client; a slower client drops frames
    long   stall_close_ms    = 10000;  // a client with queued data and no progress is closed

    std::string static_root;   // empty: embedded viewer

    RobotBodyVisual robot_body;
};

// node may be invalid (no Inspection element): defaults, disabled.
inline bool parseInspectionConfig(const ConfigNode& node, InspectionConfig& out,
                                  std::string& err) {
    out = InspectionConfig{};
    if (!node.valid()) {
        return true;
    }
    if (!node.getBool("enabled", false, out.enabled, err) ||
        !node.getInt("port", out.port, out.port, err) ||
        !node.getDouble("snapshot_hz", out.snapshot_hz, out.snapshot_hz, err) ||
        !node.getDouble("preview_hz", out.preview_hz, out.preview_hz, err) ||
        !node.getInt("preview_quality", out.preview_quality, out.preview_quality, err) ||
        !node.getInt("preview_max_width", out.preview_max_width, out.preview_max_width,
                     err) ||
        !node.getInt("max_clients", out.max_clients, out.max_clients, err) ||
        !node.getInt("client_buffer_kb", out.client_buffer_kb, out.client_buffer_kb, err) ||
        !node.getInt("stall_close_ms", out.stall_close_ms, out.stall_close_ms, err)) {
        return false;
    }
    if (node.hasAttr("bind")) {
        out.bind = node.attr("bind");
    }
    if (node.hasAttr("static_root")) {
        out.static_root = node.attr("static_root");
    }
    if (out.port < 0 || out.port > 65535) {
        err = node.path() + ": port must be 0..65535 (0 = ephemeral, for tests)";
        return false;
    }
    if (out.snapshot_hz <= 0.0 || out.snapshot_hz > 200.0) {
        err = node.path() + ": snapshot_hz must be in (0, 200]";
        return false;
    }
    if (out.preview_hz < 0.0 || out.preview_hz > 60.0) {
        err = node.path() + ": preview_hz must be in [0, 60]";
        return false;
    }
    if (out.preview_quality < 1 || out.preview_quality > 100) {
        err = node.path() + ": preview_quality must be 1..100";
        return false;
    }
    if (out.preview_max_width < 32) {
        err = node.path() + ": preview_max_width must be at least 32";
        return false;
    }
    if (out.max_clients < 1 || out.max_clients > 64) {
        err = node.path() + ": max_clients must be 1..64";
        return false;
    }
    if (out.client_buffer_kb < 64) {
        err = node.path() + ": client_buffer_kb must be at least 64";
        return false;
    }
    if (out.stall_close_ms < 100) {
        err = node.path() + ": stall_close_ms must be at least 100";
        return false;
    }
    for (ConfigNode c = node.child(); c.valid(); c = c.next()) {
        if (std::string(c.name()) != "RobotBody") {
            err = node.path() + " has unknown element " + c.name();
            return false;
        }
        RobotBodyVisual& b = out.robot_body;
        if (!c.getDouble("length_m", b.length_m, b.length_m, err) ||
            !c.getDouble("width_m", b.width_m, b.width_m, err) ||
            !c.getDouble("height_m", b.height_m, b.height_m, err) ||
            !c.getDouble("origin_x_m", b.origin_x_m, b.origin_x_m, err) ||
            !c.getDouble("origin_y_m", b.origin_y_m, b.origin_y_m, err)) {
            return false;
        }
        if (b.length_m <= 0.0 || b.width_m <= 0.0 || b.height_m <= 0.0) {
            err = c.path() + ": body dimensions must be positive";
            return false;
        }
    }
    return true;
}

} // namespace navigatr
