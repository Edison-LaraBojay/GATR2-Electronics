// serial_source.cpp

#include "receiver/sources/serial_source.h"

#include <optional>
#include <string>
#include <utility>

namespace spectagatr::receiver::sources
{

namespace
{

void report(const SourceHooks& hooks, SourceKind src, PortState state, std::string port,
            std::string detail, const TransportStats& stats) {
    SourceStatus status;
    status.src    = src;
    status.state  = state;
    status.at_us  = hooks.clock();
    status.port   = std::move(port);
    status.detail = std::move(detail);
    status.stats  = stats;
    hooks.on_status(status);
}

} // namespace

void runSerialSource(const SerialSourceConfig& config, const SourceHooks& hooks,
                     platform::StopSignal& stop) {
    TransportStats stats;
    SourceKind     last_kind = config.searching_kind;
    while (!stop.requested()) {
        std::string         err;
        std::optional<Port> port = hooks.open_port(err);
        if (!port) {
            report(hooks, config.searching_kind, PortState::kSearching, "", err, stats);
        } else {
            last_kind = port->kind;
            const std::optional<CloseReason> closed =
                port->kind == SourceKind::kController
                    ? runControllerSession(*port, config.controller, hooks, stop, stats)
                    : runBrainUsbSession(*port, config.brain, hooks, stop, stats);
            const std::string path = port->path;
            port.reset();
            if (!closed) {
                break;
            }
            report(hooks, last_kind, PortState::kClosed, path, *closed, stats);
        }
        if (stop.waitFor(config.reconnect_wait)) {
            break;
        }
    }
    report(hooks, last_kind, PortState::kStopped, "", "", stats);
}

} // namespace spectagatr::receiver::sources
