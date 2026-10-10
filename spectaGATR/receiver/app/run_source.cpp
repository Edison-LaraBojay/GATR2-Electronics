// run_source.cpp

#include "receiver/app/run_source.h"

#include <cstdio>

#include "receiver/platform/usb_ports.h"
#include "receiver/platform/vex_ports.h"
#include "receiver/sim/sim_source.h"
#include "receiver/sources/replay.h"
#include "receiver/sources/serial_source.h"
#include "receiver/sources/vex_port.h"

namespace spectagatr::receiver::app
{

namespace
{

sources::PortChoice portChoiceOf(SourceChoice choice) {
    switch (choice) {
    case SourceChoice::kController:
        return sources::PortChoice::kController;
    case SourceChoice::kBrain:
        return sources::PortChoice::kBrain;
    default:
        return sources::PortChoice::kAuto;
    }
}

sources::SourceKind searchingKindOf(SourceChoice choice) {
    switch (choice) {
    case SourceChoice::kController:
        return sources::SourceKind::kController;
    case SourceChoice::kBrain:
        return sources::SourceKind::kBrainUsb;
    default:
        return sources::SourceKind::kNone;
    }
}

std::string hex4(unsigned value) {
    char text[8];
    std::snprintf(text, sizeof(text), "%04x", value);
    return text;
}

} // namespace

void runSource(const Options& options, sources::SourceHooks hooks, platform::StopSignal& stop) {
    switch (options.source) {
    case SourceChoice::kSim: {
        sim::SimConfig config;
        config.mode    = options.sim_mode;
        config.rate_hz = options.rate_hz;
        config.seed    = options.seed;
        sim::runSimSource(config, hooks, stop);
        return;
    }
    case SourceChoice::kFile: {
        sources::ReplayConfig config;
        config.path = options.replay_path;
        config.loop = options.loop;
        if (options.replay_rate_bps) {
            config.chunk_bytes =
                sources::replayChunkBytes(*options.replay_rate_bps, config.interval);
        }
        sources::runReplaySource(config, hooks, stop);
        return;
    }
    case SourceChoice::kAuto:
    case SourceChoice::kController:
    case SourceChoice::kBrain: {
        sources::SerialSourceConfig config;
        config.searching_kind = searchingKindOf(options.source);
        hooks.open_port = sources::makeVexPortOpener(portChoiceOf(options.source), options.port);
        sources::runSerialSource(config, hooks, stop);
        return;
    }
    }
}

std::string describePorts() {
    const platform::UsbSerialPorts listed = platform::listUsbSerialPorts();
    std::string                    text;
    if (listed.empty()) {
        text += "no USB serial ports found\n";
    }
    for (const platform::UsbSerialPort& port : listed) {
        const platform::VexRole role = platform::vexRoleOf(port);
        text += "  " + port.path + "  ";
        if (port.vendor_id != platform::kVexVendorId) {
            text += "not VEX (" + hex4(port.vendor_id) + ":" + hex4(port.product_id) + ")";
        } else {
            text += std::string(platform::vexRoleName(role)) + "  " + platform::vexIdentityOf(port);
            if (role == platform::VexRole::kBrainSystem) {
                text += "  (never opened: uploads use it)";
            }
        }
        text += "\n";
    }
    std::string                            err;
    const std::optional<sources::PortPick> pick = sources::pickPort(
        platform::classifyVexPorts(listed), sources::PortChoice::kAuto, std::nullopt, err);
    if (pick) {
        text += "auto picks " + pick->path + " (" + sources::sourceKindName(pick->kind) + ")\n";
    } else {
        text += "auto: " + err + "\n";
    }
    return text;
}

} // namespace spectagatr::receiver::app
