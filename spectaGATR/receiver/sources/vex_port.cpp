// vex_port.cpp

#include "receiver/sources/vex_port.h"

#include <cctype>
#include <memory>
#include <utility>

#include "receiver/platform/serial_port.h"
#include "receiver/platform/usb_ports.h"

namespace spectagatr::receiver::sources
{

namespace
{

using SharedSerialPort = std::shared_ptr<platform::SerialPort>;

// COM7, com7 and \\.\COM7 name one Windows port.
std::string comparable(const std::string& path) {
    std::string text = path.rfind("\\\\.\\", 0) == 0 ? path.substr(4) : path;
#if defined(_WIN32)
    for (char& c : text) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
#endif
    return text;
}

std::optional<platform::VexPort> listed(const platform::VexPorts& ports, const DevicePath& path) {
    for (const platform::VexPort& port : ports) {
        if (comparable(port.port.path) == comparable(path)) {
            return port;
        }
    }
    return std::nullopt;
}

std::size_t countRole(const platform::VexPorts& ports, platform::VexRole role) {
    std::size_t count = 0;
    for (const platform::VexPort& port : ports) {
        count += port.role == role ? 1 : 0;
    }
    return count;
}

SourceKind kindOf(PortChoice choice, const std::optional<platform::VexPort>& port) {
    switch (choice) {
    case PortChoice::kController:
        return SourceKind::kController;
    case PortChoice::kBrain:
        return SourceKind::kBrainUsb;
    case PortChoice::kAuto:
        break;
    }
    return port && port->role == platform::VexRole::kController ? SourceKind::kController
                                                                : SourceKind::kBrainUsb;
}

std::optional<PortPick> pickForced(const platform::VexPorts& ports, PortChoice choice,
                                   const DevicePath& path, std::string& err) {
    const std::optional<platform::VexPort> port = listed(ports, path);
    if (port && port->role == platform::VexRole::kBrainSystem) {
        err = path + " is the Brain system port (uploads); give the user port";
        return std::nullopt;
    }
    // The controller source writes CDC2 queries; on a Brain port they would
    // land in the program's stdin.
    if (choice == PortChoice::kController && port && port->role == platform::VexRole::kBrainUser) {
        err = path + " is a Brain user port, not a controller; use --source brain";
        return std::nullopt;
    }
    PortPick pick;
    pick.path = path;
    pick.kind = kindOf(choice, port);
    pick.name = port ? platform::describeVexPort(*port) : path;
    return pick;
}

std::optional<PortPick> pickRole(const platform::VexPorts& ports, platform::VexRole role,
                                 std::string& err) {
    const std::optional<platform::VexPort> port = platform::pickVexPort(ports, role, {}, err);
    if (!port) {
        return std::nullopt;
    }
    PortPick pick;
    pick.path = port->port.path;
    pick.kind =
        role == platform::VexRole::kController ? SourceKind::kController : SourceKind::kBrainUsb;
    pick.name = platform::describeVexPort(*port);
    return pick;
}

Port portOf(SharedSerialPort serial, const PortPick& pick) {
    Port port;
    port.read = [serial](Byte* destination, std::size_t capacity, Milliseconds wait) {
        return serial->read(destination, capacity, wait);
    };
    port.write = [serial](const Byte* source, std::size_t size, Milliseconds budget) {
        return serial->writeAll(source, size, budget);
    };
    port.kind = pick.kind;
    port.path = pick.path;
    port.name = pick.name;
    return port;
}

} // namespace

std::optional<PortPick> pickPort(const platform::VexPorts& ports, PortChoice choice,
                                 const std::optional<DevicePath>& forced, std::string& err) {
    if (forced) {
        return pickForced(ports, choice, *forced, err);
    }
    switch (choice) {
    case PortChoice::kController:
        return pickRole(ports, platform::VexRole::kController, err);
    case PortChoice::kBrain:
        return pickRole(ports, platform::VexRole::kBrainUser, err);
    case PortChoice::kAuto:
        break;
    }
    if (countRole(ports, platform::VexRole::kController) > 0) {
        return pickRole(ports, platform::VexRole::kController, err); // two: refused, listed
    }
    if (countRole(ports, platform::VexRole::kBrainUser) > 0) {
        return pickRole(ports, platform::VexRole::kBrainUser, err);
    }
    err = "no VEX controller or Brain user port found";
    if (!ports.empty()) {
        err += "; VEX ports present:";
        for (const platform::VexPort& port : ports) {
            err += " " + platform::describeVexPort(port);
        }
    }
    return std::nullopt;
}

PortOpener makeVexPortOpener(PortChoice choice, const std::optional<DevicePath>& forced) {
    return [choice, forced](std::string& err) -> std::optional<Port> {
        const platform::VexPorts ports = platform::classifyVexPorts(platform::listUsbSerialPorts());
        const std::optional<PortPick> pick = pickPort(ports, choice, forced, err);
        if (!pick) {
            return std::nullopt;
        }
        std::optional<platform::SerialPort> opened =
            platform::openSerialPort(pick->path, platform::SerialSettings{}, err);
        if (!opened) {
            return std::nullopt;
        }
        return portOf(std::make_shared<platform::SerialPort>(std::move(*opened)), *pick);
    };
}

} // namespace spectagatr::receiver::sources
