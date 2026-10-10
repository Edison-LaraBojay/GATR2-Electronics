// vex_ports.cpp

#include "receiver/platform/vex_ports.h"

#include <array>
#include <cstdio>

namespace spectagatr::receiver::platform
{
namespace
{

std::string hex4(UsbId id) {
    std::array<char, 5> text{};
    std::snprintf(text.data(), text.size(), "%04x", static_cast<unsigned>(id));
    return text.data();
}

// "V5 Brain - A9EC2700" gives "A9EC2700".
std::string productSuffix(const std::string& product) {
    const std::size_t dash = product.rfind(" - ");
    if (dash == std::string::npos) {
        return {};
    }
    return product.substr(dash + 3);
}

std::string describeAll(const VexPorts& ports) {
    std::string text;
    for (const VexPort& port : ports) {
        text += (text.empty() ? "" : ", ") + describeVexPort(port);
    }
    return text;
}

} // namespace

VexRole vexRoleOf(const UsbSerialPort& port) {
    if (port.vendor_id != kVexVendorId) {
        return VexRole::kUnknown;
    }
    if (port.product_id == kV5ControllerProductId) {
        return VexRole::kController;
    }
    if (port.product_id != kV5BrainProductId || !port.interface_number) {
        return VexRole::kUnknown;
    }
    switch (*port.interface_number) {
    case kBrainSystemInterface:
        return VexRole::kBrainSystem;
    case kBrainUserInterface:
        return VexRole::kBrainUser;
    default:
        return VexRole::kUnknown;
    }
}

std::string vexIdentityOf(const UsbSerialPort& port) {
    const std::string device = "vex:" + hex4(port.product_id);
    if (!port.serial.empty()) {
        return device + ":" + port.serial;
    }
    const std::string suffix = productSuffix(port.product);
    if (!suffix.empty()) {
        return device + ":" + suffix;
    }
    if (!port.location.empty()) {
        return device + "@" + port.location;
    }
    return device + "@" + port.path;
}

VexPorts classifyVexPorts(const UsbSerialPorts& ports) {
    VexPorts vex;
    for (const UsbSerialPort& port : ports) {
        if (port.vendor_id == kVexVendorId) {
            vex.push_back({port, vexRoleOf(port), vexIdentityOf(port)});
        }
    }
    return vex;
}

std::optional<VexPort> pickVexPort(const VexPorts& ports, VexRole role,
                                   const std::optional<std::string>& identity, std::string& err) {
    if (role == VexRole::kUnknown) {
        err = "a VEX port with an unknown role is never picked";
        return std::nullopt;
    }
    VexPorts matches;
    for (const VexPort& port : ports) {
        if (port.role == role && (!identity || port.identity == *identity)) {
            matches.push_back(port);
        }
    }
    if (matches.size() == 1) {
        return matches.front();
    }
    const std::string wanted =
        std::string(vexRoleName(role)) + (identity ? " (" + *identity + ")" : std::string());
    if (matches.empty()) {
        err = "no VEX " + wanted + " port found";
        if (!ports.empty()) {
            err += "; VEX ports present: " + describeAll(ports);
        }
        return std::nullopt;
    }
    err = std::to_string(matches.size()) + " VEX " + wanted + " ports: " + describeAll(matches) +
          "; choose one with --port";
    return std::nullopt;
}

std::string describeVexPort(const VexPort& port) {
    return port.port.path + " (" + vexRoleName(port.role) + ", " + port.identity + ")";
}

const char* vexRoleName(VexRole role) {
    switch (role) {
    case VexRole::kBrainSystem:
        return "brain_system";
    case VexRole::kBrainUser:
        return "brain_user";
    case VexRole::kController:
        return "controller";
    case VexRole::kUnknown:
        return "unknown";
    }
    return "unknown";
}

} // namespace spectagatr::receiver::platform
