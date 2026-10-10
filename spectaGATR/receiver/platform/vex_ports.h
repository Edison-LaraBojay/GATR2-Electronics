// vex_ports.h
// VEX roles from USB facts only, never from port names or COM order. Brain 2888:0501: interface
// 0 system, 2 user. Controller 2888:0503: its one port. Anything else is kUnknown, never picked.

#pragma once
#include <optional>
#include <string>
#include <vector>

#include "receiver/platform/usb_ports.h"

namespace spectagatr::receiver::platform
{

inline constexpr UsbId              kVexVendorId           = 0x2888;
inline constexpr UsbId              kV5BrainProductId      = 0x0501;
inline constexpr UsbId              kV5ControllerProductId = 0x0503;
inline constexpr UsbInterfaceNumber kBrainSystemInterface  = 0;
inline constexpr UsbInterfaceNumber kBrainUserInterface    = 2;

enum class VexRole : std::uint8_t { kBrainSystem, kBrainUser, kController, kUnknown };

struct VexPort {
    UsbSerialPort port;
    VexRole       role = VexRole::kUnknown;
    std::string   identity; // one per physical device, shared by its ports
};
using VexPorts = std::vector<VexPort>;

VexRole vexRoleOf(const UsbSerialPort& port);

// vex:<pid>:<serial>, else vex:<pid>:<product suffix> ("V5 Brain - A9EC2700"), else
// vex:<pid>@<location>, else vex:<pid>@<path>.
std::string vexIdentityOf(const UsbSerialPort& port);

// VEX vendor ports only, in listing order.
VexPorts classifyVexPorts(const UsbSerialPorts& ports);

// Exactly one port with this role (and identity when given); refuses kUnknown, none and several,
// with the candidates in err.
[[nodiscard]] std::optional<VexPort> pickVexPort(const VexPorts& ports, VexRole role,
                                                 const std::optional<std::string>& identity,
                                                 std::string&                      err);

// "COM7 (brain_user, vex:0501:A9EC2700)"
std::string describeVexPort(const VexPort& port);

const char* vexRoleName(VexRole role);

} // namespace spectagatr::receiver::platform
