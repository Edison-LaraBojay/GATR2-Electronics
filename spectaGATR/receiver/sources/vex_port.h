// vex_port.h
// The real PortOpener: a VEX port found by USB role, or a forced path. Auto prefers a controller,
// else a Brain user port. The Brain system port is never opened (uploads use it).

#pragma once
#include <cstdint>
#include <optional>
#include <string>

#include "receiver/platform/vex_ports.h"
#include "receiver/sources/source.h"

namespace spectagatr::receiver::sources
{

using DevicePath = std::string;

enum class PortChoice : uint8_t { kAuto, kController, kBrain };

// Pure: which port and kind to open from a listing. err says why not.
struct PortPick {
    DevicePath  path;
    SourceKind  kind = SourceKind::kBrainUsb;
    std::string name;
};
[[nodiscard]] std::optional<PortPick> pickPort(const platform::VexPorts& ports, PortChoice choice,
                                               const std::optional<DevicePath>& forced,
                                               std::string&                     err);

// Lists, picks and opens on every call, so a replug is found again.
PortOpener makeVexPortOpener(PortChoice choice, const std::optional<DevicePath>& forced);

} // namespace spectagatr::receiver::sources
