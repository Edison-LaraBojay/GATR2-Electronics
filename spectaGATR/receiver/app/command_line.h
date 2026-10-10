// command_line.h
// spectagatr's options. Pure: parse and validate, no I/O.

#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "receiver/feed/feed_json.h"
#include "receiver/sim/trajectory.h"

namespace spectagatr::receiver::app
{

enum class SourceChoice : uint8_t { kAuto, kController, kBrain, kSim, kFile };

struct Options {
    SourceChoice               source = SourceChoice::kAuto;
    std::optional<std::string> port; // forces a serial port
    bool                       list_ports = false;

    sim::SimMode sim_mode = sim::SimMode::kFigure8;
    double       rate_hz  = 10.0;
    uint32_t     seed     = 1;

    std::string           replay_path; // "-" is stdin
    std::optional<double> replay_rate_bps;
    bool                  loop = false;

    int         http_port = 8790;
    std::string bind      = "127.0.0.1";
    std::string field     = "override";

    std::optional<std::string> web_dir;
    std::optional<std::string> fields_dir;

    double         stale_ms = 750.0;
    feed::RobotBox robot;
    bool           open_browser = true;
    bool           help         = false;
};

struct ParsedCommandLine {
    std::optional<Options> options; // none: error says why
    std::string            error;
};

// args without the program name.
ParsedCommandLine parseCommandLine(const std::vector<std::string>& args);

std::string usageText();

const char* sourceChoiceName(SourceChoice choice);

} // namespace spectagatr::receiver::app
