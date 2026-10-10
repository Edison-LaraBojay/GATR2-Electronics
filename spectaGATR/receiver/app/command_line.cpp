// command_line.cpp

#include "receiver/app/command_line.h"

#include <cmath>
#include <cstdlib>
#include <string_view>

namespace spectagatr::receiver::app
{

namespace
{

constexpr double kMinStaleMs   = 50.0;
constexpr double kMaxStaleMs   = 60000.0;
constexpr double kMaxRobotM    = 5.0;
constexpr double kMinReplayBps = 1.0;

std::optional<double> parseNumber(const std::string& text) {
    if (text.empty()) {
        return std::nullopt;
    }
    char*        end   = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    if (end != text.c_str() + text.size() || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::optional<long long> parseInteger(const std::string& text) {
    if (text.empty()) {
        return std::nullopt;
    }
    char*           end   = nullptr;
    const long long value = std::strtoll(text.c_str(), &end, 10);
    if (end != text.c_str() + text.size()) {
        return std::nullopt;
    }
    return value;
}

std::optional<SourceChoice> sourceOf(const std::string& name) {
    for (const SourceChoice choice :
         {SourceChoice::kAuto, SourceChoice::kController, SourceChoice::kBrain, SourceChoice::kSim,
          SourceChoice::kFile}) {
        if (name == sourceChoiceName(choice)) {
            return choice;
        }
    }
    return std::nullopt;
}

// front,back,left,right[,height], meters, each in (0, 5].
std::optional<feed::RobotBox> robotOf(const std::string& text) {
    std::vector<double> values;
    std::size_t         start = 0;
    while (start <= text.size()) {
        const std::size_t           comma = text.find(',', start);
        const std::string           part  = text.substr(start, comma - start);
        const std::optional<double> value = parseNumber(part);
        if (!value || *value <= 0.0 || *value > kMaxRobotM) {
            return std::nullopt;
        }
        values.push_back(*value);
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }
    if (values.size() != 4 && values.size() != 5) {
        return std::nullopt;
    }
    feed::RobotBox robot;
    robot.front = values[0];
    robot.back  = values[1];
    robot.left  = values[2];
    robot.right = values[3];
    if (values.size() == 5) {
        robot.height = values[4];
    }
    return robot;
}

bool takesValue(std::string_view flag) {
    for (const std::string_view name :
         {"--source", "--port", "--sim", "--rate", "--seed", "--replay", "--replay-rate-bps",
          "--http-port", "--bind", "--field", "--web", "--fields", "--stale-ms", "--robot"}) {
        if (flag == name) {
            return true;
        }
    }
    return false;
}

// One flag and its value. Empty on success.
std::string apply(Options& options, const std::string& flag, const std::string& value,
                  bool& source_given, bool& sim_given, bool& replay_given) {
    if (flag == "--source") {
        const std::optional<SourceChoice> choice = sourceOf(value);
        if (!choice) {
            return "--source: auto, controller, brain, sim or file, not '" + value + "'";
        }
        options.source = *choice;
        source_given   = true;
    } else if (flag == "--port") {
        options.port = value;
    } else if (flag == "--sim") {
        const std::optional<sim::SimMode> mode = sim::simModeOf(value);
        if (!mode) {
            return "--sim: figure8, circle, square, attitude, restart or dropout, not '" + value +
                   "'";
        }
        options.sim_mode = *mode;
        sim_given        = true;
    } else if (flag == "--rate") {
        const std::optional<double> rate = parseNumber(value);
        if (!rate || *rate < sim::kMinRateHz || *rate > sim::kMaxRateHz) {
            return "--rate: Hz from 1 to 50";
        }
        options.rate_hz = *rate;
    } else if (flag == "--seed") {
        const std::optional<long long> seed = parseInteger(value);
        if (!seed || *seed < 0 || *seed > 0xFFFFFFFFll) {
            return "--seed: 0 to 4294967295";
        }
        options.seed = static_cast<uint32_t>(*seed);
    } else if (flag == "--replay") {
        options.replay_path = value;
        replay_given        = true;
    } else if (flag == "--replay-rate-bps") {
        const std::optional<double> rate = parseNumber(value);
        if (!rate || *rate < kMinReplayBps) {
            return "--replay-rate-bps: bytes per second, at least 1";
        }
        options.replay_rate_bps = *rate;
    } else if (flag == "--http-port") {
        const std::optional<long long> port = parseInteger(value);
        if (!port || *port < 0 || *port > 65535) {
            return "--http-port: 0 to 65535 (0 picks a free one)";
        }
        options.http_port = static_cast<int>(*port);
    } else if (flag == "--bind") {
        options.bind = value;
    } else if (flag == "--field") {
        options.field = value;
    } else if (flag == "--web") {
        options.web_dir = value;
    } else if (flag == "--fields") {
        options.fields_dir = value;
    } else if (flag == "--stale-ms") {
        const std::optional<double> stale = parseNumber(value);
        if (!stale || *stale < kMinStaleMs || *stale > kMaxStaleMs) {
            return "--stale-ms: 50 to 60000";
        }
        options.stale_ms = *stale;
    } else if (flag == "--robot") {
        const std::optional<feed::RobotBox> robot = robotOf(value);
        if (!robot) {
            return "--robot: front,back,left,right[,height] in meters, each above 0 and at most 5";
        }
        options.robot = *robot;
    }
    return {};
}

} // namespace

ParsedCommandLine parseCommandLine(const std::vector<std::string>& args) {
    ParsedCommandLine parsed;
    Options           options;
    bool              source_given = false;
    bool              sim_given    = false;
    bool              replay_given = false;
    for (std::size_t i = 0; i < args.size(); ++i) {
        std::string       flag = args[i];
        std::string       value;
        const std::size_t equals       = flag.find('=');
        const bool        inline_value = flag.rfind("--", 0) == 0 && equals != std::string::npos;
        if (inline_value) {
            value = flag.substr(equals + 1);
            flag  = flag.substr(0, equals);
        }
        if (flag == "--help" || flag == "-h") {
            options.help = true;
            continue;
        }
        if (flag == "--list-ports" || flag == "--no-browser" || flag == "--loop") {
            if (inline_value) {
                parsed.error = flag + " takes no value";
                return parsed;
            }
            options.list_ports   = options.list_ports || flag == "--list-ports";
            options.open_browser = options.open_browser && flag != "--no-browser";
            options.loop         = options.loop || flag == "--loop";
            continue;
        }
        if (!takesValue(flag)) {
            parsed.error = "unknown option '" + args[i] + "'";
            return parsed;
        }
        if (!inline_value) {
            if (i + 1 >= args.size()) {
                parsed.error = flag + " needs a value";
                return parsed;
            }
            value = args[++i];
        }
        const std::string error =
            apply(options, flag, value, source_given, sim_given, replay_given);
        if (!error.empty()) {
            parsed.error = error;
            return parsed;
        }
    }
    // --sim and --replay name their source when --source does not.
    if (sim_given && !source_given) {
        options.source = SourceChoice::kSim;
    }
    if (replay_given && !source_given) {
        options.source = SourceChoice::kFile;
    }
    if (sim_given && options.source != SourceChoice::kSim) {
        parsed.error = "--sim needs --source sim";
    } else if (replay_given && options.source != SourceChoice::kFile) {
        parsed.error = "--replay needs --source file";
    } else if (options.source == SourceChoice::kFile && options.replay_path.empty()) {
        parsed.error = "--source file needs --replay <path> (or - for stdin)";
    } else if (options.port &&
               (options.source == SourceChoice::kSim || options.source == SourceChoice::kFile)) {
        parsed.error = "--port is for the controller and brain sources";
    } else if (options.loop && options.source != SourceChoice::kFile) {
        parsed.error = "--loop is for --source file";
    }
    if (!parsed.error.empty()) {
        return parsed;
    }
    parsed.options = options;
    return parsed;
}

std::string usageText() {
    return "usage: spectagatr [options]\n"
           "  --source auto|controller|brain|sim|file   where poses come from (default auto:\n"
           "                       a V5 controller if one is plugged in, else a Brain user port)\n"
           "  --port NAME          use this serial port (COM7, /dev/ttyACM1) instead of searching\n"
           "  --list-ports         print the VEX ports found and exit\n"
           "  --sim MODE           figure8|circle|square|attitude|restart|dropout (default "
           "figure8)\n"
           "  --rate HZ            sim poses per second, 1 to 50 (default 10)\n"
           "  --seed N             sim session seed (default 1)\n"
           "  --replay PATH        replay raw bytes from a file, or - for stdin (--source file)\n"
           "  --replay-rate-bps N  replay pace in bytes per second (default 12800)\n"
           "  --loop               start the replay file again at its end\n"
           "  --http-port N        viewer port (default 8790)\n"
           "  --bind ADDRESS       viewer address (default 127.0.0.1)\n"
           "  --field NAME         field shown first (default override)\n"
           "  --web DIR            viewer files (default: next to the program)\n"
           "  --fields DIR         field definitions (default: next to the program)\n"
           "  --stale-ms MS        a pose older than this is stale (default 750)\n"
           "  --robot F,B,L,R[,H]  robot box in meters from the pose point\n"
           "                       (default 0.1685,0.1685,0.1465,0.1465,0.25)\n"
           "  --no-browser         do not open the viewer in a browser\n"
           "  --help               this text\n";
}

const char* sourceChoiceName(SourceChoice choice) {
    switch (choice) {
    case SourceChoice::kAuto:
        return "auto";
    case SourceChoice::kController:
        return "controller";
    case SourceChoice::kBrain:
        return "brain";
    case SourceChoice::kSim:
        return "sim";
    case SourceChoice::kFile:
        return "file";
    }
    return "auto";
}

} // namespace spectagatr::receiver::app
