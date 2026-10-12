#include "app/command_line.h"

#include <charconv>
#include <system_error>

namespace navigatr
{

bool parseCommandLine(int argc, const char* const* argv, const std::string& default_config,
                      CommandLine& out, std::string& err) {
    out = CommandLine{};
    out.config_path = default_config;
    err.clear();
    bool selected_config = false;

    const auto selectConfig = [&](const std::string& path) {
        if (path.empty()) {
            err = "configuration filename must not be empty";
            return false;
        }
        if (selected_config) {
            err = "specify only one configuration filename";
            return false;
        }
        selected_config = true;
        out.config_path = path;
        return true;
    };

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            out.help = true;
            continue;
        }
        if (arg == "--inline") {
            out.inline_mode = true;
            continue;
        }
        const auto eq = arg.find('=');
        const std::string option = arg.substr(0, eq);
        if (option == "--config_file" || option == "--config-file" ||
            option == "--replay" || option == "--cycles" || option == "--inspect-port") {
            std::string value;
            if (eq != std::string::npos) {
                value = arg.substr(eq + 1);
            } else if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
                value = argv[++i];
            }
            if (value.empty()) {
                err = option + " requires a value";
                return false;
            }
            if (option == "--config_file" || option == "--config-file") {
                if (!selectConfig(value)) return false;
            } else if (option == "--replay") {
                const auto split = value.find('=');
                if (split == std::string::npos || split == 0 || split + 1 == value.size()) {
                    err = "--replay wants <resource_id>=<capture.bin>";
                    return false;
                }
                out.build.replay[value.substr(0, split)] = value.substr(split + 1);
            } else {
                long number = 0;
                const auto parsed = std::from_chars(value.data(), value.data() + value.size(), number);
                if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size()) {
                    err = option + " requires an integer";
                    return false;
                }
                if (option == "--cycles") {
                    if (number < -1) {
                        err = "--cycles wants a nonnegative count or -1 for unlimited";
                        return false;
                    }
                    out.max_cycles = number;
                } else {
                    if (number <= 0 || number > 65535) {
                        err = "--inspect-port wants 1..65535";
                        return false;
                    }
                    out.inspect_port = number;
                }
            }
        } else if (!arg.empty() && arg[0] == '-') {
            err = "unknown option " + arg;
            return false;
        } else if (!selectConfig(arg)) {
            return false;
        }
    }
    return true;
}

} // namespace navigatr
