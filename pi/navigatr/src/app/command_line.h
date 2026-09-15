#pragma once

#include <string>

#include "runtime/build_options.h"

namespace navigatr
{

struct CommandLine {
    std::string config_path;
    BuildOptions build;
    long max_cycles = -1;
    long inspect_port = 0;
    bool inline_mode = false;
    bool help = false;
};

bool parseCommandLine(int argc, const char* const* argv, const std::string& default_config,
                      CommandLine& out, std::string& err);

} // namespace navigatr
