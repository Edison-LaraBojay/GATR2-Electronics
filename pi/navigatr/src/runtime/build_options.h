// build_options.h
// Command-line policy that changes how a configuration is constructed.

#pragma once
#include <map>
#include <string>

namespace navigatr
{

struct BuildOptions {
    // Declared resource id to capture file path. The id must exist in the
    // configuration; its declared implementation is replaced with a replay
    // link for this run.
    std::map<std::string, std::string> replay;

};

} // namespace navigatr
