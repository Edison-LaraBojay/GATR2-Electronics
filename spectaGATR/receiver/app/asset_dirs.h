// asset_dirs.h
// Where the viewer files (web/) and field definitions (fields/) are: the flag, else next to the
// program (the build copies them there), else the install layout
// <prefix>/bin/spectagatr + <prefix>/share/spectagatr/{web,fields}, else the source tree.

#pragma once
#include <filesystem>
#include <optional>
#include <string>

namespace spectagatr::receiver::app
{

using Path = std::filesystem::path;

struct AssetDirs {
    Path web;
    Path fields;
};

struct AssetSearch {
    std::optional<std::string> web_flag;
    std::optional<std::string> fields_flag;
    Path                       executable_dir; // empty when unknown
    Path                       source_web;     // baked in at configure time
    Path                       source_fields;
};

// The first candidate that is a directory; the last one when none is.
AssetDirs findAssetDirs(const AssetSearch& search);

} // namespace spectagatr::receiver::app
