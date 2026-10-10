// asset_dirs.cpp

#include "receiver/app/asset_dirs.h"

#include <system_error>
#include <vector>

namespace spectagatr::receiver::app
{

namespace
{

Path firstDirectory(const std::optional<std::string>& flag, const Path& executable_dir,
                    const char* name, const Path& source_dir) {
    if (flag) {
        return Path(*flag);
    }
    std::vector<Path> candidates;
    if (!executable_dir.empty()) {
        candidates.push_back(executable_dir / name);
        candidates.push_back(executable_dir.parent_path() / "share" / "spectagatr" / name);
    }
    candidates.push_back(source_dir);
    for (const Path& candidate : candidates) {
        std::error_code error;
        if (std::filesystem::is_directory(candidate, error)) {
            return candidate;
        }
    }
    return candidates.back();
}

} // namespace

AssetDirs findAssetDirs(const AssetSearch& search) {
    AssetDirs dirs;
    dirs.web = firstDirectory(search.web_flag, search.executable_dir, "web", search.source_web);
    dirs.fields =
        firstDirectory(search.fields_flag, search.executable_dir, "fields", search.source_fields);
    return dirs;
}

} // namespace spectagatr::receiver::app
