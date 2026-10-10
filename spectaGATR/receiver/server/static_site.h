// static_site.h
// The HTTP side of the browser feed: web/ at "/" (index.html by default), field definitions at
// /fields/<name>.json, and /api/fields listing them. Read from disk per request, so an edited file
// shows on reload. Paths that could leave the folders (.., backslashes, drive letters, hidden
// names) are refused.

#pragma once
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "receiver/feed/feed_json.h"
#include "receiver/server/http_server.h"

namespace spectagatr::receiver::server
{

using Path = std::filesystem::path;

struct SiteConfig {
    Path        web_dir;
    Path        fields_dir;
    std::string default_field = "override";
};

HttpResponse serveSite(const SiteConfig& site, const HttpRequest& request);

// Names of the *.json files in dir, without the extension, sorted. Empty when dir is missing.
feed::FieldNames listFields(const Path& dir);

// A URL path ("/a/b.js") as a relative path inside a folder, or none when it could leave it.
std::optional<Path> safeRelativePath(std::string_view url_path);

// By extension: html, js, mjs, css, json, png, svg and a few more; else octet-stream.
std::string mimeTypeOf(const Path& file);

} // namespace spectagatr::receiver::server
