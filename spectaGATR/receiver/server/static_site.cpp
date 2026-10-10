// static_site.cpp

#include "receiver/server/static_site.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iterator>
#include <system_error>

namespace spectagatr::receiver::server
{

namespace
{

constexpr std::string_view kFieldsPrefix  = "/fields/";
constexpr std::string_view kFieldsApi     = "/api/fields";
constexpr std::uintmax_t   kMaxFileBytes  = 32u * 1024u * 1024u;
constexpr std::string_view kJsonExtension = ".json";

std::string lowerCase(std::string text) {
    for (char& c : text) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return text;
}

bool segmentAllowed(std::string_view segment) {
    if (segment.empty() || segment.front() == '.') {
        return false; // empty, ".", ".." and hidden names
    }
    for (const char c : segment) {
        const auto byte = static_cast<unsigned char>(c);
        if (byte < 0x20 || byte == 0x7F || c == '\\' || c == ':' || c == '*' || c == '?' ||
            c == '"' || c == '<' || c == '>' || c == '|') {
            return false;
        }
    }
    return true;
}

HttpResponse status(int code, const char* text) {
    HttpResponse response;
    response.status = code;
    response.body   = text;
    return response;
}

HttpResponse fileResponse(const Path& root, const Path& relative) {
    const Path      file = root / relative;
    std::error_code error;
    if (!std::filesystem::is_regular_file(file, error)) {
        return status(404, "not found\n");
    }
    const std::uintmax_t size = std::filesystem::file_size(file, error);
    if (error || size > kMaxFileBytes) {
        return status(500, "cannot read file\n");
    }
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        return status(500, "cannot read file\n");
    }
    HttpResponse response;
    response.body.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    response.content_type                      = mimeTypeOf(file);
    response.headers["Cache-Control"]          = "no-store";
    response.headers["X-Content-Type-Options"] = "nosniff";
    return response;
}

} // namespace

std::optional<Path> safeRelativePath(std::string_view url_path) {
    if (url_path.empty() || url_path.front() != '/') {
        return std::nullopt;
    }
    url_path.remove_prefix(1);
    Path relative;
    while (true) {
        const std::size_t      slash   = url_path.find('/');
        const std::string_view segment = url_path.substr(0, slash);
        if (!segmentAllowed(segment)) {
            return std::nullopt;
        }
        relative /= Path(std::string(segment));
        if (slash == std::string_view::npos) {
            break;
        }
        url_path.remove_prefix(slash + 1);
    }
    return relative;
}

std::string mimeTypeOf(const Path& file) {
    const std::string extension = lowerCase(file.extension().string());
    if (extension == ".html" || extension == ".htm") {
        return "text/html; charset=utf-8";
    }
    if (extension == ".js" || extension == ".mjs") {
        return "text/javascript; charset=utf-8";
    }
    if (extension == ".css") {
        return "text/css; charset=utf-8";
    }
    if (extension == ".json") {
        return "application/json; charset=utf-8";
    }
    if (extension == ".png") {
        return "image/png";
    }
    if (extension == ".svg") {
        return "image/svg+xml";
    }
    if (extension == ".ico") {
        return "image/x-icon";
    }
    if (extension == ".txt" || extension == ".md") {
        return "text/plain; charset=utf-8";
    }
    return "application/octet-stream";
}

feed::FieldNames listFields(const Path& dir) {
    feed::FieldNames                    names;
    std::error_code                     error;
    std::filesystem::directory_iterator it(dir, error);
    for (; !error && it != std::filesystem::directory_iterator(); it.increment(error)) {
        const Path& file = it->path();
        if (lowerCase(file.extension().string()) != kJsonExtension) {
            continue;
        }
        std::error_code   type_error;
        const std::string stem = file.stem().string();
        if (it->is_regular_file(type_error) && segmentAllowed(stem)) {
            names.push_back(stem);
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

HttpResponse serveSite(const SiteConfig& site, const HttpRequest& request) {
    if (request.path == kFieldsApi) {
        HttpResponse response;
        response.content_type             = "application/json; charset=utf-8";
        response.headers["Cache-Control"] = "no-store";
        response.body = feed::fieldsJson(listFields(site.fields_dir), site.default_field);
        return response;
    }
    if (request.path.compare(0, kFieldsPrefix.size(), kFieldsPrefix) == 0) {
        const std::string_view name = std::string_view(request.path).substr(kFieldsPrefix.size());
        const std::optional<Path> relative = safeRelativePath("/" + std::string(name));
        if (!relative || relative->has_parent_path() ||
            lowerCase(relative->extension().string()) != kJsonExtension) {
            return status(403, "forbidden\n");
        }
        return fileResponse(site.fields_dir, *relative);
    }
    std::string path = request.path;
    if (!path.empty() && path.back() == '/') {
        path += "index.html";
    }
    const std::optional<Path> relative = safeRelativePath(path);
    if (!relative) {
        return status(403, "forbidden\n");
    }
    return fileResponse(site.web_dir, *relative);
}

} // namespace spectagatr::receiver::server
