// browser.h
// Opens the viewer in the default browser: ShellExecuteW, open, or xdg-open. The caller prints
// the URL either way.

#pragma once
#include <string>
#include <string_view>

namespace spectagatr::receiver::platform
{

// Linux launch conditions.
struct DesktopSession {
    bool display = false; // DISPLAY or WAYLAND_DISPLAY set
    bool root    = false;
};

[[nodiscard]] bool openBrowser(const std::string& url, std::string& err);

// http:// or https:// with no spaces, quotes or control characters.
bool browserUrlAllowed(std::string_view url);

// Only inside a desktop session and never as root, as xdg-open asks.
[[nodiscard]] bool linuxLaunchAllowed(const DesktopSession& session, std::string& err);

} // namespace spectagatr::receiver::platform
