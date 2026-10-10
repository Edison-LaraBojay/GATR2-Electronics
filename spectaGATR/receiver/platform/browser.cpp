// browser.cpp

#include "receiver/platform/browser.h"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <objbase.h>
#include <shellapi.h>

#include "receiver/platform/win32_text.h"
#else
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;
#endif

namespace spectagatr::receiver::platform
{
namespace
{

bool startsWith(std::string_view text, std::string_view prefix) {
    return text.substr(0, prefix.size()) == prefix;
}

#if !defined(_WIN32)
bool spawnOpener(const char* opener, const std::string& url, std::string& err) {
    // sh checks the opener exists, starts it in the background and exits at once.
    constexpr int kNotFound = 127;
    std::string   shell     = "sh";
    std::string   flag      = "-c";
    std::string   script =
        "command -v \"$0\" >/dev/null 2>&1 || exit 127; \"$0\" \"$1\" </dev/null >/dev/null 2>&1 &";
    std::string name        = opener;
    std::string target      = url;
    char*       arguments[] = {shell.data(), flag.data(),   script.data(),
                               name.data(),  target.data(), nullptr};
    pid_t       child       = 0;
    const int   spawned = ::posix_spawn(&child, "/bin/sh", nullptr, nullptr, arguments, environ);
    if (spawned != 0) {
        err = std::string("cannot start /bin/sh: ") + std::strerror(spawned);
        return false;
    }
    int status = 0;
    while (::waitpid(child, &status, 0) < 0) {
        const int error = errno;
        if (error != EINTR) {
            err = std::string("waitpid: ") + std::strerror(error);
            return false;
        }
    }
    if (WIFEXITED(status) && WEXITSTATUS(status) == kNotFound) {
        err = std::string(opener) + " not found";
        return false;
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        err = std::string("cannot run ") + opener;
        return false;
    }
    return true;
}
#endif

} // namespace

bool browserUrlAllowed(std::string_view url) {
    if (!startsWith(url, "http://") && !startsWith(url, "https://")) {
        return false;
    }
    for (char c : url) {
        const auto byte = static_cast<unsigned char>(c);
        if (byte <= ' ' || byte == 0x7F || c == '"' || c == '\'' || c == '`' || c == '\\') {
            return false;
        }
    }
    return true;
}

bool linuxLaunchAllowed(const DesktopSession& session, std::string& err) {
    if (session.root) {
        err = "not opening a browser as root";
        return false;
    }
    if (!session.display) {
        err = "no desktop session (DISPLAY and WAYLAND_DISPLAY unset)";
        return false;
    }
    return true;
}

bool openBrowser(const std::string& url, std::string& err) {
    if (!browserUrlAllowed(url)) {
        err = "not an http URL: " + url;
        return false;
    }
#if defined(_WIN32)
    const HRESULT com = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    const HINSTANCE result =
        ShellExecuteW(nullptr, L"open", toWide(url).c_str(), nullptr, nullptr, SW_SHOWNORMAL);
    if (SUCCEEDED(com)) {
        CoUninitialize();
    }
    const auto code = reinterpret_cast<std::intptr_t>(result);
    if (code <= 32) { // above 32 is success
        err = "cannot open a browser (ShellExecute " + std::to_string(code) + ")";
        return false;
    }
    return true;
#elif defined(__APPLE__)
    return spawnOpener("open", url, err);
#else
    DesktopSession session;
    session.display =
        std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
    session.root = ::geteuid() == 0;
    if (!linuxLaunchAllowed(session, err)) {
        return false;
    }
    return spawnOpener("xdg-open", url, err);
#endif
}

} // namespace spectagatr::receiver::platform
