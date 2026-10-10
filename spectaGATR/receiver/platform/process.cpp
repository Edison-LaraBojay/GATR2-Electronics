// process.cpp

#include "receiver/platform/process.h"

#include <atomic>
#include <cerrno>
#include <climits>
#include <csignal>
#include <cstdio>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <fcntl.h>
#include <io.h>
#else
#include <unistd.h>
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#endif

namespace spectagatr::receiver::platform
{

namespace
{

std::atomic<bool> g_interrupted{false};

#if defined(_WIN32)
BOOL WINAPI onConsoleEvent(DWORD event) {
    switch (event) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
        g_interrupted = true;
        return TRUE;
    default:
        return FALSE;
    }
}
#else
void onSignal(int) { g_interrupted = true; }
#endif

} // namespace

void installInterruptHandler() {
#if defined(_WIN32)
    SetConsoleCtrlHandler(onConsoleEvent, TRUE);
#else
    struct sigaction action{};
    action.sa_handler = onSignal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    std::signal(SIGPIPE, SIG_IGN);
#endif
}

bool interruptRequested() { return g_interrupted.load(); }

Path executablePath() {
#if defined(_WIN32)
    std::vector<wchar_t> buffer(MAX_PATH);
    while (true) {
        const DWORD length =
            GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        if (length == 0) {
            return {};
        }
        if (length < buffer.size()) {
            return Path(std::wstring(buffer.data(), length));
        }
        buffer.resize(buffer.size() * 2);
    }
#elif defined(__APPLE__)
    uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::vector<char> buffer(size + 1);
    if (_NSGetExecutablePath(buffer.data(), &size) != 0) {
        return {};
    }
    std::error_code error;
    const Path      resolved = std::filesystem::canonical(buffer.data(), error);
    return error ? Path(buffer.data()) : resolved;
#else
    std::error_code error;
    const Path      resolved = std::filesystem::read_symlink("/proc/self/exe", error);
    return error ? Path() : resolved;
#endif
}

void useBinaryStandardInput() {
#if defined(_WIN32)
    _setmode(_fileno(stdin), _O_BINARY);
#endif
}

long readStandardInput(std::uint8_t* destination, std::size_t capacity) {
#if defined(_WIN32)
    const unsigned wanted = static_cast<unsigned>(capacity > INT_MAX ? INT_MAX : capacity);
    return _read(_fileno(stdin), destination, wanted);
#else
    while (true) {
        const ssize_t count = ::read(STDIN_FILENO, destination, capacity);
        if (count >= 0) {
            return static_cast<long>(count);
        }
        if (errno != EINTR) {
            return -1;
        }
    }
#endif
}

} // namespace spectagatr::receiver::platform
