// serial_port_win32.cpp
// Windows: synchronous handle, share mode 0, COMMTIMEOUTS that return on the first bytes.

#if defined(_WIN32)

#include "receiver/platform/serial_port.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include "receiver/platform/win32_text.h"

namespace spectagatr::receiver::platform
{
namespace
{

constexpr DWORD kQueueBytes    = 4096;
constexpr DWORD kLongestWaitMs = MAXDWORD - 1; // MAXDWORD is special in COMMTIMEOUTS

DWORD timeoutMs(Milliseconds duration) {
    return static_cast<DWORD>(std::clamp<Milliseconds::rep>(
        duration.count(), 0, static_cast<Milliseconds::rep>(kLongestWaitMs)));
}

// \\.\COMn reaches every port number; plain COMn only 1 to 9.
std::string devicePath(const std::string& path) {
    const bool bare_com = path.size() > 3 && _strnicmp(path.c_str(), "COM", 3) == 0;
    return bare_com ? "\\\\.\\" + path : path;
}

bool timedOut(DWORD code) { return code == ERROR_SEM_TIMEOUT || code == ERROR_TIMEOUT; }

std::string openError(const std::string& path, DWORD code) {
    switch (code) {
    case ERROR_FILE_NOT_FOUND:
    case ERROR_PATH_NOT_FOUND:
        return path + ": no such port";
    case ERROR_ACCESS_DENIED:
    case ERROR_SHARING_VIOLATION:
        return path + ": in use by another program (PROS terminal, VEXcode)";
    default:
        return path + ": " + win32ErrorText(code);
    }
}

bool configure(HANDLE handle, const SerialSettings& settings) {
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle, &dcb)) {
        return false;
    }
    dcb.BaudRate        = settings.baud;
    dcb.ByteSize        = 8;
    dcb.Parity          = NOPARITY;
    dcb.StopBits        = ONESTOPBIT;
    dcb.fBinary         = TRUE;
    dcb.fParity         = FALSE;
    dcb.fOutxCtsFlow    = FALSE;
    dcb.fOutxDsrFlow    = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX           = FALSE;
    dcb.fInX            = FALSE;
    dcb.fErrorChar      = FALSE;
    dcb.fNull           = FALSE; // keep 0x00 bytes
    dcb.fAbortOnError   = FALSE;
    dcb.fDtrControl     = settings.assert_dtr_rts ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;
    dcb.fRtsControl     = settings.assert_dtr_rts ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE;
    return SetCommState(handle, &dcb) && SetupComm(handle, kQueueBytes, kQueueBytes) &&
           PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

// False once the device is gone.
bool present(HANDLE handle) {
    DWORD   errors = 0;
    COMSTAT status{};
    return ClearCommError(handle, &errors, &status) != FALSE;
}

} // namespace

std::optional<SerialPort> openSerialPort(const std::string& path, const SerialSettings& settings,
                                         std::string& err) {
    if (path.empty()) {
        err = "no serial port given";
        return std::nullopt;
    }
    if (settings.baud == 0) {
        err = path + ": baud 0";
        return std::nullopt;
    }
    const std::wstring wide   = toWide(devicePath(path));
    HANDLE             handle = CreateFileW(wide.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                                            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        err = openError(path, GetLastError());
        return std::nullopt;
    }
    SerialPort port(handle, path);
    if (!configure(handle, settings)) {
        const DWORD code = GetLastError();
        err = path + ": not a serial port, or settings refused: " + win32ErrorText(code);
        return std::nullopt;
    }
    return port;
}

void SerialPort::close() {
    if (handle_ != kNoHandle) {
        CloseHandle(handle_);
        handle_ = kNoHandle;
    }
}

IoResult SerialPort::read(Byte* destination, std::size_t capacity, Milliseconds wait) {
    if (!open()) {
        return {IoStatus::kClosed, 0, path_ + ": not open"};
    }
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    const DWORD wait_ms          = timeoutMs(wait);
    if (wait_ms > 0) {
        timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
        timeouts.ReadTotalTimeoutConstant   = wait_ms;
    }
    if (!SetCommTimeouts(handle_, &timeouts)) {
        const DWORD code = GetLastError();
        return lose(path_ + ": " + win32ErrorText(code));
    }
    const DWORD wanted   = static_cast<DWORD>(std::min<std::size_t>(capacity, MAXDWORD));
    DWORD       received = 0;
    if (!ReadFile(handle_, destination, wanted, &received, nullptr)) {
        const DWORD code = GetLastError();
        if (timedOut(code)) {
            return {IoStatus::kTimedOut, 0, {}};
        }
        return lose(path_ + ": read failed: " + win32ErrorText(code));
    }
    if (received > 0) {
        return {IoStatus::kOk, received, {}};
    }
    if (!present(handle_)) {
        return lose(path_ + ": device gone");
    }
    return {IoStatus::kTimedOut, 0, {}};
}

IoResult SerialPort::writeAll(const Byte* source, std::size_t size, Milliseconds budget) {
    if (!open()) {
        return {IoStatus::kClosed, 0, path_ + ": not open"};
    }
    if (size == 0) {
        return {IoStatus::kOk, 0, {}};
    }
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout       = MAXDWORD;
    timeouts.WriteTotalTimeoutConstant = std::max<DWORD>(1, timeoutMs(budget)); // 0 is no limit
    if (!SetCommTimeouts(handle_, &timeouts)) {
        const DWORD code = GetLastError();
        return lose(path_ + ": " + win32ErrorText(code));
    }
    const DWORD wanted = static_cast<DWORD>(std::min<std::size_t>(size, MAXDWORD));
    DWORD       sent   = 0;
    if (!WriteFile(handle_, source, wanted, &sent, nullptr)) {
        const DWORD code = GetLastError();
        if (timedOut(code)) {
            return {IoStatus::kTimedOut, sent, {}};
        }
        return lose(path_ + ": write failed: " + win32ErrorText(code));
    }
    if (sent < size) {
        return {IoStatus::kTimedOut, sent, {}};
    }
    return {IoStatus::kOk, sent, {}};
}

} // namespace spectagatr::receiver::platform

#endif
