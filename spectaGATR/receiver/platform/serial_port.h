// serial_port.h
// One serial device, raw 8N1 and exclusive to this process, DTR and RTS asserted as pros-cli
// does. Reads wake on data; writes are bounded by a budget. One thread owns a port.

#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "receiver/platform/clock.h"

namespace spectagatr::receiver::platform
{

using Byte     = std::uint8_t;
using BaudRate = std::uint32_t;

#if defined(_WIN32)
using NativeHandle                      = void*; // HANDLE
inline constexpr NativeHandle kNoHandle = nullptr;
#else
using NativeHandle                      = int; // file descriptor
inline constexpr NativeHandle kNoHandle = -1;
#endif

struct SerialSettings {
    BaudRate baud           = 115200; // USB CDC ignores it
    bool     assert_dtr_rts = true;   // pyserial default, which pros-cli uses
};

enum class IoStatus : std::uint8_t {
    kOk,       // read: some bytes; write: every byte
    kTimedOut, // read: nothing within the wait; write: bytes is what went out
    kClosed,   // device gone or failed; the port is closed now
};

struct [[nodiscard]] IoResult {
    IoStatus    status = IoStatus::kOk;
    std::size_t bytes  = 0;
    std::string error; // with kClosed
};

class SerialPort {
  public:
    // Takes ownership of an open, configured handle.
    SerialPort(NativeHandle handle, std::string path);
    ~SerialPort();
    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;
    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Returns once any bytes are in, else kTimedOut after wait. 0 returns what is buffered.
    IoResult read(Byte* destination, std::size_t capacity, Milliseconds wait);

    // Every byte within budget, else kTimedOut with the count that went out.
    IoResult writeAll(const Byte* source, std::size_t size, Milliseconds budget);

    void close();

    bool               open() const { return handle_ != kNoHandle; }
    const std::string& path() const { return path_; }

  private:
    IoResult lose(std::string error);

    NativeHandle handle_;
    std::string  path_;
};

// COM7 or \\.\COM7 on Windows, /dev/cu.* on macOS, /dev/ttyACM* on Linux.
[[nodiscard]] std::optional<SerialPort>
openSerialPort(const std::string& path, const SerialSettings& settings, std::string& err);

const char* ioStatusName(IoStatus status);

} // namespace spectagatr::receiver::platform
