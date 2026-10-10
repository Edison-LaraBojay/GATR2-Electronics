// serial_port.cpp
// The OS-neutral part of SerialPort. Open, read, write and close are per OS.

#include "receiver/platform/serial_port.h"

#include <utility>

namespace spectagatr::receiver::platform
{

SerialPort::SerialPort(NativeHandle handle, std::string path)
    : handle_(handle), path_(std::move(path)) {}

SerialPort::~SerialPort() { close(); }

SerialPort::SerialPort(SerialPort&& other) noexcept
    : handle_(std::exchange(other.handle_, kNoHandle)), path_(std::move(other.path_)) {}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        close();
        handle_ = std::exchange(other.handle_, kNoHandle);
        path_   = std::move(other.path_);
    }
    return *this;
}

IoResult SerialPort::lose(std::string error) {
    close();
    return {IoStatus::kClosed, 0, std::move(error)};
}

const char* ioStatusName(IoStatus status) {
    switch (status) {
    case IoStatus::kOk:
        return "ok";
    case IoStatus::kTimedOut:
        return "timed_out";
    case IoStatus::kClosed:
        return "closed";
    }
    return "unknown";
}

} // namespace spectagatr::receiver::platform
