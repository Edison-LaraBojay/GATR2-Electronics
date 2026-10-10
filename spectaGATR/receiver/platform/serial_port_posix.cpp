// serial_port_posix.cpp
// macOS and Linux: termios raw, TIOCEXCL, select for reads and writes. Not poll: macOS poll does
// not support devices.

#if !defined(_WIN32)

#include "receiver/platform/serial_port.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace spectagatr::receiver::platform
{
namespace
{

enum class Direction { kRead, kWrite };

std::optional<speed_t> speedOf(BaudRate baud) {
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
#ifdef B460800
    case 460800:
        return B460800;
#endif
#ifdef B921600
    case 921600:
        return B921600;
#endif
    default:
        return std::nullopt;
    }
}

// 1 ready, 0 timed out, -1 failed with errno.
int waitReady(int fd, Direction direction, Milliseconds wait) {
    const Milliseconds::rep ms = std::max<Milliseconds::rep>(wait.count(), 0);
    timeval                 timeout{};
    timeout.tv_sec  = static_cast<decltype(timeout.tv_sec)>(ms / 1000);
    timeout.tv_usec = static_cast<decltype(timeout.tv_usec)>((ms % 1000) * 1000);
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    fd_set* const readable = direction == Direction::kRead ? &set : nullptr;
    fd_set* const writable = direction == Direction::kWrite ? &set : nullptr;
    return ::select(fd + 1, readable, writable, nullptr, &timeout);
}

std::string openError(const std::string& path, int error) {
    switch (error) {
    case ENOENT:
        return path + ": no such port";
    case EBUSY:
        return path + ": in use by another program (PROS terminal, VEXcode)";
#if defined(__linux__)
    case EACCES:
        return path + ": permission denied: add your user to the dialout group (uucp on Arch) "
                      "and log in again";
#endif
    default:
        return path + ": " + std::strerror(error);
    }
}

bool wouldBlock(int error) { return error == EAGAIN || error == EWOULDBLOCK || error == EINTR; }

bool configure(int fd, const SerialSettings& settings, speed_t speed, std::string& err) {
    termios tio{};
    if (::tcgetattr(fd, &tio) != 0) {
        err = "not a serial port";
        return false;
    }
    if (::ioctl(fd, TIOCEXCL) != 0) {
        err = std::string("no exclusive access: ") + std::strerror(errno);
        return false;
    }
    ::cfmakeraw(&tio);
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    tio.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
        err = std::string("settings refused: ") + std::strerror(errno);
        return false;
    }
    const int lines = TIOCM_DTR | TIOCM_RTS;
    static_cast<void>(::ioctl(fd, settings.assert_dtr_rts ? TIOCMBIS : TIOCMBIC, &lines));
    ::tcflush(fd, TCIOFLUSH);
    return true;
}

} // namespace

std::optional<SerialPort> openSerialPort(const std::string& path, const SerialSettings& settings,
                                         std::string& err) {
    if (path.empty()) {
        err = "no serial port given";
        return std::nullopt;
    }
    const std::optional<speed_t> speed = speedOf(settings.baud);
    if (!speed) {
        err = path + ": unsupported baud " + std::to_string(settings.baud);
        return std::nullopt;
    }
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        err = openError(path, errno);
        return std::nullopt;
    }
    SerialPort port(fd, path);
    if (fd >= FD_SETSIZE) {
        err = path + ": too many open files for select";
        return std::nullopt;
    }
    std::string why;
    if (!configure(fd, settings, *speed, why)) {
        err = path + ": " + why;
        return std::nullopt;
    }
    return port;
}

void SerialPort::close() {
    if (handle_ != kNoHandle) {
        ::close(handle_);
        handle_ = kNoHandle;
    }
}

IoResult SerialPort::read(Byte* destination, std::size_t capacity, Milliseconds wait) {
    if (!open()) {
        return {IoStatus::kClosed, 0, path_ + ": not open"};
    }
    const int ready = waitReady(handle_, Direction::kRead, wait);
    if (ready < 0) {
        const int error = errno;
        if (error == EINTR) {
            return {IoStatus::kTimedOut, 0, {}};
        }
        return lose(path_ + ": select failed: " + std::strerror(error));
    }
    if (ready == 0) {
        return {IoStatus::kTimedOut, 0, {}};
    }
    const ssize_t count = ::read(handle_, destination, capacity);
    const int     error = errno;
    if (count > 0) {
        return {IoStatus::kOk, static_cast<std::size_t>(count), {}};
    }
    if (count == 0) {
        return lose(path_ + ": device gone"); // readable and empty: hung up
    }
    if (wouldBlock(error)) {
        return {IoStatus::kTimedOut, 0, {}};
    }
    return lose(path_ + ": read failed: " + std::strerror(error));
}

IoResult SerialPort::writeAll(const Byte* source, std::size_t size, Milliseconds budget) {
    if (!open()) {
        return {IoStatus::kClosed, 0, path_ + ": not open"};
    }
    const SteadyClock::time_point deadline = SteadyClock::now() + budget;
    std::size_t                   sent     = 0;
    while (sent < size) {
        const ssize_t count = ::write(handle_, source + sent, size - sent);
        const int     error = errno;
        if (count > 0) {
            sent += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && !wouldBlock(error)) {
            return lose(path_ + ": write failed: " + std::strerror(error));
        }
        const Milliseconds left = std::chrono::ceil<Milliseconds>(deadline - SteadyClock::now());
        if (left.count() <= 0) {
            return {IoStatus::kTimedOut, sent, {}};
        }
        const int ready        = waitReady(handle_, Direction::kWrite, left);
        const int select_error = errno;
        if (ready < 0 && select_error != EINTR) {
            return lose(path_ + ": select failed: " + std::strerror(select_error));
        }
    }
    return {IoStatus::kOk, sent, {}};
}

} // namespace spectagatr::receiver::platform

#endif
