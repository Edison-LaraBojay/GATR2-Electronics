// serial_port.cpp

#include "transport/serial_port.h"

#if defined(__unix__) || defined(__APPLE__)

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace navigatr
{

namespace
{

bool baudConstant(int baud, speed_t& out) {
    switch (baud) {
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
#ifdef B460800
    case 460800: out = B460800; return true;
#endif
#ifdef B921600
    case 921600: out = B921600; return true;
#endif
    default: return false;
    }
}

} // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& device, int baud, std::string& err) {
    speed_t speed;
    if (!baudConstant(baud, speed)) {
        err = "unsupported baud " + std::to_string(baud);
        return false;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        err = "cannot open " + device;
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        err = "tcgetattr failed on " + device;
        close();
        return false;
    }

    cfmakeraw(&tio);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        err = "tcsetattr failed on " + device;
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialPort::read(uint8_t* dst, int cap) {
    if (fd_ < 0) {
        return -1;
    }
    const ssize_t n = ::read(fd_, dst, static_cast<size_t>(cap));
    if (n < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }
    return static_cast<int>(n);
}

bool SerialPort::write(const uint8_t* src, int len) {
    if (fd_ < 0 || len < 0) {
        return false;
    }
    int at = 0;
    while (at < len) {
        const ssize_t n = ::write(fd_, src + at, static_cast<size_t>(len - at));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            return false;
        }
        at += static_cast<int>(n);
    }
    return true;
}

} // namespace navigatr

#else

// Non-POSIX build. Serial hardware only exists on the Pi; this keeps host
// builds compiling.

namespace navigatr
{

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& device, int baud, std::string& err) {
    (void)baud;
    err = "serial not supported on this platform: " + device;
    return false;
}

void SerialPort::close() { fd_ = -1; }

int SerialPort::read(uint8_t* dst, int cap) {
    (void)dst;
    (void)cap;
    return -1;
}

bool SerialPort::write(const uint8_t* src, int len) {
    (void)src;
    (void)len;
    return false;
}

} // namespace navigatr

#endif
