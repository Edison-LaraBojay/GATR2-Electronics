// serial_port.cpp

#include "transport/serial_port.h"

#include <chrono>
#include <thread>

namespace navigatr
{

int64_t SerialPort::nowUs() { return steadyNowUs(); }

void SerialPort::sleepUs(int64_t us) {
    if (us > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }
}

} // namespace navigatr

#if defined(__unix__) || defined(__APPLE__)

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace navigatr
{

namespace
{

constexpr int64_t kWriteMarginUs = 2000;

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

// errno of the failing call on false
bool writeSysfs(const std::string& path, const char* value) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        return false;
    }
    const ssize_t len = static_cast<ssize_t>(std::strlen(value));
    const bool    ok  = ::write(fd, value, static_cast<size_t>(len)) == len;
    const int     e   = errno;
    ::close(fd);
    errno = e;
    return ok;
}

} // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& device, int baud, std::string& err) {
    speed_t speed;
    if (!baudConstant(baud, speed)) {
        err = "unsupported baud " + std::to_string(baud);
        return false;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
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
    baud_            = baud;
    lsr_unsupported_ = false;
    return true;
}

void SerialPort::close() {
    if (driver_fd_ >= 0) {
        setDriver(false);
        ::close(driver_fd_);
        driver_fd_ = -1;
    }
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
    const std::size_t size     = static_cast<std::size_t>(len);
    const int64_t     deadline = nowUs() + transmitBudgetUs(size, baud_, kWriteMarginUs);
    return writeAll(*this, ByteSpan{src, size}, deadline) == nullptr;
}

bool SerialPort::openDriverEnable(int gpio, std::string& err) {
    const std::string number = std::to_string(gpio);
    const std::string base   = "/sys/class/gpio/gpio" + number;
    // export fails when the pin is already exported
    writeSysfs("/sys/class/gpio/export", number.c_str());
    // "low" makes it an output driven low in one step; udev may need a
    // moment to grant access to a freshly exported pin
    bool direction = writeSysfs(base + "/direction", "low");
    for (int attempt = 0; !direction && errno == EACCES && attempt < 20; ++attempt) {
        sleepUs(10'000);
        direction = writeSysfs(base + "/direction", "low");
    }
    const int fd = direction ? ::open((base + "/value").c_str(), O_WRONLY | O_CLOEXEC) : -1;
    if (fd < 0) {
        err = "cannot drive gpio " + number + " via sysfs";
        return false;
    }
    if (driver_fd_ >= 0) {
        ::close(driver_fd_);
    }
    driver_fd_ = fd;
    if (!setDriver(false)) {
        err = "cannot write gpio " + number + " value";
        return false;
    }
    return true;
}

bool SerialPort::setDriver(bool on) {
    if (driver_fd_ < 0) {
        return false;
    }
    return ::pwrite(driver_fd_, on ? "1" : "0", 1, 0) == 1;
}

bool SerialPort::inputPending() {
    int n = 0;
    return fd_ >= 0 && ::ioctl(fd_, FIONREAD, &n) == 0 && n > 0;
}

int SerialPort::writeSome(const uint8_t* data, std::size_t size) {
    if (fd_ < 0) {
        return -1;
    }
    const ssize_t n = ::write(fd_, data, std::min<std::size_t>(size, INT_MAX));
    if (n < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : -1;
    }
    return static_cast<int>(n);
}

bool SerialPort::waitWritable(int64_t timeout_us) {
    if (fd_ < 0) {
        return false;
    }
    pollfd pfd{};
    pfd.fd     = fd_;
    pfd.events = POLLOUT;
    const int64_t ms = std::min<int64_t>(timeout_us <= 0 ? 0 : (timeout_us + 999) / 1000, INT_MAX);
    const int     n  = ::poll(&pfd, 1, static_cast<int>(ms));
    if (n < 0) {
        return errno == EINTR;
    }
    return (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) == 0;
}

int SerialPort::transmitterEmpty() {
    if (fd_ < 0) {
        return -1;
    }
#ifdef TIOCSERGETLSR
    if (!lsr_unsupported_) {
        unsigned int lsr = 0;
        if (::ioctl(fd_, TIOCSERGETLSR, &lsr) == 0) {
            return (lsr & TIOCSER_TEMT) != 0 ? 1 : 0;
        }
        if (errno != ENOTTY && errno != EINVAL) {
            return -1;
        }
        lsr_unsupported_ = true;
    }
#endif
    // blocks until sent, no deadline of its own
    return tcdrain(fd_) == 0 ? 1 : -1;
}

void SerialPort::discardOutput() {
    if (fd_ >= 0) {
        tcflush(fd_, TCOFLUSH);
    }
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

void SerialPort::close() {
    fd_        = -1;
    driver_fd_ = -1;
}

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

bool SerialPort::openDriverEnable(int gpio, std::string& err) {
    err = "gpio not supported on this platform: " + std::to_string(gpio);
    return false;
}

bool SerialPort::setDriver(bool on) {
    (void)on;
    return false;
}

bool SerialPort::inputPending() { return false; }

int SerialPort::writeSome(const uint8_t* data, std::size_t size) {
    (void)data;
    (void)size;
    return -1;
}

bool SerialPort::waitWritable(int64_t timeout_us) {
    (void)timeout_us;
    return false;
}

int SerialPort::transmitterEmpty() { return -1; }

void SerialPort::discardOutput() {}

} // namespace navigatr

#endif
