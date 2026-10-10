// serial_port.cpp

#include "serial_port.h"

#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace relay
{

namespace
{

bool toSpeed(int baud, speed_t& speed) {
    switch (baud) {
    case 9600:
        speed = B9600;
        return true;
    case 19200:
        speed = B19200;
        return true;
    case 38400:
        speed = B38400;
        return true;
    case 57600:
        speed = B57600;
        return true;
    case 115200:
        speed = B115200;
        return true;
    case 230400:
        speed = B230400;
        return true;
#ifdef B460800
    case 460800:
        speed = B460800;
        return true;
#endif
#ifdef B921600
    case 921600:
        speed = B921600;
        return true;
#endif
    default:
        return false;
    }
}

} // namespace

bool supportedBaud(int baud) {
    speed_t speed;
    return toSpeed(baud, speed);
}

int openSerial(const std::string& path, int baud) {
    speed_t speed;
    if (!toSpeed(baud, speed)) {
        errno = EINVAL;
        return -1;
    }
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return -1;
    }
    // One owner: another relay holding the lock, or a later open by anyone
    // but root, fails with EBUSY.
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0 || ::ioctl(fd, TIOCEXCL) != 0) {
        ::close(fd);
        errno = EBUSY;
        return -1;
    }
    termios settings{};
    if (::tcgetattr(fd, &settings) != 0) {
        const int saved = errno;
        ::close(fd);
        errno = saved;
        return -1;
    }
    ::cfmakeraw(&settings);
    ::cfsetispeed(&settings, speed);
    ::cfsetospeed(&settings, speed);
    settings.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    settings.c_cflag &= ~CRTSCTS;
#endif
    settings.c_cc[VMIN]  = 0;
    settings.c_cc[VTIME] = 0;
    if (::tcsetattr(fd, TCSANOW, &settings) != 0) {
        const int saved = errno;
        ::close(fd);
        errno = saved;
        return -1;
    }
    // Stale bytes from before this open are never forwarded.
    ::tcflush(fd, TCIOFLUSH);
    return fd;
}

} // namespace relay
