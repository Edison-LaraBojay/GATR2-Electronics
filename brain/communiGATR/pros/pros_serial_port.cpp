// pros_serial_port.cpp
// PROS only.

#include "communigatr/pros_serial_port.h"

#include "pros/error.h"
#include "pros/serial.h"

namespace communigatr
{

ProsSerialPort::ProsSerialPort(uint8_t port, int32_t baud) : port_(port), baud_(baud) {}

bool ProsSerialPort::open() {
    open_ = pros::c::serial_enable(port_) == 1 && pros::c::serial_set_baudrate(port_, baud_) == 1 &&
            pros::c::serial_flush(port_) == 1;
    return open_;
}

// serial_read returns only bytes already received; it never waits.
int ProsSerialPort::read(uint8_t* buf, int max) {
    if (!open_) {
        return -1;
    }
    if (max <= 0) {
        return 0;
    }
    const int32_t n = pros::c::serial_read(port_, buf, max);
    return n == PROS_ERR || n < 0 ? -1 : static_cast<int>(n);
}

// All or nothing: the whole frame must fit the output FIFO.
bool ProsSerialPort::write(const uint8_t* data, int len) {
    if (!open_) {
        return false;
    }
    if (len <= 0) {
        return len == 0;
    }
    const int32_t free = pros::c::serial_get_write_free(port_);
    if (free == PROS_ERR || free < len) {
        return false;
    }
    // serial_write takes a non-const buffer but does not modify it.
    return pros::c::serial_write(port_, const_cast<uint8_t*>(data), len) == len;
}

} // namespace communigatr
