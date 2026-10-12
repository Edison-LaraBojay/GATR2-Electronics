// pros_serial_port.cpp
// PROS only.

#include "communigatr/pros_serial_port.h"

#include "pros/error.h"
#include "pros/rtos.hpp"
#include "pros/serial.h"

namespace communigatr
{

namespace
{

constexpr uint32_t kSettleStepMs = 2;
constexpr uint32_t kSettleMs     = 100;

} // namespace

ProsSerialPort::ProsSerialPort(uint8_t port, int32_t baud) : port_(port), baud_(baud) {}

bool ProsSerialPort::open() {
    open_   = false;
    errors_ = 0;
    if (pros::c::serial_enable(port_) != 1) {
        return false;
    }
    // Right after serial_enable the port may still refuse its settings.
    for (uint32_t waited = 0;; waited += kSettleStepMs) {
        if (pros::c::serial_set_baudrate(port_, baud_) == 1 && pros::c::serial_flush(port_) == 1) {
            open_ = true;
            return true;
        }
        if (waited >= kSettleMs) {
            return false;
        }
        pros::delay(kSettleStepMs);
    }
}

void ProsSerialPort::failed() {
    if (++errors_ >= kErrorsBeforeClose && open_) {
        open_ = false;
        ++closes_;
    }
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
    if (n == PROS_ERR || n < 0) {
        failed();
        return -1;
    }
    errors_ = 0;
    return static_cast<int>(n);
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
    if (free == PROS_ERR) {
        failed();
        return false;
    }
    if (free < len) {
        return false; // FIFO full, not an error
    }
    // serial_write takes a non-const buffer but does not modify it.
    if (pros::c::serial_write(port_, const_cast<uint8_t*>(data), len) != len) {
        failed();
        return false;
    }
    errors_ = 0;
    return true;
}

} // namespace communigatr
