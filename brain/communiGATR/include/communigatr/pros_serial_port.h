// pros_serial_port.h
// BytePort over a V5 smart port in generic serial mode (PROS serial_* C API).
// Source in pros/, PROS builds only. The V5 port handles its own RS-485
// direction.

#pragma once
#include <cstdint>

#include "communigatr/byte_port.h"

namespace communigatr
{

class ProsSerialPort : public BytePort {
public:
    ProsSerialPort(uint8_t port, int32_t baud);

    // Generic serial on, baud set, buffers cleared. False when PROS refuses
    // (errno set); read and write fail until it succeeds.
    bool open();
    bool isOpen() const { return open_; }

    uint8_t port() const { return port_; }
    int32_t baud() const { return baud_; }

    int  read(uint8_t* buf, int max) override;
    bool write(const uint8_t* data, int len) override;

private:
    uint8_t port_;
    int32_t baud_;
    bool    open_ = false;
};

} // namespace communigatr
