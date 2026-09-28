// pros_serial_port.h
// BytePort over a V5 smart port in generic serial mode (PROS serial_* C API),
// wired to the RS-485 link. Source in pros/, PROS builds only. The V5 port
// handles its own RS-485 direction. One task uses it: the link poll task.

#pragma once
#include <cstdint>

#include "communigatr/byte_port.h"

namespace communigatr
{

class ProsSerialPort : public BytePort {
public:
    ProsSerialPort(uint8_t port, int32_t baud);

    // Generic serial on, then baud and buffer flush, retried every 2 ms for
    // up to about 100 ms while the port settles. Blocks for that long at most.
    // False when PROS refuses (errno set); read and write fail until a later
    // open() succeeds.
    bool open();
    bool isOpen() const { return open_; }

    // Consecutive read or write errors after which the port counts as closed
    // and needs open() again.
    static constexpr int kErrorsBeforeClose = 50;

    uint8_t  port() const { return port_; }
    int32_t  baud() const { return baud_; }
    uint32_t closes() const { return closes_; } // closed after errors

    int  read(uint8_t* buf, int max) override;
    bool write(const uint8_t* data, int len) override;

private:
    void failed();

    uint8_t  port_;
    int32_t  baud_;
    bool     open_   = false;
    int      errors_ = 0;
    uint32_t closes_ = 0;
};

} // namespace communigatr
