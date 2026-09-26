// serial_port.h
// Nonblocking raw serial. Real implementation on POSIX; on other platforms
// open fails so the rest of the app still builds and replay still works.
// Optional RS-485 driver enable on a sysfs GPIO for half-duplex use.

#pragma once
#include <string>

#include "transport/byte_stream.h"
#include "transport/half_duplex.h"

namespace navigatr
{

class SerialPort : public ByteSource, public ByteSink, public HalfDuplexPort
{
public:
    ~SerialPort() override;

    bool open(const std::string& device, int baud, std::string& err);
    // Also drives the driver enable low and closes it.
    void close();
    bool isOpen() const { return fd_ >= 0; }
    int  baud() const { return baud_; }

    int read(uint8_t* dst, int cap) override;
    // Waits for output space with poll(); false after the frame budget.
    bool write(const uint8_t* src, int len) override;

    // Exports the GPIO, drives it low and keeps its value file open.
    bool openDriverEnable(int gpio, std::string& err);
    bool driverEnableOpen() const { return driver_fd_ >= 0; }

    int64_t nowUs() override;
    void    sleepUs(int64_t us) override;
    bool    setDriver(bool on) override;
    bool    inputPending() override;   // FIONREAD
    int     writeSome(const uint8_t* data, std::size_t size) override;
    bool    waitWritable(int64_t timeout_us) override;
    int     transmitterEmpty() override;   // TIOCSERGETLSR, tcdrain fallback
    void    discardOutput() override;      // tcflush TCOFLUSH

private:
    int  fd_              = -1;
    int  baud_            = 0;
    int  driver_fd_       = -1;
    bool lsr_unsupported_ = false;
};

} // namespace navigatr
