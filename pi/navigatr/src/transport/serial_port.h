// serial_port.h
// Nonblocking raw serial. Real implementation on POSIX; on other platforms
// open fails so the rest of the app still builds and replay still works.

#pragma once
#include <string>

#include "transport/byte_stream.h"

namespace navigatr
{

class SerialPort : public ByteSource, public ByteSink
{
public:
    ~SerialPort() override;

    bool open(const std::string& device, int baud, std::string& err);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    int  read(uint8_t* dst, int cap) override;
    bool write(const uint8_t* src, int len) override;

private:
    int fd_ = -1;
};

} // namespace navigatr
