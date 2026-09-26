// byte_port.h
// Nonblocking byte stream under the Navigatr client. The PROS build wraps a
// V5 smart port; host tests use a fake half-duplex bus.

#pragma once
#include <cstdint>

namespace communigatr
{

class BytePort {
public:
    virtual ~BytePort() = default;

    // Copies up to max received bytes into buf. Returns the count, 0 when
    // nothing is waiting, negative on error. Never blocks.
    virtual int read(uint8_t* buf, int max) = 0;

    // Queues all len bytes for transmission. False when they could not all
    // be queued. Never blocks.
    virtual bool write(const uint8_t* data, int len) = 0;
};

} // namespace communigatr
