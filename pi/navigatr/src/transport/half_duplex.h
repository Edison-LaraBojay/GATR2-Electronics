// half_duplex.h
// RS-485 half-duplex transmit: listen while idle, drive the bus only while a
// frame goes out, release once the transmitter is empty. Portable over
// HalfDuplexPort; SerialPort implements it on Linux, tests use a fake.

#pragma once
#include <cstddef>
#include <cstdint>

#include "resources/serial_link.h"

namespace navigatr
{

// One 8N1 character (10 bits), rounded up. 0 for baud <= 0.
int64_t characterTimeUs(int baud);

// bytes * 10 / baud + margin_us
int64_t transmitBudgetUs(std::size_t bytes, int baud, int64_t margin_us);

class HalfDuplexPort
{
public:
    virtual ~HalfDuplexPort() = default;

    virtual int64_t nowUs()             = 0;
    virtual void    sleepUs(int64_t us) = 0;

    // Transceiver driver enable. False when it cannot be set.
    virtual bool setDriver(bool on) = 0;

    virtual bool inputPending() = 0;

    // Bytes accepted, 0 when the output buffer is full, negative on error.
    virtual int writeSome(const uint8_t* data, std::size_t size) = 0;

    // Blocks until output space frees up or timeout_us passes. False on error.
    virtual bool waitWritable(int64_t timeout_us) = 0;

    // 1 when the last stop bit is out, 0 while sending, negative on error.
    virtual int transmitterEmpty() = 0;

    // Drops output not yet sent.
    virtual void discardOutput() = 0;
};

struct HalfDuplexTiming {
    int     baud          = 115200;
    int64_t post_guard_us = 174;    // driver held after transmitter empty, 2 characters
    int64_t margin_us     = 2000;   // added to the airtime for the transmit deadline
};

// Sleeps until window.not_before_us. False when the window is missed.
bool waitForWindow(HalfDuplexPort& port, const TransmitWindow& window);

// Writes every byte before deadline_us. nullptr on success, else the reason.
const char* writeAll(HalfDuplexPort& port, ByteSpan bytes, int64_t deadline_us);

// Window check, input check, driver on, write, wait for transmitter empty,
// post guard, driver off. The driver is released on every path that set it
// and failures discard unsent output. late_release: driver released after
// driver on + transmitBudgetUs + post guard.
SerialWriteResult transmitHalfDuplex(HalfDuplexPort& port, ByteSpan frame,
                                     const TransmitWindow&   window,
                                     const HalfDuplexTiming& timing);

} // namespace navigatr
