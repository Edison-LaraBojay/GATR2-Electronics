// pros_usb_port.h
// BytePort over the V5 USB user console, the brain link's USB transport.
// Each frame travels as one NG1 line (usb_line.h). Output goes through the
// named stream /ser/ngtr with PROS output COBS disabled for the whole
// program; input is stdin. Two I/O tasks do all USB calls, so a blocked USB
// write (cable out) stalls only the transmit task, never the caller: read and
// write only touch lock-free single-producer single-consumer queues. Source
// in pros/, PROS builds only. One task uses open, read and write: the link
// poll task.

#pragma once
#include <cstdint>

#include "communigatr/byte_port.h"
#include "communigatr/usb_line.h"

namespace communigatr
{

struct ProsUsbStats {
    uint32_t frames_out    = 0; // lines written
    uint32_t expired       = 0; // queued frames older than the lifetime, never written
    uint32_t short_writes  = 0; // write calls that did not take the whole line
    uint32_t frames_in     = 0; // decoded frames queued for the client
    uint32_t input_full    = 0; // decoded frames dropped, input queue full
    uint32_t read_errors   = 0; // stdin reads that failed
    uint32_t lines_ignored = 0; // lines without a marker (console text)
    uint32_t lines_dropped = 0; // marker lines with bad digits, overlong lines
};

class ProsUsbPort : public BytePort {
public:
    // tx_lifetime_ms: a queued frame not written by then is dropped, so a
    // stalled writer never replays expired requests. At least 1.
    explicit ProsUsbPort(uint32_t tx_lifetime_ms);

    // Stops the I/O tasks. Their state stays allocated if a task is still
    // inside a USB call after a short wait; keep the port for the program's
    // life.
    ~ProsUsbPort() override;
    ProsUsbPort(const ProsUsbPort&)            = delete;
    ProsUsbPort& operator=(const ProsUsbPort&) = delete;

    // Opens /ser/ngtr, disables output COBS, starts whichever I/O task is not
    // running. False (errno set) when any step fails; call again to retry.
    bool open();
    bool isOpen() const;

    int  read(uint8_t* buf, int max) override;
    bool write(const uint8_t* data, int len) override;

    ProsUsbStats stats() const;

private:
    struct Io;
    Io* io_;
};

} // namespace communigatr
