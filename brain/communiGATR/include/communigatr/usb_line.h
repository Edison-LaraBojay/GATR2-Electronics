// usb_line.h
// NG1 line codec for the brain link over the V5 USB user console. Each link
// frame travels as "NG1:" + uppercase hex of every frame byte + "\n", so the
// PROS console never sees binary. Same rules as the Pi pros_usb_link parser:
// the last marker in a line counts, text before it is ignored, a trailing
// "\r" is stripped, the digits are uppercase, even in number and 2..256; any
// fault drops the whole line. Portable, no I/O.

#pragma once
#include <cstddef>
#include <cstdint>

#include "common/frames.h"

namespace communigatr
{

constexpr std::size_t kUsbMarkerLen = 4; // "NG1:"

// Longest encoded line, newline included.
constexpr std::size_t kUsbLineMax = kUsbMarkerLen + 2 * gatr2::kMaxFrameLen + 1;

// Characters a line may hold before its newline; longer lines are dropped.
// Room for a diagnostic prefix before the marker.
constexpr std::size_t kUsbLineLimit = kUsbMarkerLen + 2 * gatr2::kMaxFrameLen + 256;

// Writes the line for one frame. Returns the characters written, 0 when len
// is 0 or over kMaxFrameLen or the line does not fit in cap.
std::size_t encodeUsbLine(const uint8_t* frame, std::size_t len, char* out, std::size_t cap);

struct UsbLineStats {
    uint32_t frames  = 0; // lines that carried bytes
    uint32_t ignored = 0; // lines without a marker, such as console text
    uint32_t dropped = 0; // marker lines with bad digits, and overlong lines
};

class UsbLineDecoder {
public:
    // Feeds one character. True when it ended a line carrying bytes, which
    // stay in bytes() until the next push.
    bool push(char c);

    // Forgets a partial line, as after a reconnect.
    void reset();

    const uint8_t*      bytes() const { return bytes_; }
    std::size_t         length() const { return length_; }
    const UsbLineStats& stats() const { return stats_; }

private:
    bool finish();

    char         line_[kUsbLineLimit]        = {};
    std::size_t  used_                       = 0;
    bool         discarding_                 = false;
    uint8_t      bytes_[gatr2::kMaxFrameLen] = {};
    std::size_t  length_                     = 0;
    UsbLineStats stats_;
};

} // namespace communigatr
