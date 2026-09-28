// shtp_header.h
// Inbound SHTP transfer header check. sh2 v1.4.0 indexes its channel table
// before bounds checking, so invalid headers never reach it.

#pragma once
#include <stdint.h>

namespace bno08x
{

constexpr uint16_t kShtpHeaderLen  = 4;
constexpr uint16_t kShtpMaxRxLen   = 1024;
constexpr uint8_t  kShtpMaxChannel = 5;

// Transfer length from a 4-byte header, continuation bit masked off. 0 for a
// null header, 0xFFFF, a channel above 5, or a length of 1-3 or over 1024.
constexpr uint16_t shtpRxLen(const uint8_t* hdr) {
    if (hdr[0] == 0xFF && hdr[1] == 0xFF) {
        return 0;
    }
    const uint16_t len = static_cast<uint16_t>((hdr[0] | (hdr[1] << 8)) & 0x7FFF);
    if (hdr[2] > kShtpMaxChannel || len < kShtpHeaderLen || len > kShtpMaxRxLen) {
        return 0;
    }
    return len;
}

} // namespace bno08x
