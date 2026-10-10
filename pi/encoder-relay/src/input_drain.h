// input_drain.h
// Bound each pass so a continuously readable UART cannot monopolize the relay.

#pragma once

#include "forwarder.h"

namespace relay
{

constexpr std::size_t kInputReadChunk = 512;
constexpr std::size_t kInputReadLimit = 64;

struct DrainResult {
    std::size_t records = 0;
    bool limited = false; // input may remain; service other I/O before the next pass
};

// read(buffer, capacity) is nonblocking: positive means bytes read, zero means
// no more input (or closed), and negative means interrupted; retry it. It must
// never return more than capacity bytes. Complete records replace older unsent
// ones; the scanner retains any partial tail for the next pass.
template <typename Read>
DrainResult drainInput(Forwarder& forwarder, Read read) {
    DrainResult result;
    uint8_t buffer[kInputReadChunk];
    for (std::size_t reads = 0; reads < kInputReadLimit; ++reads) {
        const std::ptrdiff_t count = read(buffer, sizeof(buffer));
        if (count == 0) {
            return result;
        }
        if (count > 0) {
            result.records += forwarder.receive(buffer, static_cast<std::size_t>(count));
        }
    }
    result.limited = true;
    return result;
}

} // namespace relay
