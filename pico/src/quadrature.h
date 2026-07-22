// quadrature.h
// x4 decode for ABI encoders. Counts accumulate in the background so a reader
// can sample at any time without losing edges.
//
// This is an interrupt driven implementation. PIO decode is the intended
// replacement, see pico/README.md.

#pragma once
#include <stdint.h>

namespace encoder
{

constexpr uint8_t kChannels = 3;

void begin(uint8_t ch, uint8_t pin_a, uint8_t pin_b);

// Accumulated count. Safe to call from the main loop at any time.
int32_t count(uint8_t ch);

// True once begin has run for this channel.
bool started(uint8_t ch);

} // namespace encoder
