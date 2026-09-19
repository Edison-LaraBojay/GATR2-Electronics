// quadrature.h
// Interrupt-driven x4 decode of encoder A/B signals. Counts accumulate in
// the background and can be sampled independently of the UART send loop.

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
