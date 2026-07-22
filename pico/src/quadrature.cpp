// quadrature.cpp

#include "quadrature.h"

#include <Arduino.h>

namespace encoder
{
namespace
{

// Delta for a (previous state, new state) pair, indexed (prev << 2) | now.
// State is (A << 1) | B. Illegal transitions, meaning both lines changed
// between samples, contribute zero rather than guessing a direction.
constexpr int8_t kDelta[16] = {
    0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0,
};

struct Channel {
    uint8_t          pin_a   = 0;
    uint8_t          pin_b   = 0;
    uint8_t          state   = 0;
    bool             started = false;
    volatile int32_t count   = 0;
};

Channel g_ch[kChannels];

uint8_t readState(const Channel& c) {
    return static_cast<uint8_t>((digitalRead(c.pin_a) << 1) | digitalRead(c.pin_b));
}

void step(uint8_t ch) {
    Channel&      c   = g_ch[ch];
    const uint8_t now = readState(c);
    c.count += kDelta[(c.state << 2) | now];
    c.state = now;
}

void isr0() { step(0); }
void isr1() { step(1); }
void isr2() { step(2); }

void (*const kIsr[kChannels])() = {isr0, isr1, isr2};

} // namespace

void begin(uint8_t ch, uint8_t pin_a, uint8_t pin_b) {
    if (ch >= kChannels) {
        return;
    }
    Channel& c = g_ch[ch];
    c.pin_a    = pin_a;
    c.pin_b    = pin_b;

    pinMode(pin_a, INPUT_PULLUP);
    pinMode(pin_b, INPUT_PULLUP);

    c.state = readState(c);
    c.count = 0;

    attachInterrupt(digitalPinToInterrupt(pin_a), kIsr[ch], CHANGE);
    attachInterrupt(digitalPinToInterrupt(pin_b), kIsr[ch], CHANGE);
    c.started = true;
}

int32_t count(uint8_t ch) {
    if (ch >= kChannels) {
        return 0;
    }
    return g_ch[ch].count;
}

bool started(uint8_t ch) {
    return ch < kChannels && g_ch[ch].started;
}

} // namespace encoder
