// encoder_demo_config.h
// Encoder demo firmware (env hat2_encoder_demo): record rate, UART and pins.
// Encoder pins are in board.h: channel 0 GP0/GP1, channel 1 GP2/GP3.

#pragma once
#include <stddef.h>
#include <stdint.h>

#include "board.h"

namespace demo_cfg
{

// Encoder records to the Pi.
constexpr uint32_t kRecordHz = 50;
constexpr uint32_t kTickUs   = 1000000u / kRecordHz;

// Channel 0 sideways wheel, channel 1 forward wheel. Channel 2 is not started.
constexpr uint8_t kChannelsUsed = 2;

// UART to the Pi, Serial1 (UART0). RX is set so UART0 leaves GP1 alone;
// nothing is read from it.
constexpr uint8_t  kUartTxPin = 16; // pin 21
constexpr uint8_t  kUartRxPin = 17; // pin 22
constexpr uint32_t kUartBaud  = 115200;

// Local USB text status. Never waits for a monitor.
constexpr bool     kUsbDebug         = true;
constexpr uint32_t kUsbBaud          = 115200;
constexpr uint32_t kUsbDebugPeriodMs = 500;

constexpr uint8_t kAllPins[] = {
    board::kEncPinA[0], board::kEncPinB[0], board::kEncPinA[1],
    board::kEncPinB[1], kUartTxPin,         kUartRxPin,
};

constexpr bool pinsUnique() {
    for (size_t i = 0; i < sizeof(kAllPins); ++i) {
        for (size_t j = i + 1; j < sizeof(kAllPins); ++j) {
            if (kAllPins[i] == kAllPins[j]) {
                return false;
            }
        }
    }
    return true;
}
static_assert(pinsUnique(), "pin assigned twice");

} // namespace demo_cfg
