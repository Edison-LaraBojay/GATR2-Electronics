// config.h
// Build selection, rates, and the pin budget. Pins live in board.h.
//
// Usable GPIO is GP0-GP22 and GP26-GP28, 26 total. Assigned 15 with the
// BNO08X (11 free), 13 with the ASM330 (13 free).

#pragma once
#include <stddef.h>
#include <stdint.h>

#include "board.h"

#if defined(GATR2_IMU_BNO08X) == defined(GATR2_IMU_ASM330)
#error "define exactly one of GATR2_IMU_BNO08X, GATR2_IMU_ASM330"
#endif

namespace cfg
{

// UART to the Pi.
constexpr uint32_t kPiBaud = 115200;

// Sample and send rate. One tick produces at most one frame.
constexpr uint32_t kSampleHz = 50;
constexpr uint32_t kTickUs   = 1000000u / kSampleHz;

// Human-readable USB serial diagnostics, independent of the Pi UART.
// No host connection is required; full USB buffers drop log lines.
constexpr bool     kUsbDebug         = true;
constexpr uint32_t kUsbBaud          = 115200;
constexpr uint32_t kUsbDebugPeriodMs = 200;

// ---------------------------------------------------------------------------
// Pin budget. Add every new assignment here so collisions fail the build.
// ---------------------------------------------------------------------------

constexpr uint8_t kAllPins[] = {
    board::kEncPinA[0], board::kEncPinB[0], board::kEncPinA[1], board::kEncPinB[1],
    board::kEncPinA[2], board::kEncPinB[2], board::kImuMisoPin, board::kImuCsPin,
    board::kImuSckPin,  board::kImuMosiPin, board::kImuIntPin,
#if defined(GATR2_IMU_BNO08X)
    board::kImuRstPin,  board::kImuWakePin,
#endif
    board::kPiTxPin,    board::kPiRxPin,
};

constexpr uint8_t kPinsAssigned = sizeof(kAllPins);
constexpr uint8_t kPinsUsable   = 26;

constexpr bool pinFree(uint8_t p) { return p <= 22 || (p >= 26 && p <= 28); }

constexpr bool pinsUnique(const uint8_t* p, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        if (!pinFree(p[i])) {
            return false;
        }
        for (size_t j = i + 1; j < n; ++j) {
            if (p[i] == p[j]) {
                return false;
            }
        }
    }
    return true;
}

static_assert(pinsUnique(kAllPins, kPinsAssigned), "pin assigned twice or not a usable Pico GPIO");
static_assert(kPinsAssigned <= kPinsUsable, "out of pins");

} // namespace cfg
