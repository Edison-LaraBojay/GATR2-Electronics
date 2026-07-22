// config.h
// Pin assignments and rates. Pins are placeholders until the board is wired.
//
// Pico usable GPIO is GP0-GP22 and GP26-GP28, 26 total.
// GP23-GP25 are onboard functions and are not available.
// Assigned 13, free 13.

#pragma once
#include <stddef.h>
#include <stdint.h>

namespace cfg
{

// Encoder channels, A and B of each ABI encoder.
constexpr uint8_t kEncPinA[3] = {2, 4, 6};
constexpr uint8_t kEncPinB[3] = {3, 5, 7};

// IMU on SPI0.
constexpr uint8_t kImuSckPin  = 18;
constexpr uint8_t kImuMosiPin = 19;
constexpr uint8_t kImuMisoPin = 16;
constexpr uint8_t kImuCsPin   = 17;
constexpr uint8_t kImuIntPin  = 20; // data ready, routed but unused

// UART to the Pi.
constexpr uint8_t  kPiTxPin = 0;
constexpr uint8_t  kPiRxPin = 1;
constexpr uint32_t kPiBaud  = 115200;

// Sample and send rate. One tick produces exactly one frame.
constexpr uint32_t kSampleHz = 50;
constexpr uint32_t kTickUs   = 1000000u / kSampleHz;

// ---------------------------------------------------------------------------
// Pin budget. Add every new assignment here so collisions fail the build.
// ---------------------------------------------------------------------------

constexpr uint8_t kAllPins[] = {
    kEncPinA[0], kEncPinB[0], kEncPinA[1], kEncPinB[1], kEncPinA[2],
    kEncPinB[2], kImuSckPin,  kImuMosiPin, kImuMisoPin, kImuCsPin,
    kImuIntPin,  kPiTxPin,    kPiRxPin,
};

constexpr uint8_t kPinsAssigned = sizeof(kAllPins);
constexpr uint8_t kPinsUsable   = 26;

constexpr bool pinFree(uint8_t p) {
    return p <= 22 || (p >= 26 && p <= 28);
}

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

static_assert(pinsUnique(kAllPins, kPinsAssigned),
              "pin assigned twice or not a usable Pico GPIO");
static_assert(kPinsAssigned <= kPinsUsable, "out of pins");

} // namespace cfg
