// config.h
// Pin assignments and rates. Values are RP2040 GPIO numbers (GPn), not header
// pin numbers. The header pin is noted alongside for wiring.
//
// Usable GPIO is GP0-GP22 and GP26-GP28, 26 total. Assigned 13, free 13.

#pragma once
#include <stddef.h>
#include <stdint.h>

namespace cfg
{

// Encoder channels, A and B per ABI encoder.
// enc0 GP0/GP1 (pins 1/2), enc1 GP2/GP3 (pins 4/5), enc2 GP4/GP5 (pins 6/7).
constexpr uint8_t kEncPinA[3] = {0, 2, 4};
constexpr uint8_t kEncPinB[3] = {1, 3, 5};

// IMU on SPI1. MISO GP8 (pin 11), CS GP9 (pin 12), SCK GP10 (pin 14),
// MOSI GP11 (pin 15), INT GP22 (pin 29).
constexpr uint8_t kImuMisoPin = 8;
constexpr uint8_t kImuCsPin   = 9;
constexpr uint8_t kImuSckPin  = 10;
constexpr uint8_t kImuMosiPin = 11;
constexpr uint8_t kImuIntPin  = 22;

// UART to the Pi, Serial1 (UART0). TX GP16 (pin 21), RX GP17 (pin 22).
constexpr uint8_t  kPiTxPin = 16;
constexpr uint8_t  kPiRxPin = 17;
constexpr uint32_t kPiBaud  = 115200;

// Sample and send rate. One tick produces exactly one frame.
constexpr uint32_t kSampleHz = 50;
constexpr uint32_t kTickUs   = 1000000u / kSampleHz;

// ---------------------------------------------------------------------------
// Pin budget. Add every new assignment here so collisions fail the build.
// ---------------------------------------------------------------------------

constexpr uint8_t kAllPins[] = {
    kEncPinA[0], kEncPinB[0], kEncPinA[1], kEncPinB[1], kEncPinA[2],
    kEncPinB[2], kImuMisoPin, kImuCsPin,  kImuSckPin,  kImuMosiPin,
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
