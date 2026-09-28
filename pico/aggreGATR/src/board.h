// board.h
// Pico pin map per Pi HAT revision, selected by GATR2_HAT_REV. Values are
// RP2040 GPIO numbers (GPn); Pico header pins and HAT connectors are noted.

#pragma once
#include <stdint.h>

#if !defined(GATR2_HAT_REV)
#error "GATR2_HAT_REV is not set"
#elif GATR2_HAT_REV == 2

namespace board
{

// Encoder A/B inputs. enc0 GP0/GP1 (pins 1/2, J2), enc1 GP2/GP3 (pins 4/5, J3),
// enc2 GP4/GP5 (pins 6/7, J4).
constexpr uint8_t kEncPinA[3] = {0, 2, 4};
constexpr uint8_t kEncPinB[3] = {1, 3, 5};

// IMU on SPI1, J7. J7-7 is 3.3 V, J7-1 and J7-8 are GND.
// BNO08X module labels: SDA is MISO, ADO is MOSI, SCL is SCK.
constexpr uint8_t kImuIntPin  = 22; // pin 29, J7-2
constexpr uint8_t kImuMisoPin = 8;  // pin 11, J7-3
constexpr uint8_t kImuMosiPin = 11; // pin 15, J7-4
constexpr uint8_t kImuSckPin  = 10; // pin 14, J7-5
constexpr uint8_t kImuCsPin   = 9;  // pin 12, J7-6

// BNO08X only, direct wires off the PCB. PS1 is tied to 3.3 V.
constexpr uint8_t kImuRstPin  = 12; // pin 16, RST
constexpr uint8_t kImuWakePin = 13; // pin 17, PS0/WAKE

// UART to the Pi, Serial1 (UART0).
constexpr uint8_t kPiTxPin = 16; // pin 21
constexpr uint8_t kPiRxPin = 17; // pin 22

} // namespace board

#else
#error "unsupported GATR2_HAT_REV"
#endif
