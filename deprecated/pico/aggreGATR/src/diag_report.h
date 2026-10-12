// diag_report.h
// The optional diagnostic frame (translagatr::PicoDiag) from state the
// firmware already keeps: pin levels read back, IMU driver counters, Pi
// command link rejections and skipped sensor ticks. No hardware
// dependencies; main.cpp samples, this packs.

#pragma once
#include <stddef.h>
#include <stdint.h>

#include "frames.h"
#include "imu.h"

namespace pilink
{

// One pin of the frame: its translagatr::PicoDiagPin bit and RP2040 GPIO.
struct DiagPin {
    uint16_t bit;
    uint8_t  gpio;
};

// Levels of the listed pins through read(gpio), true = HIGH. Every listed
// bit is set in known; unlisted bits stay clear in both.
template <typename Read>
void readPins(const DiagPin* pins, size_t count, Read read, uint16_t& levels, uint16_t& known) {
    levels = 0;
    known  = 0;
    for (size_t i = 0; i < count; ++i) {
        known = static_cast<uint16_t>(known | pins[i].bit);
        if (read(pins[i].gpio)) {
            levels = static_cast<uint16_t>(levels | pins[i].bit);
        }
    }
}

struct DiagInputs {
    uint16_t     boot_id    = 0;
    uint8_t      seq        = 0;
    uint8_t      firmware   = translagatr::kPicoFirmwareUnknown;
    uint16_t     pins       = 0;
    uint16_t     pins_known = 0;
    bool         imu_present = false; // this build drives an IMU
    imu::Counters imu;
    uint32_t     link_rx_bad   = 0; // FrameReader length + check errors on the Pi link
    uint32_t     ticks_skipped = 0;
};

// Counters keep their low bits and wrap; the IMU error saturates to int8;
// report_age_ms saturates at 0xFFFE, 0xFFFF when no report was accepted.
translagatr::PicoDiag makeDiag(const DiagInputs& in);

} // namespace pilink
