// Report decoding and startup alignment, independent of the SPI transport.
#pragma once

#include <stdint.h>

#include "gravity_alignment.h"
#include "gyro_window.h"

namespace bno08x {

constexpr uint8_t kAccelReportId = 0x01;
constexpr uint8_t kGyroReportId = 0x07;

class Bno08xReports {
  public:
    static constexpr uint32_t kMaxReportAgeUs = 100000;

    // Used on every hub reset or report re-enable. Pending yaw is discarded.
    void reset();
    void acknowledge(uint8_t report_id, uint32_t interval_us);
    bool acknowledged() const { return ack_mask_ == 3; }

    // SH-2 report header + payload. Timestamps are Pico sample/receipt times.
    // Reject malformed, delayed, duplicate and out-of-order reports.
    bool add(uint8_t report_id, const uint8_t* report, uint16_t len,
             uint32_t sample_us, uint32_t received_us);
    bool healthy(uint32_t now_us) const;
    bool aligned() const { return alignment_.ready(); }
    uint32_t alignmentSamples() const { return alignment_.samples(); }
    uint8_t ackMask() const { return ack_mask_; } // bit 0 accel, bit 1 gyro

    // Existing scalar yaw wire units. No output until aligned and healthy.
    bool take(uint32_t cut_us, int32_t& yaw_mdps);

  private:
    struct Stream {
        bool seen = false;
        uint32_t stamp_us = 0;
    };

    uint8_t ack_mask_ = 0;
    Stream accel_;
    Stream gyro_;
    GravityAlignment alignment_;
    GyroWindow window_;
};

} // namespace bno08x
