// pi_link.h
// Pico -> Pi link helpers: the boot identity and the status frame schedule.
// No hardware dependencies.

#pragma once
#include <stdint.h>

namespace pilink
{

// Nonzero boot id from hardware entropy mixed with a time stamp.
uint16_t makeBootId(uint32_t entropy, uint64_t time_us);

// Wire time of a frame at 10 bits per byte, rounded up.
constexpr uint32_t airtimeUs(uint32_t bytes, uint32_t baud) {
    return static_cast<uint32_t>((static_cast<uint64_t>(bytes) * 10000000u + baud - 1) / baud);
}

// Status frames go every period_us, and at most min_gap_us after a request,
// only while the TX FIFO is empty and at least guard_us before the next
// sensor tick, so a due sensor frame always finds the FIFO empty.
class StatusSchedule {
  public:
    StatusSchedule(uint32_t period_us, uint32_t min_gap_us, uint32_t guard_us);

    // A command was answered or the IMU state changed.
    void request() { requested_ = true; }

    bool ready(uint32_t now_us, uint32_t next_tick_us, bool tx_empty) const;

    void sent(uint32_t now_us);

  private:
    uint32_t period_us_;
    uint32_t min_gap_us_;
    uint32_t guard_us_;
    uint32_t last_us_   = 0;
    bool     sent_      = false;
    bool     requested_ = true;
};

} // namespace pilink
