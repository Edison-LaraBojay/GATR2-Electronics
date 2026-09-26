// gyro_window.h
// BNO08X yaw rate reports grouped by sample time into one mean per telemetry
// tick. Times are on the 32-bit microsecond clock and may wrap.

#pragma once
#include <stdint.h>

namespace bno08x
{

// True when a is at or before b. Valid while the two are within 2^31 us.
constexpr bool atOrBefore(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) <= 0; }

// Mean of n rates in Q9 units, in mdeg/s, rounded half away from zero.
// Projected XYZ rates retain fractional Q9 units. n must be nonzero.
int32_t q9MeanToMdps(double sum_q9, uint32_t n);

class GyroWindow {
  public:
    static constexpr uint8_t kCapacity = 32;

    // Report sampled at t_us. Dropped and counted when at or before the
    // previous cut (late) or when the window is full.
    // A projection of three int16 axes can exceed int16 and be fractional.
    // Nonfinite/out-of-range input is ignored; 65536 covers every unit-vector
    // projection of three signed 16-bit readings.
    void add(uint32_t t_us, float rate_q9);

    // Mean in mdeg/s of the reports sampled in (previous cut, cut_us]. Those
    // are consumed, later ones stay. False when there are none.
    bool take(uint32_t cut_us, int32_t& mdps);

    // Drops every pending report. The previous cut stays.
    void clear();

    uint8_t  pending() const { return n_; }
    uint32_t late() const { return late_; }
    uint32_t overflowed() const { return overflowed_; }

  private:
    struct Report {
        uint32_t t_us;
        float    rate_q9;
    };

    Report   buf_[kCapacity] = {};
    uint8_t  n_              = 0;
    bool     has_cut_        = false;
    uint32_t cut_us_         = 0;
    uint32_t late_           = 0;
    uint32_t overflowed_     = 0;
};

} // namespace bno08x
