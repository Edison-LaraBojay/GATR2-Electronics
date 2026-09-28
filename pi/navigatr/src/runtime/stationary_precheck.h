// stationary_precheck.h
// The CONTROL precheck before recalibrate and reinitialize: over the last
// kWindowMs every profile encoder moved less than kMaxTravelM of wheel
// travel and, with a Pico IMU, every gyro rate stayed below kMaxRateRadS.
// Each source must be fresh, cover the whole window, and arrive without a
// gap longer than kMaxGapMs or a discontinuity. Missing, repeated, stale or
// discontinuous data never counts as still. Host receipt times; estimation
// worker only.

#pragma once
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/records.h"

namespace navigatr
{

class StationaryPrecheck
{
public:
    static constexpr int64_t kWindowMs    = 300;
    static constexpr int64_t kFreshMs     = 150;
    static constexpr int64_t kMaxGapMs    = 100;
    static constexpr double  kMaxTravelM  = 0.001;
    static constexpr double  kMaxRateRadS = 2.0 * 3.14159265358979323846 / 180.0;

    struct Wheel {
        SensorId sensor;   // EncoderSample
        double   radius_m = 0.0;
    };

    // Replaces the sources and forgets every sample. imu may be empty.
    void configure(const std::vector<Wheel>& wheels, const SensorId& imu);

    // Records every new sample of the configured sources.
    void update(const SensorMap& sensors, MonotonicTime now);

    // why names the first source that is not still.
    bool still(MonotonicTime now, std::string* why = nullptr) const;

private:
    struct Sample {
        int64_t  at_ms = 0;
        double   value = 0.0;   // travel m, or rate rad/s
        uint64_t discontinuity = 0;
    };
    struct Source {
        SensorId           sensor;
        double             radius_m = 0.0;
        bool               gyro     = false;
        bool               seen     = false;
        uint64_t           sequence = 0;
        uint64_t           epoch    = 0;
        std::deque<Sample> samples;
    };

    std::vector<Source> sources_;
};

} // namespace navigatr
