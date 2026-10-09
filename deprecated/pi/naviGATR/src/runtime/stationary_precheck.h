// stationary_precheck.h
// The CONTROL precheck before recalibrate, reinitialize and an acquisition
// restart: over the last kWindowMs every profile encoder moved less than
// kMaxTravelM of wheel travel and, with a Pico IMU, every gyro rate stayed
// below kMaxRateRadS; with the Brain VEX IMU its rotation changed less than
// kMaxRateRadS over the window. Each source must be fresh, cover the whole
// window, and arrive without a gap longer than kMaxGapMs or a
// discontinuity. Missing, repeated, stale or discontinuous data never counts
// as still. Host receipt times; estimation worker only.

#pragma once
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/records.h"
#include "resources/brain_imu_bench.h"

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

    // Replaces the sources and forgets every sample. imu and vex may be
    // empty.
    void configure(const std::vector<Wheel>& wheels, const SensorId& imu,
                   std::shared_ptr<const BrainImuBench> vex = nullptr);

    // Records every new sample of the configured sources.
    void update(const SensorMap& sensors, MonotonicTime now);

    // why names the first source that is not still.
    bool still(MonotonicTime now, std::string* why = nullptr) const;

private:
    enum class Kind : uint8_t { kWheel, kGyro, kRotation };
    struct Sample {
        int64_t  at_ms = 0;
        double   value = 0.0;   // travel m, rate rad/s or rotation rad
        uint64_t discontinuity = 0;
    };
    struct Source {
        SensorId           sensor;
        Kind               kind     = Kind::kWheel;
        double             radius_m = 0.0;
        bool               seen     = false;
        uint64_t           sequence = 0;
        uint64_t           epoch    = 0;
        std::deque<Sample> samples;
    };

    void record(Source& s, const Sample& sample, uint64_t sequence, uint64_t epoch);

    std::vector<Source>                  sources_;
    std::shared_ptr<const BrainImuBench> vex_;
};

} // namespace navigatr
