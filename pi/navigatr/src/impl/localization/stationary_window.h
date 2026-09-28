// stationary_window.h
// Stationary evidence shared by every IMU bias path and the stationary
// status: the HeadingConstraint of tracking_wheel_motion,
// imu_heading_increment and the Brain IMU bench models.
//
// A window collects new samples from each configured source: wheel travel
// (m), a gyro rate (rad/s, with the producer's accumulated angle when it has
// one) and a continuous rotation (rad, the Brain VEX IMU). A sample counts
// only when its sequence advanced; a retained record never does. The window
// restarts waiting for data when a source changes identity (record epoch,
// discontinuity or source epoch), when two of its samples are more than
// max_gap_ms apart or out of order, when the gyro accumulator drops an
// interval, or when a source goes quiet for max_gap_ms of host receipt time
// (poll). It restarts waiting for stillness on movement: a wheel travel range
// above still_travel_m, a gyro rate above max_rate or further than still_rate
// from the window mean, a rotation range above still_rate times the window.
// It qualifies once every source spans window_ms of its own sample time with
// min_samples samples; the next window starts from the newest samples. A
// window that qualified and has not restarted since is stationary.
//
// GyroBiasCalibration turns qualified windows into a gyro bias. After a
// start the first qualified window gives the bias, its accumulated angle over
// its elapsed time (the trapezoid of its rates without an accumulator). No
// qualified window within attempt_ms fails the attempt until the next start.
// Once calibrated each later qualified window moves the bias by
// maintain_weight of the difference, at most maintain_step per window; a
// window that restarts contributes nothing.
//
// Estimation worker only.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "core/time.h"
#include "math/angles.h"
#include "state/stillness.h"

namespace navigatr
{

struct StillnessConfig {
    int64_t window_ms        = 2000;
    long    min_samples      = 20;   // per source
    int64_t max_gap_ms       = 100;
    double  still_travel_m   = 0.001;
    double  still_rate_rad_s = degToRad(1.0);
    double  max_rate_rad_s   = degToRad(5.0);
};

// Reads window_ms, still_rate_dps, max_rate_dps and evidence_gap_ms from
// node into c, keeping c's values as defaults, and checks them.
bool readStillness(const ConfigNode& node, StillnessConfig& c, std::string& err);

struct StillSample {
    uint64_t      sequence      = 0;
    uint64_t      epoch         = 0;   // record epoch
    uint64_t      discontinuity = 0;   // encoder discontinuity or source epoch
    MonotonicTime at;                  // sample time on the source clock
    MonotonicTime received;            // host receipt
    double        value = 0.0;         // travel m, rate rad/s or rotation rad

    bool     has_accumulated   = false;   // gyro only
    double   accumulated       = 0.0;     // rad
    uint64_t accumulated_epoch = 0;
};

class StationaryWindow
{
public:
    enum class Kind : uint8_t { kWheel, kGyro, kRotation };
    enum class Phase : uint8_t { kWaitingData, kWaitingStill, kCollecting };

    struct Qualified {
        bool   gyro        = false;
        double gyro_rate   = 0.0;   // rad/s: accumulated angle over elapsed time
        double elapsed_s   = 0.0;   // the gyro's span
    };

    // Replaces the sources; source i is the i-th add. Waiting for data.
    void   configure(const StillnessConfig& config);
    size_t addSource(Kind kind, const std::string& name);

    const StillnessConfig& config() const { return config_; }
    bool                   empty() const { return sources_.empty(); }

    // False when the sample is not new evidence.
    bool add(size_t source, const StillSample& s);

    // Host freshness of every source; once per cycle after the samples.
    void poll(MonotonicTime now);

    void restart(Phase phase, const std::string& why);

    Phase              phase() const;
    bool               stationary() const { return stationary_; }
    const std::string& reason() const { return reason_; }
    int64_t            progressMs() const;
    uint64_t           windows() const { return windows_; }
    uint64_t           restarts() const { return restarts_; }

    // The window the latest add qualified, once.
    bool takeQualified(Qualified& out);

private:
    struct Source {
        Kind        kind = Kind::kWheel;
        std::string name;

        bool          seen = false;
        uint64_t      sequence      = 0;
        uint64_t      epoch         = 0;
        uint64_t      discontinuity = 0;
        MonotonicTime received;
        bool          stale = false;

        long          count = 0;   // samples in the current window
        MonotonicTime first_at;
        MonotonicTime last_at;
        double        lo = 0.0, hi = 0.0, sum = 0.0;
        double        last_value = 0.0;
        double        integral   = 0.0;   // gyro trapezoid, rad
        bool          accumulated = false;   // every window sample had one, same epoch
        double        first_accum = 0.0, last_accum = 0.0;
        uint64_t      accum_epoch = 0;
    };

    void    begin(Source& s, const StillSample& sample);
    void    append(Source& s, const StillSample& sample);
    bool    moves(const Source& s, const StillSample& sample, std::string& why) const;
    void    qualify();
    int64_t span(const Source& s) const;

    StillnessConfig     config_;
    std::vector<Source> sources_;
    Phase               restart_phase_ = Phase::kWaitingData;
    bool                stationary_    = false;
    std::string         reason_        = "no samples yet";
    uint64_t            windows_       = 0;
    uint64_t            restarts_      = 0;
    bool                qualified_     = false;
    Qualified           last_;
};

class GyroBiasCalibration
{
public:
    struct Config {
        bool    enabled             = true;   // false: the bias stays zero, nothing collected
        int64_t attempt_ms          = 60000;
        double  maintain_weight     = 0.2;
        double  maintain_step_rad_s = degToRad(0.2);
    };

    // Reads attempt_s from node into c (default kept) and checks it.
    static bool read(const ConfigNode& node, Config& c, std::string& err);

    void configure(const Config& config);

    // A new attempt; the bias is invalid until a window qualifies.
    void start(const std::string& why);

    // A qualified window's rate: the bias while calibrating, a bounded step
    // once calibrated.
    void qualified(double rate_rad_s, MonotonicTime now);

    // The attempt bound, host clock.
    void poll(MonotonicTime now);

    bool               enabled() const { return config_.enabled; }
    bool               calibrated() const { return calibrated_; }
    bool               failed() const { return failed_; }
    double             bias() const { return bias_; }
    uint32_t           attempts() const { return attempts_; }
    uint32_t           steps() const { return steps_; }
    const std::string& note() const { return note_; }

    BiasCalibration state(StationaryWindow::Phase phase) const;

private:
    Config        config_;
    bool          calibrated_ = false;
    bool          failed_     = false;
    double        bias_       = 0.0;
    MonotonicTime attempt_start_;   // host clock; unset until the first poll
    uint32_t      attempts_ = 0;
    uint32_t      steps_    = 0;
    std::string   note_;
};

// What a function with a window (and optionally a calibration) reports.
StillnessStatus stillnessOf(const StationaryWindow& window,
                            const GyroBiasCalibration* calibration);

} // namespace navigatr
