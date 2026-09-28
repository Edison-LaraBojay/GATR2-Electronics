// stillness.h
// Stationary evidence and IMU bias calibration as an observation function
// reports them: what publishing, inspection and the executor read. The
// StationaryWindow and GyroBiasCalibration helpers
// (impl/localization/stationary_window.h) produce it.

#pragma once
#include <cstdint>
#include <string>

namespace navigatr
{

// Values equal gatr2::CalibrationState.
enum class BiasCalibration : uint8_t {
    kNone         = 0,   // nothing to calibrate
    kRunning      = 1,   // collecting a stationary window
    kDone         = 2,
    kWaitingStill = 3,   // movement seen; the window restarts when still
    kWaitingData  = 4,   // samples missing, stale or discontinuous
    kFailed       = 5,   // no qualified window within the attempt bound
};

struct StillnessStatus {
    bool            monitored   = false;   // the function runs a stationary window
    bool            stationary  = false;   // qualified, and no movement or gap since
    BiasCalibration calibration = BiasCalibration::kNone;
    std::string     reason;                // last window restart or calibration note

    int64_t  progress_ms = 0;   // sample time the current window spans
    int64_t  window_ms   = 0;   // what a window needs
    uint64_t windows     = 0;   // qualified windows
    uint64_t restarts    = 0;   // window restarts
    uint64_t movements   = 0;   // restarts for movement (waiting for stillness)
    uint32_t attempts    = 0;   // calibration starts
    uint32_t steps       = 0;   // bias maintenance steps
    bool     has_bias    = false;
    double   bias_rad_s  = 0.0;
};

inline const char* toString(BiasCalibration state) {
    switch (state) {
    case BiasCalibration::kNone: return "none";
    case BiasCalibration::kRunning: return "collecting";
    case BiasCalibration::kDone: return "done";
    case BiasCalibration::kWaitingStill: return "waiting for stillness";
    case BiasCalibration::kWaitingData: return "waiting for data";
    case BiasCalibration::kFailed: return "failed";
    }
    return "unknown";
}

} // namespace navigatr
