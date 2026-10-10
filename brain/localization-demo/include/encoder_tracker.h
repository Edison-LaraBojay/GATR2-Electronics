// encoder_tracker.h
// Decides what each received encoder record means for odometry: a new
// baseline, a duplicate, a step to integrate, or a discontinuity that rebases
// instead of integrating. Portable, no PROS.

#pragma once
#include <cstdint>

#include "encoder_record.h"

namespace localization
{

struct TrackerLimits {
    uint32_t stale_ms        = 250;   // no record this long: link lost
    int32_t  max_step_counts = 20000; // a bigger change in one step is a discontinuity
};

enum class StepKind : uint8_t {
    kBaseline,  // first record, or first after rebase(): nothing to integrate
    kDuplicate, // same run, seq equal to or up to 256 behind the last accepted: ignored
    kAdvance,   // integrate the deltas
    kRestart,   // new Pico run id: counts started over, rebased
    kResume,    // first record after a link gap: travel during the gap is lost, rebased
    kJump,      // seq far out of order or counts changed implausibly: rebased
    kImuInvalid, // never from accept(): applyRecord dropped the record, IMU unusable
};

struct Step {
    StepKind kind     = StepKind::kBaseline;
    int32_t  channel0 = 0; // count deltas, kAdvance only
    int32_t  channel1 = 0;
};

class EncoderTracker {
  public:
    explicit EncoderTracker(const TrackerLimits& limits) : limits_(limits) {}

    // arrived_ms: Brain time the record was received.
    Step accept(const encoder_record::Record& record, uint32_t arrived_ms);

    // The next record becomes a new baseline.
    void rebase() { have_last_ = false; }

    // No record accepted within stale_ms, or none yet.
    bool stale(uint32_t now_ms) const;

  private:
    TrackerLimits          limits_;
    bool                   have_last_ = false;
    bool                   ever_      = false;
    encoder_record::Record last_;
    uint32_t               last_ms_ = 0;
};

const char* stepName(StepKind kind);

} // namespace localization
