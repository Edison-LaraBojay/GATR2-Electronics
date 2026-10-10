// encoder_tracker.cpp

#include "encoder_tracker.h"

#include <cstdlib>

namespace localization
{

namespace
{

// Up to 256 records behind the last accepted one: old data, ignored.
constexpr uint16_t kLateWindowStart = 0xFF00;

// Wrap safe: counts are free running int32.
int32_t countDelta(int32_t now, int32_t before) {
    return static_cast<int32_t>(static_cast<uint32_t>(now) - static_cast<uint32_t>(before));
}

bool tooBig(int32_t delta, int32_t limit) {
    return std::llabs(static_cast<long long>(delta)) > limit;
}

} // namespace

Step EncoderTracker::accept(const encoder_record::Record& record, uint32_t arrived_ms) {
    Step step;
    // The last accepted record again, or one slightly older that arrived late.
    const uint16_t seq_step = static_cast<uint16_t>(record.seq - last_.seq);
    if (have_last_ && record.run_id == last_.run_id &&
        (seq_step == 0 || seq_step >= kLateWindowStart)) {
        step.kind = StepKind::kDuplicate;
        return step;
    }

    if (!have_last_) {
        step.kind = StepKind::kBaseline;
    } else if (record.run_id != last_.run_id) {
        step.kind = StepKind::kRestart;
    } else if (arrived_ms - last_ms_ > limits_.stale_ms) {
        step.kind = StepKind::kResume;
    } else {
        const int32_t d0 = countDelta(record.channel0, last_.channel0);
        const int32_t d1 = countDelta(record.channel1, last_.channel1);
        if (seq_step >= 0x8000 || tooBig(d0, limits_.max_step_counts) ||
            tooBig(d1, limits_.max_step_counts)) {
            step.kind = StepKind::kJump;
        } else {
            step.kind     = StepKind::kAdvance;
            step.channel0 = d0;
            step.channel1 = d1;
        }
    }

    last_      = record;
    last_ms_   = arrived_ms;
    have_last_ = true;
    ever_      = true;
    return step;
}

bool EncoderTracker::stale(uint32_t now_ms) const {
    return !ever_ || now_ms - last_ms_ > limits_.stale_ms;
}

const char* stepName(StepKind kind) {
    switch (kind) {
    case StepKind::kBaseline:
        return "baseline";
    case StepKind::kDuplicate:
        return "duplicate";
    case StepKind::kAdvance:
        return "advance";
    case StepKind::kRestart:
        return "pico restart";
    case StepKind::kResume:
        return "resume after gap";
    case StepKind::kJump:
        return "jump";
    case StepKind::kImuInvalid:
        return "imu invalid";
    }
    return "?";
}

} // namespace localization
