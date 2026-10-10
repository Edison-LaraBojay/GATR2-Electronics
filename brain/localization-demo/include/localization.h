// localization.h
// One encoder record plus one IMU reading in, pose out. Portable, no PROS.

#pragma once
#include <cstdint>

#include "encoder_record.h"
#include "encoder_tracker.h"
#include "odometry.h"

namespace localization
{

struct ImuReading {
    bool   valid            = false; // installed, calibrated, finite
    double rotation_degrees = 0;     // VEX rotation, clockwise positive
};

// Lost information since start. Any nonzero count means the pose may be off
// by whatever happened during those events.
struct Health {
    uint32_t restarts   = 0; // Pico restarted
    uint32_t gaps       = 0; // link gaps longer than stale_ms
    uint32_t jumps      = 0;
    uint32_t imu_faults = 0; // records dropped for an invalid IMU reading
    uint32_t duplicates = 0;
    uint32_t advances   = 0;

    bool uncertain() const { return restarts + gaps + jumps + imu_faults > 0; }
};

struct State {
    explicit State(const odometry::Pose& initial, const TrackerLimits& limits)
        : pose(initial), encoders(limits) {}

    odometry::Pose      pose;
    odometry::ImuAnchor imu;
    EncoderTracker      encoders;
    Health              health;
    bool                have_baseline = false;
};

// Integrates one record. Returns what the record was.
StepKind applyRecord(State& state, const odometry::WheelSetup& wheels,
                     const encoder_record::Record& record, uint32_t arrived_ms,
                     const ImuReading& imu);

int32_t channelCount(const encoder_record::Record& record, int channel);
int32_t channelDelta(const Step& step, int channel);

} // namespace localization
