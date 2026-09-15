// payloads/pico_telemetry_samples.h
// Named outputs of the pico_telemetry resource: decoded wire values, not
// yet calibrated. Channel sensors turn these into SI samples; nothing
// downstream of a sensor sees them.

#pragma once
#include <cstdint>

namespace navigatr
{

namespace payload_names
{
constexpr const char* kPicoEncoderCounts = "pico.encoder_counts";
constexpr const char* kPicoGyroRate      = "pico.gyro_rate";
} // namespace payload_names

// Absolute counter as the Pico sends it; wraps at 32 bits.
struct PicoEncoderCounts {
    int32_t counts = 0;
};

struct PicoGyroRate {
    int32_t rate_mdps = 0;   // millidegrees per second, latest packet

    // Integrated over every decoded packet in millidegrees, so a drained
    // batch loses no rotation. accumulated_epoch bumps on every dropped
    // interval; a difference across epochs is a discontinuity.
    double   accumulated_mdeg  = 0.0;
    uint64_t accumulated_epoch = 0;
};

} // namespace navigatr
