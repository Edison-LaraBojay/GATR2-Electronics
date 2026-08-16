// payloads/sensor_samples.h
// Sensor publication payloads, SI units, electrically normalized by the
// sensor that produced them. Consumers know these contracts, never the
// hardware behind them; a sample is the same whether it came from Pico
// telemetry, direct SPI, replay, or simulation.

#pragma once
#include <cstdint>

namespace navigatr
{

namespace payload_names
{
constexpr const char* kEncoderSample = "sensor.encoder_sample";
constexpr const char* kImuSample     = "sensor.imu_sample";
} // namespace payload_names

// Accumulated, unwrapped shaft angle. Counts-per-revolution and electrical
// sign are already applied by the sensor.
struct EncoderSample {
    double angle_rad = 0.0;
};

struct ImuSample {
    double yaw_rate_rad_s = 0.0;   // sign normalized, bias not removed

    // Yaw angle integrated by the producer across every decoded packet,
    // bias included. Encoder counts survive packet batching because they
    // are absolute; a latest-rate-only gyro sample does not, so consumers
    // difference this accumulator when it is available instead of
    // integrating published rates. accumulated_epoch bumps whenever the
    // producer dropped an interval; a difference across epochs is a
    // discontinuity, never zero rotation, and consumers must not take it.
    double   accumulated_angle_rad = 0.0;
    uint64_t accumulated_epoch     = 0;
    bool     has_accumulated       = false;
};

} // namespace navigatr
