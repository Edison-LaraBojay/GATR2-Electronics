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
};

} // namespace navigatr
