// pico_channels.h
// Logical channel sensors decoding from the shared Pico telemetry resource.
// The Pico packs several physical sensors into one packet; each configured
// channel here is its own measurement producer with its own id, calibration,
// and payload contract. The generic sensor system does not know they share
// a link.
//
//   <Sensor id="tracking_encoder_a" type="sensor/pico_encoder_channel">
//       <Source resource_id="pico_telemetry" channel="0"/>
//       <Calibration counts_per_revolution="4000" invert="false"/>
//   </Sensor>
//
//   <Sensor id="robot_imu" type="sensor/pico_imu_channel">
//       <Source resource_id="pico_telemetry" channel="imu"/>
//   </Sensor>
//
// Per section 11.1 ownership: counts per revolution and electrical sign are
// sensor calibration; wheel radius and geometry are preprocessing concerns
// and do not appear here. The encoder publishes an accumulated unwrapped
// shaft angle in radians.

#pragma once
#include <memory>
#include <string>

#include "contracts/sensor.h"
#include "payloads/sensor_samples.h"

namespace navigatr
{

class PicoTelemetry;

class PicoEncoderChannelSensor : public Sensor
{
public:
    static std::unique_ptr<Sensor> create(const ConfigNode& node,
                                          SensorInitializationContext& context,
                                          std::string& err);

    SensorPollResult poll(const SensorExecutionInput& input) override;

    const PayloadDescriptor& outputPayload() const override { return payload_; }

    void reset() override;

private:
    std::shared_ptr<PicoTelemetry> telemetry_;
    int                            channel_ = -1;
    double                         radians_per_count_ = 0.0;   // signed
    PayloadDescriptor              payload_;

    bool     have_prev_    = false;
    int32_t  prev_counts_  = 0;
    double   angle_rad_    = 0.0;
    uint64_t last_updates_ = 0;
};

class PicoImuChannelSensor : public Sensor
{
public:
    static std::unique_ptr<Sensor> create(const ConfigNode& node,
                                          SensorInitializationContext& context,
                                          std::string& err);

    SensorPollResult poll(const SensorExecutionInput& input) override;

    const PayloadDescriptor& outputPayload() const override { return payload_; }

    void reset() override;

private:
    std::shared_ptr<PicoTelemetry> telemetry_;
    bool                           invert_ = false;
    PayloadDescriptor              payload_;
    uint64_t                       last_updates_ = 0;
};

} // namespace navigatr
