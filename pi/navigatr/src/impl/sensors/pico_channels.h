// pico_channels.h
// Channel sensors over the pico_telemetry resource outputs. Each configured
// sensor is its own measurement producer with its own id, calibration,
// freshness policy, and payload contract; it reads one named output of the
// ResourceMap and never touches the link or the decoder.
//
//   <Sensor id="tracking_encoder_a" type="pico_encoder_channel">
//       <Source resource_id="pico_telemetry" output_id="encoder_a"/>
//       <Calibration counts_per_revolution="4000" invert="false"/>
//       <Freshness stale_after_ms="250"/>
//   </Sensor>
//
//   <Sensor id="robot_imu" type="pico_imu_channel">
//       <Source resource_id="pico_telemetry" output_id="imu"/>
//       <Calibration invert="false"/>
//   </Sensor>
//
// Ownership per the architecture: counts per revolution and electrical sign
// are sensor calibration; wheel radius and geometry belong to the tracking
// motion observation model. The encoder publishes an accumulated
// unwrapped shaft angle in radians. The IMU sensor converts wire units and
// forwards the resource's accumulated angle; bias removal stays downstream.
//
// Freshness: a link that stays open but goes silent turns the sensor
// Unavailable after stale_after_ms (default 250, 0 disables) instead of
// staying Valid forever. History is retained either way.

#pragma once
#include <memory>
#include <optional>
#include <string>

#include "contracts/sensor.h"
#include "payloads/sensor_samples.h"

namespace navigatr
{

std::optional<SensorExecutable> make_pico_encoder_channel(
    const ConfigNode& node, SensorInitializationContext& context, std::string& err);

std::optional<SensorExecutable> make_pico_imu_channel(const ConfigNode& node,
                                                      SensorInitializationContext& context,
                                                      std::string& err);

} // namespace navigatr
