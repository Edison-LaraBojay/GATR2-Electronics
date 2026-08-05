// pico_channels.h
// Logical channel sensors decoding from the shared Pico telemetry resource.
// The Pico packs several physical sensors into one packet; each configured
// channel here is its own measurement producer with its own id, calibration,
// freshness policy, and payload contract. The generic sensor system does not
// know they share a link.
//
//   <Sensor id="tracking_encoder_a" type="pico_encoder_channel">
//       <Source resource_id="pico_telemetry" channel="0"/>
//       <Calibration counts_per_revolution="4000" invert="false"/>
//       <Freshness stale_after_ms="250"/>
//   </Sensor>
//
//   <Sensor id="robot_imu" type="pico_imu_channel">
//       <Source resource_id="pico_telemetry" channel="imu"/>
//   </Sensor>
//
// Ownership per the architecture: counts per revolution and electrical sign
// are sensor calibration; wheel radius and geometry are preprocessing
// concerns and do not appear here. The encoder publishes an accumulated
// unwrapped shaft angle in radians.
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
