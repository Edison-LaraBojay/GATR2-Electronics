// attitude_channel.h
// attitude_channel sensor: forwards an attitude resource output into the
// standard sensor records, rotated from the sensor's mounting frame into
// the robot body frame. The mounting calibration is the one place the
// IMU-to-body orientation lives; downstream models see body attitude only.
//
//   <Sensor id="robot_attitude" type="attitude_channel">
//       <Source resource_id="rig" output_id="attitude"/>
//       <Mounting calibration_status="verified"
//                 roll_deg="0" pitch_deg="0" yaw_deg="0"/>   optional, identity
//       <Freshness stale_after_ms="250"/>                    optional
//   </Sensor>
//
// Mounting gives the sensor frame's orientation in the body frame,
// R_body_sensor. A sample q_reference_sensor becomes
// q_reference_body = q_reference_sensor * inverse(q_body_sensor).

#pragma once
#include <optional>
#include <string>

#include "contracts/sensor.h"

namespace navigatr
{

std::optional<SensorExecutable> make_attitude_channel(const ConfigNode&            node,
                                                      SensorInitializationContext& context,
                                                      std::string&                 err);

} // namespace navigatr
