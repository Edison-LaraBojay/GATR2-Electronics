// camera_channel.h
// camera_frame sensor: polls a CameraDevice resource into the standard
// sensor records. A new device frame publishes a CameraFramePayload with
// measuredAt set to the exposure timestamp (host clock); a live camera
// between frames is a healthy quiet cycle; a dead camera is unavailable,
// never a silent absence.
//
//   <Sensor id="front_camera" type="camera_frame">
//       <Source resource_id="front_camera_device"/>
//   </Sensor>

#pragma once
#include <optional>
#include <string>

#include "contracts/sensor.h"

namespace navigatr
{

std::optional<SensorExecutable> make_camera_frame(const ConfigNode&            node,
                                                  SensorInitializationContext& context,
                                                  std::string&                 err);

} // namespace navigatr
