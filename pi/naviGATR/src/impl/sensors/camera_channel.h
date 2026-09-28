// camera_channel.h
// camera_frame sensor: forwards a camera resource's frame output into the
// standard sensor records. A new frame publishes the CameraFramePayload
// with measuredAt at exposure (host clock); a live camera between frames is
// a healthy quiet cycle; a dead camera is unavailable, never a silent
// absence. Perception binds to this sensor id.
//
//   <Sensor id="front_camera" type="camera_frame">
//       <Source resource_id="front_camera_device" output_id="frame"/>
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
