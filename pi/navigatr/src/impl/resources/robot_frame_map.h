// robot_frame_map.h
// Robot geometry resource: named frames measured relative to the robot pose
// origin (or chained through other frames), full SE(3). Every value is
// required and calibration gated; an unmeasured frame must fail the build.
//
//   <Resource id="robot_geometry" type="robot_frame_map">
//       <Frame id="front_camera_engineering" parent_frame_id="robot_body"
//              calibration_status="verified">
//           <PoseOfChildInParent x_m="0.18" y_m="0.0" z_m="0.31"
//               roll_deg="0" pitch_deg="12" yaw_deg="0"/>
//       </Frame>
//   </Resource>

#pragma once
#include <string>

#include "config/config_node.h"
#include "resources/resource_instance.h"
#include "resources/resource_map.h"

namespace navigatr
{

ResourceInstance make_robot_frame_map(const ConfigNode&              node,
                                      ResourceInitializationContext& context,
                                      std::string&                   err);

} // namespace navigatr
