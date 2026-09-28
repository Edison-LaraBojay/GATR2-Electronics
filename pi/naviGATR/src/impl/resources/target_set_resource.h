// target_set_resource.h
// Target declarations as a typed shared resource, validated at build against
// the field map (landmarks, approach frames, tag mounts) and the robot frame
// map (controlled frames). Target resolution consumes the selected side,
// stand-off, and vision policy from this set.
//
//   <Resource id="targets" type="target_set">
//       <FieldMap resource_id="game_field"/>
//       <RobotFrames resource_id="robot_geometry"/>
//
//       <Target id="back_to_center_goal" type="landmark_relative" wire_id="1"
//               landmark_id="center_goal"
//               approach_frame_id="center_goal_east_face"
//               controlled_frame_id="rear_contact">
//           <DesiredControlledFramePose calibration_status="verified"
//               x_m="0.05" y_m="0" heading_deg="180"/>
//           <VisionCorrection type="acquire_once"
//               on_acquisition_timeout="use_nominal_target"
//               minimum_consistent_observations="3"
//               maximum_observation_age_ms="150"
//               maximum_robot_angular_speed_deg_s="45"
//               acquisition_timeout_ms="1500"
//               consistency_translation_m="0.05"
//               consistency_heading_deg="3">
//               <PreferredCamera sensor_id="front_camera"/>
//               <AllowedTagMount instance_id="center_goal_east_tag"/>
//           </VisionCorrection>
//       </Target>
//
//       <Target id="scored_to_matchloader" type="robot_relative" wire_id="2"
//               controlled_frame_id="robot_body">
//           <Snapshot frame_id="robot_body" delta_axes="robot_at_activation"/>
//           <Delta x_m="-0.6" y_m="0.3" heading_deg="0"/>
//           <VisionCorrection type="none"/>
//       </Target>
//   </Resource>

#pragma once
#include <string>

#include "config/config_node.h"
#include "resources/resource_instance.h"
#include "resources/resource_store.h"

namespace navigatr
{

ResourceInstance make_target_set(const ConfigNode&              node,
                                 ResourceInitializationContext& context,
                                 std::string&                   err);

} // namespace navigatr
