// field_map_resource.h
// Field landmark data as a typed shared resource. The core runtime never
// reads it; world prediction, association, and target implementations
// reference it by resource_id.
//
//   <Resource id="game_field" type="field_map">
//       <Landmark id="center_goal">
//           <NominalPose calibration_status="verified"
//               x_m="1.7832" y_m="1.7832" heading_deg="0"/>
//           <ApproachFrame id="center_goal_east_face"
//                          calibration_status="verified">
//               <PoseOfApproachFrameInLandmark x_m="0.14" y_m="0" z_m="0"
//                   roll_deg="0" pitch_deg="0" yaw_deg="0"/>
//           </ApproachFrame>
//           <TagMount instance_id="center_goal_east_tag"
//                     calibration_status="verified" family="tag36h11"
//                     observed_id="0" detection_size_m="0.06">
//               <PoseOfTagSurfaceInLandmark x_m="0.14" y_m="0" z_m="0.25"
//                   roll_deg="0" pitch_deg="0" yaw_deg="0"/>
//           </TagMount>
//       </Landmark>
//   </Resource>

#pragma once
#include "config/field_map.h"
#include "resources/resource_store.h"

namespace navigatr
{

ResourceInstance make_field_map(const ConfigNode& node,
                             ResourceInitializationContext& context, std::string& err);

} // namespace navigatr
