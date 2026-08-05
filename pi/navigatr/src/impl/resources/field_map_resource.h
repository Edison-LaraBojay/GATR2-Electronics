// field_map_resource.h
// Field landmark data as a typed shared resource. The core runtime never
// reads it; world prediction and future association implementations
// reference it by resource_id.
//
//   <Resource id="override_field" type="resource/field_map">
//       <Landmark id="center_goal">
//           <NominalPose x_m="1.8" y_m="1.8" heading_deg="0"/>
//           <Tag instance="goal_front" family="tag36h11" observed_id="7"
//                x_m="0.15" y_m="0" heading_deg="180"/>
//       </Landmark>
//   </Resource>

#pragma once
#include "config/field_map.h"
#include "resources/resource_map.h"

namespace navigatr
{

ResourceInstance make_field_map(const ConfigNode& node,
                             ResourceInitializationContext& context, std::string& err);

} // namespace navigatr
