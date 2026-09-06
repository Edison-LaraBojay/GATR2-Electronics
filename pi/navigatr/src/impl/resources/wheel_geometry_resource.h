// wheel_geometry_resource.h
// Tracking-wheel geometry resource. Every value is required, calibration
// gated, and measured on the assembled robot; an unmeasured wheel must fail
// the build, never run as zero.
//
//   <Resource id="wheel_geometry" type="wheel_geometry">
//       <Wheel id="left_wheel" sensor_id="tracking_encoder_a" label="left"
//              calibration_status="verified"
//              radius_m="0.0254"
//              position_x_m="0.000" position_y_m="0.130"
//              measurement_angle_deg="0" direction="positive"/>
//   </Resource>

#pragma once
#include <string>

#include "config/config_node.h"
#include "resources/resource_instance.h"
#include "resources/resource_map.h"

namespace navigatr
{

ResourceInstance make_wheel_geometry(const ConfigNode&              node,
                                     ResourceInitializationContext& context,
                                     std::string&                   err);

} // namespace navigatr
