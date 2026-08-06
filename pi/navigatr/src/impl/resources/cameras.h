// cameras.h
// Camera device resources. libcamera_camera is the Pi CSI camera behind
// libcamera; its configuration is fully validated everywhere, and on builds
// without a capture backend the device comes up dead with a warning, the
// same degrade-not-hang policy as an unplugged serial device.
//
//   <Resource id="front_camera_device" type="libcamera_camera">
//       <Device index="0"/>
//       <Capture width_px="1456" height_px="1088" pixel_format="Y8"
//                frame_rate_hz="30"/>
//       <Calibration calibration_status="verified"
//                    calibration_id="front_camera_2026_08">
//           <Intrinsics model="brown_conrady"
//               calibrated_width_px="1456" calibrated_height_px="1088"
//               fx_px="1130.2" fy_px="1129.8" cx_px="728.4" cy_px="544.1"
//               k1="-0.31" k2="0.11" p1="0.0002" p2="-0.0001" k3="-0.02"
//               rms_reprojection_px="0.4"/>
//           <Extrinsic frame_id="front_camera_engineering"/>
//       </Calibration>
//   </Resource>
//
// Startup fails when capture resolution and calibration resolution disagree.

#pragma once
#include <string>

#include "config/config_node.h"
#include "resources/resource_instance.h"
#include "resources/resource_map.h"

namespace navigatr
{

ResourceInstance make_libcamera_camera(const ConfigNode&              node,
                                       ResourceInitializationContext& context,
                                       std::string&                   err);

} // namespace navigatr
