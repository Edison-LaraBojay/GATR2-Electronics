// cameras.h
// Camera device resources.
//
// libcamera_camera is the Raspberry Pi camera behind libcamera. It exists
// only in a binary configured with NAVIGATR_WITH_LIBCAMERA; selecting it
// elsewhere is a build error naming the missing backend, never a device
// that silently produces nothing.
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
//       </Calibration>                         optional: absent = uncalibrated
//       <Output id="frame"/>                   optional, default frame
//   </Resource>
//
// Without a Calibration element the camera runs uncalibrated: frames flow
// for preview and 2D decoding, consumers see null intrinsics, and no
// metric tag pose is ever solved from them. Startup fails when capture
// resolution and calibration resolution disagree. The resource publishes
// one output, a CameraFramePayload per new device frame.

#pragma once
#include <memory>
#include <string>

#include "config/config_node.h"
#include "resources/camera.h"
#include "resources/resource_instance.h"
#include "resources/resource_store.h"

namespace navigatr
{

// What every camera factory parses before choosing a backend.
struct CameraCaptureConfig {
    long        device_index = 0;
    long        width_px     = 0;
    long        height_px    = 0;
    std::string pixel_format = "Y8";
    double      frame_rate_hz = 0.0;

    bool             calibrated = false;
    std::string      calibration_id;
    CameraIntrinsics intrinsics;
    FrameId          engineering_frame;   // empty when uncalibrated
};

// Parses Device, Capture and the optional Calibration. False with err.
bool parseCameraConfig(const ConfigNode& node, CameraCaptureConfig& out,
                       std::string& err);

ResourceInstance make_libcamera_camera(const ConfigNode&              node,
                                       ResourceInitializationContext& context,
                                       std::string&                   err);

// Any CameraDevice as an executable resource publishing frames under
// output. Test and synthetic cameras go through this too.
ResourceInstance cameraResource(std::shared_ptr<CameraDevice> device, OutputId output);

// Reads the optional <Output id=.../> child; "frame" when absent.
bool cameraOutputId(const ConfigNode& node, OutputId& out, std::string& err);

} // namespace navigatr
