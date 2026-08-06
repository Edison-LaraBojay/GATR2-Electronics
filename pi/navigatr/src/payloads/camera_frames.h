// payloads/camera_frames.h
// Camera frame publication payload. Downstream perception binds to this
// contract by sensor id; replay, synthetic, and live cameras are
// indistinguishable behind it. The engineering frame id rides along so
// consumers can resolve the camera's mounting transform without knowing
// which device produced the frame.

#pragma once
#include <memory>

#include "core/ids.h"
#include "resources/camera.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kCameraFrame = "sensor.camera_frame";
} // namespace payload_names

struct CameraFramePayload {
    CameraFrameData frame;
    FrameId         engineering_frame;   // resolve in the robot frame map

    // The calibration the frame was captured under; consumers that project
    // or recover pose use exactly this, never a device lookup.
    std::shared_ptr<const CameraIntrinsics> intrinsics;
};

} // namespace navigatr
