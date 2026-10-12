// payloads/camera_frames.h
// Camera frame publication payload. Downstream perception binds to this
// contract by sensor id; replay, synthetic, and live cameras are
// indistinguishable behind it. The engineering frame id rides along so
// consumers can resolve the camera's mounting transform without knowing
// which device produced the frame, and the calibration the frame was
// captured under rides along too: null intrinsics means metric processing
// is unavailable for this frame, never that a default applies.

#pragma once
#include <memory>
#include <string>

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
    FrameId         engineering_frame;   // empty when the camera has no extrinsic

    // The calibration the frame was captured under; consumers that project
    // or recover pose use exactly this, never a device lookup. Null when
    // the camera runs uncalibrated.
    std::shared_ptr<const CameraIntrinsics> intrinsics;
};

} // namespace navigatr
