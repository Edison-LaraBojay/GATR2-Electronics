// camera.h
// Typed camera contracts. A camera device is a resource that produces
// timestamped frames; the camera_frame sensor polls it into the standard
// sensor records, and perception implementations consume the frames.
//
// Intrinsics are tied to the exact camera, lens, focus, resolution, crop
// mode, pixel format, and distortion model that produced them; a device
// whose capture resolution disagrees with its calibration resolution must
// fail at startup, never scale silently.
//
// Every camera declares its engineering frame Ce: origin at the optical
// center, +x the looking direction, +y camera-left, +z camera-up. Mounting
// yaw, pitch, roll, and position live in the robot frame map under that
// frame id; detector-native optical axes never appear outside perception.

#pragma once
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/time.h"

namespace navigatr
{

struct CameraIntrinsics {
    std::string model;   // brown_conrady
    int         calibrated_width_px  = 0;
    int         calibrated_height_px = 0;
    double      fx_px = 0.0;
    double      fy_px = 0.0;
    double      cx_px = 0.0;
    double      cy_px = 0.0;
    double      k1 = 0.0;
    double      k2 = 0.0;
    double      p1 = 0.0;
    double      p2 = 0.0;
    double      k3 = 0.0;
    double      rms_reprojection_px = 0.0;
};

struct CameraFrameData {
    uint32_t      sequence = 0;   // monotonically increasing per device
    MonotonicTime exposureAt;     // host clock at exposure
    int           width_px  = 0;
    int           height_px = 0;

    // Y8 pixels, row major. May be empty for injected/synthetic frames that
    // carry only detections downstream.
    std::shared_ptr<const std::vector<uint8_t>> y8;
};

class CameraDevice
{
public:
    virtual ~CameraDevice() = default;

    // False when the device could not open or died; consumers degrade.
    virtual bool alive() const = 0;

    virtual const CameraIntrinsics& intrinsics() const = 0;

    // The engineering frame id this camera's extrinsic calibration names.
    virtual FrameId engineeringFrame() const = 0;

    // The newest frame with sequence greater than after, or nothing.
    virtual std::optional<CameraFrameData> latestFrame(uint32_t after) = 0;
};

} // namespace navigatr
