// camera.h
// Typed camera contracts. A camera device is a resource that produces
// timestamped frames; the camera resource publishes them as a named output,
// the camera_frame sensor forwards them, and perception consumes them.
//
// Intrinsics are tied to the exact camera, lens, focus, resolution, crop
// mode, pixel format, and distortion model that produced them; a device
// whose capture resolution disagrees with its calibration resolution must
// fail at startup, never scale silently. A device may run uncalibrated:
// frames still flow for preview and 2D decoding, and every consumer sees a
// null intrinsics pointer instead of an invented calibration.
//
// Every calibrated camera declares its engineering frame Ce: origin at the
// optical center, +x the looking direction, +y camera-left, +z camera-up.
// Mounting yaw, pitch, roll, and position live in the robot frame map under
// that frame id; detector-native optical axes never appear outside
// perception.
//
// Frame timing: exposureAt is the backend's best estimate of mid-exposure
// on the host clock, receivedAt is when the frame became available to the
// runtime. When exposure_time_reliable is true, exposure_uncertainty_ms
// bounds the backend's exposure-time estimate; receipt-time fallback has
// no claimed exposure bound and sets exposure_time_reliable false. epoch counts device
// restarts; sequence is monotonic within an epoch. Pixel data is an
// immutable copy owned by the frame, so buffer reuse inside a backend can
// never touch a frame being processed or displayed.

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

    bool hasDistortion() const {
        return k1 != 0.0 || k2 != 0.0 || p1 != 0.0 || p2 != 0.0 || k3 != 0.0;
    }
};

struct CameraFrameData {
    uint32_t      sequence = 0;   // monotonically increasing within an epoch
    uint64_t      epoch    = 0;   // device restart generation
    MonotonicTime exposureAt;     // host clock, mid-exposure estimate
    MonotonicTime receivedAt;     // host clock, frame available to the runtime
    int64_t       exposure_uncertainty_ms = 0;
    bool          exposure_time_reliable  = true;

    int width_px  = 0;
    int height_px = 0;

    // Y8 pixels, packed rows (stride == width). May be empty for injected
    // frames that carry only detections downstream.
    std::shared_ptr<const std::vector<uint8_t>> y8;
};

class CameraDevice
{
public:
    virtual ~CameraDevice() = default;

    // False when the device could not open or died; consumers degrade.
    virtual bool alive() const = 0;

    // Why the device is not alive, or other backend notes.
    virtual std::string diagnostic() const { return {}; }

    // Null when the device runs uncalibrated.
    virtual const CameraIntrinsics* intrinsics() const = 0;

    // The engineering frame id this camera's extrinsic calibration names;
    // empty when there is none.
    virtual FrameId engineeringFrame() const = 0;

    // The newest frame identified later than (after_epoch, after_sequence),
    // or nothing.
    virtual std::optional<CameraFrameData> latestFrame(uint64_t after_epoch,
                                                       uint32_t after_sequence) = 0;

    virtual void reset() {}
};

} // namespace navigatr
