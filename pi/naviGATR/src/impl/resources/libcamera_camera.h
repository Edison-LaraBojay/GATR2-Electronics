// libcamera_camera.h
// The Raspberry Pi camera behind libcamera as a CameraDevice. Compiled only
// with NAVIGATR_WITH_LIBCAMERA (Linux, libcamera-dev); this header keeps
// every libcamera type behind a pimpl so cameras.cpp can include it without
// the library's headers.
//
// open() starts capture at exactly the configured size and rate, YUV420
// from the ISP with the Y plane published as the Y8 the runtime wants. A
// capture thread copies each completed frame out of the driver buffer and
// requeues the buffer at once, so a frame handed to the runtime is never
// touched by later capture. Frame timing follows camera.h: exposureAt is
// derived from the driver's SensorTimestamp and ExposureTime metadata when
// present, receipt time with exposure_time_reliable false otherwise.
//
// Nothing in this backend has run on a Pi yet; docs/pi_camera_setup.md
// lists the checks still to perform.

#pragma once
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "impl/resources/cameras.h"
#include "resources/camera.h"

namespace navigatr
{

class LibcameraCamera : public CameraDevice
{
public:
    // Opens, configures and starts the camera at config.device_index. Null
    // with err when any step fails; nothing partial stays open.
    static std::shared_ptr<CameraDevice> open(const CameraCaptureConfig& config,
                                              std::string&               err);

    // A device whose initial open failed. reset() retries acquisition;
    // until then alive() is false and diagnostic() supplies the reason.
    static std::shared_ptr<CameraDevice> dead(const CameraCaptureConfig& config,
                                              const std::string&         reason);

    ~LibcameraCamera() override;

    bool                    alive() const override;
    std::string             diagnostic() const override;
    const CameraIntrinsics* intrinsics() const override;
    FrameId                 engineeringFrame() const override;

    std::optional<CameraFrameData> latestFrame(uint64_t after_epoch,
                                               uint32_t after_sequence) override;

    // Releases and reacquires the device with the same configuration in a
    // new epoch, also retrying failed opens/disconnections. A restart
    // failure leaves the device not alive with the reason.
    void reset() override;

private:
    struct Impl;
    explicit LibcameraCamera(std::unique_ptr<Impl> impl);

    std::unique_ptr<Impl> impl_;
};

} // namespace navigatr
