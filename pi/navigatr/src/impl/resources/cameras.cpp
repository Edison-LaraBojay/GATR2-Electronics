// cameras.cpp

#include "impl/resources/cameras.h"

#include <memory>

#include "config/calibration.h"
#include "resources/camera.h"

namespace navigatr
{

namespace
{

// Fully configured camera without a live capture backend. Frames never
// arrive; the camera_frame sensor reports it unavailable.
class DeadCamera : public CameraDevice
{
public:
    DeadCamera(CameraIntrinsics intrinsics, FrameId frame)
        : intrinsics_(std::move(intrinsics)), frame_(std::move(frame)) {}

    bool alive() const override { return false; }
    const CameraIntrinsics& intrinsics() const override { return intrinsics_; }
    FrameId engineeringFrame() const override { return frame_; }
    std::optional<CameraFrameData> latestFrame(uint32_t) override { return std::nullopt; }

private:
    CameraIntrinsics intrinsics_;
    FrameId          frame_;
};

} // namespace

ResourceInstance make_libcamera_camera(const ConfigNode&              node,
                                       ResourceInitializationContext& context,
                                       std::string&                   err) {
    const ConfigNode device = node.child("Device");
    if (!device.valid()) {
        err = node.path() + ": needs <Device index=.../>";
        return ResourceInstance{};
    }
    long index = 0;
    if (!device.requireInt("index", index, err)) {
        return ResourceInstance{};
    }
    if (index < 0) {
        err = device.path() + ": index cannot be negative";
        return ResourceInstance{};
    }

    const ConfigNode capture = node.child("Capture");
    if (!capture.valid()) {
        err = node.path() + ": needs <Capture .../>";
        return ResourceInstance{};
    }
    long   cap_w = 0, cap_h = 0;
    double rate = 0.0;
    std::string pixel_format;
    if (!capture.requireInt("width_px", cap_w, err) ||
        !capture.requireInt("height_px", cap_h, err) ||
        !capture.requireAttr("pixel_format", pixel_format, err) ||
        !capture.requireDouble("frame_rate_hz", rate, err)) {
        return ResourceInstance{};
    }
    if (cap_w <= 0 || cap_h <= 0 || rate <= 0.0) {
        err = capture.path() + ": width_px, height_px, and frame_rate_hz must be positive";
        return ResourceInstance{};
    }
    if (pixel_format != "Y8") {
        err = capture.path() + ": pixel_format " + pixel_format +
              " is not supported; use Y8";
        return ResourceInstance{};
    }

    const ConfigNode calibration = node.child("Calibration");
    if (!calibration.valid()) {
        err = node.path() + ": needs <Calibration .../>";
        return ResourceInstance{};
    }
    std::string calibration_id;
    if (!checkCalibration(calibration, context.allow_provisional, err) ||
        !calibration.requireAttr("calibration_id", calibration_id, err)) {
        return ResourceInstance{};
    }

    const ConfigNode intr = calibration.child("Intrinsics");
    if (!intr.valid()) {
        err = calibration.path() + ": needs <Intrinsics .../>";
        return ResourceInstance{};
    }
    CameraIntrinsics intrinsics;
    long             cal_w = 0, cal_h = 0;
    if (!intr.requireAttr("model", intrinsics.model, err) ||
        !intr.requireInt("calibrated_width_px", cal_w, err) ||
        !intr.requireInt("calibrated_height_px", cal_h, err) ||
        !intr.requireDouble("fx_px", intrinsics.fx_px, err) ||
        !intr.requireDouble("fy_px", intrinsics.fy_px, err) ||
        !intr.requireDouble("cx_px", intrinsics.cx_px, err) ||
        !intr.requireDouble("cy_px", intrinsics.cy_px, err) ||
        !intr.requireDouble("k1", intrinsics.k1, err) ||
        !intr.requireDouble("k2", intrinsics.k2, err) ||
        !intr.requireDouble("p1", intrinsics.p1, err) ||
        !intr.requireDouble("p2", intrinsics.p2, err) ||
        !intr.requireDouble("k3", intrinsics.k3, err) ||
        !intr.requireDouble("rms_reprojection_px", intrinsics.rms_reprojection_px, err)) {
        return ResourceInstance{};
    }
    if (intrinsics.model != "brown_conrady") {
        err = intr.path() + ": model " + intrinsics.model +
              " is not supported; use brown_conrady";
        return ResourceInstance{};
    }
    if (intrinsics.fx_px <= 0.0 || intrinsics.fy_px <= 0.0) {
        err = intr.path() + ": fx_px and fy_px must be positive";
        return ResourceInstance{};
    }
    intrinsics.calibrated_width_px  = static_cast<int>(cal_w);
    intrinsics.calibrated_height_px = static_cast<int>(cal_h);

    if (cal_w != cap_w || cal_h != cap_h) {
        err = calibration.path() + ": calibration resolution " + std::to_string(cal_w) +
              "x" + std::to_string(cal_h) + " does not match capture resolution " +
              std::to_string(cap_w) + "x" + std::to_string(cap_h) +
              "; recalibrate or change the capture mode";
        return ResourceInstance{};
    }

    const ConfigNode extrinsic = calibration.child("Extrinsic");
    std::string      frame_raw;
    if (!extrinsic.valid() || !extrinsic.requireAttr("frame_id", frame_raw, err)) {
        if (err.empty()) {
            err = calibration.path() + ": needs <Extrinsic frame_id=.../>";
        }
        return ResourceInstance{};
    }

    // No capture backend is compiled into this binary yet; the device is
    // fully validated but dead, and consumers degrade like an unplugged
    // serial device.
    if (context.warnings != nullptr) {
        context.warnings->push_back(
            node.path() + ": libcamera capture backend is not built into this binary; "
            "camera is configured but dead");
    }
    auto camera = std::make_shared<DeadCamera>(std::move(intrinsics), FrameId{frame_raw});
    return ResourceInstance::asContract<CameraDevice>(std::move(camera));
}

} // namespace navigatr
