// cameras.cpp

#include "impl/resources/cameras.h"

#include <cmath>
#include <limits>
#include <memory>

#include "payloads/camera_frames.h"
#include "resources/camera.h"

#if NAVIGATR_HAVE_LIBCAMERA
#include "impl/resources/libcamera_camera.h"
#endif

namespace navigatr
{

bool parseCameraConfig(const ConfigNode& node, CameraCaptureConfig& out,
                       std::string& err) {
    const ConfigNode device = node.child("Device");
    if (!device.valid()) {
        err = node.path() + ": needs <Device index=.../>";
        return false;
    }
    if (!device.requireInt("index", out.device_index, err)) {
        return false;
    }
    if (out.device_index < 0) {
        err = device.path() + ": index cannot be negative";
        return false;
    }

    const ConfigNode capture = node.child("Capture");
    if (!capture.valid()) {
        err = node.path() + ": needs <Capture .../>";
        return false;
    }
    if (!capture.requireInt("width_px", out.width_px, err) ||
        !capture.requireInt("height_px", out.height_px, err) ||
        !capture.requireAttr("pixel_format", out.pixel_format, err) ||
        !capture.requireDouble("frame_rate_hz", out.frame_rate_hz, err)) {
        return false;
    }
    if (out.width_px <= 0 || out.height_px <= 0 || out.frame_rate_hz <= 0.0) {
        err = capture.path() + ": width_px, height_px, and frame_rate_hz must be positive";
        return false;
    }
    if (out.width_px > std::numeric_limits<int>::max() ||
        out.height_px > std::numeric_limits<int>::max()) {
        err = capture.path() + ": capture dimensions exceed supported integer range";
        return false;
    }
    const double frame_duration_us = 1e6 / out.frame_rate_hz;
    if (!std::isfinite(out.frame_rate_hz) || !std::isfinite(frame_duration_us) ||
        frame_duration_us < 1.0 ||
        frame_duration_us >= static_cast<double>(std::numeric_limits<int64_t>::max())) {
        err = capture.path() + ": frame_rate_hz must produce a supported positive frame duration";
        return false;
    }
    if (out.pixel_format != "Y8") {
        err = capture.path() + ": pixel_format " + out.pixel_format +
              " is not supported; use Y8";
        return false;
    }

    const ConfigNode calibration = node.child("Calibration");
    if (!calibration.valid()) {
        out.calibrated         = false;
        return true;
    }
    if (!calibration.requireAttr("calibration_id", out.calibration_id, err)) {
        return false;
    }

    const ConfigNode intr = calibration.child("Intrinsics");
    if (!intr.valid()) {
        err = calibration.path() + ": needs <Intrinsics .../>";
        return false;
    }
    CameraIntrinsics& K = out.intrinsics;
    long              cal_w = 0, cal_h = 0;
    if (!intr.requireAttr("model", K.model, err) ||
        !intr.requireInt("calibrated_width_px", cal_w, err) ||
        !intr.requireInt("calibrated_height_px", cal_h, err) ||
        !intr.requireDouble("fx_px", K.fx_px, err) ||
        !intr.requireDouble("fy_px", K.fy_px, err) ||
        !intr.requireDouble("cx_px", K.cx_px, err) ||
        !intr.requireDouble("cy_px", K.cy_px, err) || !intr.requireDouble("k1", K.k1, err) ||
        !intr.requireDouble("k2", K.k2, err) || !intr.requireDouble("p1", K.p1, err) ||
        !intr.requireDouble("p2", K.p2, err) || !intr.requireDouble("k3", K.k3, err) ||
        !intr.requireDouble("rms_reprojection_px", K.rms_reprojection_px, err)) {
        return false;
    }
    if (K.model != "brown_conrady") {
        err = intr.path() + ": model " + K.model + " is not supported; use brown_conrady";
        return false;
    }
    if (K.fx_px <= 0.0 || K.fy_px <= 0.0) {
        err = intr.path() + ": fx_px and fy_px must be positive";
        return false;
    }
    if (K.rms_reprojection_px < 0.0) {
        err = intr.path() + ": rms_reprojection_px cannot be negative";
        return false;
    }
    K.calibrated_width_px  = static_cast<int>(cal_w);
    K.calibrated_height_px = static_cast<int>(cal_h);
    if (cal_w != out.width_px || cal_h != out.height_px) {
        err = calibration.path() + ": calibration resolution " + std::to_string(cal_w) + "x" +
              std::to_string(cal_h) + " does not match capture resolution " +
              std::to_string(out.width_px) + "x" + std::to_string(out.height_px) +
              "; recalibrate or change the capture mode";
        return false;
    }

    const ConfigNode extrinsic = calibration.child("Extrinsic");
    std::string      frame_raw;
    if (!extrinsic.valid() || !extrinsic.requireAttr("frame_id", frame_raw, err)) {
        if (err.empty()) {
            err = calibration.path() + ": needs <Extrinsic frame_id=.../>";
        }
        return false;
    }
    out.engineering_frame = FrameId{frame_raw};
    out.calibrated        = true;
    return true;
}

bool cameraOutputId(const ConfigNode& node, OutputId& out, std::string& err) {
    const ConfigNode output = node.child("Output");
    if (!output.valid()) {
        out = OutputId{"frame"};
        return true;
    }
    out = OutputId{output.attr("id")};
    if (out.empty()) {
        err = output.path() + ": Output needs id";
        return false;
    }
    if (output.next("Output").valid()) {
        err = node.path() + ": a camera publishes exactly one Output";
        return false;
    }
    return true;
}

ResourceInstance make_libcamera_camera(const ConfigNode&              node,
                                       ResourceInitializationContext& context,
                                       std::string&                   err) {
    CameraCaptureConfig config;
    if (!parseCameraConfig(node, config, err)) {
        return ResourceInstance{};
    }
    OutputId output;
    if (!cameraOutputId(node, output, err)) {
        return ResourceInstance{};
    }
#if NAVIGATR_HAVE_LIBCAMERA
    std::string inner;
    auto        camera = LibcameraCamera::open(config, inner);
    if (camera == nullptr) {
        // an unpluggable device is a warning and a dead camera; the robot
        // keeps running without it, but the reason is visible
        if (context.warnings != nullptr) {
            context.warnings->push_back(node.path() + ": " + inner);
        }
        camera = LibcameraCamera::dead(config, inner);
    }
    return cameraResource(std::move(camera), std::move(output));
#else
    (void)context;
    err = node.path() + ": libcamera_camera needs the libcamera backend, which is not "
          "compiled into this binary; configure with -DNAVIGATR_WITH_LIBCAMERA=ON on the "
          "Pi (libcamera-dev) or select synthetic_rig for a hardware-free run";
    return ResourceInstance{};
#endif
}

ResourceInstance cameraResource(std::shared_ptr<CameraDevice> device, OutputId output) {
    struct State {
        uint64_t last_epoch    = 0;
        uint32_t last_sequence = 0;
        bool     published     = false;
    };
    auto          state             = std::make_shared<State>();
    const FrameId engineering_frame = device->engineeringFrame();
    std::shared_ptr<const CameraIntrinsics> intrinsics;
    if (device->intrinsics() != nullptr) {
        intrinsics = std::make_shared<const CameraIntrinsics>(*device->intrinsics());
    }

    ResourceExecutable executable;
    executable.outputs.push_back(ResourceOutputDecl{
        output, PayloadDescriptor::of<CameraFramePayload>(payload_names::kCameraFrame)});
    executable.execute = [device, state, engineering_frame, intrinsics,
                          output](const ExecutionContext& context) {
        ResourcePollResult result;
        OutputPoll         poll;
        poll.id = output;

        std::optional<CameraFrameData> frame = device->latestFrame(
            state->published ? state->last_epoch : 0, state->published ? state->last_sequence : 0);
        if (frame.has_value()) {
            state->last_epoch    = frame->epoch;
            state->last_sequence = frame->sequence;
            state->published     = true;
            CameraFramePayload payload;
            payload.frame              = *frame;
            payload.engineering_frame  = engineering_frame;
            payload.intrinsics         = intrinsics;
            Publication publication;
            publication.measuredAt        = frame->exposureAt;
            publication.receivedAt        = frame->receivedAt.isSet() ? frame->receivedAt : context.now;
            publication.upstream.source   = "camera";
            publication.upstream.clock    = "host";
            publication.upstream.sequence = frame->sequence;
            publication.upstream.epoch    = frame->epoch;
            publication.payload =
                TypedPayload::store(std::move(payload), payload_names::kCameraFrame);
            poll.result.state       = SourceState::kValid;
            poll.result.publication = std::move(publication);
        } else if (!device->alive()) {
            poll.result.state      = SourceState::kUnavailable;
            poll.result.diagnostic = device->diagnostic().empty() ? "camera device is dead"
                                                                  : device->diagnostic();
        } else {
            poll.result.state = state->published ? SourceState::kValid : SourceState::kNoDataYet;
        }
        result.state      = poll.result.state;
        result.diagnostic = poll.result.diagnostic;
        result.outputs.push_back(std::move(poll));
        return result;
    };
    executable.reset = [device, state] {
        *state = State{};
        device->reset();
    };

    auto instance = ResourceInstance::asContract<CameraDevice>(std::move(device));
    instance.setExecutable(std::move(executable));
    return instance;
}

} // namespace navigatr
