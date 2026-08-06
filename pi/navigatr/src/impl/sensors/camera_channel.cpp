// camera_channel.cpp

#include "impl/sensors/camera_channel.h"

#include <memory>

#include "core/payload_descriptor.h"
#include "payloads/camera_frames.h"
#include "resources/camera.h"
#include "resources/resource_map.h"

namespace navigatr
{

std::optional<SensorExecutable> make_camera_frame(const ConfigNode&            node,
                                                  SensorInitializationContext& context,
                                                  std::string&                 err) {
    const ResourceId device_id{node.child("Source").attr("resource_id")};
    if (device_id.empty()) {
        err = node.path() + ": needs <Source resource_id=.../>";
        return std::nullopt;
    }
    if (context.resources == nullptr) {
        err = node.path() + ": no resources available";
        return std::nullopt;
    }
    std::string inner;
    std::shared_ptr<CameraDevice> device =
        context.resources->require<CameraDevice>(device_id, inner);
    if (device == nullptr) {
        err = node.path() + ": " + inner;
        return std::nullopt;
    }

    struct State {
        uint32_t last_sequence  = 0;
        bool     published_once = false;
    };
    auto state = std::make_shared<State>();
    const FrameId engineering_frame = device->engineeringFrame();
    auto intrinsics = std::make_shared<const CameraIntrinsics>(device->intrinsics());

    SensorExecutable executable;
    executable.outputPayload =
        PayloadDescriptor::of<CameraFramePayload>(payload_names::kCameraFrame);
    executable.execute = [device, state, engineering_frame,
                          intrinsics](const SensorExecutionInput&) {
        SensorPollResult result;
        std::optional<CameraFrameData> frame =
            device->latestFrame(state->published_once ? state->last_sequence : 0);
        if (frame.has_value()) {
            state->last_sequence  = frame->sequence;
            state->published_once = true;
            CameraFramePayload payload;
            payload.frame             = std::move(*frame);
            payload.engineering_frame = engineering_frame;
            payload.intrinsics        = intrinsics;
            SensorPublication publication;
            publication.measuredAt = payload.frame.exposureAt;
            publication.payload =
                TypedPayload::store(std::move(payload), payload_names::kCameraFrame);
            result.state       = SensorState::kValid;
            result.publication = std::move(publication);
            return result;
        }
        if (!device->alive()) {
            result.state      = SensorState::kUnavailable;
            result.diagnostic = "camera device is dead";
            return result;
        }
        result.state = state->published_once ? SensorState::kValid
                                             : SensorState::kNoDataYet;
        return result;
    };
    executable.reset = [state]() { *state = State{}; };
    return executable;
}

} // namespace navigatr
