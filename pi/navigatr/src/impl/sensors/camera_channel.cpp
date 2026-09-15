// camera_channel.cpp

#include "impl/sensors/camera_channel.h"

#include <memory>

#include "core/payload_descriptor.h"
#include "payloads/camera_frames.h"
#include "runtime/resource_catalog.h"

namespace navigatr
{

std::optional<SensorExecutable> make_camera_frame(const ConfigNode&            node,
                                                  SensorInitializationContext& context,
                                                  std::string&                 err) {
    const ConfigNode source = node.child("Source");
    const ResourceId resource{source.attr("resource_id")};
    const OutputId   output{source.attr("output_id")};
    if (resource.empty() || output.empty()) {
        err = node.path() + ": needs <Source resource_id=... output_id=.../>";
        return std::nullopt;
    }
    if (context.outputs == nullptr) {
        err = node.path() + ": no resource outputs available";
        return std::nullopt;
    }

    struct State {
        TypedOutputBinding<CameraFramePayload> binding;
        uint64_t                               last_sequence = 0;
        uint64_t                               last_epoch    = 0;
        bool                                   forwarded     = false;
    };
    auto state = std::make_shared<State>();
    if (!context.outputs->bind<CameraFramePayload>(resource, output, node.path(),
                                                   state->binding, err)) {
        return std::nullopt;
    }

    SensorExecutable executable;
    executable.outputPayload =
        PayloadDescriptor::of<CameraFramePayload>(payload_names::kCameraFrame);
    executable.execute = [state](const ResourceMap& resources, const ExecutionContext&) {
        PollResult               result;
        const MeasurementRecord* record = state->binding.record(resources);
        if (record == nullptr) {
            result.state      = SourceState::kFault;
            result.diagnostic = "bound resource output is missing";
            return result;
        }
        if (record->latest.has_value() &&
            (!state->forwarded || record->latest->sequence != state->last_sequence ||
             record->latest->epoch != state->last_epoch)) {
            // a frame is forwarded once with its original receipt and
            // identity; the same retained frame is never republished as
            // new evidence
            const StoredSample& stored = *record->latest;
            state->forwarded           = true;
            state->last_sequence       = stored.sequence;
            state->last_epoch          = stored.epoch;
            result.state               = SourceState::kValid;
            Publication p;
            p.measuredAt        = stored.measuredAt;
            p.receivedAt        = stored.receivedAt;
            p.upstream.source   = state->binding.resource.value + "." + state->binding.output.value;
            p.upstream.clock    = stored.upstream.clock;
            p.upstream.sequence = stored.sequence;
            p.upstream.epoch    = stored.epoch + stored.upstream.epoch;
            p.payload           = stored.payload;
            result.publication  = std::move(p);
            return result;
        }
        result.state      = record->state;
        result.diagnostic = record->diagnostic;
        return result;
    };
    executable.reset = [state] {
        state->forwarded     = false;
        state->last_sequence = 0;
        state->last_epoch    = 0;
    };
    return executable;
}

} // namespace navigatr
