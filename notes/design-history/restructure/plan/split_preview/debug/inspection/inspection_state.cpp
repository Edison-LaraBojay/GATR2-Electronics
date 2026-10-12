// inspection_state.cpp

#include "runtime/inspection_state.h"

#include "core/host_clock.h"
#include "runtime/system.h"

namespace navigatr
{

std::shared_ptr<const SourceHealthSnapshot> System::sourceHealth() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return source_health_;
}

std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>>
System::detectionFrames() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return detection_frames_;
}

void System::publishSourceHealth(MonotonicTime now) {
    auto snapshot   = std::make_shared<SourceHealthSnapshot>();
    snapshot->at    = now;
    snapshot->cycle = cycle_.load();
    const auto fill = [](SourceHealthEntry& e, const MeasurementRecord& r) {
        e.state        = r.state;
        e.diagnostic   = r.diagnostic;
        e.lastPolledAt = r.lastPolledAt;
        e.epoch        = r.epoch;
        if (r.latest.has_value()) {
            e.has_sample        = true;
            e.payload           = r.latest->payload.stableTypeName();
            e.measuredAt        = r.latest->measuredAt;
            e.receivedAt        = r.latest->receivedAt;
            e.sequence          = r.latest->sequence;
            e.epoch             = r.latest->epoch;
            e.upstream_source   = r.latest->upstream.source;
            e.upstream_clock    = r.latest->upstream.clock;
            e.upstream_sequence = r.latest->upstream.sequence;
            e.upstream_epoch    = r.latest->upstream.epoch;
        }
    };
    for (const auto& kv : execute_resources_.retained()) {
        if (kv.second.outputs.empty()) {
            SourceHealthEntry e;
            e.kind         = "resource";
            e.id           = kv.first.value;
            e.state        = kv.second.state;
            e.diagnostic   = kv.second.diagnostic;
            e.lastPolledAt = kv.second.lastPolledAt;
            snapshot->entries.push_back(std::move(e));
            continue;
        }
        for (const auto& out : kv.second.outputs) {
            SourceHealthEntry e;
            e.kind = "resource";
            e.id   = kv.first.value + "." + out.first.value;
            fill(e, out.second);
            if (kv.second.state != SourceState::kValid && e.diagnostic.empty()) {
                e.diagnostic = kv.second.diagnostic;
            }
            snapshot->entries.push_back(std::move(e));
        }
    }
    for (const auto& kv : execute_sensors_.retained()) {
        SourceHealthEntry e;
        e.kind = "sensor";
        e.id   = kv.first.value;
        fill(e, kv.second);
        snapshot->entries.push_back(std::move(e));
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    source_health_ = std::move(snapshot);
}

void System::publishDetectionFrames(const SensorMap& sensors, const FieldSnapshot& snapshot,
                                    MonotonicTime now) {
    const auto frameSnapshot = [&](const SensorId& camera, const CameraFramePayload& frame) {
        auto d                     = std::make_shared<DetectionFrameSnapshot>();
        d->camera                  = camera;
        d->engineering_frame       = frame.engineering_frame;
        d->frame_epoch             = frame.frame.epoch;
        d->frame_sequence          = frame.frame.sequence;
        d->exposureAt              = frame.frame.exposureAt;
        d->receivedAt              = frame.frame.receivedAt;
        d->processedAt             = running_.load() ? HostClock::now() : now;
        d->exposure_uncertainty_ms = frame.frame.exposure_uncertainty_ms;
        d->exposure_time_reliable  = frame.frame.exposure_time_reliable;
        d->width_px                = frame.frame.width_px;
        d->height_px               = frame.frame.height_px;
        d->y8                      = frame.frame.y8;
        d->intrinsics              = frame.intrinsics;
        d->field_invocation        = snapshot.invocation;
        return d;
    };

    // Every tag observation set published this invocation is bound to the
    // exact frame it was decoded from: the pixels come from the sensor
    // record whose identity matches, never from "the latest image".
    std::map<SensorId, std::shared_ptr<const DetectionFrameSnapshot>> produced;
    for (const auto& kv : snapshot.observations) {
        const TagObservationSet* set = kv.second.payload.get<TagObservationSet>();
        if (set == nullptr) {
            continue;
        }
        const auto sensor_it = sensors.find(set->camera);
        if (sensor_it == sensors.end() || !sensor_it->second.latest.has_value()) {
            continue;
        }
        const CameraFramePayload* frame =
            sensor_it->second.latest->payload.get<CameraFramePayload>();
        if (frame == nullptr || frame->frame.epoch != set->frame_epoch ||
            frame->frame.sequence != set->frame_sequence) {
            continue;   // the retained frame is not the one these came from
        }
        auto d              = frameSnapshot(set->camera, *frame);
        d->has_observations = true;
        d->observations     = *set;
        for (const auto& akv : snapshot.associations) {
            const TagAssociationTraceSet* trace =
                akv.second.payload.get<TagAssociationTraceSet>();
            if (trace != nullptr && trace->camera == set->camera &&
                trace->frame_epoch == set->frame_epoch &&
                trace->frame_sequence == set->frame_sequence) {
                d->has_trace = true;
                d->trace     = *trace;
                break;
            }
        }
        if (d->has_trace && d->trace.has_exposure_context) {
            d->pose_at_exposure     = d->trace.exposure_context.pose;
            d->attitude_at_exposure = d->trace.exposure_context.attitude;
            d->field_from_odom      = d->trace.field_from_odom;
            d->anchor_revision      = d->trace.anchor_revision;
        } else {
            const auto exposure     = feed_->sampleSnapshotAt(set->exposureAt);
            d->pose_at_exposure     = exposure.sample.pose;
            d->attitude_at_exposure = exposure.sample.attitude;
            d->field_from_odom      = exposure.current.robot.field_from_odom;
            d->anchor_revision      = exposure.current.robot.anchor_revision;
        }
        produced[set->camera] = std::move(d);
    }

    // Preview without detection: a camera frame nothing decoded this
    // invocation is still published under its own identity with
    // has_observations false, so preview works with noop world estimation.
    // A frame already published, decoded or not, is never replaced by a
    // raw copy of itself.
    for (const auto& kv : sensors) {
        if (produced.count(kv.first) != 0 || !kv.second.latest.has_value()) {
            continue;
        }
        const CameraFramePayload* frame = kv.second.latest->payload.get<CameraFramePayload>();
        if (frame == nullptr) {
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            const auto                  it = detection_frames_.find(kv.first);
            if (it != detection_frames_.end() && it->second->frame_epoch == frame->frame.epoch &&
                it->second->frame_sequence == frame->frame.sequence) {
                continue;
            }
        }
        auto       d        = frameSnapshot(kv.first, *frame);
        const auto exposure     = feed_->sampleSnapshotAt(frame->frame.exposureAt);
        d->pose_at_exposure     = exposure.sample.pose;
        d->attitude_at_exposure = exposure.sample.attitude;
        d->field_from_odom      = exposure.current.robot.field_from_odom;
        d->anchor_revision      = exposure.current.robot.anchor_revision;
        produced[kv.first]      = std::move(d);
    }
    if (produced.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    for (auto& kv : produced) {
        detection_frames_[kv.first] = std::move(kv.second);
    }
}

} // namespace navigatr
