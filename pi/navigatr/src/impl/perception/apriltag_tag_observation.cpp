// apriltag_tag_observation.cpp

#include "impl/perception/apriltag_tag_observation.h"

#include <chrono>

#include "payloads/tag_observations.h"
#include "resources/resource_map.h"

namespace navigatr
{

std::unique_ptr<Perception> AprilTagObservationPerception::create(
    const ConfigNode& node, SlotInitializationContext& context, std::string& err) {
    auto perception = std::make_unique<AprilTagObservationPerception>();

    if (context.resources == nullptr || context.sensors == nullptr) {
        err = node.path() + ": no resources available";
        return nullptr;
    }

    const SensorId camera_id{node.child("Camera").attr("sensor_id")};
    if (camera_id.empty()) {
        err = node.path() + ": needs <Camera sensor_id=.../>";
        return nullptr;
    }
    if (!context.sensors->bind<CameraFramePayload>(camera_id, node.path(),
                                                   perception->camera_, err)) {
        return nullptr;
    }

    const ResourceId detector_id{node.child("Detector").attr("resource_id")};
    if (detector_id.empty()) {
        err = node.path() + ": needs <Detector resource_id=.../>";
        return nullptr;
    }
    std::string inner;
    perception->detector_ = context.resources->require<TagDetector>(detector_id, inner);
    if (perception->detector_ == nullptr) {
        err = node.path() + ": " + inner;
        return nullptr;
    }

    perception->output_ = ObservationId{node.child("Output").attr("observation_id")};
    if (perception->output_.empty()) {
        err = node.path() + ": needs <Output observation_id=.../>";
        return nullptr;
    }

    const std::string detect = node.attr("detect", "on_demand");
    if (detect == "always") {
        perception->always_detect_ = true;
    } else if (detect != "on_demand") {
        err = node.path() + ": detect must be on_demand or always, not \"" + detect +
              "\"";
        return nullptr;
    }
    return perception;
}

std::vector<ObservationOutputDecl> AprilTagObservationPerception::produces() const {
    return {ObservationOutputDecl{
        output_,
        PayloadDescriptor::of<TagObservationSet>(payload_names::kTagObservationSet)}};
}

PerceptionOutput AprilTagObservationPerception::run(const PerceptionInput& in) {
    PerceptionOutput out;

    // The camera stays warm; the detector only earns its cycle time while
    // something is actually acquiring, unless diagnostics ask for always.
    if (!always_detect_ &&
        !(in.target.active && in.target.status == TargetStatus::kPendingAcquisition)) {
        return out;
    }

    const StoredSensorSample* stored = camera_.freshStored(in.sensorResults);
    if (stored == nullptr) {
        out.status = FunctionStatus::kNoData;   // camera dead or nothing yet
        return out;
    }
    const CameraFramePayload* frame = stored->payload.get<CameraFramePayload>();
    if (frame == nullptr || frame->intrinsics == nullptr) {
        out.status = FunctionStatus::kFault;
        return out;
    }
    if (has_processed_ && frame->frame.sequence == last_processed_sequence_) {
        return out;   // no new frame this cycle; evidence is per frame
    }
    has_processed_           = true;
    last_processed_sequence_ = frame->frame.sequence;

    std::vector<NativeTagDetection> native;
    std::string                     detect_err;
    const auto                      detect_start = std::chrono::steady_clock::now();
    if (!detector_->detect(frame->frame, *frame->intrinsics, native, detect_err)) {
        out.status = FunctionStatus::kFault;
        return out;
    }
    const double detect_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  detect_start)
            .count();

    // The one fixed normalization: engineering camera from detector optical,
    // canonical tag surface from detector-native tag. Nothing else in the
    // repository touches native axes.
    Transform3 T_ce_cd;
    T_ce_cd.R = rotationEngineeringFromOptical();
    Transform3 T_sd_s;
    T_sd_s.R = rotationCanonicalTagFromNative();

    TagObservationSet set;
    set.camera                = camera_.id;
    set.camera_frame          = frame->engineering_frame;
    set.frame_sequence        = frame->frame.sequence;
    set.exposureAt            = frame->frame.exposureAt;
    set.intrinsics            = frame->intrinsics;
    set.detector_processing_ms = detect_ms;
    for (const NativeTagDetection& d : native) {
        TagObservation obs;
        obs.family          = d.family;
        obs.observed_id     = d.observed_id;
        obs.hamming         = d.hamming;
        obs.decision_margin = d.decision_margin;
        obs.has_reprojection_error = d.has_reprojection_error;
        obs.reprojection_error_px  = d.reprojection_error_px;
        obs.has_alternate_pose_ambiguity = d.has_alternate_pose_ambiguity;
        obs.alternate_pose_ambiguity     = d.alternate_pose_ambiguity;
        for (int c = 0; c < 4; ++c) {
            obs.corners_px[c][0] = d.corners_px[c][0];
            obs.corners_px[c][1] = d.corners_px[c][1];
        }
        obs.center_px[0] = d.center_px[0];
        obs.center_px[1] = d.center_px[1];
        obs.T_camera_tag = compose(compose(T_ce_cd, d.T_optical_tag_native), T_sd_s);
        set.tags.push_back(std::move(obs));
    }

    ObservationRecord record;
    record.measuredAt = set.exposureAt;
    record.payload = TypedPayload::store(std::move(set), payload_names::kTagObservationSet);
    out.observations.emplace(output_, std::move(record));
    return out;
}

} // namespace navigatr
