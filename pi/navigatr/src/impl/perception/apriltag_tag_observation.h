// apriltag_tag_observation.h
// Perception: runs a fiducial detector over one camera's new frames and
// publishes canonical tag observations. This is the only place the
// detector-native optical and tag axes are converted; the conversion is one
// fixed rotation pair, never per-configuration angle fudging.
//
// Extraction is continuous: every new camera frame is processed exactly
// once, whether or not any navigation target exists. Scheduling policy, if
// one is ever needed, is a source-side implementation concern and never
// navigation state.
//
//   <Perception type="apriltag_tag_observation">
//       <Camera sensor_id="front_camera"/>
//       <Detector resource_id="tag_detector"/>
//       <Output observation_id="tag_observations"/>
//   </Perception>

#pragma once
#include <memory>
#include <string>

#include "contracts/perception.h"
#include "payloads/camera_frames.h"
#include "resources/tag_detector.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

class AprilTagObservationPerception : public Perception
{
public:
    static std::unique_ptr<Perception> create(const ConfigNode& node,
                                              SlotInitializationContext& context,
                                              std::string& err);

    PerceptionOutput run(const PerceptionInput& in) override;

    std::vector<ObservationOutputDecl> produces() const override;

    void reset() override {
        has_processed_           = false;
        last_processed_sequence_ = 0;
    }

private:
    TypedSensorBinding<CameraFramePayload> camera_;
    std::shared_ptr<TagDetector>           detector_;
    ObservationId                          output_;

    // has_processed_ distinguishes "never ran" from a first frame whose
    // sequence happens to be zero.
    bool     has_processed_           = false;
    uint32_t last_processed_sequence_ = 0;
};

} // namespace navigatr
