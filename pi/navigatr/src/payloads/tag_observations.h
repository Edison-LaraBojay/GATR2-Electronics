// payloads/tag_observations.h
// Fiducial observations after the one fixed detector normalization: every
// pose is T_cameraEngineering_tagSurface with the canonical conventions
// (engineering camera +x looking, +y left, +z up; tag surface +x outward
// normal toward the viewer, +z printed top). No detector-native axis ever
// appears in this payload.

#pragma once
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/time.h"
#include "math/se3.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kTagObservationSet = "perception.tag_observation_set";
} // namespace payload_names

struct TagObservation {
    std::string family;
    int         observed_id = -1;
    Transform3  T_camera_tag;   // T_Ce_S, engineering to canonical surface
    double      decision_margin = 0.0;
};

struct TagObservationSet {
    SensorId      camera;            // producing sensor
    FrameId       camera_frame;      // engineering frame id in the robot frame map
    uint32_t      frame_sequence = 0;
    MonotonicTime exposureAt;        // host clock
    std::vector<TagObservation> tags;
};

} // namespace navigatr
