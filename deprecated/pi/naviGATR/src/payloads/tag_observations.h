// payloads/tag_observations.h
// Fiducial observations after the one fixed detector normalization: every
// pose is T_cameraEngineering_tagSurface with the canonical conventions
// (engineering camera +x looking, +y left, +z up; tag surface +x outward
// normal toward the viewer, +z printed top). No detector-native axis ever
// appears in this payload.

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "core/ids.h"
#include "core/time.h"
#include "math/se3.h"
#include "resources/camera.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kTagObservationSet = "perception.tag_observation_set";
} // namespace payload_names

struct TagObservation {
    std::string family;
    int         observed_id = -1;

    // Metric pose exists only when the frame carried calibrated intrinsics
    // and the family has a configured physical size; a 2D decode without
    // it is still an observation, never a zero pose.
    bool       has_pose = false;
    Transform3 T_camera_tag;   // T_Ce_S, engineering to canonical surface

    // Pixel geometry in the captured image, ordering as NativeTagDetection
    // (corner 0 bottom-left of the printed tag, counter-clockwise); kept
    // for association diagnostics and the inspection overlays.
    double corners_px[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    double center_px[2]     = {0, 0};

    // Detector quality carried through for association gating and the
    // inspection tools; semantics and has_ flags as NativeTagDetection.
    int    hamming                 = 0;
    double decision_margin         = 0.0;
    bool   has_reprojection_error  = false;
    double reprojection_error_px   = 0.0;
    bool   has_alternate_pose_ambiguity = false;
    double alternate_pose_ambiguity     = 0.0;
};

struct TagObservationSet {
    SensorId      camera;            // producing sensor
    FrameId       camera_frame;      // engineering frame id in the robot frame map
    uint32_t      frame_sequence = 0;
    uint64_t      frame_epoch    = 0;   // camera restart generation
    MonotonicTime exposureAt;           // host clock
    int64_t       exposure_uncertainty_ms = 0;
    bool          exposure_time_reliable  = true;
    int           width_px  = 0;
    int           height_px = 0;

    // The calibration the frame was captured under, for consumers that
    // project (field-of-view checks, overlays).
    std::shared_ptr<const CameraIntrinsics> intrinsics;

    // Wall time perception spent in the detector for this frame.
    double detector_processing_ms = 0.0;

    std::vector<TagObservation> tags;
};

} // namespace navigatr
