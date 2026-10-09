// tag_detector.h
// Typed fiducial detector contract. A detector turns one camera frame into
// detections expressed in DETECTOR-NATIVE axes; perception owns the one
// fixed conversion into engineering axes and no native axis ever escapes it.
//
// Native conventions (the adapter for a real upstream detector must produce
// exactly this form):
//   camera optical frame Cd: +x image-right, +y image-down, +z optical
//     forward
//   native tag frame Sd: +x image-right on the tag, +y image-down, +z
//     optical-forward; a head-on tag has translation (0, 0, distance) and
//     identity rotation
//
// A metric pose needs calibrated intrinsics and a configured physical size
// for the family; without them the detection is a 2D decode with has_pose
// false, and its pose fields mean nothing.

#pragma once
#include <string>
#include <vector>

#include "math/se3.h"
#include "resources/camera.h"

namespace navigatr
{

struct NativeTagDetection {
    std::string family;
    int         observed_id = -1;

    // Metric pose availability: false when no calibrated intrinsics or
    // physical size existed for the solve; the 2D decode still stands.
    bool       has_pose = false;
    Transform3 T_optical_tag_native;   // T_Cd_Sd

    // Pixel corners in the captured (distorted) image, corner 0 at the
    // printed tag's bottom-left proceeding counter-clockwise as seen in the
    // image; center is the detection center. Association and inspection
    // tools depend on this ordering staying fixed.
    double corners_px[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    double center_px[2]     = {0, 0};

    // Detector quality. hamming is corrected bit errors (0 best);
    // decision_margin is detector confidence, larger better;
    // reprojection_error_px is the pose fit residual;
    // alternate_pose_ambiguity is best/alternate pose error ratio in 0..1,
    // smaller meaning the reported pose is more clearly the right one.
    // A detector that does not compute a value leaves its has_ flag false;
    // absent is never conflated with a genuine zero.
    int    hamming                 = 0;
    double decision_margin         = 0.0;
    bool   has_reprojection_error  = false;
    double reprojection_error_px   = 0.0;
    bool   has_alternate_pose_ambiguity = false;
    double alternate_pose_ambiguity     = 0.0;

    double processing_ms = 0.0;   // detector time for this frame
};

class TagDetector
{
public:
    virtual ~TagDetector() = default;

    // Detections for one frame. intrinsics may be null (2D decoding only).
    // False and err on detector failure (not on an empty result).
    virtual bool detect(const CameraFrameData& frame, const CameraIntrinsics* intrinsics,
                        std::vector<NativeTagDetection>& out, std::string& err) = 0;
};

} // namespace navigatr
