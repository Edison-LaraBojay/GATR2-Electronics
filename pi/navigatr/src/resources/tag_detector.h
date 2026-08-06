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
    Transform3  T_optical_tag_native;   // T_Cd_Sd
    double      decision_margin = 0.0;  // detector confidence, larger is better
};

class TagDetector
{
public:
    virtual ~TagDetector() = default;

    // Detections for one frame. False and err on detector failure (not on
    // an empty result).
    virtual bool detect(const CameraFrameData& frame, const CameraIntrinsics& intrinsics,
                        std::vector<NativeTagDetection>& out, std::string& err) = 0;
};

} // namespace navigatr
