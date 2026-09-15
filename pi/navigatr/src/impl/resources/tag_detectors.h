// tag_detectors.h
// Fiducial detector resources. apriltag_detector is the upstream
// AprilRobotics detector (vendored under third_party/apriltag) behind the
// TagDetector contract.
//
//   <Resource id="tag_detector" type="apriltag_detector">
//       <Family name="tagCircle21h7" detection_size_m="0.01761272"/>
//       <Detector quad_decimate="2.0" quad_sigma="0.0" nthreads="1"
//                 refine_edges="true" decode_sharpening="0.25"/>   optional
//       <Pose iterations="50"/>                                    optional
//   </Resource>
//
// detection_size_m is the physical edge length of the detector's four
// pose-estimation corners: the square where the tag's black and white
// borders meet (width_at_border cells), not the outer printed dimension.
// For tagCircle21h7 and tagStandard41h12 that square is 5/9 of the printed
// tag side, for tag36h11 it is 8/10.
//
// Metric poses are solved on undistorted corners with the frame's
// intrinsics; the raw corners stay attached to the detection for overlays.
// A frame without intrinsics, or a family without a configured size,
// yields 2D detections with has_pose false. One detector instance is
// serialized by a mutex: the upstream detector is not reentrant.

#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "resources/resource_instance.h"
#include "resources/resource_store.h"
#include "resources/tag_detector.h"

extern "C" {
struct nav_apriltag_detector;
}

namespace navigatr
{

struct TagFamilyConfig {
    std::string name;
    double      detection_size_m = 0.0;
};

struct AprilTagDetectorConfig {
    std::vector<TagFamilyConfig> families;
    double                       quad_decimate     = 2.0;
    double                       quad_sigma        = 0.0;
    int                          nthreads          = 1;
    bool                         refine_edges      = true;
    double                       decode_sharpening = 0.25;
    int                          pose_iterations   = 50;
};

class AprilTagDetector : public TagDetector
{
public:
    static std::unique_ptr<AprilTagDetector> create(const AprilTagDetectorConfig& config,
                                                    std::string&                  err);
    ~AprilTagDetector() override;

    bool detect(const CameraFrameData& frame, const CameraIntrinsics* intrinsics,
                std::vector<NativeTagDetection>& out, std::string& err) override;

    // Physical size configured for a family, or 0 when unknown.
    double detectionSizeFor(const std::string& family) const;

    // The family's cell geometry: the corner square spans width_at_border
    // cells of the total_width printed cells. False for an unknown family.
    static bool familyGeometry(const std::string& family, int& width_at_border,
                               int& total_width, std::string& err);

    // The printed tag as the family defines it, total_width cells per side,
    // one byte per cell (0 black, 255 white). False for an unknown family
    // or id.
    static bool renderTag(const std::string& family, int id, std::vector<uint8_t>& cells,
                          int& total_width, std::string& err);

    static std::vector<std::string> supportedFamilies();

private:
    AprilTagDetector() = default;

    std::mutex                   mutex_;
    nav_apriltag_detector*       detector_ = nullptr;
    std::vector<TagFamilyConfig> families_;
    int                  pose_iterations_ = 50;
    std::vector<uint8_t> scratch_;   // private copy: the detector may write in place
};

ResourceInstance make_apriltag_detector(const ConfigNode&              node,
                                        ResourceInitializationContext& context,
                                        std::string&                   err);

} // namespace navigatr
