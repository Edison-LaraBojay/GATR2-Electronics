// apriltag_gtest.cpp
// The real AprilRobotics detector behind the TagDetector contract: a
// correctly generated configured-family tag is decoded with the documented
// corner order, its metric pose follows the pinhole model only when a
// calibration exists, distortion is undone before the solve, and
// configuration errors name the problem.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "impl/resources/tag_detectors.h"
#include "math/angles.h"
#include "math/camera_model.h"
#include "runtime/register_all.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

// A frame holding one rendered tag, nearest-neighbour scaled so the corner
// square spans corner_px pixels, centered at (cx, cy) on a mid-gray field.
CameraFrameData frameWithTag(const std::string& family, int id, int width, int height,
                             double cx, double cy, int corner_px) {
    std::vector<uint8_t> cells;
    int                  total = 0;
    std::string          err;
    EXPECT_TRUE(AprilTagDetector::renderTag(family, id, cells, total, err)) << err;
    int width_at_border = 0, total_width = 0;
    EXPECT_TRUE(AprilTagDetector::familyGeometry(family, width_at_border, total_width, err));
    EXPECT_EQ(total, total_width);

    auto pixels = std::make_shared<std::vector<uint8_t>>(
        static_cast<std::size_t>(width) * height, 110);
    const double cell_px = static_cast<double>(corner_px) / width_at_border;
    const double half    = cell_px * (total_width / 2.0 + 1.0);   // one cell of quiet zone
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const double x = (u + 0.5) - cx;
            const double y = (v + 0.5) - cy;
            if (std::fabs(x) > half || std::fabs(y) > half) {
                continue;
            }
            uint8_t      value = 255;
            const double tag_half = cell_px * total_width / 2.0;
            if (std::fabs(x) < tag_half && std::fabs(y) < tag_half) {
                const int col = static_cast<int>(std::floor((x + tag_half) / cell_px));
                const int row = static_cast<int>(std::floor((y + tag_half) / cell_px));
                value = cells[static_cast<std::size_t>(std::min(std::max(row, 0), total - 1)) * total +
                              std::min(std::max(col, 0), total - 1)];
            }
            (*pixels)[static_cast<std::size_t>(v) * width + u] = value;
        }
    }
    CameraFrameData frame;
    frame.sequence   = 1;
    frame.exposureAt = hostTime(1);
    frame.receivedAt = hostTime(2);
    frame.width_px   = width;
    frame.height_px  = height;
    frame.y8         = pixels;
    return frame;
}

std::unique_ptr<AprilTagDetector> makeDetector(const char* family, double size_m,
                                               double decimate = 1.0) {
    AprilTagDetectorConfig config;
    config.families      = {TagFamilyConfig{family, size_m}};
    config.quad_decimate = decimate;
    config.nthreads      = 2;
    std::string err;
    auto        detector = AprilTagDetector::create(config, err);
    EXPECT_NE(detector, nullptr) << err;
    return detector;
}

CameraIntrinsics pinhole(int w, int h, double f) {
    CameraIntrinsics K;
    K.model                = "brown_conrady";
    K.calibrated_width_px  = w;
    K.calibrated_height_px = h;
    K.fx_px = K.fy_px = f;
    K.cx_px           = w / 2.0;
    K.cy_px           = h / 2.0;
    return K;
}

} // namespace

TEST(AprilTag, DecodesTheConfiguredCircleFamilyWithDocumentedCorners) {
    const CameraFrameData frame = frameWithTag("tagCircle21h7", 4, 400, 300, 200.0, 150.0, 60);
    auto                  detector = makeDetector("tagCircle21h7", 0.01761272);
    ASSERT_NE(detector, nullptr);

    std::vector<NativeTagDetection> out;
    std::string                     err;
    ASSERT_TRUE(detector->detect(frame, nullptr, out, err)) << err;
    ASSERT_EQ(out.size(), 1u);
    const NativeTagDetection& d = out[0];
    EXPECT_EQ(d.family, "tagCircle21h7");
    EXPECT_EQ(d.observed_id, 4);
    EXPECT_EQ(d.hamming, 0);
    EXPECT_GT(d.decision_margin, 20.0);
    EXPECT_FALSE(d.has_pose);   // no calibration was given
    EXPECT_NEAR(d.center_px[0], 200.0, 1.0);
    EXPECT_NEAR(d.center_px[1], 150.0, 1.0);
    // corner 0 bottom-left, then counter-clockwise as displayed (y down)
    EXPECT_NEAR(d.corners_px[0][0], 170.0, 1.5);
    EXPECT_NEAR(d.corners_px[0][1], 180.0, 1.5);
    EXPECT_NEAR(d.corners_px[1][0], 230.0, 1.5);
    EXPECT_NEAR(d.corners_px[1][1], 180.0, 1.5);
    EXPECT_NEAR(d.corners_px[2][0], 230.0, 1.5);
    EXPECT_NEAR(d.corners_px[2][1], 120.0, 1.5);
    EXPECT_NEAR(d.corners_px[3][0], 170.0, 1.5);
    EXPECT_NEAR(d.corners_px[3][1], 120.0, 1.5);
    EXPECT_GT(d.processing_ms, 0.0);
}

TEST(AprilTag, MetricPoseFollowsThePinholeOnlyWithCalibration) {
    const int             corner_px = 60;
    const CameraFrameData frame = frameWithTag("tagCircle21h7", 3, 400, 300, 200.0, 150.0, corner_px);
    auto                  detector = makeDetector("tagCircle21h7", 0.02);
    const CameraIntrinsics K        = pinhole(400, 300, 500.0);

    std::vector<NativeTagDetection> out;
    std::string                     err;
    ASSERT_TRUE(detector->detect(frame, &K, out, err)) << err;
    ASSERT_EQ(out.size(), 1u);
    const NativeTagDetection& d = out[0];
    ASSERT_TRUE(d.has_pose);
    // head on at the principal point: z = f * size / pixels, x = y = 0
    EXPECT_NEAR(d.T_optical_tag_native.z_m, 500.0 * 0.02 / corner_px, 0.002);
    EXPECT_NEAR(d.T_optical_tag_native.x_m, 0.0, 0.002);
    EXPECT_NEAR(d.T_optical_tag_native.y_m, 0.0, 0.002);
    EXPECT_NEAR(d.T_optical_tag_native.R.m[0][0], 1.0, 0.05);
    EXPECT_NEAR(d.T_optical_tag_native.R.m[2][2], 1.0, 0.05);
    EXPECT_TRUE(d.has_reprojection_error);
    EXPECT_LT(d.reprojection_error_px, 1.0);
    EXPECT_TRUE(d.has_alternate_pose_ambiguity);

    // a family without a configured size stays a 2D decode even when
    // calibrated
    auto other = makeDetector("tag36h11", 0.06);
    ASSERT_TRUE(other->detect(frame, &K, out, err)) << err;
    EXPECT_TRUE(out.empty());   // the circle tag is not a tag36h11

    // resolution mismatch between frame and calibration is refused
    const CameraIntrinsics wrong = pinhole(640, 480, 500.0);
    EXPECT_FALSE(detector->detect(frame, &wrong, out, err));
    EXPECT_NE(err.find("resolution"), std::string::npos);
}

TEST(AprilTag, DistortionIsUndoneBeforeTheSolve) {
    // render an off-center tag, then warp the whole image with a strong
    // barrel distortion the way a real lens would, and check the solve
    // through the calibrated model recovers the undistorted geometry
    const int      W = 480, H = 360;
    const double   f = 400.0;
    CameraIntrinsics K = pinhole(W, H, f);
    K.k1               = -0.3;
    K.k2               = 0.05;

    // far from the principal point, where the barrel term is large
    const CameraFrameData ideal = frameWithTag("tagCircle21h7", 7, W, H, 410.0, 90.0, 40);
    auto                  warped_pixels = std::make_shared<std::vector<uint8_t>>(
        static_cast<std::size_t>(W) * H, 110);
    for (int v = 0; v < H; ++v) {
        for (int u = 0; u < W; ++u) {
            // the distorted pixel (u, v) sees the ideal pixel of the same ray
            double xn = 0.0, yn = 0.0;
            undistortPixel(K, u + 0.5, v + 0.5, xn, yn);
            const int iu = static_cast<int>(std::floor(f * xn + K.cx_px));
            const int iv = static_cast<int>(std::floor(f * yn + K.cy_px));
            if (iu >= 0 && iu < W && iv >= 0 && iv < H) {
                (*warped_pixels)[static_cast<std::size_t>(v) * W + u] =
                    (*ideal.y8)[static_cast<std::size_t>(iv) * W + iu];
            }
        }
    }
    CameraFrameData warped = ideal;
    warped.y8              = warped_pixels;

    auto                            detector = makeDetector("tagCircle21h7", 0.02);
    std::vector<NativeTagDetection> from_ideal, from_warped;
    std::string                     err;
    CameraIntrinsics                pin = pinhole(W, H, f);
    ASSERT_TRUE(detector->detect(ideal, &pin, from_ideal, err)) << err;
    ASSERT_TRUE(detector->detect(warped, &K, from_warped, err)) << err;
    ASSERT_EQ(from_ideal.size(), 1u);
    ASSERT_EQ(from_warped.size(), 1u);
    ASSERT_TRUE(from_ideal[0].has_pose);
    ASSERT_TRUE(from_warped[0].has_pose);

    // raw corners differ (they live in different images)...
    EXPECT_GT(std::fabs(from_ideal[0].corners_px[0][0] - from_warped[0].corners_px[0][0]), 4.0);
    // ...but the metric pose agrees once distortion is undone
    EXPECT_NEAR(from_warped[0].T_optical_tag_native.z_m, from_ideal[0].T_optical_tag_native.z_m,
                0.03 * from_ideal[0].T_optical_tag_native.z_m);
    EXPECT_NEAR(from_warped[0].T_optical_tag_native.x_m, from_ideal[0].T_optical_tag_native.x_m,
                0.01);
    EXPECT_NEAR(from_warped[0].T_optical_tag_native.y_m, from_ideal[0].T_optical_tag_native.y_m,
                0.01);
    EXPECT_LT(from_warped[0].reprojection_error_px, 1.5);

    // solving the warped corners as if the camera were a pinhole is wrong
    // by a visible margin: barrel distortion shrinks the tag, so the naive
    // range is too long (the bearing survives because position and size
    // shrink together)
    std::vector<NativeTagDetection> naive;
    ASSERT_TRUE(detector->detect(warped, &pin, naive, err)) << err;
    ASSERT_EQ(naive.size(), 1u);
    EXPECT_GT(std::fabs(naive[0].T_optical_tag_native.z_m - from_ideal[0].T_optical_tag_native.z_m),
              0.03 * from_ideal[0].T_optical_tag_native.z_m);
}

TEST(CameraModel, DistortionRoundTripsAndFrameConversionsAgree) {
    CameraIntrinsics K = pinhole(640, 480, 500.0);
    K.k1 = -0.3;
    K.k2 = 0.1;
    K.p1 = 0.001;
    K.p2 = -0.0005;
    K.k3 = -0.02;
    for (double x = -0.6; x <= 0.6; x += 0.3) {
        for (double y = -0.45; y <= 0.45; y += 0.3) {
            double u = 0.0, v = 0.0;
            ASSERT_TRUE(projectOptical(K, x, y, 1.0, u, v));
            double xn = 0.0, yn = 0.0;
            undistortPixel(K, u, v, xn, yn);
            EXPECT_NEAR(xn, x, 1e-6);
            EXPECT_NEAR(yn, y, 1e-6);
        }
    }
    // engineering forward maps to optical depth
    double x_o = 0.0, y_o = 0.0, z_o = 0.0;
    opticalFromEngineering(2.0, 0.5, -0.25, x_o, y_o, z_o);
    EXPECT_NEAR(x_o, -0.5, 1e-12);
    EXPECT_NEAR(y_o, 0.25, 1e-12);
    EXPECT_NEAR(z_o, 2.0, 1e-12);
    double x_e = 0.0, y_e = 0.0, z_e = 0.0;
    engineeringFromOptical(x_o, y_o, z_o, x_e, y_e, z_e);
    EXPECT_NEAR(x_e, 2.0, 1e-12);
    EXPECT_NEAR(y_e, 0.5, 1e-12);
    EXPECT_NEAR(z_e, -0.25, 1e-12);
    // behind the camera never projects
    double u = 0.0, v = 0.0;
    EXPECT_FALSE(projectEngineering(K, -1.0, 0.0, 0.0, u, v));
}

TEST(AprilTag, ResourceFactoryValidatesFamiliesAndTunables) {
    FunctionRegistry functions;
    register_resources(functions);
    tinyxml2::XMLDocument doc;
    std::string           err;
    const auto            make = [&](const char* xml) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        ResourceInitializationContext context;
        context.functions = &functions;
        err.clear();
        return make_apriltag_detector(ConfigNode{doc.RootElement()}, context, err);
    };

    EXPECT_FALSE(make(R"(<Resource id="d" type="apriltag_detector"/>)").empty() == false);
    EXPECT_NE(err.find("Family"), std::string::npos);

    EXPECT_TRUE(make(R"(<Resource id="d" type="apriltag_detector">
        <Family name="tagWeird99h1" detection_size_m="0.05"/></Resource>)")
                    .empty());
    EXPECT_NE(err.find("tagWeird99h1"), std::string::npos);
    EXPECT_NE(err.find("tagCircle21h7"), std::string::npos);   // the message lists what exists

    EXPECT_TRUE(make(R"(<Resource id="d" type="apriltag_detector">
        <Family name="tag36h11" detection_size_m="0"/></Resource>)")
                    .empty());
    EXPECT_NE(err.find("detection_size_m"), std::string::npos);

    EXPECT_TRUE(make(R"(<Resource id="d" type="apriltag_detector">
        <Family name="tag36h11" detection_size_m="0.06"/>
        <Detector quad_decimate="0.5"/></Resource>)")
                    .empty());
    EXPECT_NE(err.find("quad_decimate"), std::string::npos);

    const ResourceInstance good = make(R"(<Resource id="d" type="apriltag_detector">
        <Family name="tagCircle21h7" detection_size_m="0.01761272"/>
        <Family name="tag36h11" detection_size_m="0.06"/>
        <Detector quad_decimate="1.0" nthreads="2"/><Pose iterations="30"/></Resource>)");
    ASSERT_FALSE(good.empty()) << err;
    std::string inner;
    EXPECT_NE(good.require<TagDetector>(inner), nullptr) << inner;
}
