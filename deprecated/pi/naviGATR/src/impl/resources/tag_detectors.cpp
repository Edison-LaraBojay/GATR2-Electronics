// tag_detectors.cpp

#include "impl/resources/tag_detectors.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "impl/resources/apriltag_c.h"
#include "math/camera_model.h"

namespace navigatr
{

namespace
{

std::string supportedList() {
    std::string out;
    for (int i = 0; nav_apriltag_supported_family(i) != nullptr; ++i) {
        if (!out.empty()) {
            out += ", ";
        }
        out += nav_apriltag_supported_family(i);
    }
    return out;
}

} // namespace

bool AprilTagDetector::familyGeometry(const std::string& family, int& width_at_border,
                                      int& total_width, std::string& err) {
    if (nav_apriltag_family_geometry(family.c_str(), &width_at_border, &total_width) != 0) {
        err = "unknown tag family " + family + "; supported: " + supportedList();
        return false;
    }
    return true;
}

bool AprilTagDetector::renderTag(const std::string& family, int id, std::vector<uint8_t>& cells,
                                 int& total_width, std::string& err) {
    uint8_t   buffer[64 * 64];
    const int result = nav_apriltag_render(family.c_str(), id, buffer, sizeof(buffer));
    if (result == -1) {
        err = "unknown tag family " + family + "; supported: " + supportedList();
        return false;
    }
    if (result == -2) {
        err = "family " + family + " has no id " + std::to_string(id);
        return false;
    }
    if (result <= 0) {
        err = "family " + family + " renders larger than the render buffer";
        return false;
    }
    total_width = result;
    cells.assign(buffer, buffer + static_cast<std::size_t>(total_width) * total_width);
    return true;
}

std::unique_ptr<AprilTagDetector> AprilTagDetector::create(const AprilTagDetectorConfig& config,
                                                           std::string&                  err) {
    if (config.families.empty()) {
        err = "apriltag_detector needs at least one family";
        return nullptr;
    }
    if (config.quad_decimate < 1.0 || config.nthreads < 1 || config.pose_iterations < 1 ||
        config.decode_sharpening < 0.0) {
        err = "apriltag_detector: quad_decimate must be at least 1, nthreads and pose "
              "iterations at least 1, decode_sharpening non-negative";
        return nullptr;
    }
    std::unique_ptr<AprilTagDetector> detector(new AprilTagDetector());
    detector->detector_ =
        nav_apriltag_create(config.quad_decimate, config.quad_sigma, config.nthreads,
                            config.refine_edges ? 1 : 0, config.decode_sharpening);
    if (detector->detector_ == nullptr) {
        err = "apriltag_detector: upstream detector could not be created";
        return nullptr;
    }
    detector->pose_iterations_ = config.pose_iterations;
    for (const TagFamilyConfig& f : config.families) {
        const int added = nav_apriltag_add_family(detector->detector_, f.name.c_str());
        if (added == -1) {
            err = "unknown tag family " + f.name + "; supported: " + supportedList();
            return nullptr;
        }
        if (added == -2) {
            err = "tag family " + f.name + " configured twice";
            return nullptr;
        }
        detector->families_.push_back(f);
    }
    return detector;
}

AprilTagDetector::~AprilTagDetector() {
    if (detector_ != nullptr) {
        nav_apriltag_destroy(detector_);
    }
}

double AprilTagDetector::detectionSizeFor(const std::string& family) const {
    for (const TagFamilyConfig& f : families_) {
        if (f.name == family) {
            return f.detection_size_m;
        }
    }
    return 0.0;
}

bool AprilTagDetector::detect(const CameraFrameData& frame, const CameraIntrinsics* intrinsics,
                              std::vector<NativeTagDetection>& out, std::string& err) {
    out.clear();
    if (frame.y8 == nullptr || frame.width_px <= 0 || frame.height_px <= 0) {
        err = "frame carries no pixels";
        return false;
    }
    const std::size_t expected =
        static_cast<std::size_t>(frame.width_px) * static_cast<std::size_t>(frame.height_px);
    if (frame.y8->size() < expected) {
        err = "frame pixel buffer smaller than width x height";
        return false;
    }
    if (frame.width_px >= 32768 || frame.height_px >= 32768) {
        err = "frame too large for the detector";
        return false;
    }
    if (intrinsics != nullptr &&
        (intrinsics->calibrated_width_px != frame.width_px ||
         intrinsics->calibrated_height_px != frame.height_px)) {
        err = "frame resolution does not match the calibration resolution";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto                  start = std::chrono::steady_clock::now();

    // private copy: with quad_decimate <= 1 and quad_sigma != 0 the
    // upstream detector blurs the buffer it is given
    scratch_.assign(frame.y8->begin(), frame.y8->begin() + static_cast<std::ptrdiff_t>(expected));

    struct Sink {
        std::vector<NativeTagDetection>* out;
    } sink{&out};
    const int count = nav_apriltag_detect(
        detector_, frame.width_px, frame.height_px, frame.width_px, scratch_.data(),
        [](void* user, const nav_apriltag_detection* det) {
            NativeTagDetection d;
            d.family          = det->family;
            d.observed_id     = det->id;
            d.hamming         = det->hamming;
            d.decision_margin = det->decision_margin;
            for (int c = 0; c < 4; ++c) {
                d.corners_px[c][0] = det->corners[c][0];
                d.corners_px[c][1] = det->corners[c][1];
            }
            d.center_px[0] = det->center[0];
            d.center_px[1] = det->center[1];
            static_cast<Sink*>(user)->out->push_back(std::move(d));
        },
        &sink);
    if (count < 0) {
        err = "apriltag detector failed";
        return false;
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
            .count();

    for (NativeTagDetection& d : out) {
        d.processing_ms     = elapsed_ms;
        const double size_m = detectionSizeFor(d.family);
        if (intrinsics == nullptr || size_m <= 0.0) {
            continue;   // 2D decode only
        }
        // solve on undistorted corners: the pose model is a pinhole
        nav_apriltag_pose_request request;
        for (int c = 0; c < 4; ++c) {
            undistortedPixel(*intrinsics, d.corners_px[c][0], d.corners_px[c][1],
                             request.corners[c][0], request.corners[c][1]);
        }
        request.tagsize_m  = size_m;
        request.fx         = intrinsics->fx_px;
        request.fy         = intrinsics->fy_px;
        request.cx         = intrinsics->cx_px;
        request.cy         = intrinsics->cy_px;
        request.iterations = pose_iterations_;
        nav_apriltag_pose_result pose;
        if (nav_apriltag_pose(&request, &pose) != 0) {
            continue;
        }
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                d.T_optical_tag_native.R.m[r][c] = pose.R[r * 3 + c];
            }
        }
        d.T_optical_tag_native.x_m = pose.t[0];
        d.T_optical_tag_native.y_m = pose.t[1];
        d.T_optical_tag_native.z_m = pose.t[2];
        d.has_pose = std::isfinite(pose.t[2]) && std::isfinite(pose.t[0]) &&
                     std::isfinite(pose.t[1]);
        if (!d.has_pose) {
            continue;
        }

        // ambiguity: best over alternate object-space error, 0 when no
        // competing minimum exists
        d.has_alternate_pose_ambiguity = true;
        if (pose.has_second && std::isfinite(pose.err2) && std::isfinite(pose.err1)) {
            const double hi            = std::max(pose.err1, pose.err2);
            const double lo            = std::min(pose.err1, pose.err2);
            d.alternate_pose_ambiguity = hi > 0.0 ? lo / hi : 0.0;
        } else {
            d.alternate_pose_ambiguity = 0.0;
        }

        // reprojection residual of the chosen pose against the undistorted
        // corners, pixels
        const double half      = size_m / 2.0;
        const double obj[4][3] = {{-half, half, 0.0}, {half, half, 0.0}, {half, -half, 0.0},
                                  {-half, -half, 0.0}};
        double       sum       = 0.0;
        bool         ok        = true;
        for (int c = 0; c < 4; ++c) {
            double x_o = 0.0, y_o = 0.0, z_o = 0.0;
            transformPoint3(d.T_optical_tag_native, obj[c][0], obj[c][1], obj[c][2], x_o, y_o,
                            z_o);
            if (z_o <= 1e-9) {
                ok = false;
                break;
            }
            const double u  = intrinsics->fx_px * x_o / z_o + intrinsics->cx_px;
            const double v  = intrinsics->fy_px * y_o / z_o + intrinsics->cy_px;
            const double du = u - request.corners[c][0];
            const double dv = v - request.corners[c][1];
            sum += du * du + dv * dv;
        }
        if (ok) {
            d.has_reprojection_error = true;
            d.reprojection_error_px  = std::sqrt(sum / 4.0);
        }
    }
    return true;
}

ResourceInstance make_apriltag_detector(const ConfigNode& node,
                                        ResourceInitializationContext&,
                                        std::string& err) {
    AprilTagDetectorConfig config;
    bool                   ok = true;
    node.forEach("Family", [&](const ConfigNode& fam) {
        if (!ok) {
            return;
        }
        TagFamilyConfig f;
        if (!fam.requireAttr("name", f.name, err) ||
            !fam.requireDouble("detection_size_m", f.detection_size_m, err)) {
            ok = false;
            return;
        }
        if (f.detection_size_m <= 0.0) {
            err = fam.path() + ": detection_size_m must be positive";
            ok  = false;
            return;
        }
        config.families.push_back(f);
    });
    if (!ok) {
        return ResourceInstance{};
    }
    if (config.families.empty()) {
        err = node.path() + ": needs at least one <Family name=... detection_size_m=.../>";
        return ResourceInstance{};
    }
    const ConfigNode tuning  = node.child("Detector");
    long             threads = 1;
    if (!tuning.getDouble("quad_decimate", 2.0, config.quad_decimate, err) ||
        !tuning.getDouble("quad_sigma", 0.0, config.quad_sigma, err) ||
        !tuning.getInt("nthreads", 1, threads, err) ||
        !tuning.getBool("refine_edges", true, config.refine_edges, err) ||
        !tuning.getDouble("decode_sharpening", 0.25, config.decode_sharpening, err)) {
        return ResourceInstance{};
    }
    config.nthreads = static_cast<int>(threads);
    long iterations = 50;
    if (!node.child("Pose").getInt("iterations", 50, iterations, err)) {
        return ResourceInstance{};
    }
    config.pose_iterations = static_cast<int>(iterations);

    std::string inner;
    auto        detector = AprilTagDetector::create(config, inner);
    if (detector == nullptr) {
        err = node.path() + ": " + inner;
        return ResourceInstance{};
    }
    std::shared_ptr<AprilTagDetector> shared(std::move(detector));
    return ResourceInstance::asContract<TagDetector>(shared);
}

} // namespace navigatr
