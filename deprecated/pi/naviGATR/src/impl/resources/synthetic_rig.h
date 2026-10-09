// synthetic_rig.h
// One hardware-free rig behind the normal resource boundary: a scripted
// truth trajectory drives Pico-shaped encoder and gyro outputs, an
// optional attitude output, and rendered camera frames of the configured
// field's tag mounts. Every consumer downstream is the real one: the same
// channel sensors, the same localization models, the real AprilTag
// detector, the same association and estimation. Nothing here is an
// estimate; it is labeled synthetic truth wherever it is exposed.
//
//   <Resource id="rig" type="synthetic_rig">
//       <Field resource_id="override_field"/>                  needed by Camera
//       <Wheels resource_id="wheel_geometry" counts_per_revolution="4000"/>
//       <Trajectory center_x_m="1.7832" center_y_m="1.7832" radius_m="0.7"
//                   period_s="40" facing="center" start_deg="180"/>
//       <Displace landmark_id="neutral_goal_0_center" dx_m="0.05"
//                 dy_m="-0.03" dyaw_deg="4"/>                     repeatable
//       <Telemetry tick_hz="50" device_offset_ms="5000" gyro_bias_mdps="1500"/>
//       <Attitude mode="measured" rock_deg="3" period_s="2.5"/>   or unavailable
//       <Camera frame_id="front_camera_engineering"
//               robot_frames_resource_id="robot_geometry"
//               width_px="1280" height_px="960" fx_px="1100" fy_px="1100"
//               cx_px="640" cy_px="480" k1="0" k2="0" p1="0" p2="0" k3="0"
//               frame_rate_hz="10" latency_ms="30" background="110"/>
//       <Output id="encoder_a" wheel_id="left_wheel"/>
//       <Output id="imu" channel="imu"/>
//       <Output id="attitude" channel="attitude"/>
//       <Output id="frame" channel="camera"/>
//   </Resource>
//
// Encoder outputs publish pico.encoder_counts, the imu output publishes
// pico.gyro_rate with the same lossless per-tick accumulator the Pico
// resource keeps, attitude publishes sensor.attitude_sample (or stays
// Unavailable in mode unavailable), and the camera publishes
// sensor.camera_frame with the synthetic camera intrinsics. Rocking is
// physical in both attitude modes: the camera sees it either way, the
// estimator only learns about it when measured. Displaced landmarks are
// the truth the estimation is expected to discover.

#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "config/field_map.h"
#include "core/time.h"
#include "math/se3.h"
#include "math/transforms.h"
#include "resources/camera.h"
#include "resources/resource_instance.h"
#include "resources/resource_store.h"
#include "resources/robot_frames.h"
#include "resources/wheel_geometry.h"

namespace navigatr
{

struct RigTruth {
    Pose2D pose;   // field frame, robot origin
    double roll_rad  = 0.0;
    double pitch_rad = 0.0;
};

class SyntheticRig
{
public:
    struct TrajectoryConfig {
        double      center_x_m = 0.0;
        double      center_y_m = 0.0;
        double      radius_m   = 0.5;
        double      period_s   = 40.0;
        std::string facing     = "center";   // center | tangent
        double      start_rad  = 0.0;
        double      hold_s     = 0.0;   // stationary at the start, for bias calibration
    };

    struct AttitudeConfig {
        bool   measured = false;
        double rock_rad = 0.0;
        double period_s = 2.5;
    };

    struct CameraConfig {
        bool                             configured = false;
        FrameId                          frame_id;
        std::shared_ptr<const CameraIntrinsics> intrinsics;
        double                           frame_rate_hz = 10.0;
        int64_t                          latency_ms    = 30;
        uint8_t                          background    = 110;
    };

    // Truth of the trajectory at a host time.
    RigTruth truthAt(MonotonicTime host) const;

    // Landmark truth: nominal from the field map plus the configured
    // displacement, field frame.
    Pose2D landmarkTruth(const FieldObjectId& id) const;

    const CameraConfig& camera() const { return camera_; }

    uint64_t framesRendered() const { return frames_rendered_; }

    // Renders the camera view at a truth pose into a packed Y8 image.
    void render(const RigTruth& truth, std::vector<uint8_t>& y8) const;

private:
    friend ResourceInstance make_synthetic_rig(const ConfigNode&, ResourceInitializationContext&,
                                               std::string&);
    friend class SyntheticRigExecutable;

    std::shared_ptr<const FieldMap>       field_;
    std::shared_ptr<const RobotFrameMap>  frames_;
    std::shared_ptr<const WheelGeometryMap> wheels_;
    double                                counts_per_revolution_ = 0.0;
    TrajectoryConfig                      trajectory_;
    AttitudeConfig                        attitude_;
    CameraConfig                          camera_;
    Transform3                            T_robot_camera_;
    std::map<std::string, Pose2D>         displacements_;
    double                                tick_hz_          = 50.0;
    int64_t                               device_offset_ms_ = 5000;
    double                                gyro_bias_mdps_   = 0.0;

    // rendered tag cells per family/id, filled on first use
    mutable std::map<std::string, std::pair<int, std::vector<uint8_t>>> tag_cache_;

    uint64_t frames_rendered_ = 0;
    uint64_t ticks_           = 0;
};

ResourceInstance make_synthetic_rig(const ConfigNode&              node,
                                    ResourceInitializationContext& context,
                                    std::string&                   err);

} // namespace navigatr
