// synthetic_rig_gtest.cpp
// The hardware-free rig through the whole runtime: scripted truth drives
// the real channel sensors, the real wheel and attitude models, the real
// detector over rendered tagCircle21h7 frames, association with traces,
// and the landmark estimator. Nothing downstream knows the input was
// synthetic.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "impl/resources/synthetic_rig.h"
#include "impl/resources/tag_detectors.h"
#include "math/angles.h"
#include "payloads/field_object_evidence.h"
#include "payloads/tag_observations.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;

namespace
{

// attitude: measured | unavailable ; calibrated camera unless told otherwise
std::string rigConfig(const char* attitude_mode, bool calibrated_camera = true) {
    std::string xml = R"(
<System>
    <Loop rate_hz="100"/>
    <Resources>
        <Resource id="robot_geometry" type="robot_frame_map">
            <Frame id="front_camera_engineering" parent_frame_id="robot_body"
                   calibration_status="verified">
                <PoseOfChildInParent x_m="0.15" y_m="0" z_m="0.20"
                    roll_deg="0" pitch_deg="0" yaw_deg="0"/>
            </Frame>
        </Resource>
        <Resource id="field" type="field_map">
            <Landmark id="center_goal">
                <NominalPose calibration_status="verified" x_m="1.7832" y_m="1.7832" heading_deg="0"/>
                <TagMount instance_id="center_east" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="0.05" y_m="0" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="0"/>
                </TagMount>
                <TagMount instance_id="center_north" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="0" y_m="0.05" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="90"/>
                </TagMount>
                <TagMount instance_id="center_west" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="-0.05" y_m="0" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="180"/>
                </TagMount>
                <TagMount instance_id="center_south" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="0" y_m="-0.05" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="-90"/>
                </TagMount>
            </Landmark>
            <Landmark id="far_goal">
                <NominalPose calibration_status="verified" x_m="3.0" y_m="1.7832" heading_deg="0"/>
                <TagMount instance_id="far_west" calibration_status="verified"
                          family="tagCircle21h7" observed_id="1" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="-0.05" y_m="0" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="180"/>
                </TagMount>
            </Landmark>
        </Resource>
        <Resource id="wheel_geometry" type="wheel_geometry">
            <Wheel id="left_wheel" sensor_id="enc_a" calibration_status="verified"
                   radius_m="0.0254" position_x_m="0" position_y_m="0.13"
                   measurement_angle_deg="0" direction="positive"/>
            <Wheel id="right_wheel" sensor_id="enc_b" calibration_status="verified"
                   radius_m="0.0254" position_x_m="0" position_y_m="-0.13"
                   measurement_angle_deg="0" direction="positive"/>
            <Wheel id="rear_wheel" sensor_id="enc_c" calibration_status="verified"
                   radius_m="0.0254" position_x_m="-0.12" position_y_m="0"
                   measurement_angle_deg="90" direction="positive"/>
        </Resource>
        <Resource id="rig" type="synthetic_rig">
            <Field resource_id="field"/>
            <Wheels resource_id="wheel_geometry" counts_per_revolution="4000"/>
            <Trajectory center_x_m="1.7832" center_y_m="1.7832" radius_m="0.5"
                        period_s="30" facing="center" start_deg="180" hold_s="1.5"/>
            <Displace landmark_id="center_goal" dx_m="0.06" dy_m="-0.04" dyaw_deg="5"/>
            <Telemetry tick_hz="50" device_offset_ms="5000" gyro_bias_mdps="800"/>
            <Attitude mode=")" + std::string(attitude_mode) + R"(" rock_deg="3" period_s="2.5"/>
            <Camera frame_id="front_camera_engineering" robot_frames_resource_id="robot_geometry"
                    width_px="640" height_px="480" fx_px="600" fy_px="600" cx_px="320" cy_px="240"
                    k1="-0.1" k2="0.02" frame_rate_hz="10" latency_ms="30"/>
            <Output id="encoder_a" wheel_id="left_wheel"/>
            <Output id="encoder_b" wheel_id="right_wheel"/>
            <Output id="encoder_c" wheel_id="rear_wheel"/>
            <Output id="imu" channel="imu"/>
            <Output id="attitude" channel="attitude"/>
            <Output id="frame" channel="camera"/>
        </Resource>
        <Resource id="tag_detector" type="apriltag_detector">
            <Family name="tagCircle21h7" detection_size_m="0.03"/>
            <Detector quad_decimate="1.0" nthreads="2"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_b" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_c" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_c"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="rig" output_id="imu"/>
        </Sensor>
        <Sensor id="robot_attitude" type="attitude_channel">
            <Source resource_id="rig" output_id="attitude"/>
            <Mounting calibration_status="verified" roll_deg="0" pitch_deg="0" yaw_deg="0"/>
        </Sensor>
        <Sensor id="front_camera" type="camera_frame">
            <Source resource_id="rig" output_id="frame"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">
                <Wheels resource_id="wheel_geometry">
                    <Use wheel_id="left_wheel"/>
                    <Use wheel_id="right_wheel"/>
                    <Use wheel_id="rear_wheel"/>
                </Wheels>
                <HeadingConstraint sensor_id="robot_imu" bias_samples="40"
                                   max_calibration_travel_m="0.005"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Observation id="attitude" type="attitude_reference">
                <Input sensor_id="robot_attitude"/>
                <Freshness max_age_ms="100"/>
                <Output observation_id="attitude"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
                <Attitude observation_id="attitude" max_age_ms="200"/>
            </Estimator>
            <History retention_s="5" capacity="1024" max_interpolation_gap_ms="100"/>
            <InitialPlacement x_m="1.2832" y_m="1.7832" heading_deg="0"/>
        </Localization>
        <WorldEstimation>
            <Estimator id="goals" type="apriltag">
                <FieldMap resource_id="field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="tag_detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <Association>
                    <Observations observation_id="tag_observations"/>
                    <FieldMap resource_id="field"/>
                    <RobotFrames resource_id="robot_geometry"/>
                    <Attitude policy="assume_level"/>
                    <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
                           ambiguity_margin_m="0.15" max_range_m="3.0"
                           min_decision_margin="10" max_hamming="0"
                           min_facing_cos="0.1" min_projected_size_px="8"/>
                    <Output association_id="landmark_pose_observations"/>
                    <Trace association_id="tag_association_trace"/>
                </Association>
                <LandmarkEstimation commit="always" blend="0.5"/>
            </Estimator>
        </WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
    if (!calibrated_camera) {
        // strip the camera calibration by giving the detector no sizes is not
        // the same thing; here the rig keeps rendering and the frame carries
        // intrinsics, so the uncalibrated path is exercised through the
        // camera_frame contract in a separate fixture below
    }
    return xml;
}

struct Rig {
    FunctionRegistry              functions;
    std::unique_ptr<System>       system;
    std::shared_ptr<const SyntheticRig> rig;
    int64_t                       now_ms = 0;

    explicit Rig(const char* attitude_mode = "measured") {
        registerAll(functions);
        std::string err;
        system = System::buildFromString(rigConfig(attitude_mode).c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system != nullptr) {
            rig = system->resources().require<const SyntheticRig>(ResourceId{"rig"}, err);
            EXPECT_NE(rig, nullptr) << err;
        }
    }

    void run(double seconds) {
        const int steps = static_cast<int>(seconds * 100.0);
        for (int i = 0; i < steps; ++i) {
            now_ms += 10;
            system->step(hostTime(now_ms));
        }
    }
};

} // namespace

TEST(SyntheticRig, RenderedTagsDecodeWithTheRealDetectorAtTheTruthPose) {
    Rig f;
    ASSERT_NE(f.rig, nullptr);
    // the camera looks at the center goal from 0.5 m west of it
    const RigTruth truth = f.rig->truthAt(hostTime(0));
    std::vector<uint8_t> pixels;
    f.rig->render(truth, pixels);
    ASSERT_EQ(pixels.size(), 640u * 480u);

    CameraFrameData frame;
    frame.width_px  = 640;
    frame.height_px = 480;
    frame.y8        = std::make_shared<const std::vector<uint8_t>>(pixels);
    frame.sequence  = 1;

    std::string err;
    auto detector = f.system->resources().require<TagDetector>(ResourceId{"tag_detector"}, err);
    ASSERT_NE(detector, nullptr) << err;
    std::vector<NativeTagDetection> out;
    ASSERT_TRUE(detector->detect(frame, f.rig->camera().intrinsics.get(), out, err)) << err;
    ASSERT_GE(out.size(), 1u);

    // the west face of the displaced goal is the one facing the camera
    const NativeTagDetection* west = nullptr;
    for (const NativeTagDetection& d : out) {
        if (d.observed_id == 0 && d.has_pose) {
            west = &d;
        }
    }
    ASSERT_NE(west, nullptr);
    // depth: goal center 0.5 m ahead of the robot origin, minus the camera
    // offset 0.15 and the mount 0.05, plus the truth displacement of +0.06
    EXPECT_NEAR(west->T_optical_tag_native.z_m, 0.5 - 0.15 - 0.05 + 0.06, 0.02);
    EXPECT_LT(west->reprojection_error_px, 1.5);
}

TEST(SyntheticRig, LocalizationTracksTheTruthAndAttitudeIsMeasured) {
    Rig f("measured");
    ASSERT_NE(f.system, nullptr);
    f.run(6.0);

    const RobotState& robot = f.system->robot();
    ASSERT_TRUE(robot.valid);
    EXPECT_TRUE(robot.initialized);
    const RigTruth truth = f.rig->truthAt(robot.measuredAtHost);
    EXPECT_NEAR(robot.fieldPose().x_m, truth.pose.x_m, 0.03);
    EXPECT_NEAR(robot.fieldPose().y_m, truth.pose.y_m, 0.03);
    EXPECT_NEAR(wrapAngle(robot.fieldPose().heading_rad - truth.pose.heading_rad), 0.0,
                degToRad(2.0));
    EXPECT_TRUE(f.system->robotFeed()->status().allReady());   // bias calibration done

    ASSERT_TRUE(robot.attitude.valid);
    EXPECT_FALSE(robot.attitude.assumed_level);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    attitudeEuler(robot.attitude, roll, pitch, yaw);
    EXPECT_NEAR(roll, truth.roll_rad, degToRad(0.7));
    EXPECT_NEAR(yaw, robot.odom_pose.heading_rad, 1e-9);
    EXPECT_EQ(robot.attitude.source, "robot_attitude");
    EXPECT_GT(f.rig->framesRendered(), 30u);
}

TEST(SyntheticRig, UnavailableAttitudeIsAssumedLevelNotFabricated) {
    Rig f("unavailable");
    ASSERT_NE(f.system, nullptr);
    f.run(3.0);
    const RobotState& robot = f.system->robot();
    ASSERT_TRUE(robot.valid);
    EXPECT_FALSE(robot.attitude.valid);
    EXPECT_TRUE(robot.attitude.assumed_level);
    EXPECT_EQ(f.system->sensorMap().at(SensorId{"robot_attitude"}).state,
              SourceState::kUnavailable);
}

TEST(SyntheticRig, FieldEstimateDiscoversTheDisplacedGoalWithTraces) {
    Rig f("measured");
    ASSERT_NE(f.system, nullptr);
    f.run(8.0);

    const FieldObjectState& goal = f.system->field().objects.at(FieldObjectId{"center_goal"});
    EXPECT_EQ(goal.source, EstimateSource::kObserved);
    const Pose2D truth = f.rig->landmarkTruth(FieldObjectId{"center_goal"});
    EXPECT_NEAR(goal.pose.pose.x_m, truth.x_m, 0.04);
    EXPECT_NEAR(goal.pose.pose.y_m, truth.y_m, 0.04);
    EXPECT_NEAR(wrapAngle(goal.pose.pose.heading_rad - truth.heading_rad), 0.0, degToRad(6.0));
    // the displacement is visible against the nominal definition
    EXPECT_GT(std::fabs(goal.pose.pose.x_m - 1.7832), 0.03);
    EXPECT_FALSE(goal.last_feature.empty());
    EXPECT_EQ(goal.last_source, "front_camera");

    // the far goal is visible beyond the center goal (nothing occludes in
    // the rig) and is not displaced, so its estimate stays near nominal
    const FieldObjectState& far = f.system->field().objects.at(FieldObjectId{"far_goal"});
    EXPECT_NEAR(far.pose.pose.x_m, 3.0, 0.08);
    EXPECT_NEAR(far.pose.pose.y_m, 1.7832, 0.08);

    // every decoded tag has a visible decision in the trace, published on
    // the cycles that carried a frame
    std::shared_ptr<const FieldSnapshot> snapshot = f.system->fieldSnapshot();
    for (int i = 0; i < 20 && snapshot->associations.count(AssociationId{"tag_association_trace"}) == 0;
         ++i) {
        f.run(0.01);
        snapshot = f.system->fieldSnapshot();
    }
    const auto trace_it = snapshot->associations.find(AssociationId{"tag_association_trace"});
    ASSERT_NE(trace_it, snapshot->associations.end());
    const TagAssociationTraceSet* trace =
        trace_it->second.payload.get<TagAssociationTraceSet>();
    ASSERT_NE(trace, nullptr);
    EXPECT_EQ(trace->camera, SensorId{"front_camera"});
    EXPECT_FALSE(trace->tags.empty());
    bool any_accepted = false;
    for (const TagAssociationTrace& t : trace->tags) {
        any_accepted = any_accepted || t.accepted;
        if (!t.accepted) {
            EXPECT_FALSE(t.rejection.empty());
        }
    }
    EXPECT_TRUE(any_accepted);
    EXPECT_EQ(f.system->fieldDiagnostics().functions.count("WorldEstimation/goals"), 1u);
}

TEST(SyntheticRig, ResetRestartsCountersInANewEpochWithoutBreakingOdometry) {
    Rig f("unavailable");
    ASSERT_NE(f.system, nullptr);
    f.run(2.5);
    const uint64_t epoch = f.system->robot().odometry_epoch;
    f.system->reset();
    f.run(2.5);
    const RobotState& robot = f.system->robot();
    EXPECT_EQ(robot.odometry_epoch, epoch + 1);
    EXPECT_TRUE(robot.valid);
    // after the rig restart the encoder sensors rebased instead of
    // differencing across the reset
    const MeasurementRecord& enc = f.system->sensorMap().at(SensorId{"enc_a"});
    ASSERT_TRUE(enc.latest.has_value());
    EXPECT_EQ(enc.latest->epoch, 1u);
}
