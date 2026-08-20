// vision_targets_gtest.cpp
// The one-camera acquire-once flow end to end through the System: synthetic
// detections generated from ground truth, full-chain pose recovery, target
// latching in the odometry frame, correction gating, generation and epoch
// invalidation, and repeated-printed-id disambiguation. The camera, the
// detector, the command source, and the odometry are scripted test doubles
// registered as ordinary factories.

#include <gtest/gtest.h>

#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "math/angles.h"
#include "math/se3.h"
#include "resources/camera.h"
#include "resources/tag_detector.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;

namespace
{

struct CameraScript {
    std::optional<CameraFrameData> pending;
    uint32_t                       next_sequence = 0;
};

class ScriptedCamera : public CameraDevice
{
public:
    ScriptedCamera(std::shared_ptr<CameraScript> script) : script_(std::move(script)) {
        intrinsics_.model                = "brown_conrady";
        intrinsics_.calibrated_width_px  = 640;
        intrinsics_.calibrated_height_px = 480;
        intrinsics_.fx_px                = 500;
        intrinsics_.fy_px                = 500;
        intrinsics_.cx_px                = 320;
        intrinsics_.cy_px                = 240;
    }

    bool alive() const override { return true; }
    const CameraIntrinsics& intrinsics() const override { return intrinsics_; }
    FrameId engineeringFrame() const override {
        return FrameId{"front_camera_engineering"};
    }
    std::optional<CameraFrameData> latestFrame(uint32_t after) override {
        if (script_->pending.has_value() && script_->pending->sequence > after) {
            return script_->pending;
        }
        return std::nullopt;
    }

private:
    std::shared_ptr<CameraScript> script_;
    CameraIntrinsics              intrinsics_;
};

using DetectionScript = std::map<uint32_t, std::vector<NativeTagDetection>>;

class ScriptedDetector : public TagDetector
{
public:
    ScriptedDetector(std::shared_ptr<DetectionScript> script) : script_(std::move(script)) {}
    bool detect(const CameraFrameData& frame, const CameraIntrinsics&,
                std::vector<NativeTagDetection>& out, std::string&) override {
        const auto it = script_->find(frame.sequence);
        out           = it == script_->end() ? std::vector<NativeTagDetection>{}
                                             : it->second;
        return true;
    }

private:
    std::shared_ptr<DetectionScript> script_;
};

class ScriptedCommands : public Commands
{
public:
    ScriptedCommands(std::shared_ptr<CommandState> next) : next_(std::move(next)) {}
    CommandsOutput run(const CommandsInput&) override {
        return CommandsOutput{*next_, FunctionStatus::kOk};
    }

private:
    std::shared_ptr<CommandState> next_;
};

struct RobotScript {
    Pose2D   odom;
    uint64_t epoch    = 0;
    double   yaw_rate = 0.0;
    bool     valid    = true;
};

class ScriptedLocalization : public Localization
{
public:
    ScriptedLocalization(std::shared_ptr<RobotScript> script) : script_(std::move(script)) {}
    LocalizationOutput run(const LocalizationInput& in) override {
        LocalizationOutput out;
        out.robot                 = in.previous;
        out.robot.odom_pose       = script_->odom;
        out.robot.odometry_epoch  = script_->epoch;
        out.robot.yaw_rate_rad_s  = script_->yaw_rate;
        out.robot.valid           = script_->valid;
        return out;
    }

private:
    std::shared_ptr<RobotScript> script_;
};

std::string configXml(double camera_yaw_deg, double ambiguity_margin_m) {
    char head[256];
    std::snprintf(head, sizeof(head),
                  "yaw_deg=\"%.6f\"", camera_yaw_deg);
    char margin[128];
    std::snprintf(margin, sizeof(margin), "ambiguity_margin_m=\"%.6f\"",
                  ambiguity_margin_m);

    std::string xml = R"(<System>
  <Loop rate_hz="100"/>
  <Resources>
    <Resource id="robot_geometry" type="robot_frame_map">
      <Frame id="front_camera_engineering" parent_frame_id="robot_body"
             calibration_status="verified">
        <PoseOfChildInParent x_m="0.2" y_m="0" z_m="0.3"
            roll_deg="0" pitch_deg="0" @CAMYAW@/>
      </Frame>
      <Frame id="rear_contact" parent_frame_id="robot_body"
             calibration_status="verified">
        <PoseOfChildInParent x_m="-0.40" y_m="0" z_m="0.05"
            roll_deg="0" pitch_deg="0" yaw_deg="180"/>
      </Frame>
    </Resource>
    <Resource id="camera_device" type="scripted_camera"/>
    <Resource id="detector" type="scripted_detector"/>
    <Resource id="game_field" type="field_map">
      <Landmark id="center_goal">
        <NominalPose calibration_status="verified"
                     x_m="1.7832" y_m="1.7832" heading_deg="0"/>
        <ApproachFrame id="center_goal_west_face" calibration_status="verified">
          <PoseOfApproachFrameInLandmark x_m="-0.14" y_m="0" z_m="0"
              roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </ApproachFrame>
        <TagMount instance_id="center_goal_west_tag" calibration_status="verified"
                  family="tag36h11" observed_id="0" detection_size_m="0.06">
          <PoseOfTagSurfaceInLandmark x_m="-0.14" y_m="0" z_m="0.25"
              roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </TagMount>
      </Landmark>
      <Landmark id="left_goal">
        <NominalPose calibration_status="verified"
                     x_m="0.6" y_m="3.0" heading_deg="0"/>
        <ApproachFrame id="left_goal_west_face" calibration_status="verified">
          <PoseOfApproachFrameInLandmark x_m="-0.14" y_m="0" z_m="0"
              roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </ApproachFrame>
        <TagMount instance_id="left_goal_tag" calibration_status="verified"
                  family="tag36h11" observed_id="1" detection_size_m="0.06">
          <PoseOfTagSurfaceInLandmark x_m="-0.14" y_m="0" z_m="0.25"
              roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </TagMount>
      </Landmark>
      <Landmark id="right_goal">
        <NominalPose calibration_status="verified"
                     x_m="3.0" y_m="0.6" heading_deg="0"/>
        <ApproachFrame id="right_goal_west_face" calibration_status="verified">
          <PoseOfApproachFrameInLandmark x_m="-0.14" y_m="0" z_m="0"
              roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </ApproachFrame>
        <TagMount instance_id="right_goal_tag" calibration_status="verified"
                  family="tag36h11" observed_id="1" detection_size_m="0.06">
          <PoseOfTagSurfaceInLandmark x_m="-0.14" y_m="0" z_m="0.25"
              roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </TagMount>
      </Landmark>
    </Resource>
    <Resource id="targets" type="target_set">
      <FieldMap resource_id="game_field"/>
      <RobotFrames resource_id="robot_geometry"/>
      <Target id="back_to_center" type="landmark_relative" wire_id="1"
              landmark_id="center_goal"
              approach_frame_id="center_goal_west_face"
              controlled_frame_id="rear_contact">
        <DesiredControlledFramePose calibration_status="verified"
            x_m="0.05" y_m="0" heading_deg="180"/>
        <VisionCorrection type="acquire_once"
            on_acquisition_timeout="use_nominal_target"
            minimum_consistent_observations="3"
            maximum_observation_age_ms="200"
            maximum_robot_angular_speed_deg_s="60"
            acquisition_timeout_ms="1000"
            consistency_translation_m="0.05"
            consistency_heading_deg="3">
          <PreferredCamera sensor_id="front_camera"/>
          <AllowedTagMount instance_id="center_goal_west_tag"/>
        </VisionCorrection>
      </Target>
      <Target id="relative_move" type="robot_relative" wire_id="2"
              controlled_frame_id="robot_body">
        <Snapshot frame_id="robot_body" delta_axes="robot_at_activation"/>
        <Delta x_m="-0.6" y_m="0.3" heading_deg="0"/>
        <VisionCorrection type="none"/>
      </Target>
      <Target id="nominal_center" type="landmark_relative" wire_id="3"
              landmark_id="center_goal"
              approach_frame_id="center_goal_west_face"
              controlled_frame_id="rear_contact">
        <DesiredControlledFramePose calibration_status="verified"
            x_m="0.05" y_m="0" heading_deg="180"/>
        <VisionCorrection type="none"/>
      </Target>
      <Target id="cancel_on_timeout" type="landmark_relative" wire_id="4"
              landmark_id="center_goal"
              approach_frame_id="center_goal_west_face"
              controlled_frame_id="rear_contact">
        <DesiredControlledFramePose calibration_status="verified"
            x_m="0.05" y_m="0" heading_deg="180"/>
        <VisionCorrection type="acquire_once"
            on_acquisition_timeout="cancel"
            minimum_consistent_observations="3"
            maximum_observation_age_ms="200"
            maximum_robot_angular_speed_deg_s="60"
            acquisition_timeout_ms="1000"
            consistency_translation_m="0.05"
            consistency_heading_deg="3"/>
      </Target>
      <Target id="back_to_right" type="landmark_relative" wire_id="5"
              landmark_id="right_goal"
              approach_frame_id="right_goal_west_face"
              controlled_frame_id="rear_contact">
        <DesiredControlledFramePose calibration_status="verified"
            x_m="0.05" y_m="0" heading_deg="180"/>
        <VisionCorrection type="acquire_once"
            on_acquisition_timeout="cancel"
            minimum_consistent_observations="1"
            maximum_observation_age_ms="200"
            maximum_robot_angular_speed_deg_s="60"
            acquisition_timeout_ms="1000"
            consistency_translation_m="0.05"
            consistency_heading_deg="3"/>
      </Target>
      <Target id="back_to_left" type="landmark_relative" wire_id="6"
              landmark_id="left_goal"
              approach_frame_id="left_goal_west_face"
              controlled_frame_id="rear_contact">
        <DesiredControlledFramePose calibration_status="verified"
            x_m="0.05" y_m="0" heading_deg="180"/>
        <VisionCorrection type="acquire_once"
            on_acquisition_timeout="cancel"
            minimum_consistent_observations="1"
            maximum_observation_age_ms="200"
            maximum_robot_angular_speed_deg_s="60"
            acquisition_timeout_ms="1000"
            consistency_translation_m="0.05"
            consistency_heading_deg="3"/>
      </Target>
    </Resource>
  </Resources>
  <Sensors>
    <Sensor id="front_camera" type="camera_frame">
      <Source resource_id="camera_device"/>
    </Sensor>
  </Sensors>
  <Pipeline>
    <CommandCollection type="scripted_commands"/>
    <Preprocessing type="noop"/>
    <Localization type="scripted_localization"/>
    <WorldEstimation type="landmark_world">
      <FieldMap resource_id="game_field"/>
      <Pipeline>
        <ObservationExtraction type="apriltag_tag_observation">
          <Camera sensor_id="front_camera"/>
          <Detector resource_id="detector"/>
          <Output observation_id="tag_observations"/>
        </ObservationExtraction>
        <Association type="tag_mount_association">
          <Observations observation_id="tag_observations"/>
          <FieldMap resource_id="game_field"/>
          <RobotFrames resource_id="robot_geometry"/>
          <Targets resource_id="targets"/>
          <Gates max_translation_error_m="0.5" max_heading_error_deg="30" @MARGIN@
                 max_range_m="3.0" min_decision_margin="20"
                 min_projected_size_px="4"/>
          <Output association_id="landmark_pose_observations"/>
        </Association>
        <Estimator type="landmark_estimator" commit="on_target_lock"/>
      </Pipeline>
    </WorldEstimation>
    <TargetResolution type="configured_targets">
      <Targets resource_id="targets"/>
      <FieldMap resource_id="game_field"/>
      <Evidence association_id="landmark_pose_observations"/>
    </TargetResolution>
    <Publishing type="noop"/>
  </Pipeline>
</System>)";
    xml.replace(xml.find("@CAMYAW@"), 8, head);
    xml.replace(xml.find("@MARGIN@"), 8, margin);
    return xml;
}

struct Fixture {
    std::shared_ptr<CameraScript>    camera    = std::make_shared<CameraScript>();
    std::shared_ptr<DetectionScript> detections = std::make_shared<DetectionScript>();
    std::shared_ptr<CommandState>    command   = std::make_shared<CommandState>();
    std::shared_ptr<RobotScript>     robot     = std::make_shared<RobotScript>();

    FunctionRegistry        functions;
    std::unique_ptr<System> system;
    int64_t                 now_ms = 0;
    double                  camera_yaw_rad = 0.0;

    explicit Fixture(double camera_yaw_deg = 0.0, double ambiguity_margin_m = 0.15) {
        camera_yaw_rad = degToRad(camera_yaw_deg);
        registerAll(functions);
        auto cam = camera;
        functions.add(FunctionKey{"scripted_camera"},
                      ResourceMakeFunction([cam](const ConfigNode&,
                                                 ResourceInitializationContext&,
                                                 std::string&) {
                          return ResourceInstance::asContract<CameraDevice>(
                              std::make_shared<ScriptedCamera>(cam));
                      }));
        auto det = detections;
        functions.add(FunctionKey{"scripted_detector"},
                      ResourceMakeFunction([det](const ConfigNode&,
                                                 ResourceInitializationContext&,
                                                 std::string&) {
                          return ResourceInstance::asContract<TagDetector>(
                              std::make_shared<ScriptedDetector>(det));
                      }));
        auto cmd = command;
        functions.add(FunctionKey{"scripted_commands"},
                      CommandsMakeFunction([cmd](const ConfigNode&,
                                                 SlotInitializationContext&,
                                                 std::string&) {
                          return std::make_unique<ScriptedCommands>(cmd);
                      }));
        auto rob = robot;
        functions.add(FunctionKey{"scripted_localization"},
                      LocalizationMakeFunction([rob](const ConfigNode&,
                                                     SlotInitializationContext&,
                                                     std::string&) {
                          return std::make_unique<ScriptedLocalization>(rob);
                      }));

        std::string err;
        system = System::buildFromString(
            configXml(camera_yaw_deg, ambiguity_margin_m).c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;   // ASSERT cannot live in a ctor
    }

    void stepOnce() {
        now_ms += 10;
        system->step(hostTime(now_ms));
    }

    Transform3 cameraExtrinsic() const {
        return makeTransform3(0.2, 0, 0.3, 0, 0, camera_yaw_rad);
    }

    // Synthetic detection from ground truth: project the mounted tag into
    // the camera and undo the fixed normalizations so the detector reports
    // native axes, exactly as a real adapter would.
    NativeTagDetection detectionFor(const Pose2D& landmark_field,
                                    const Transform3& T_landmark_surface,
                                    int observed_id) const {
        const Transform3 T_odom_surface =
            compose(transform3FromPlanar(landmark_field), T_landmark_surface);
        const Transform3 T_odom_camera =
            compose(transform3FromPlanar(robot->odom), cameraExtrinsic());
        const Transform3 T_ce_s = compose(inverse(T_odom_camera), T_odom_surface);

        Transform3 T_ce_cd;
        T_ce_cd.R = rotationEngineeringFromOptical();
        Transform3 T_sd_s;
        T_sd_s.R = rotationCanonicalTagFromNative();

        NativeTagDetection d;
        d.family              = "tag36h11";
        d.observed_id         = observed_id;
        d.decision_margin     = 60.0;
        d.T_optical_tag_native =
            compose(compose(inverse(T_ce_cd), T_ce_s), inverse(T_sd_s));
        return d;
    }

    void pushFrame(const std::vector<NativeTagDetection>& tags) {
        CameraFrameData frame;
        frame.sequence   = ++camera->next_sequence;
        frame.exposureAt = hostTime(now_ms + 10);   // the cycle that consumes it
        frame.width_px   = 640;
        frame.height_px  = 480;
        camera->pending  = frame;
        (*detections)[frame.sequence] = tags;
    }

    void select(uint8_t wire_id) {
        command->object_requested = true;
        command->object_wire_id   = wire_id;
        command->object_sequence += 1;
    }
};

const Transform3 kCenterMount = makeTransform3(-0.14, 0, 0.25, 0, 0, kPi);
const Pose2D     kCenterGoal{1.7832, 1.7832, 0.0};
// analytic expected body target: goal x - face 0.14 - (0.40 + 0.05)
const Pose2D kExpectedBodyTarget{1.7832 - 0.14 - 0.45, 1.7832, kPi};

} // namespace

TEST(VisionTargets, AcquireOnceLocksLatchSurvivesFrameLossAndMotion) {
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};   // facing the west face of the goal

    f.select(1);
    f.stepOnce();   // activation
    ASSERT_TRUE(f.system->target().active);
    EXPECT_EQ(f.system->target().status, TargetStatus::kPendingAcquisition);

    // three consistent frames latch the target
    for (int i = 0; i < 3; ++i) {
        f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
        f.stepOnce();
    }
    ASSERT_EQ(f.system->target().status, TargetStatus::kLockedVision);
    const Pose2D latched = f.system->target().T_odom_robot_target;
    EXPECT_NEAR(latched.x_m, kExpectedBodyTarget.x_m, 1e-9);
    EXPECT_NEAR(latched.y_m, kExpectedBodyTarget.y_m, 1e-9);
    // exactly the two legitimate 180s, cancelled: no third one appears
    EXPECT_NEAR(std::fabs(latched.heading_rad), kPi, 1e-9);

    // camera frames disappear; the robot turns 180 and drives backward
    f.stepOnce();
    f.robot->odom = Pose2D{1.05, 1.7832, kPi};
    f.stepOnce();
    f.robot->odom = Pose2D{1.15, 1.7832, kPi};
    f.stepOnce();

    // the latched target is bit for bit unchanged
    const Pose2D after = f.system->target().T_odom_robot_target;
    EXPECT_EQ(after.x_m, latched.x_m);
    EXPECT_EQ(after.y_m, latched.y_m);
    EXPECT_EQ(after.heading_rad, latched.heading_rad);
    EXPECT_EQ(f.system->target().status, TargetStatus::kLockedVision);
}

TEST(VisionTargets, SevenDegreeCameraYawMatchesStraightCamera) {
    // the configured extrinsic removes the mounting yaw; both rigs recover
    // the same landmark and the same target
    Pose2D latched[2];
    const double yaws[2] = {0.0, 7.0};
    for (int i = 0; i < 2; ++i) {
        Fixture f(yaws[i]);
        f.robot->odom = Pose2D{0.9, 1.7832, 0.0};
        f.select(1);
        f.stepOnce();
        for (int k = 0; k < 3; ++k) {
            f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
            f.stepOnce();
        }
        ASSERT_EQ(f.system->target().status, TargetStatus::kLockedVision);
        latched[i] = f.system->target().T_odom_robot_target;
        f.stepOnce();   // the estimator folds the locked window next cycle

        // synthetic projection and pose recovery agree: the observed
        // landmark equals the ground truth it was projected from
        const WorldObject& goal =
            f.system->world().objects.at(WorldObjectId{"center_goal"});
        EXPECT_EQ(goal.source, EstimateSource::kObserved);
        EXPECT_NEAR(goal.pose.pose.x_m, kCenterGoal.x_m, 1e-9);
        EXPECT_NEAR(goal.pose.pose.y_m, kCenterGoal.y_m, 1e-9);
        EXPECT_NEAR(goal.pose.pose.heading_rad, 0.0, 1e-9);
    }
    EXPECT_NEAR(latched[0].x_m, latched[1].x_m, 1e-9);
    EXPECT_NEAR(latched[0].y_m, latched[1].y_m, 1e-9);
    EXPECT_NEAR(wrapAngle(latched[0].heading_rad - latched[1].heading_rad), 0.0, 1e-9);
}

TEST(VisionTargets, NoActiveTargetCausesNoLandmarkMutation) {
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};

    // frames with a perfectly good tag, but nothing selected
    for (int i = 0; i < 3; ++i) {
        f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
        f.stepOnce();
    }
    const WorldObject& goal =
        f.system->world().objects.at(WorldObjectId{"center_goal"});
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);   // never mutated
    EXPECT_FALSE(goal.observed);
    EXPECT_FALSE(f.system->target().active);
}

TEST(VisionTargets, RobotRelativeSnapshotOncePerNewSequence) {
    Fixture f;
    f.robot->odom = Pose2D{1.0, 1.0, 0.0};

    f.select(2);
    f.stepOnce();
    ASSERT_EQ(f.system->target().status, TargetStatus::kLockedRobotRelative);
    const Pose2D first = f.system->target().T_odom_robot_target;
    EXPECT_NEAR(first.x_m, 0.4, 1e-12);   // 1.0 - 0.6
    EXPECT_NEAR(first.y_m, 1.3, 1e-12);   // 1.0 + 0.3

    // robot moves; the same command sequence never resnapshots
    f.robot->odom = Pose2D{2.0, 1.0, 0.0};
    f.stepOnce();
    EXPECT_EQ(f.system->target().T_odom_robot_target.x_m, first.x_m);
    EXPECT_EQ(f.system->target().T_odom_robot_target.y_m, first.y_m);

    // a genuinely new sequence resnapshots at the new pose
    const uint64_t old_generation = f.system->target().generation;
    f.select(2);
    f.stepOnce();
    EXPECT_GT(f.system->target().generation, old_generation);
    EXPECT_NEAR(f.system->target().T_odom_robot_target.x_m, 1.4, 1e-12);

    // robot-relative targets run no perception or association
    EXPECT_EQ(f.system->target().status, TargetStatus::kLockedRobotRelative);
}

TEST(VisionTargets, TargetSwitchRejectsLateOldGenerationDetection) {
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};

    f.select(1);
    f.stepOnce();
    const uint64_t generation_a = f.system->target().generation;

    // two consistent frames arrive (one short of the latch)...
    for (int i = 0; i < 2; ++i) {
        f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
        f.stepOnce();
    }
    EXPECT_EQ(f.system->target().status, TargetStatus::kPendingAcquisition);

    // ...then the brain switches targets in the same cycle a frame lands.
    // The association output this cycle still carries the old generation
    // and must be discarded by the new target.
    f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
    f.select(4);   // switch to the cancel_on_timeout center target
    f.stepOnce();
    EXPECT_GT(f.system->target().generation, generation_a);
    EXPECT_EQ(f.system->target().status, TargetStatus::kPendingAcquisition);
    EXPECT_FALSE(f.system->target().latched);
}

TEST(VisionTargets, AcquisitionTimeoutAppliesConfiguredFallback) {
    // use_nominal_target: zero visual correction, not a zero pose
    {
        Fixture f;
        f.robot->odom = Pose2D{0.9, 1.7832, 0.0};
        f.select(1);
        f.stepOnce();
        while (f.now_ms < 1200) {
            f.stepOnce();   // no frames at all
        }
        ASSERT_EQ(f.system->target().status, TargetStatus::kLockedNominal);
        const Pose2D latched = f.system->target().T_odom_robot_target;
        EXPECT_NEAR(latched.x_m, kExpectedBodyTarget.x_m, 1e-9);
        EXPECT_NEAR(latched.y_m, kExpectedBodyTarget.y_m, 1e-9);
    }
    // cancel: the target goes away instead
    {
        Fixture f;
        f.robot->odom = Pose2D{0.9, 1.7832, 0.0};
        f.select(4);
        f.stepOnce();
        while (f.now_ms < 1200) {
            f.stepOnce();
        }
        EXPECT_EQ(f.system->target().status, TargetStatus::kCancelled);
        EXPECT_FALSE(f.system->target().latched);
    }
}

TEST(VisionTargets, PolicyNoneLatchesNominalImmediately) {
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};
    f.select(3);
    f.stepOnce();
    ASSERT_EQ(f.system->target().status, TargetStatus::kLockedNominal);
    EXPECT_NEAR(f.system->target().T_odom_robot_target.x_m, kExpectedBodyTarget.x_m,
                1e-9);
}

TEST(VisionTargets, OdometryEpochResetCancelsLatchedTarget) {
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};
    f.select(3);   // nominal policy latches immediately
    f.stepOnce();
    ASSERT_TRUE(f.system->target().latched);

    f.robot->epoch = 1;   // odometry frame discontinuity
    f.stepOnce();
    EXPECT_EQ(f.system->target().status, TargetStatus::kCancelled);
    EXPECT_FALSE(f.system->target().latched);
}

TEST(VisionTargets, AngularSpeedGateUsesMotionAtExposureTime) {
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};

    f.select(1);
    f.stepOnce();

    // the robot genuinely spins: 2 degrees per 10 ms cycle is 200 deg/s in
    // the pose history around each exposure, over the 60 deg/s limit
    for (int i = 0; i < 3; ++i) {
        f.robot->odom.heading_rad += degToRad(2.0);
        f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
        f.stepOnce();
    }
    EXPECT_EQ(f.system->target().status, TargetStatus::kPendingAcquisition);

    // the robot settles; exposure-time motion drops to zero and evidence
    // counts again
    for (int i = 0; i < 3; ++i) {
        f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
        f.stepOnce();
    }
    EXPECT_EQ(f.system->target().status, TargetStatus::kLockedVision);
}

TEST(VisionTargets, RepeatedPrintedIdsDisambiguateByFullPose) {
    // left_goal and right_goal both carry printed id 1; near the right
    // goal the correct candidate wins decisively
    Fixture f;
    f.robot->odom = Pose2D{2.2, 0.6, 0.0};   // facing right_goal's west face

    f.select(5);
    f.stepOnce();
    f.pushFrame({f.detectionFor(Pose2D{3.0, 0.6, 0.0}, kCenterMount, 1)});
    f.stepOnce();
    ASSERT_EQ(f.system->target().status, TargetStatus::kLockedVision);
    f.stepOnce();   // the estimator folds the locked window next cycle

    const WorldObject& right = f.system->world().objects.at(WorldObjectId{"right_goal"});
    EXPECT_EQ(right.source, EstimateSource::kObserved);
    const WorldObject& left = f.system->world().objects.at(WorldObjectId{"left_goal"});
    EXPECT_EQ(left.source, EstimateSource::kFieldMap);   // untouched
}

TEST(VisionTargets, PartialEvidenceNeverMutatesWorldBeforeLock) {
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};
    f.select(1);   // minimum_consistent_observations = 3
    f.stepOnce();

    // two accepted frames: acquisition is in progress but nothing commits
    for (int i = 0; i < 2; ++i) {
        f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
        f.stepOnce();
    }
    EXPECT_EQ(f.system->target().status, TargetStatus::kPendingAcquisition);
    {
        const WorldObject& goal =
            f.system->world().objects.at(WorldObjectId{"center_goal"});
        EXPECT_EQ(goal.source, EstimateSource::kFieldMap);
        EXPECT_FALSE(goal.observed);
    }

    // the third frame locks; the estimator folds the locked window on the
    // following cycle, and nothing before the lock ever touched the world
    f.pushFrame({f.detectionFor(kCenterGoal, kCenterMount, 0)});
    f.stepOnce();
    ASSERT_EQ(f.system->target().status, TargetStatus::kLockedVision);
    {
        const WorldObject& pre =
            f.system->world().objects.at(WorldObjectId{"center_goal"});
        EXPECT_EQ(pre.source, EstimateSource::kFieldMap);   // not yet folded
    }
    f.stepOnce();
    const WorldObject& goal = f.system->world().objects.at(WorldObjectId{"center_goal"});
    EXPECT_EQ(goal.source, EstimateSource::kObserved);
    EXPECT_TRUE(goal.observed);
}

TEST(VisionTargets, SingleFrameWithManyTagsCountsAsOneObservation) {
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};
    f.select(1);
    f.stepOnce();

    // one exposure containing the tag three times must not satisfy three
    // consistent observations; frames count, not entries
    const NativeTagDetection d = f.detectionFor(kCenterGoal, kCenterMount, 0);
    f.pushFrame({d, d, d});
    f.stepOnce();
    for (int i = 0; i < 5; ++i) {
        f.stepOnce();   // no further frames
    }
    EXPECT_EQ(f.system->target().status, TargetStatus::kPendingAcquisition);
}

TEST(VisionTargets, NominalFallbackIgnoresUnconfirmedEvidence) {
    // the goal is physically displaced, but acquisition never completes:
    // the timeout fallback must come from the immutable map nominal with
    // zero trace of the unconfirmed camera evidence
    Fixture f;
    f.robot->odom = Pose2D{0.9, 1.7832, 0.0};
    f.select(1);
    f.stepOnce();

    const Pose2D displaced{kCenterGoal.x_m + 0.08, kCenterGoal.y_m, 0.0};
    for (int i = 0; i < 2; ++i) {   // one short of the lock
        f.pushFrame({f.detectionFor(displaced, kCenterMount, 0)});
        f.stepOnce();
    }
    while (f.now_ms < 1200) {
        f.stepOnce();
    }
    ASSERT_EQ(f.system->target().status, TargetStatus::kLockedNominal);
    EXPECT_NEAR(f.system->target().T_odom_robot_target.x_m, kExpectedBodyTarget.x_m,
                1e-9);
    EXPECT_NEAR(f.system->target().T_odom_robot_target.y_m, kExpectedBodyTarget.y_m,
                1e-9);
    const WorldObject& goal = f.system->world().objects.at(WorldObjectId{"center_goal"});
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);   // never touched
}

TEST(VisionTargets, SelectDuringInvalidLocalizationDefersActivation) {
    Fixture f;
    f.robot->valid = false;
    f.robot->odom  = Pose2D{1.0, 1.0, 0.0};

    f.select(2);   // robot_relative: would snapshot zeros if unguarded
    f.stepOnce();
    f.stepOnce();
    EXPECT_FALSE(f.system->target().active);

    // localization becomes real later, at a different pose; the deferred
    // command activates there
    f.robot->valid = true;
    f.robot->odom  = Pose2D{2.0, 1.0, 0.0};
    f.stepOnce();
    ASSERT_EQ(f.system->target().status, TargetStatus::kLockedRobotRelative);
    EXPECT_NEAR(f.system->target().T_odom_robot_target.x_m, 1.4, 1e-12);
    EXPECT_NEAR(f.system->target().T_odom_robot_target.y_m, 1.3, 1e-12);
}

TEST(VisionTargets, AmbiguousCandidatesAbstain) {
    // from far back both repeated-id mounts pass every expected-visibility
    // check (in front, facing, in the field of view, large enough); with an
    // enormous ambiguity margin the runner-up is never beaten decisively,
    // so the same evidence produces abstention, not a guess
    Fixture f(0.0, 50.0);
    f.robot->odom = Pose2D{-2.0, 1.8, 0.0};   // both goals ahead of the camera

    f.select(6);   // acquire the left goal, one consistent frame suffices
    f.stepOnce();
    for (int i = 0; i < 3; ++i) {
        f.pushFrame({f.detectionFor(Pose2D{0.6, 3.0, 0.0}, kCenterMount, 1)});
        f.stepOnce();
    }
    EXPECT_EQ(f.system->target().status, TargetStatus::kPendingAcquisition);
    const WorldObject& left = f.system->world().objects.at(WorldObjectId{"left_goal"});
    EXPECT_EQ(left.source, EstimateSource::kFieldMap);   // no mutation either

    // the identical scene with a sane margin is decisive
    Fixture g(0.0, 0.15);
    g.robot->odom = Pose2D{-2.0, 1.8, 0.0};
    g.select(6);
    g.stepOnce();
    g.pushFrame({g.detectionFor(Pose2D{0.6, 3.0, 0.0}, kCenterMount, 1)});
    g.stepOnce();
    EXPECT_EQ(g.system->target().status, TargetStatus::kLockedVision);
}
