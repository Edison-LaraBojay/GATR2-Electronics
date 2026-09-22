// world_estimation_gtest.cpp
// World estimation as an aggregate stage: exactly one Estimator selected
// through the registry and validated at build, driven by the coordinator
// through one contract that the noop and the apriltag estimator both
// satisfy. The apriltag estimator's landmark estimation step is tested on
// its own (seeding, commit policy, blend, deterministic fusion, non-finite
// rejection); the estimator as a whole is tested through the System with a
// scripted camera and detector (schema, optional association, no partial
// commit on a step fault, quiet cycles).

#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <typeindex>
#include <vector>

#include "impl/resources/cameras.h"
#include "impl/world_estimation/landmark_estimator.h"
#include "math/angles.h"
#include "payloads/field_object_evidence.h"
#include "payloads/tag_observations.h"
#include "resources/camera.h"
#include "resources/resource_store.h"
#include "resources/tag_detector.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

// ---- landmark estimation step --------------------------------------------

const char* kFieldResources = R"(
<Resources>
    <Resource id="override_field" type="field_map">
        <Landmark id="center_goal">
            <NominalPose calibration_status="verified"
                         x_m="1.8" y_m="1.8" heading_deg="90"/>
            <TagMount instance_id="goal_front" calibration_status="verified"
                      family="tag36h11" observed_id="7" detection_size_m="0.06">
                <PoseOfTagSurfaceInLandmark x_m="0.15" y_m="0" z_m="0.25"
                    roll_deg="0" pitch_deg="0" yaw_deg="180"/>
            </TagMount>
            <TagMount instance_id="goal_left" calibration_status="verified"
                      family="tag36h11" observed_id="7" detection_size_m="0.06">
                <PoseOfTagSurfaceInLandmark x_m="0" y_m="0.15" z_m="0.25"
                    roll_deg="0" pitch_deg="0" yaw_deg="90"/>
            </TagMount>
        </Landmark>
    </Resource>
</Resources>)";

struct LandmarkFixture {
    FunctionRegistry                functions;
    std::vector<std::string>        warnings;
    ResourceStore                   store;
    std::shared_ptr<const FieldMap> map;
    tinyxml2::XMLDocument           doc;

    RobotState robot;
    FieldState field;

    LandmarkFixture() {
        register_resources(functions);
        tinyxml2::XMLDocument resources_doc;
        EXPECT_EQ(resources_doc.Parse(kFieldResources), tinyxml2::XML_SUCCESS);
        ResourceStoreBuilder builder(functions, &warnings);
        std::string          err;
        bool                 ok = true;
        ConfigNode{resources_doc.RootElement()}.forEach("Resource", [&](const ConfigNode& r) {
            if (ok) {
                ok = builder.index(r, err);
            }
        });
        EXPECT_TRUE(ok && builder.buildAll(err)) << err;
        store = builder.take();
        map   = store.require<const FieldMap>(ResourceId{"override_field"}, err);
        EXPECT_NE(map, nullptr) << err;
    }

    std::optional<LandmarkEstimator> make(const char* xml, std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        return LandmarkEstimator::create(ConfigNode{doc.RootElement()}, map, err);
    }

    void run(const LandmarkEstimator& estimator,
             const FieldObjectPoseEvidenceSet* evidence = nullptr, int64_t now = 1) {
        field = estimator.run(field, evidence, robot, hostTime(now));
    }

    static FieldObjectPoseEvidence evidence(const char* object, double x, double y,
                                            double heading_rad, double confidence,
                                            int64_t measured_ms = 1) {
        FieldObjectPoseEvidence entry;
        entry.object           = FieldObjectId{object};
        entry.feature_instance = "goal_front";
        entry.frame            = FrameId{"odometry"};
        entry.T_frame_object   = Pose2D{x, y, heading_rad};
        entry.measuredAt       = hostTime(measured_ms);
        entry.source           = SensorId{"scripted"};
        entry.confidence       = confidence;
        return entry;
    }

    static FieldObjectPoseEvidenceSet one(const char* object, double x, double y,
                                          double heading_rad, double confidence) {
        FieldObjectPoseEvidenceSet set;
        set.entries.push_back(evidence(object, x, y, heading_rad, confidence));
        return set;
    }
};

const char* kAlways = R"(<LandmarkEstimation commit="always"/>)";
const char* kNever  = R"(<LandmarkEstimation commit="never"/>)";

// ---- the apriltag estimator through the System ---------------------------

struct CameraScript {
    std::optional<CameraFrameData> pending;
    uint32_t                       next_sequence = 0;
};

class ScriptedCamera : public CameraDevice
{
public:
    explicit ScriptedCamera(std::shared_ptr<CameraScript> script) : script_(std::move(script)) {
        intrinsics_.model                = "brown_conrady";
        intrinsics_.calibrated_width_px  = 640;
        intrinsics_.calibrated_height_px = 480;
        intrinsics_.fx_px                = 500;
        intrinsics_.fy_px                = 500;
        intrinsics_.cx_px                = 320;
        intrinsics_.cy_px                = 240;
    }
    bool                    alive() const override { return true; }
    const CameraIntrinsics* intrinsics() const override { return &intrinsics_; }
    FrameId engineeringFrame() const override { return FrameId{"front_camera_engineering"}; }
    std::optional<CameraFrameData> latestFrame(uint64_t, uint32_t after) override {
        if (script_->pending.has_value() && script_->pending->sequence > after) {
            return script_->pending;
        }
        return std::nullopt;
    }

private:
    std::shared_ptr<CameraScript> script_;
    CameraIntrinsics              intrinsics_;
};

struct DetectorScript {
    std::map<uint32_t, std::vector<NativeTagDetection>> by_sequence;
    bool                                                fail = false;
};

class ScriptedDetector : public TagDetector
{
public:
    explicit ScriptedDetector(std::shared_ptr<DetectorScript> script) : script_(std::move(script)) {}
    bool detect(const CameraFrameData& frame, const CameraIntrinsics*,
                std::vector<NativeTagDetection>& out, std::string& err) override {
        if (script_->fail) {
            err = "scripted detector failure";
            return false;
        }
        const auto it = script_->by_sequence.find(frame.sequence);
        out = it == script_->by_sequence.end() ? std::vector<NativeTagDetection>{} : it->second;
        return true;
    }

private:
    std::shared_ptr<DetectorScript> script_;
};

const char* kAprilTagFull = R"(
        <WorldEstimation>
            <Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <Association>
                    <Observations observation_id="tag_observations"/>
                    <FieldMap resource_id="game_field"/>
                    <RobotFrames resource_id="robot_geometry"/>
                    <Attitude policy="assume_level"/>
                    <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
                           ambiguity_margin_m="0.15" max_range_m="3.0"
                           min_decision_margin="10" max_hamming="0"/>
                    <Output association_id="landmark_pose_observations"/>
                    <Trace association_id="tag_association_trace"/>
                </Association>
                <LandmarkEstimation commit="always" blend="0.5"/>
            </Estimator>
        </WorldEstimation>)";

const char* kDecodesOnly = R"(
        <WorldEstimation>
            <Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <LandmarkEstimation commit="never"/>
            </Estimator>
        </WorldEstimation>)";

std::string rigXml(const std::string& world_estimation) {
    return R"(
<System>
    <Loop rate_hz="100"/>
    <Resources>
        <Resource id="robot_geometry" type="robot_frame_map">
            <Frame id="front_camera_engineering" parent_frame_id="robot_body"
                   calibration_status="verified">
                <PoseOfChildInParent x_m="0.2" y_m="0" z_m="0.3"
                    roll_deg="0" pitch_deg="0" yaw_deg="0"/>
            </Frame>
        </Resource>
        <Resource id="camera_device" type="scripted_camera"/>
        <Resource id="detector" type="scripted_detector"/>
        <Resource id="game_field" type="field_map">
            <Landmark id="center_goal">
                <NominalPose calibration_status="verified"
                             x_m="1.7832" y_m="1.7832" heading_deg="0"/>
                <TagMount instance_id="center_goal_west_tag" calibration_status="verified"
                          family="tag36h11" observed_id="0" detection_size_m="0.06">
                    <PoseOfTagSurfaceInLandmark x_m="-0.14" y_m="0" z_m="0.25"
                        roll_deg="0" pitch_deg="0" yaw_deg="180"/>
                </TagMount>
            </Landmark>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="front_camera" type="camera_frame">
            <Source resource_id="camera_device" output_id="frame"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>)" +
           world_estimation + R"(
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
}

struct RigFixture {
    std::shared_ptr<CameraScript>   camera   = std::make_shared<CameraScript>();
    std::shared_ptr<DetectorScript> detector = std::make_shared<DetectorScript>();
    FunctionRegistry                functions;
    std::unique_ptr<System>         system;
    int64_t                         now_ms = 0;

    RigFixture() {
        registerAll(functions);
        auto cam = camera;
        functions.add(FunctionKey{"scripted_camera"},
                      ResourceMakeFunction([cam](const ConfigNode&,
                                                 ResourceInitializationContext&,
                                                 std::string&) {
                          return cameraResource(std::make_shared<ScriptedCamera>(cam),
                                                OutputId{"frame"});
                      }));
        auto det = detector;
        functions.add(FunctionKey{"scripted_detector"},
                      ResourceMakeFunction([det](const ConfigNode&,
                                                 ResourceInitializationContext&,
                                                 std::string&) {
                          return ResourceInstance::asContract<TagDetector>(
                              std::make_shared<ScriptedDetector>(det));
                      }));
    }

    std::unique_ptr<System> build(const std::string& world_estimation, std::string& err) {
        return System::buildFromString(rigXml(world_estimation).c_str(), functions, err);
    }

    void stepOnce() {
        now_ms += 10;
        system->step(hostTime(now_ms));
    }

    void pushFrame(const std::vector<NativeTagDetection>& tags) {
        CameraFrameData frame;
        frame.sequence   = ++camera->next_sequence;
        frame.exposureAt = hostTime(now_ms + 10);   // the cycle that consumes it
        frame.width_px   = 640;
        frame.height_px  = 480;
        camera->pending  = frame;
        detector->by_sequence[frame.sequence] = tags;
    }

    static NativeTagDetection decode(int observed_id) {
        NativeTagDetection d;
        d.family          = "tag36h11";
        d.observed_id     = observed_id;
        d.decision_margin = 60.0;
        d.has_pose        = false;   // a 2D decode
        return d;
    }
};

// all-noop pipeline around one WorldEstimation section
std::string noopSystemWith(const std::string& world_estimation) {
    return R"(<System><Pipeline><CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>)" +
           world_estimation + R"(<TargetResolution type="noop"/><Publishing type="noop"/>
        </Pipeline></System>)";
}

} // namespace

// ---- landmark estimation step --------------------------------------------

TEST(LandmarkEstimator, SeedsFromMapAndKeepsUnseenValid) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kNever, err);
    ASSERT_TRUE(estimator.has_value()) << err;
    EXPECT_FALSE(estimator->commits());

    f.run(*estimator);
    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_TRUE(goal.valid);
    EXPECT_FALSE(goal.observed);
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);
    EXPECT_EQ(goal.pose.frame.value, "field");
    EXPECT_NEAR(goal.pose.pose.x_m, 1.8, 1e-12);
    EXPECT_NEAR(goal.pose.pose.heading_rad, kPi / 2.0, 1e-12);
}

TEST(LandmarkEstimator, CommittedEvidenceMovesLandmarksAndQuietCyclesRetain) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kAlways, err);
    ASSERT_TRUE(estimator.has_value()) << err;
    EXPECT_TRUE(estimator->commits());

    f.run(*estimator);
    const auto set = LandmarkFixture::one("center_goal", 1.9, 1.75, kPi / 2.0, 0.9);
    f.run(*estimator, &set);

    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_NEAR(goal.pose.pose.x_m, 1.9, 1e-12);
    EXPECT_NEAR(goal.pose.pose.y_m, 1.75, 1e-12);
    EXPECT_TRUE(goal.observed);
    EXPECT_EQ(goal.source, EstimateSource::kObserved);
    EXPECT_EQ(goal.last_feature, "goal_front");
    EXPECT_EQ(goal.last_source, "scripted");

    // a quiet invocation retains the estimate and clears the observed flag
    f.run(*estimator);
    EXPECT_FALSE(f.field.objects.at(FieldObjectId{"center_goal"}).observed);
    EXPECT_TRUE(f.field.objects.at(FieldObjectId{"center_goal"}).valid);
    EXPECT_NEAR(f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m, 1.9, 1e-12);
}

TEST(LandmarkEstimator, CommitNeverKeepsTheEstimatesStill) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kNever, err);
    ASSERT_TRUE(estimator.has_value()) << err;

    const auto set = LandmarkFixture::one("center_goal", 2.4, 2.4, 0.0, 0.9);
    f.run(*estimator, &set);
    EXPECT_NEAR(f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m, 1.8, 1e-12);
    EXPECT_FALSE(f.field.objects.at(FieldObjectId{"center_goal"}).observed);
}

TEST(LandmarkEstimator, BlendHalvesTheStep) {
    LandmarkFixture f;
    std::string     err;
    auto estimator = f.make(R"(<LandmarkEstimation commit="always" blend="0.5"/>)", err);
    ASSERT_TRUE(estimator.has_value()) << err;
    f.run(*estimator);
    const auto set = LandmarkFixture::one("center_goal", 2.0, 1.8, kPi / 2.0, 0.5);
    f.run(*estimator, &set);
    EXPECT_NEAR(f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m, 1.9, 1e-12);
}

TEST(LandmarkEstimator, EstimatesAreUnboundedOutsideTheNominalField) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kAlways, err);
    ASSERT_TRUE(estimator.has_value()) << err;
    f.run(*estimator);
    // far outside the 144x144 inch nominal boundary; never clamped
    const auto set = LandmarkFixture::one("center_goal", 25.0, -13.0, 2.0, 0.9);
    f.run(*estimator, &set);
    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_NEAR(goal.pose.pose.x_m, 25.0, 1e-9);
    EXPECT_NEAR(goal.pose.pose.y_m, -13.0, 1e-9);
    EXPECT_TRUE(goal.valid);
}

TEST(LandmarkEstimator, MultiEvidenceFusionIsDeterministicAndOrderFree) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kAlways, err);
    ASSERT_TRUE(estimator.has_value()) << err;
    f.run(*estimator);

    FieldObjectPoseEvidenceSet forward;
    forward.entries.push_back(LandmarkFixture::evidence("center_goal", 2.0, 2.0, 0.0, 0.9));
    forward.entries.push_back(LandmarkFixture::evidence("center_goal", 2.2, 2.4, 0.2, 0.3));
    f.run(*estimator, &forward);
    const auto first = f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose;

    // same evidence, opposite order, fresh state: identical result
    LandmarkFixture g;
    auto            other = g.make(kAlways, err);
    ASSERT_TRUE(other.has_value()) << err;
    g.run(*other);
    FieldObjectPoseEvidenceSet reversed;
    reversed.entries.push_back(LandmarkFixture::evidence("center_goal", 2.2, 2.4, 0.2, 0.3));
    reversed.entries.push_back(LandmarkFixture::evidence("center_goal", 2.0, 2.0, 0.0, 0.9));
    g.run(*other, &reversed);
    const auto second = g.field.objects.at(FieldObjectId{"center_goal"}).pose.pose;

    EXPECT_NEAR(first.x_m, second.x_m, 1e-12);
    EXPECT_NEAR(first.y_m, second.y_m, 1e-12);
    EXPECT_NEAR(first.heading_rad, second.heading_rad, 1e-12);
    // and the higher-confidence measurement dominates the weighted mean
    EXPECT_LT(first.x_m, 2.1);
}

TEST(LandmarkEstimator, NonFiniteEvidenceNeverReachesState) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kAlways, err);
    ASSERT_TRUE(estimator.has_value()) << err;
    f.run(*estimator);
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    const auto set = LandmarkFixture::one("center_goal", nan, 2.0, 0.0, 0.9);
    f.run(*estimator, &set);
    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);
    EXPECT_NEAR(goal.pose.pose.x_m, 1.8, 1e-12);
    EXPECT_FALSE(goal.observed);
}

TEST(LandmarkEstimator, OdometryEpochChangeReturnsObservedEntriesToNominal) {
    LandmarkFixture f;
    std::string     err;
    auto            estimator = f.make(kAlways, err);
    ASSERT_TRUE(estimator.has_value()) << err;
    f.run(*estimator);
    const auto set = LandmarkFixture::one("center_goal", 2.4, 2.4, 0.0, 0.9);
    f.run(*estimator, &set);
    ASSERT_EQ(f.field.objects.at(FieldObjectId{"center_goal"}).source, EstimateSource::kObserved);

    // the frame the measurement was taken in no longer exists
    f.robot.odometry_epoch += 1;
    f.run(*estimator);
    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);
    EXPECT_TRUE(goal.valid);
    EXPECT_NEAR(goal.pose.pose.x_m, 1.8, 1e-12);
    EXPECT_NEAR(goal.confidence, 0.5, 1e-12);
}

TEST(LandmarkEstimator, StrictSchema) {
    LandmarkFixture f;
    std::string     err;

    // out of range blend is an error, not a clamp
    EXPECT_FALSE(f.make(R"(<LandmarkEstimation commit="never" blend="1.5"/>)", err).has_value());
    EXPECT_NE(err.find("blend"), std::string::npos);

    // the commit policy is explicit configuration
    EXPECT_FALSE(f.make(R"(<LandmarkEstimation/>)", err).has_value());
    EXPECT_NE(err.find("commit"), std::string::npos);
    EXPECT_FALSE(f.make(R"(<LandmarkEstimation commit="sometimes"/>)", err).has_value());
    EXPECT_NE(err.find("sometimes"), std::string::npos);
}

// ---- the stage -----------------------------------------------------------

TEST(WorldEstimationStage, ExactlyOneTypedEstimatorIsConfigured) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;

    const auto rejects = [&](const std::string& world, const char* expected) {
        EXPECT_EQ(System::buildFromString(noopSystemWith(world).c_str(), functions, err),
                  nullptr)
            << world;
        EXPECT_NE(err.find(expected), std::string::npos) << err;
    };
    rejects("<WorldEstimation/>", "exactly one Estimator");
    rejects(R"(<WorldEstimation><Estimator id="a" type="noop"/>
               <Estimator id="b" type="noop"/></WorldEstimation>)",
            "more than one Estimator");
    rejects(R"(<WorldEstimation><Estimatr id="a" type="noop"/></WorldEstimation>)",
            "unknown element Estimatr");
    rejects(R"(<WorldEstimation><Estimator type="noop"/></WorldEstimation>)",
            "needs id and type");
    rejects(R"(<WorldEstimation><Estimator id="a"/></WorldEstimation>)", "needs id and type");
    rejects(R"(<WorldEstimation><Estimator id="a" type="kalman_9000"/></WorldEstimation>)",
            "kalman_9000");
    // a key from another category fails on signature, never constructs
    rejects(R"(<WorldEstimation><Estimator id="a" type="vex_brain"/></WorldEstimation>)",
            "different signature");
}

TEST(WorldEstimationStage, NoopEstimatorKeepsTheFieldEmptyAndIsLabeledById) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromString(
        noopSystemWith(R"(<WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>)")
            .c_str(),
        functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->worldEstimation().estimatorId(), "none");
    EXPECT_EQ(system->worldEstimation().estimatorType(), "noop");
    EXPECT_TRUE(system->worldEstimation().observationOutputs().empty());
    EXPECT_TRUE(system->worldEstimation().associationOutputs().empty());

    system->step(hostTime(1));
    system->step(hostTime(2));
    EXPECT_TRUE(system->field().objects.empty());
    EXPECT_EQ(system->fieldSnapshot()->status, FunctionStatus::kOk);
    EXPECT_EQ(system->fieldSnapshot()->invocation, 2u);
    ASSERT_EQ(system->fieldDiagnostics().functions.count("WorldEstimation/none"), 1u);
    EXPECT_EQ(system->fieldDiagnostics().functions.at("WorldEstimation/none").runs, 2u);
}

// ---- the apriltag estimator ----------------------------------------------

TEST(AprilTagWorldEstimator, BuildsResolvesReferencesAndDeclaresOutputs) {
    RigFixture  f;
    std::string err;
    f.system = f.build(kAprilTagFull, err);
    ASSERT_NE(f.system, nullptr) << err;

    const auto& world = f.system->worldEstimation();
    EXPECT_EQ(world.estimatorId(), "goals");
    EXPECT_EQ(world.estimatorType(), "apriltag");
    ASSERT_EQ(world.observationOutputs().size(), 1u);
    EXPECT_EQ(world.observationOutputs()[0].id.value, "tag_observations");
    EXPECT_TRUE(world.observationOutputs()[0].payload.matches(
        std::type_index(typeid(TagObservationSet))));
    ASSERT_EQ(world.associationOutputs().size(), 2u);
    EXPECT_EQ(world.associationOutputs()[0].id.value, "landmark_pose_observations");
    EXPECT_TRUE(world.associationOutputs()[0].payload.matches(
        std::type_index(typeid(FieldObjectPoseEvidenceSet))));
    EXPECT_EQ(world.associationOutputs()[1].id.value, "tag_association_trace");

    f.stepOnce();
    const auto& goal = f.system->field().objects.at(FieldObjectId{"center_goal"});
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);
    EXPECT_NEAR(goal.pose.pose.x_m, 1.7832, 1e-12);
    EXPECT_EQ(f.system->fieldDiagnostics().functions.count("WorldEstimation/goals"), 1u);
}

TEST(AprilTagWorldEstimator, QuietCyclesWithoutAFrameAreNotFaults) {
    RigFixture  f;
    std::string err;
    f.system = f.build(kAprilTagFull, err);
    ASSERT_NE(f.system, nullptr) << err;
    for (int i = 0; i < 3; ++i) {
        f.stepOnce();
    }
    const auto snapshot = f.system->fieldSnapshot();
    EXPECT_EQ(snapshot->status, FunctionStatus::kOk);
    EXPECT_TRUE(snapshot->observations.empty());
    EXPECT_TRUE(snapshot->associations.empty());
    EXPECT_EQ(snapshot->field.objects.count(FieldObjectId{"center_goal"}), 1u);
    EXPECT_EQ(f.system->fieldDiagnostics().functions.at("WorldEstimation/goals").fault, 0u);
}

TEST(AprilTagWorldEstimator, WithoutAssociationDecodesPublishAndNothingCommits) {
    RigFixture  f;
    std::string err;
    f.system = f.build(kDecodesOnly, err);
    ASSERT_NE(f.system, nullptr) << err;
    EXPECT_TRUE(f.system->worldEstimation().associationOutputs().empty());

    f.pushFrame({RigFixture::decode(0)});
    f.stepOnce();
    const auto snapshot = f.system->fieldSnapshot();
    EXPECT_EQ(snapshot->status, FunctionStatus::kOk);
    ASSERT_EQ(snapshot->observations.count(ObservationId{"tag_observations"}), 1u);
    const auto* set = snapshot->observations.at(ObservationId{"tag_observations"})
                          .payload.get<TagObservationSet>();
    ASSERT_NE(set, nullptr);
    ASSERT_EQ(set->tags.size(), 1u);
    EXPECT_EQ(set->tags[0].observed_id, 0);
    EXPECT_TRUE(snapshot->associations.empty());
    EXPECT_EQ(snapshot->field.objects.at(FieldObjectId{"center_goal"}).source,
              EstimateSource::kFieldMap);

    // the same frame is evidence once: the next cycle publishes nothing new
    f.stepOnce();
    EXPECT_TRUE(f.system->fieldSnapshot()->observations.empty());
}

TEST(AprilTagWorldEstimator, DetectorFaultLeavesThePreviousFieldAndPublishesNothing) {
    RigFixture  f;
    std::string err;
    f.system = f.build(kAprilTagFull, err);
    ASSERT_NE(f.system, nullptr) << err;
    f.stepOnce();   // seeds
    const auto seeded_x = f.system->field().objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m;

    f.detector->fail = true;
    f.pushFrame({RigFixture::decode(0)});
    f.stepOnce();
    const auto snapshot = f.system->fieldSnapshot();
    EXPECT_EQ(snapshot->status, FunctionStatus::kFault);
    EXPECT_NE(snapshot->diagnostic.find("scripted detector failure"), std::string::npos);
    EXPECT_TRUE(snapshot->observations.empty());
    EXPECT_TRUE(snapshot->associations.empty());
    EXPECT_NEAR(snapshot->field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m,
                seeded_x, 1e-12);
    EXPECT_EQ(f.system->fieldDiagnostics().functions.at("WorldEstimation/goals").fault, 1u);
}

TEST(AprilTagWorldEstimator, StrictSchemaAndReferenceChecks) {
    RigFixture  f;
    std::string err;
    const auto  rejects = [&](const char* world, const char* expected) {
        EXPECT_EQ(f.build(world, err), nullptr) << world;
        EXPECT_NE(err.find(expected), std::string::npos) << err;
    };

    // committing with no association publishing evidence is a contradiction
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <LandmarkEstimation commit="always"/>
            </Estimator></WorldEstimation>)",
            "commit");

    // out of range blend
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <LandmarkEstimation commit="never" blend="1.5"/>
            </Estimator></WorldEstimation>)",
            "blend");

    // unknown field map, camera and detector references die at build
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="ghost_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <LandmarkEstimation commit="never"/>
            </Estimator></WorldEstimation>)",
            "ghost_field");
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="ghost_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <LandmarkEstimation commit="never"/>
            </Estimator></WorldEstimation>)",
            "ghost_camera");
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="ghost_detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <LandmarkEstimation commit="never"/>
            </Estimator></WorldEstimation>)",
            "ghost_detector");

    // the association must consume the extraction's declared output
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <Association>
                    <Observations observation_id="ghost_observations"/>
                    <FieldMap resource_id="game_field"/>
                    <RobotFrames resource_id="robot_geometry"/>
                    <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
                           ambiguity_margin_m="0.15" max_range_m="3.0"
                           min_decision_margin="10"/>
                    <Output association_id="landmark_pose_observations"/>
                </Association>
                <LandmarkEstimation commit="always"/>
            </Estimator></WorldEstimation>)",
            "ghost_observations");

    // unknown element, with the full configuration path
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <Associaton/>
                <LandmarkEstimation commit="never"/>
            </Estimator></WorldEstimation>)",
            "Associaton");
    EXPECT_NE(err.find("Estimator"), std::string::npos) << err;

    // duplicate step
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <Association/>
                <Association/>
                <LandmarkEstimation commit="never"/>
            </Estimator></WorldEstimation>)",
            "more than one Association");

    // missing steps
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
            </Estimator></WorldEstimation>)",
            "LandmarkEstimation");
    rejects(R"(<WorldEstimation><Estimator id="goals" type="apriltag">
                <FieldMap resource_id="game_field"/>
                <LandmarkEstimation commit="never"/>
            </Estimator></WorldEstimation>)",
            "ObservationExtraction");
}
