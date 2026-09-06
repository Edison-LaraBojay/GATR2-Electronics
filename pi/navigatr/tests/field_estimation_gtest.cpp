// field_estimation_gtest.cpp
// Field map resource parsing and the landmark_field composite: a leaf, a
// noop, and a composite all satisfy the same FieldEstimation contract; the
// composite owns its private children, validates its nested schema, commits
// per its explicit policy, and never partially commits on a child fault.

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "impl/field_estimation/landmark_field.h"
#include "math/angles.h"
#include "payloads/field_object_evidence.h"
#include "resources/resource_map.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

// Composite child registered only in this test binary: emits a scripted
// landmark pose evidence set, or a fault, on demand.
struct EvidenceScript {
    FieldObjectPoseEvidenceSet set;
    bool                       emit  = false;
    bool                       fault = false;
};

class ScriptedAssociation : public Association
{
public:
    explicit ScriptedAssociation(std::shared_ptr<EvidenceScript> script)
        : script_(std::move(script)) {}

    AssociationOutput run(const AssociationInput&) override {
        AssociationOutput out;
        if (script_->fault) {
            out.status = FunctionStatus::kFault;
            return out;
        }
        if (script_->emit) {
            AssociationRecord record;
            record.payload = TypedPayload::store(
                script_->set, payload_names::kFieldObjectPoseEvidenceSet);
            out.associations.emplace(AssociationId{"landmark_pose_observations"},
                                     std::move(record));
        }
        return out;
    }

    std::vector<AssociationOutputDecl> produces() const override {
        return {AssociationOutputDecl{
            AssociationId{"landmark_pose_observations"},
            PayloadDescriptor::of<FieldObjectPoseEvidenceSet>(
                payload_names::kFieldObjectPoseEvidenceSet)}};
    }

private:
    std::shared_ptr<EvidenceScript> script_;
};

struct Fixture {
    tinyxml2::XMLDocument     doc;
    FunctionRegistry          functions;
    std::vector<std::string>  warnings;
    ResourceMap               store;
    SlotInitializationContext context;

    std::shared_ptr<EvidenceScript> script = std::make_shared<EvidenceScript>();

    SensorResultsMap sensorResults;
    ArtifactMap      artifacts;
    RobotState       robot;
    FieldState       field;
    CommandState     command;
    TargetState      target;

    Fixture() {
        register_resources(functions);
        register_perception(functions);
        register_association(functions);
        auto script_copy = script;
        functions.add<AssociationMakeFunction>(
            FunctionKey{"scripted_evidence"},
            [script_copy](const ConfigNode&, SlotInitializationContext&, std::string&) {
                return std::make_unique<ScriptedAssociation>(script_copy);
            });

        tinyxml2::XMLDocument resources_doc;
        EXPECT_EQ(resources_doc.Parse(R"(
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
</Resources>)"),
                  tinyxml2::XML_SUCCESS);
        ResourceMapBuilder builder(functions, &warnings);
        std::string        err;
        bool               ok = true;
        ConfigNode{resources_doc.RootElement()}.forEach("Resource",
                                                        [&](const ConfigNode& r) {
                                                            if (ok) {
                                                                ok = builder.index(r, err);
                                                            }
                                                        });
        EXPECT_TRUE(ok && builder.buildAll(err)) << err;
        store = builder.take();

        context.resources = &store;
        context.functions = &functions;
    }

    std::unique_ptr<FieldEstimation> make(const char* xml, std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        return LandmarkFieldEstimation::create(ConfigNode{doc.RootElement()}, context,
                                               err);
    }

    FieldEstimationOutput run(FieldEstimation& we, int64_t now = 1) {
        const auto out =
            we.run({sensorResults, artifacts, robot, field, hostTime(now)});
        field = out.field;
        return out;
    }

    void putEvidence(const char* object, double x, double y, double heading_rad,
                     double confidence, int64_t measured_ms = 1,
                     const char* feature = "goal_front") {
        FieldObjectPoseEvidence entry;
        entry.object           = FieldObjectId{object};
        entry.feature_instance = feature;
        entry.frame            = FrameId{"odometry"};
        entry.T_frame_object   = Pose2D{x, y, heading_rad};
        entry.measuredAt       = hostTime(measured_ms);
        entry.source           = SensorId{"scripted"};
        entry.confidence       = confidence;
        script->set.entries.clear();
        script->set.entries.push_back(entry);
        script->emit = true;
    }

    void addEvidence(const char* object, double x, double y, double heading_rad,
                     double confidence) {
        FieldObjectPoseEvidence entry;
        entry.object           = FieldObjectId{object};
        entry.feature_instance = "goal_front";
        entry.frame            = FrameId{"odometry"};
        entry.T_frame_object   = Pose2D{x, y, heading_rad};
        entry.measuredAt       = hostTime(1);
        entry.source           = SensorId{"scripted"};
        entry.confidence       = confidence;
        script->set.entries.push_back(entry);
        script->emit = true;
    }
};

const char* kSeedOnly = R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</FieldEstimation>)";

const char* kScriptedAlways = R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="scripted_evidence"/>
        <Estimator type="landmark_estimator" commit="always"/>
    </Pipeline>
</FieldEstimation>)";

} // namespace

TEST(FieldMapResource, ParsesLandmarksAndTagInstances) {
    Fixture     f;
    std::string err;
    auto        map = f.store.require<const FieldMap>(ResourceId{"override_field"}, err);
    ASSERT_NE(map, nullptr) << err;

    const LandmarkDecl* goal = map->find(FieldObjectId{"center_goal"});
    ASSERT_NE(goal, nullptr);
    EXPECT_NEAR(goal->nominal.x_m, 1.8, 1e-12);
    EXPECT_NEAR(goal->nominal.heading_rad, kPi / 2.0, 1e-12);

    // two physical mounts share one printed id; identity is the instance
    ASSERT_EQ(goal->mounts.size(), 2u);
    EXPECT_EQ(goal->mounts[0].instance_id, "goal_front");
    EXPECT_EQ(goal->mounts[1].instance_id, "goal_left");
    EXPECT_EQ(goal->mounts[0].observed_id, 7);
    EXPECT_EQ(goal->mounts[1].observed_id, 7);
    EXPECT_NEAR(goal->mounts[0].detection_size_m, 0.06, 1e-12);
    EXPECT_NEAR(goal->mounts[0].T_landmark_tag_surface.x_m, 0.15, 1e-12);
    // yaw 180: the surface outward normal points back along landmark -x
    EXPECT_NEAR(goal->mounts[0].T_landmark_tag_surface.R.m[0][0], -1.0, 1e-12);
}

TEST(LandmarkWorld, SeedsFromMapAndKeepsUnseenValid) {
    Fixture     f;
    std::string err;
    auto        we = f.make(kSeedOnly, err);
    ASSERT_NE(we, nullptr) << err;

    f.run(*we);
    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_TRUE(goal.valid);
    EXPECT_FALSE(goal.observed);
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);
    EXPECT_EQ(goal.pose.frame.value, "field");
    EXPECT_NEAR(goal.pose.pose.x_m, 1.8, 1e-12);
}

TEST(LandmarkWorld, CommittedEvidenceMovesLandmarksAndQuietCyclesRetain) {
    Fixture     f;
    std::string err;
    auto        we = f.make(kScriptedAlways, err);
    ASSERT_NE(we, nullptr) << err;

    f.run(*we);
    f.putEvidence("center_goal", 1.9, 1.75, kPi / 2.0, 0.9);
    const auto out = f.run(*we);

    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_NEAR(goal.pose.pose.x_m, 1.9, 1e-12);
    EXPECT_NEAR(goal.pose.pose.y_m, 1.75, 1e-12);
    EXPECT_TRUE(goal.observed);
    EXPECT_EQ(goal.source, EstimateSource::kObserved);
    // the accepted evidence is republished across the boundary
    EXPECT_EQ(out.associations.count(AssociationId{"landmark_pose_observations"}), 1u);

    f.script->emit = false;
    f.run(*we);
    EXPECT_FALSE(f.field.objects.at(FieldObjectId{"center_goal"}).observed);
    EXPECT_TRUE(f.field.objects.at(FieldObjectId{"center_goal"}).valid);
    EXPECT_NEAR(f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m, 1.9,
                1e-12);
}

TEST(LandmarkWorld, CommitNeverPublishesEvidenceWithoutMutation) {
    Fixture     f;
    std::string err;
    auto        we = f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="scripted_evidence"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</FieldEstimation>)",
                            err);
    ASSERT_NE(we, nullptr) << err;

    f.putEvidence("center_goal", 2.4, 2.4, 0.0, 0.9);
    const auto out = f.run(*we);
    // evidence crosses the boundary, the estimate does not move
    EXPECT_EQ(out.associations.count(AssociationId{"landmark_pose_observations"}), 1u);
    EXPECT_NEAR(f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m, 1.8,
                1e-12);
    EXPECT_FALSE(f.field.objects.at(FieldObjectId{"center_goal"}).observed);
}

TEST(LandmarkWorld, ChildFaultCommitsNothing) {
    Fixture     f;
    std::string err;
    auto        we = f.make(kScriptedAlways, err);
    ASSERT_NE(we, nullptr) << err;

    f.run(*we);   // seeds
    const auto seeded_x =
        f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m;

    f.putEvidence("center_goal", 2.4, 2.4, 0.0, 0.9);
    f.script->fault = true;
    const auto out  = f.run(*we);
    EXPECT_EQ(out.status, FunctionStatus::kFault);
    EXPECT_TRUE(out.associations.empty());
    EXPECT_TRUE(out.observations.empty());
    EXPECT_NEAR(f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m,
                seeded_x, 1e-12);
}

TEST(LandmarkWorld, BlendHalvesTheStep) {
    Fixture     f;
    std::string err;
    auto        we = f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="scripted_evidence"/>
        <Estimator type="landmark_estimator" commit="always" blend="0.5"/>
    </Pipeline>
</FieldEstimation>)",
                            err);
    ASSERT_NE(we, nullptr) << err;
    f.run(*we);
    f.putEvidence("center_goal", 2.0, 1.8, kPi / 2.0, 0.5);
    f.run(*we);
    EXPECT_NEAR(f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose.x_m, 1.9,
                1e-12);
}

TEST(LandmarkWorld, StrictNestedSchema) {
    Fixture     f;
    std::string err;

    // out of range blend is an error, not a clamp
    EXPECT_EQ(f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="never" blend="1.5"/>
    </Pipeline>
</FieldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("blend"), std::string::npos);

    // unknown field map resource
    EXPECT_EQ(f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="ghost_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</FieldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("ghost_field"), std::string::npos);

    // committing with no published evidence is a contradiction
    EXPECT_EQ(f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="always"/>
    </Pipeline>
</FieldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("commit"), std::string::npos);

    // unknown estimator implementation
    EXPECT_EQ(f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="kalman_9000" commit="never"/>
    </Pipeline>
</FieldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("kalman_9000"), std::string::npos);

    // unknown nested element, with the full configuration path
    EXPECT_EQ(f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Associaton type="noop"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</FieldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("Associaton"), std::string::npos);
    EXPECT_NE(err.find("Pipeline"), std::string::npos);

    // duplicate nested child
    EXPECT_EQ(f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</FieldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("more than one Association"), std::string::npos);

    // missing estimator
    EXPECT_EQ(f.make(R"(
<FieldEstimation type="landmark_field">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
    </Pipeline>
</FieldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("Estimator"), std::string::npos);
}

TEST(LandmarkWorld, LeafNoopAndCompositeSatisfyTheSameSlot) {
    // The coordinator accepts the explicit noop and the composite through
    // the identical contract; swapping them is a configuration change only.
    FunctionRegistry functions;
    registerAll(functions);

    const char* composite_system = R"(
<System>
    <Resources>
        <Resource id="override_field" type="field_map">
            <Landmark id="center_goal">
                <NominalPose calibration_status="verified"
                             x_m="1.8" y_m="1.8" heading_deg="0"/>
            </Landmark>
        </Resource>
    </Resources>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Preprocessing type="noop"/>
        <Localization type="noop"/>
        <FieldEstimation type="landmark_field">
            <FieldMap resource_id="override_field"/>
            <Pipeline>
                <ObservationExtraction type="noop"/>
                <Association type="noop"/>
                <Estimator type="landmark_estimator" commit="never"/>
            </Pipeline>
        </FieldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";

    std::string err;
    auto        system = System::buildFromString(composite_system, functions, err);
    ASSERT_NE(system, nullptr) << err;
    system->step(hostTime(1));
    EXPECT_EQ(system->field().objects.count(FieldObjectId{"center_goal"}), 1u);

    const char* noop_system = R"(
<System>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Preprocessing type="noop"/>
        <Localization type="noop"/>
        <FieldEstimation type="noop"/>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
    auto plain = System::buildFromString(noop_system, functions, err);
    ASSERT_NE(plain, nullptr) << err;
    plain->step(hostTime(1));
    EXPECT_TRUE(plain->field().objects.empty());
}

TEST(LandmarkField, EstimatesAreUnboundedOutsideTheNominalField) {
    Fixture     f;
    std::string err;
    auto        we = f.make(kScriptedAlways, err);
    ASSERT_NE(we, nullptr) << err;
    f.run(*we);
    // far outside the 144x144 inch nominal boundary; never clamped
    f.putEvidence("center_goal", 25.0, -13.0, 2.0, 0.9);
    f.run(*we);
    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_NEAR(goal.pose.pose.x_m, 25.0, 1e-9);
    EXPECT_NEAR(goal.pose.pose.y_m, -13.0, 1e-9);
    EXPECT_TRUE(goal.valid);
}

TEST(LandmarkField, MultiEvidenceFusionIsDeterministicAndOrderFree) {
    Fixture     f;
    std::string err;
    auto        we = f.make(kScriptedAlways, err);
    ASSERT_NE(we, nullptr) << err;

    f.run(*we);
    f.script->set.entries.clear();
    f.addEvidence("center_goal", 2.0, 2.0, 0.0, 0.9);
    f.addEvidence("center_goal", 2.2, 2.4, 0.2, 0.3);
    f.run(*we);
    const auto first = f.field.objects.at(FieldObjectId{"center_goal"}).pose.pose;

    // same evidence, opposite order, fresh state: identical result
    Fixture     g;
    auto        wg = g.make(kScriptedAlways, err);
    ASSERT_NE(wg, nullptr) << err;
    g.run(*wg);
    g.script->set.entries.clear();
    g.addEvidence("center_goal", 2.2, 2.4, 0.2, 0.3);
    g.addEvidence("center_goal", 2.0, 2.0, 0.0, 0.9);
    g.run(*wg);
    const auto second = g.field.objects.at(FieldObjectId{"center_goal"}).pose.pose;

    EXPECT_NEAR(first.x_m, second.x_m, 1e-12);
    EXPECT_NEAR(first.y_m, second.y_m, 1e-12);
    EXPECT_NEAR(first.heading_rad, second.heading_rad, 1e-12);
    // and the higher-confidence measurement dominates the weighted mean
    EXPECT_LT(first.x_m, 2.1);
}

TEST(LandmarkField, NonFiniteEvidenceNeverReachesState) {
    Fixture     f;
    std::string err;
    auto        we = f.make(kScriptedAlways, err);
    ASSERT_NE(we, nullptr) << err;
    f.run(*we);
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    f.putEvidence("center_goal", nan, 2.0, 0.0, 0.9);
    f.run(*we);
    const auto& goal = f.field.objects.at(FieldObjectId{"center_goal"});
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);
    EXPECT_NEAR(goal.pose.pose.x_m, 1.8, 1e-12);
    EXPECT_FALSE(goal.observed);
}
