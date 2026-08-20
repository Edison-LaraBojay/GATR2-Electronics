// world_estimation_gtest.cpp
// Field map resource parsing and the landmark_world composite: a leaf, a
// noop, and a composite all satisfy the same WorldEstimation contract; the
// composite owns its private children, validates its nested schema, commits
// per its explicit policy, and never partially commits on a child fault.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "impl/world_estimation/landmark_world.h"
#include "math/angles.h"
#include "payloads/landmark_pose_observations.h"
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
    LandmarkPoseObservationSet set;
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
                script_->set, payload_names::kLandmarkPoseObservationSet);
            out.associations.emplace(AssociationId{"landmark_pose_observations"},
                                     std::move(record));
        }
        return out;
    }

    std::vector<AssociationOutputDecl> produces() const override {
        return {AssociationOutputDecl{
            AssociationId{"landmark_pose_observations"},
            PayloadDescriptor::of<LandmarkPoseObservationSet>(
                payload_names::kLandmarkPoseObservationSet)}};
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
    WorldState       world;
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

    std::unique_ptr<WorldEstimation> make(const char* xml, std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        return LandmarkWorldEstimation::create(ConfigNode{doc.RootElement()}, context,
                                               err);
    }

    WorldEstimationOutput run(WorldEstimation& we, int64_t now = 1) {
        const auto out = we.run(
            {sensorResults, artifacts, robot, world, command, target, hostTime(now)});
        world = out.world;
        return out;
    }

    void putEvidence(const char* landmark, double x, double y, double heading_rad,
                     double confidence) {
        LandmarkPoseObservation entry;
        entry.landmark        = WorldObjectId{landmark};
        entry.T_odom_landmark = Pose2D{x, y, heading_rad};
        entry.confidence      = confidence;
        script->set.entries.clear();
        script->set.entries.push_back(entry);
        script->emit = true;
    }
};

const char* kSeedOnly = R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</WorldEstimation>)";

const char* kScriptedAlways = R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="scripted_evidence"/>
        <Estimator type="landmark_estimator" commit="always"/>
    </Pipeline>
</WorldEstimation>)";

} // namespace

TEST(FieldMapResource, ParsesLandmarksAndTagInstances) {
    Fixture     f;
    std::string err;
    auto        map = f.store.require<const FieldMap>(ResourceId{"override_field"}, err);
    ASSERT_NE(map, nullptr) << err;

    const LandmarkDecl* goal = map->find(WorldObjectId{"center_goal"});
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
    const auto& goal = f.world.objects.at(WorldObjectId{"center_goal"});
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

    const auto& goal = f.world.objects.at(WorldObjectId{"center_goal"});
    EXPECT_NEAR(goal.pose.pose.x_m, 1.9, 1e-12);
    EXPECT_NEAR(goal.pose.pose.y_m, 1.75, 1e-12);
    EXPECT_TRUE(goal.observed);
    EXPECT_EQ(goal.source, EstimateSource::kObserved);
    // the accepted evidence is republished across the boundary
    EXPECT_EQ(out.associations.count(AssociationId{"landmark_pose_observations"}), 1u);

    f.script->emit = false;
    f.run(*we);
    EXPECT_FALSE(f.world.objects.at(WorldObjectId{"center_goal"}).observed);
    EXPECT_TRUE(f.world.objects.at(WorldObjectId{"center_goal"}).valid);
    EXPECT_NEAR(f.world.objects.at(WorldObjectId{"center_goal"}).pose.pose.x_m, 1.9,
                1e-12);
}

TEST(LandmarkWorld, CommitNeverPublishesEvidenceWithoutMutation) {
    Fixture     f;
    std::string err;
    auto        we = f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="scripted_evidence"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</WorldEstimation>)",
                            err);
    ASSERT_NE(we, nullptr) << err;

    f.putEvidence("center_goal", 2.4, 2.4, 0.0, 0.9);
    const auto out = f.run(*we);
    // evidence crosses the boundary, the estimate does not move
    EXPECT_EQ(out.associations.count(AssociationId{"landmark_pose_observations"}), 1u);
    EXPECT_NEAR(f.world.objects.at(WorldObjectId{"center_goal"}).pose.pose.x_m, 1.8,
                1e-12);
    EXPECT_FALSE(f.world.objects.at(WorldObjectId{"center_goal"}).observed);
}

TEST(LandmarkWorld, ChildFaultCommitsNothing) {
    Fixture     f;
    std::string err;
    auto        we = f.make(kScriptedAlways, err);
    ASSERT_NE(we, nullptr) << err;

    f.run(*we);   // seeds
    const auto seeded_x =
        f.world.objects.at(WorldObjectId{"center_goal"}).pose.pose.x_m;

    f.putEvidence("center_goal", 2.4, 2.4, 0.0, 0.9);
    f.script->fault = true;
    const auto out  = f.run(*we);
    EXPECT_EQ(out.status, FunctionStatus::kFault);
    EXPECT_TRUE(out.associations.empty());
    EXPECT_TRUE(out.observations.empty());
    EXPECT_NEAR(f.world.objects.at(WorldObjectId{"center_goal"}).pose.pose.x_m,
                seeded_x, 1e-12);
}

TEST(LandmarkWorld, BlendHalvesTheStep) {
    Fixture     f;
    std::string err;
    auto        we = f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="scripted_evidence"/>
        <Estimator type="landmark_estimator" commit="always" blend="0.5"/>
    </Pipeline>
</WorldEstimation>)",
                            err);
    ASSERT_NE(we, nullptr) << err;
    f.run(*we);
    f.putEvidence("center_goal", 2.0, 1.8, kPi / 2.0, 0.5);
    f.run(*we);
    EXPECT_NEAR(f.world.objects.at(WorldObjectId{"center_goal"}).pose.pose.x_m, 1.9,
                1e-12);
}

TEST(LandmarkWorld, StrictNestedSchema) {
    Fixture     f;
    std::string err;

    // out of range blend is an error, not a clamp
    EXPECT_EQ(f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="never" blend="1.5"/>
    </Pipeline>
</WorldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("blend"), std::string::npos);

    // unknown field map resource
    EXPECT_EQ(f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="ghost_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</WorldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("ghost_field"), std::string::npos);

    // committing with no published evidence is a contradiction
    EXPECT_EQ(f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="always"/>
    </Pipeline>
</WorldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("commit"), std::string::npos);

    // unknown estimator implementation
    EXPECT_EQ(f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Estimator type="kalman_9000" commit="never"/>
    </Pipeline>
</WorldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("kalman_9000"), std::string::npos);

    // unknown nested element, with the full configuration path
    EXPECT_EQ(f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Associaton type="noop"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</WorldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("Associaton"), std::string::npos);
    EXPECT_NE(err.find("Pipeline"), std::string::npos);

    // duplicate nested child
    EXPECT_EQ(f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
        <Association type="noop"/>
        <Estimator type="landmark_estimator" commit="never"/>
    </Pipeline>
</WorldEstimation>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("more than one Association"), std::string::npos);

    // missing estimator
    EXPECT_EQ(f.make(R"(
<WorldEstimation type="landmark_world">
    <FieldMap resource_id="override_field"/>
    <Pipeline>
        <ObservationExtraction type="noop"/>
        <Association type="noop"/>
    </Pipeline>
</WorldEstimation>)",
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
        <WorldEstimation type="landmark_world">
            <FieldMap resource_id="override_field"/>
            <Pipeline>
                <ObservationExtraction type="noop"/>
                <Association type="noop"/>
                <Estimator type="landmark_estimator" commit="never"/>
            </Pipeline>
        </WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";

    std::string err;
    auto        system = System::buildFromString(composite_system, functions, err);
    ASSERT_NE(system, nullptr) << err;
    system->step(hostTime(1));
    EXPECT_EQ(system->world().objects.count(WorldObjectId{"center_goal"}), 1u);

    const char* noop_system = R"(
<System>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Preprocessing type="noop"/>
        <Localization type="noop"/>
        <WorldEstimation type="noop"/>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
    auto plain = System::buildFromString(noop_system, functions, err);
    ASSERT_NE(plain, nullptr) << err;
    plain->step(hostTime(1));
    EXPECT_TRUE(plain->world().objects.empty());
}
