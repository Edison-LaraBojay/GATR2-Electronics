// world_prediction_gtest.cpp
// Field map resource parsing and the landmark map world prediction.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "impl/world_prediction/landmark_map.h"
#include "math/angles.h"
#include "payloads/landmark_associations.h"
#include "resources/resource_store.h"
#include "runtime/register_all.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

struct Fixture {
    tinyxml2::XMLDocument     doc;
    FunctionRegistry          functions;
    std::vector<std::string>  warnings;
    ResourceStore             store;
    SlotInitializationContext context;

    ObservationMap observations;
    AssociationMap associations;
    RobotState     robot;
    WorldState     world;

    Fixture() {
        register_resources(functions);

        tinyxml2::XMLDocument resources_doc;
        EXPECT_EQ(resources_doc.Parse(R"(
<Resources>
    <Resource id="override_field" type="resource/field_map">
        <Landmark id="center_goal">
            <NominalPose x_m="1.8" y_m="1.8" heading_deg="90"/>
            <Tag instance="goal_front" family="tag36h11" observed_id="7"
                 x_m="0.15" y_m="0" heading_deg="180"/>
            <Tag instance="goal_left" family="tag36h11" observed_id="7"
                 x_m="0" y_m="0.15" heading_deg="90"/>
        </Landmark>
    </Resource>
</Resources>)"),
                  tinyxml2::XML_SUCCESS);
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

        context.resources    = &store;
        context.functions    = &functions;
        context.associations = {AssociationOutputDecl{
            AssociationId{"landmarks"},
            PayloadDescriptor::of<LandmarkAssociationSet>(
                payload_names::kLandmarkAssociationSet)}};
    }

    std::unique_ptr<WorldPrediction> make(const char* xml, std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        return LandmarkMapWorldPrediction::create(ConfigNode{doc.RootElement()}, context,
                                                  err);
    }

    WorldPredictionOutput run(WorldPrediction& wp, int64_t now = 1) {
        const WorldPredictionOutput out =
            wp.run({observations, associations, robot, world, hostTime(now)});
        world = out.world;
        return out;
    }

    void putAssociation(const char* landmark, double field_x, double field_y,
                        double quality) {
        LandmarkAssociationSet set;
        LandmarkAssociationEntry entry;
        entry.landmark  = WorldObjectId{landmark};
        entry.field_x_m = field_x;
        entry.field_y_m = field_y;
        entry.quality   = quality;
        set.entries.push_back(entry);

        AssociationRecord record;
        record.payload = TypedPayload::store(std::move(set),
                                             payload_names::kLandmarkAssociationSet);
        associations[AssociationId{"landmarks"}] = std::move(record);
    }
};

const char* kMapOnly = R"(
<WorldPrediction type="world_prediction/landmark_map">
    <FieldMap resource_id="override_field"/>
</WorldPrediction>)";

const char* kWithAssociations = R"(
<WorldPrediction type="world_prediction/landmark_map">
    <FieldMap resource_id="override_field"/>
    <Input association_id="landmarks"/>
</WorldPrediction>)";

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

    // two physical instances share one printed id; identity is the instance
    ASSERT_EQ(goal->tags.size(), 2u);
    EXPECT_EQ(goal->tags[0].instance, "goal_front");
    EXPECT_EQ(goal->tags[1].instance, "goal_left");
    EXPECT_EQ(goal->tags[0].observed_id, 7);
    EXPECT_EQ(goal->tags[1].observed_id, 7);
    EXPECT_NEAR(goal->tags[0].mount.x_m, 0.15, 1e-12);
}

TEST(LandmarkMap, SeedsFromMapAndKeepsUnseenValid) {
    Fixture     f;
    std::string err;
    auto        wp = f.make(kMapOnly, err);
    ASSERT_NE(wp, nullptr) << err;

    f.run(*wp);
    const WorldObject& goal = f.world.objects.at(WorldObjectId{"center_goal"});
    EXPECT_TRUE(goal.valid);
    EXPECT_FALSE(goal.observed);
    EXPECT_EQ(goal.source, EstimateSource::kFieldMap);
    EXPECT_EQ(goal.pose.frame.value, "field");
    EXPECT_NEAR(goal.pose.pose.x_m, 1.8, 1e-12);
}

TEST(LandmarkMap, AssociationsMoveObjectsKeepHeading) {
    Fixture     f;
    std::string err;
    auto        wp = f.make(kWithAssociations, err);
    ASSERT_NE(wp, nullptr) << err;

    f.run(*wp);
    f.putAssociation("center_goal", 1.9, 1.75, 0.9);
    f.run(*wp);

    const WorldObject& goal = f.world.objects.at(WorldObjectId{"center_goal"});
    EXPECT_NEAR(goal.pose.pose.x_m, 1.9, 1e-12);
    EXPECT_NEAR(goal.pose.pose.y_m, 1.75, 1e-12);
    EXPECT_NEAR(goal.pose.pose.heading_rad, kPi / 2.0, 1e-12);   // bearing only
    EXPECT_TRUE(goal.observed);
    EXPECT_EQ(goal.source, EstimateSource::kObserved);

    f.associations.clear();
    f.run(*wp);
    EXPECT_FALSE(f.world.objects.at(WorldObjectId{"center_goal"}).observed);
    EXPECT_TRUE(f.world.objects.at(WorldObjectId{"center_goal"}).valid);
    EXPECT_NEAR(f.world.objects.at(WorldObjectId{"center_goal"}).pose.pose.x_m, 1.9,
                1e-12);
}

TEST(LandmarkMap, BlendAndConfigErrors) {
    Fixture     f;
    std::string err;

    auto wp = f.make(R"(
<WorldPrediction type="world_prediction/landmark_map" blend="0.5">
    <FieldMap resource_id="override_field"/>
    <Input association_id="landmarks"/>
</WorldPrediction>)",
                     err);
    ASSERT_NE(wp, nullptr) << err;
    f.run(*wp);
    f.putAssociation("center_goal", 2.0, 1.8, 0.5);
    f.run(*wp);
    EXPECT_NEAR(f.world.objects.at(WorldObjectId{"center_goal"}).pose.pose.x_m, 1.9,
                1e-12);

    // out of range blend is an error, not a clamp
    EXPECT_EQ(f.make(R"(
<WorldPrediction type="world_prediction/landmark_map" blend="1.5">
    <FieldMap resource_id="override_field"/>
</WorldPrediction>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("blend"), std::string::npos);

    // unknown field map resource
    EXPECT_EQ(f.make(R"(
<WorldPrediction type="world_prediction/landmark_map">
    <FieldMap resource_id="ghost_field"/>
</WorldPrediction>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("ghost_field"), std::string::npos);

    // unknown association reference
    EXPECT_EQ(f.make(R"(
<WorldPrediction type="world_prediction/landmark_map">
    <FieldMap resource_id="override_field"/>
    <Input association_id="ghost"/>
</WorldPrediction>)",
                     err),
              nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);
}
