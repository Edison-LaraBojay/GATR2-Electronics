// brain_io_gtest.cpp
// The brain-facing wire implementations: command decoding into command state
// and pose publishing with publisher-owned wire object mapping and health.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "common/frame_codec.h"
#include "impl/commands/vex_brain_serial.h"
#include "impl/publishing/vex_brain.h"
#include "impl/resources/serial_links.h"
#include "math/angles.h"
#include "payloads/landmark_associations.h"
#include "payloads/sensor_samples.h"
#include "runtime/register_all.h"
#include "runtime/sensor_catalog.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

std::vector<uint8_t> commandBytes(const gatr2::CommandFrame& c) {
    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeCommandFrame(c, buf.data(), gatr2::kMaxFrameLen));
    return buf;
}

struct Fixture {
    tinyxml2::XMLDocument     doc;
    FunctionRegistry          functions;
    std::vector<std::string>  warnings;
    ResourceStore             store;
    SensorCatalog             catalog;
    SlotInitializationContext context;
    MemoryLink*               brain = nullptr;

    SensorMap          results;
    LocalizationStatus localization;
    ObservationMap     observations;
    AssociationMap   associations;
    RobotState       robot;
    FieldState       field;
    CommandState     command;
    TargetState      target;

    Fixture() {
        register_resources(functions);

        tinyxml2::XMLDocument resources_doc;
        EXPECT_EQ(resources_doc.Parse(R"(
<Resources><Resource id="brain_uart" type="memory_link"/></Resources>)"),
                  tinyxml2::XML_SUCCESS);
        ResourceStoreBuilder builder(functions, &warnings);
        std::string          err;
        bool                 ok = true;
        ConfigNode{resources_doc.RootElement()}.forEach("Resource",
                                                        [&](const ConfigNode& r) {
                                                            if (ok) {
                                                                ok = builder.index(r, err);
                                                            }
                                                        });
        EXPECT_TRUE(ok && builder.buildAll(err)) << err;
        store = builder.take();

        auto link = store.require<SerialLink>(ResourceId{"brain_uart"}, err);
        brain     = dynamic_cast<MemoryLink*>(link.get());
        EXPECT_NE(brain, nullptr);

        catalog.add(SensorId{"enc_a"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"imu"},
                    PayloadDescriptor::of<ImuSample>(payload_names::kImuSample));

        context.resources = &store;
        context.sensors   = &catalog;
        context.functions = &functions;
        context.observation_functions = {"tracking_motion"};
        context.associations = {AssociationOutputDecl{
            AssociationId{"landmarks"},
            PayloadDescriptor::of<LandmarkAssociationSet>(
                payload_names::kLandmarkAssociationSet)}};
    }

    ConfigNode parse(const char* xml) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        return ConfigNode{doc.RootElement()};
    }

    void putFreshSensor(const char* id, int64_t received_ms) {
        MeasurementRecord record;
        record.state = SourceState::kValid;
        StoredSample stored;
        stored.receivedAt = hostTime(received_ms);
        stored.sequence   = 1;
        record.latest     = std::move(stored);
        results[SensorId{id}] = std::move(record);
    }

    PublishingInput publishingInput(int64_t now_ms = 100) {
        return PublishingInput{results, observations, associations, robot, localization,
                               field,   command,      target,       hostTime(now_ms)};
    }

    gatr2::PoseFrame decode() {
        const std::vector<uint8_t> bytes = brain->output().takeAll();
        gatr2::PoseFrame           p{};
        EXPECT_TRUE(
            gatr2::decodePoseFrame(bytes.data(), static_cast<uint16_t>(bytes.size()), p));
        return p;
    }
};

} // namespace

TEST(BrainCommands, AppliesAndDeduplicates) {
    Fixture     f;
    std::string err;
    auto        commands = VexBrainSerialCommands::create(
        f.parse(R"(<CommandCollection type="vex_brain_serial">
                    <Serial resource_id="brain_uart"/></CommandCollection>)"),
        f.context, err);
    ASSERT_NE(commands, nullptr) << err;

    gatr2::CommandFrame init{};
    init.seq          = 9;
    init.command      = gatr2::kCmdInitPose;
    init.x_mm         = 610;
    init.y_mm         = 457;
    init.heading_cdeg = 9000;
    f.brain->input().feed(commandBytes(init));

    CommandState state;
    CommandsOutput out = commands->run({state, hostTime(1), nullptr});
    state              = out.command;
    EXPECT_EQ(state.init_sequence, 1u);
    EXPECT_NEAR(state.init_pose.x_m, 0.610, 1e-12);   // wire mm becomes meters
    EXPECT_NEAR(state.init_pose.heading_rad, kPi / 2.0, 1e-9);

    f.brain->input().feed(commandBytes(init));   // resend, same wire seq
    out = commands->run({state, hostTime(2), nullptr});
    EXPECT_EQ(out.command.init_sequence, 1u);

    init.seq = 10;   // genuinely new command
    f.brain->input().feed(commandBytes(init));
    out = commands->run({out.command, hostTime(3), nullptr});
    EXPECT_EQ(out.command.init_sequence, 2u);
}

TEST(BrainCommands, NoUpdatePersistsState) {
    Fixture     f;
    std::string err;
    auto        commands = VexBrainSerialCommands::create(
        f.parse(R"(<CommandCollection type="vex_brain_serial">
                    <Serial resource_id="brain_uart"/></CommandCollection>)"),
        f.context, err);
    ASSERT_NE(commands, nullptr) << err;

    CommandState state;
    state.object_requested = true;
    state.object_wire_id   = 7;
    state.stream_on        = false;

    const CommandsOutput out = commands->run({state, hostTime(1), nullptr});
    EXPECT_TRUE(out.command.object_requested);
    EXPECT_EQ(out.command.object_wire_id, 7);
    EXPECT_FALSE(out.command.stream_on);   // no update is not a default command
}

TEST(BrainPublisher, WireMappingHealthAndUnits) {
    Fixture     f;
    std::string err;
    auto        publisher = VexBrainPublisher::create(
        f.parse(R"(<Publishing type="vex_brain">
                    <Serial resource_id="brain_uart"/>
                    <Health fresh_ms="150">
                        <Encoder sensor_id="enc_a"/>
                        <Gyro sensor_id="imu"/>
                        <BiasCal function_id="tracking_motion"/>
                    </Health>
                    <FieldObject object_id="center_goal" wire_id="1"/>
                   </Publishing>)"),
        f.context, err);
    ASSERT_NE(publisher, nullptr) << err;

    f.robot.odom_pose.x_m        = 1.5004;
    f.robot.odom_pose.y_m        = -0.2502;
    f.robot.odom_pose.heading_rad = kPi / 2.0;
    f.robot.valid            = true;
    f.robot.initialized      = true;
    f.robot.measuredAt       = deviceTime(5000);
    f.putFreshSensor("enc_a", 90);
    f.putFreshSensor("imu", 95);
    f.localization.functions = {
        ObservationFunctionStatus{"tracking_motion", "tracking_wheel_motion", true, ""}};

    f.command.object_requested = true;
    f.command.object_wire_id   = 1;
    FieldObjectState goal;
    goal.valid          = true;
    goal.pose.pose.x_m  = 1.8;
    goal.pose.pose.y_m  = 1.8;
    f.field.objects[FieldObjectId{"center_goal"}] = goal;

    EXPECT_EQ(publisher->run(f.publishingInput()).status, FunctionStatus::kOk);
    const gatr2::PoseFrame p = f.decode();

    EXPECT_EQ(p.stamp_ms, 5000u);
    EXPECT_EQ(p.x_mm, 1500);   // meters become wire millimeters
    EXPECT_EQ(p.y_mm, -250);
    EXPECT_EQ(p.heading_cdeg, 9000);
    EXPECT_TRUE(p.status & gatr2::kStatusPoseValid);
    EXPECT_TRUE(p.status & gatr2::kStatusLocInit);
    EXPECT_TRUE(p.status & gatr2::kStatusEncHealthy);
    EXPECT_TRUE(p.status & gatr2::kStatusGyroHealthy);
    EXPECT_TRUE(p.status & gatr2::kStatusBiasCal);
    EXPECT_TRUE(p.status & gatr2::kStatusObjRequested);
    EXPECT_TRUE(p.status & gatr2::kStatusObjValid);
    EXPECT_EQ(p.object_id, 1);
    EXPECT_EQ(p.obj_x_mm, 1800);
}

TEST(BrainPublisher, UnmappedWireIdStaysInvalid) {
    Fixture     f;
    std::string err;
    auto        publisher = VexBrainPublisher::create(
        f.parse(R"(<Publishing type="vex_brain">
                    <Serial resource_id="brain_uart"/>
                   </Publishing>)"),
        f.context, err);
    ASSERT_NE(publisher, nullptr) << err;

    f.command.object_requested = true;
    f.command.object_wire_id   = 99;   // no FieldObject mapping configured
    publisher->run(f.publishingInput());
    const gatr2::PoseFrame p = f.decode();
    EXPECT_TRUE(p.status & gatr2::kStatusObjRequested);
    EXPECT_FALSE(p.status & gatr2::kStatusObjValid);
    EXPECT_EQ(p.object_id, 99);
}

TEST(BrainPublisher, LandmarkEntriesMapAndClamp) {
    Fixture     f;
    std::string err;
    auto        publisher = VexBrainPublisher::create(
        f.parse(R"(<Publishing type="vex_brain">
                    <Serial resource_id="brain_uart"/>
                    <FieldObject object_id="center_goal" wire_id="9"/>
                    <Landmarks association_id="landmarks"/>
                   </Publishing>)"),
        f.context, err);
    ASSERT_NE(publisher, nullptr) << err;

    LandmarkAssociationSet set;
    LandmarkAssociationEntry seen;
    seen.landmark    = FieldObjectId{"center_goal"};
    seen.dx_m        = 0.1234;
    seen.dy_m        = -99.0;   // beyond int16 mm on the wire
    seen.bearing_rad = 0.5;
    seen.quality     = 0.5;
    set.entries.push_back(seen);

    LandmarkAssociationEntry unmapped;
    unmapped.landmark = FieldObjectId{"mystery"};   // no wire id: not publishable
    unmapped.quality  = 0.9;
    set.entries.push_back(unmapped);

    AssociationRecord record;
    record.payload = TypedPayload::store(std::move(set),
                                         payload_names::kLandmarkAssociationSet);
    f.associations[AssociationId{"landmarks"}] = std::move(record);

    publisher->run(f.publishingInput());
    const gatr2::PoseFrame p = f.decode();
    ASSERT_EQ(p.n_landmarks, 1);
    EXPECT_EQ(p.landmarks[0].id, 9);
    EXPECT_EQ(p.landmarks[0].dx_mm, 123);
    EXPECT_EQ(p.landmarks[0].dy_mm, -32768);   // clamped
    EXPECT_EQ(p.landmarks[0].bearing_cdeg, 2865);
    EXPECT_EQ(p.landmarks[0].quality, 128);
}

TEST(BrainPublisher, StreamOffSendsNothingAndConfigErrors) {
    Fixture     f;
    std::string err;
    auto        publisher = VexBrainPublisher::create(
        f.parse(R"(<Publishing type="vex_brain">
                    <Serial resource_id="brain_uart"/>
                   </Publishing>)"),
        f.context, err);
    ASSERT_NE(publisher, nullptr) << err;

    f.command.stream_on = false;
    EXPECT_EQ(publisher->run(f.publishingInput()).status, FunctionStatus::kOk);
    EXPECT_TRUE(f.brain->output().takeAll().empty());

    // duplicate wire mapping is a configuration error
    EXPECT_EQ(VexBrainPublisher::create(
                  f.parse(R"(<Publishing type="vex_brain">
                    <Serial resource_id="brain_uart"/>
                    <FieldObject object_id="a" wire_id="1"/>
                    <FieldObject object_id="b" wire_id="1"/>
                   </Publishing>)"),
                  f.context, err),
              nullptr);
    EXPECT_NE(err.find("duplicate"), std::string::npos);

    // unknown localization function behind the bias flag
    EXPECT_EQ(VexBrainPublisher::create(
                  f.parse(R"(<Publishing type="vex_brain">
                    <Serial resource_id="brain_uart"/>
                    <Health><BiasCal function_id="ghost_model"/></Health>
                   </Publishing>)"),
                  f.context, err),
              nullptr);
    EXPECT_NE(err.find("ghost_model"), std::string::npos);

    // unknown health sensor reference
    EXPECT_EQ(VexBrainPublisher::create(
                  f.parse(R"(<Publishing type="vex_brain">
                    <Serial resource_id="brain_uart"/>
                    <Health><Encoder sensor_id="ghost"/></Health>
                   </Publishing>)"),
                  f.context, err),
              nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);
}
