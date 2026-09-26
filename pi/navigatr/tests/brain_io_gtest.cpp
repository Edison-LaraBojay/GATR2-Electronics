// brain_io_gtest.cpp
// The brain link v3 slots. LinkHarness drives brain_link commands and
// publishing directly over one memory link with a fake link clock and
// hand-built robot and field state: sessions, dedupe, the reply window and
// reply bodies. PiRig builds a whole System with real localization for
// placement acknowledgements, target latches, pi_instance and build checks.

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common/frame_codec.h"
#include "impl/resources/serial_links.h"
#include "inspection/inspection_document.h"
#include "math/angles.h"
#include "payloads/sensor_samples.h"
#include "runtime/register_all.h"
#include "runtime/sensor_catalog.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

// ---- wire helpers ----------------------------------------------------------

std::vector<uint8_t> requestBytes(const gatr2::BrainRequest& r) {
    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeBrainRequest(r, buf.data(), gatr2::kMaxFrameLen));
    EXPECT_FALSE(buf.empty());
    return buf;
}

gatr2::BrainRequest helloRequest(uint16_t rid, uint32_t nonce) {
    gatr2::BrainRequest r;
    r.op         = gatr2::kOpHello;
    r.request_id = rid;
    r.nonce      = nonce;
    return r;
}

gatr2::BrainRequest setPoseRequest(uint32_t session, uint16_t rid, int32_t x_mm, int32_t y_mm,
                                   int32_t heading_cdeg) {
    gatr2::BrainRequest r;
    r.op           = gatr2::kOpSetPose;
    r.session      = session;
    r.request_id   = rid;
    r.x_mm         = x_mm;
    r.y_mm         = y_mm;
    r.heading_cdeg = heading_cdeg;
    return r;
}

gatr2::BrainRequest selectRequest(uint32_t session, uint16_t rid, uint8_t id, bool selected) {
    gatr2::BrainRequest r;
    r.op           = gatr2::kOpSelectLandmark;
    r.session      = session;
    r.request_id   = rid;
    r.landmark_id  = id;
    r.select_flags = selected ? gatr2::kSelectFlagSelected : 0;
    return r;
}

gatr2::BrainRequest getStateRequest(uint32_t session, uint16_t rid) {
    gatr2::BrainRequest r;
    r.op         = gatr2::kOpGetState;
    r.session    = session;
    r.request_id = rid;
    return r;
}

// Every reply frame the Pi wrote since the last call.
std::vector<gatr2::BrainReply> takeReplies(MemoryLink& link) {
    std::vector<gatr2::BrainReply> out;
    gatr2::FrameReader             reader;
    for (uint8_t b : link.output().takeAll()) {
        if (!reader.push(b)) {
            continue;
        }
        do {
            gatr2::BrainReply reply;
            EXPECT_EQ(reader.frameType(), gatr2::kFrameBrainReply);
            EXPECT_TRUE(gatr2::decodeBrainReply(reader.frame(), reader.frameLen(), reply));
            out.push_back(reply);
        } while (reader.next());
    }
    return out;
}

// ---- slot harness ----------------------------------------------------------

const char* kCommandsXml = R"(<CommandCollection type="brain_link">
    <Serial resource_id="brain_uart"/></CommandCollection>)";

const char* kPublishingXml = R"(<Publishing type="brain_link">
    <Serial resource_id="brain_uart"/>
    <Health fresh_ms="150">
        <Encoder sensor_id="enc_a"/>
        <Gyro sensor_id="imu"/>
        <BiasCal function_id="tracking_motion"/>
    </Health>
    <FieldObject object_id="center_goal" wire_id="1"/>
</Publishing>)";

struct LinkHarness {
    tinyxml2::XMLDocument     doc;
    FunctionRegistry          functions;
    std::vector<std::string>  warnings;
    ResourceStore             store;
    SensorCatalog             catalog;
    SlotInitializationContext context;
    MemoryLink*               brain    = nullptr;
    int64_t                   clock_us = 1000000;
    int64_t                   now_ms   = 1000;

    std::unique_ptr<Commands>   commands;
    std::unique_ptr<Publishing> publisher;

    SensorMap          results;
    LocalizationStatus localization;
    ObservationMap     observations;
    AssociationMap     associations;
    RobotState         robot;
    FieldState         field;
    CommandState       command;
    TargetState        target;
    Diagnostics        diagnostics;

    explicit LinkHarness(bool world_noop = false, bool idle_first = true) {
        register_resources(functions);
        register_commands(functions);
        register_publishers(functions);

        tinyxml2::XMLDocument resources_doc;
        EXPECT_EQ(resources_doc.Parse(R"(
<Resources><Resource id="brain_uart" type="memory_link"/></Resources>)"),
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

        auto link = store.require<SerialLink>(ResourceId{"brain_uart"}, err);
        brain     = dynamic_cast<MemoryLink*>(link.get());
        EXPECT_NE(brain, nullptr);
        brain->setClock([this] { return clock_us; });

        catalog.add(SensorId{"enc_a"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"imu"}, PayloadDescriptor::of<ImuSample>(payload_names::kImuSample));

        context.resources             = &store;
        context.sensors               = &catalog;
        context.functions             = &functions;
        context.observation_functions = {"tracking_motion"};
        context.commands_type         = FunctionKey{"brain_link"};
        context.commands_serial       = ResourceId{"brain_uart"};
        context.world_estimation_noop = world_noop;

        commands = make<CommandsMakeFunction>(kCommandsXml, "brain_link", err);
        EXPECT_NE(commands, nullptr) << err;
        publisher = make<PublishingMakeFunction>(kPublishingXml, "brain_link", err);
        EXPECT_NE(publisher, nullptr) << err;

        FieldObjectState goal;
        goal.valid         = true;
        goal.source        = EstimateSource::kFieldMap;
        goal.pose.pose.x_m = 1.8;
        goal.pose.pose.y_m = 1.8;
        field.objects[FieldObjectId{"center_goal"}] = goal;

        if (idle_first) {
            cycle();   // the first drain never replies
        }
    }

    template <typename MakeFunction>
    typename MakeFunction::result_type make(const char* xml, const char* key,
                                           std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        const MakeFunction* factory = functions.find<MakeFunction>(FunctionKey{key}, err);
        EXPECT_NE(factory, nullptr) << err;
        return (*factory)(ConfigNode{doc.RootElement()}, context, err);
    }

    // commands then publishing, with between() in the gap
    std::vector<gatr2::BrainReply> cycle(int64_t advance_us = 5000,
                                         const std::function<void()>& between = nullptr) {
        clock_us += advance_us;
        now_ms += advance_us / 1000;
        const CommandsOutput c = commands->run({command, hostTime(now_ms), &diagnostics});
        command                = c.command;
        if (between) {
            between();
        }
        publisher->run({results, observations, associations, robot, localization, field,
                        command, target, hostTime(now_ms), &diagnostics});
        return takeReplies(*brain);
    }

    std::vector<gatr2::BrainReply> exchange(const gatr2::BrainRequest& r) {
        brain->input().feed(requestBytes(r));
        return cycle();
    }

    gatr2::BrainReply one(const gatr2::BrainRequest& r) {
        const std::vector<gatr2::BrainReply> replies = exchange(r);
        EXPECT_EQ(replies.size(), 1u);
        return replies.empty() ? gatr2::BrainReply{} : replies.front();
    }

    uint32_t open(uint16_t rid = 1, uint32_t nonce = 0x12345678) {
        const gatr2::BrainReply reply = one(helloRequest(rid, nonce));
        EXPECT_EQ(reply.result, gatr2::kResultOk);
        EXPECT_NE(reply.session, 0u);
        return reply.session;
    }

    LinkStats& stats() { return diagnostics.links["brain_uart"]; }
};

// ---- whole Pi --------------------------------------------------------------

// Real localization over Pico wheel channels (a placement applies without
// motion), one robot-relative configured target and the brain link.
std::string piXml(const std::string& estimator = "planar_motion_integrator",
                  const std::string& rate_hz = "200",
                  const std::string& publishing = R"(<Publishing type="brain_link">
            <Serial resource_id="brain_uart"/>
            <FieldObject object_id="center_goal" wire_id="1"/>
        </Publishing>)",
                  const std::string& commands = R"(<CommandCollection type="brain_link">
            <Serial resource_id="brain_uart"/>
        </CommandCollection>)") {
    std::string localization;
    if (estimator == "noop") {
        localization = R"(<Localization><Estimator type="noop"/></Localization>)";
    } else {
        localization = R"(<Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">
                <TrackingWheel sensor_id="enc_a" label="left" radius_m="0.0254"
                               position_x_m="0" position_y_m="0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_b" label="right" radius_m="0.0254"
                               position_x_m="0" position_y_m="-0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_c" label="rear" radius_m="0.0254"
                               position_x_m="-0.12" position_y_m="0"
                               measurement_angle_deg="90" direction="positive"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
            </Estimator>
        </Localization>)";
    }
    return R"(
<System>
    <Loop rate_hz=")" +
           rate_hz + R"("/>
    <Resources>
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="encoder_c" channel="2"/>
        </Resource>
        <Resource id="brain_uart" type="memory_link"/>
        <Resource id="spare_uart" type="memory_link"/>
        <Resource id="game_field" type="field_map">
            <Landmark id="center_goal">
                <NominalPose calibration_status="verified" x_m="1.8" y_m="1.8" heading_deg="0"/>
            </Landmark>
        </Resource>
        <Resource id="robot_geometry" type="robot_frame_map"/>
        <Resource id="targets" type="target_set">
            <FieldMap resource_id="game_field"/>
            <RobotFrames resource_id="robot_geometry"/>
            <Target id="relative_move" type="robot_relative" wire_id="2"
                    controlled_frame_id="robot_body">
                <Snapshot frame_id="robot_body" delta_axes="robot_at_activation"/>
                <Delta x_m="-0.6" y_m="0.3" heading_deg="0"/>
                <VisionCorrection type="none"/>
            </Target>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_c" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_c"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        )" + commands +
           localization + R"(
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="configured_targets">
            <Targets resource_id="targets"/>
        </TargetResolution>
        )" + publishing +
           R"(
    </Pipeline>
</System>)";
}

// A System stepped inline with the brain link clock under test control:
// every step advances the link clock 5 ms and the host clock 5 ms.
struct PiRig {
    FunctionRegistry        functions;
    std::unique_ptr<System> system;
    MemoryLink*             brain    = nullptr;
    int64_t                 clock_us = 1000000;
    int64_t                 now_ms   = 0;

    explicit PiRig(const std::string& xml, bool idle_first = true) {
        registerAll(functions);
        std::string err;
        system = System::buildFromString(xml.c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system == nullptr) {
            return;
        }
        auto link = system->resources().require<SerialLink>(ResourceId{"brain_uart"}, err);
        brain     = dynamic_cast<MemoryLink*>(link.get());
        EXPECT_NE(brain, nullptr);
        brain->setClock([this] { return clock_us; });
        if (idle_first) {
            step();   // the first drain never replies
        }
    }

    std::vector<gatr2::BrainReply> step() {
        clock_us += 5000;
        now_ms += 5;
        system->step(hostTime(now_ms));
        return takeReplies(*brain);
    }

    gatr2::BrainReply one(const gatr2::BrainRequest& r) {
        brain->input().feed(requestBytes(r));
        const std::vector<gatr2::BrainReply> replies = step();
        EXPECT_EQ(replies.size(), 1u);
        return replies.empty() ? gatr2::BrainReply{} : replies.front();
    }

    uint32_t open(uint16_t rid = 1, uint32_t nonce = 0x12345678) {
        const gatr2::BrainReply reply = one(helloRequest(rid, nonce));
        EXPECT_EQ(reply.result, gatr2::kResultOk);
        return reply.session;
    }

    const LinkStats& stats() { return system->diagnostics().links["brain_uart"]; }
};

} // namespace

// ---- sessions ----------------------------------------------------------------

TEST(BrainLinkSession, HelloOpensASessionAndClearsClientState) {
    LinkHarness f;
    f.command.object_requested = true;   // left over from an earlier session
    f.command.object_wire_id   = 1;
    f.command.object_sequence  = 4;

    const gatr2::BrainReply reply = f.one(helloRequest(1, 0xCAFE));
    EXPECT_EQ(reply.version, gatr2::kBrainLinkVersion);
    EXPECT_EQ(reply.op, gatr2::kOpHello);
    EXPECT_EQ(reply.request_id, 1u);
    EXPECT_EQ(reply.result, gatr2::kResultOk);
    EXPECT_EQ(reply.nonce, 0xCAFEu);
    EXPECT_NE(reply.session, 0u);
    EXPECT_NE(reply.pi_instance, 0u);
    EXPECT_EQ(f.command.session, reply.session);
    EXPECT_FALSE(f.command.object_requested);
    EXPECT_EQ(f.command.object_wire_id, 0u);
    EXPECT_EQ(f.command.object_sequence, 5u);   // the edge releases a target latch
    EXPECT_EQ(f.command.init_sequence, 0u);     // never touches placement
}

TEST(BrainLinkSession, HelloRetryIsIdempotentUntilTheSessionIsUsed) {
    LinkHarness    f;
    const uint32_t session = f.open(1, 0xAB);
    const uint64_t edge    = f.command.object_sequence;

    const gatr2::BrainReply retry = f.one(helloRequest(1, 0xAB));
    EXPECT_EQ(retry.result, gatr2::kResultOk);
    EXPECT_EQ(retry.session, session);
    EXPECT_EQ(f.command.object_sequence, edge);   // nothing changed

    EXPECT_EQ(f.one(getStateRequest(session, 2)).result, gatr2::kResultOk);
    const gatr2::BrainReply late = f.one(helloRequest(1, 0xAB));   // used: no longer a retry
    EXPECT_EQ(late.result, gatr2::kResultStale);
    EXPECT_EQ(late.nonce, 0xABu);
    EXPECT_EQ(f.command.session, session);
}

TEST(BrainLinkSession, HelloWithARecentNonceIsStale) {
    LinkHarness    f;
    const uint32_t a = f.open(1, 0x1111);
    const uint32_t b = f.open(1, 0x2222);   // brain reboot: counters restart
    EXPECT_NE(a, b);

    const gatr2::BrainReply reply = f.one(helloRequest(7, 0x1111));
    EXPECT_EQ(reply.result, gatr2::kResultStale);
    EXPECT_EQ(f.command.session, b);
    EXPECT_EQ(f.stats().stale, 1u);

    // the ring holds four nonces: the oldest falls out
    f.open(1, 0x3333);
    f.open(1, 0x4444);
    f.open(1, 0x5555);
    EXPECT_EQ(f.one(helloRequest(2, 0x2222)).result, gatr2::kResultStale);
    EXPECT_EQ(f.one(helloRequest(2, 0x1111)).result, gatr2::kResultOk);
}

TEST(BrainLinkSession, UnknownSessionChangesNothing) {
    LinkHarness f;
    EXPECT_EQ(f.one(getStateRequest(0, 1)).result, gatr2::kResultUnknownSession);   // none open

    const uint32_t          session = f.open();
    const gatr2::BrainReply reply   = f.one(selectRequest(session + 1, 2, 1, true));
    EXPECT_EQ(reply.result, gatr2::kResultUnknownSession);
    EXPECT_EQ(reply.session, session + 1);   // echo
    EXPECT_FALSE(f.command.object_requested);
    EXPECT_EQ(f.stats().unknown_session, 2u);
}

TEST(BrainLinkSession, VersionAndOpErrors) {
    LinkHarness    f;
    const uint32_t session = f.open();

    gatr2::BrainRequest future = getStateRequest(session, 2);
    future.version             = 4;
    const gatr2::BrainReply v  = f.one(future);
    EXPECT_EQ(v.result, gatr2::kResultUnsupportedVersion);
    EXPECT_EQ(v.version, gatr2::kBrainLinkVersion);   // the Pi's version

    gatr2::BrainRequest unknown = getStateRequest(session, 3);
    unknown.op                  = 9;
    const gatr2::BrainReply op  = f.one(unknown);
    EXPECT_EQ(op.result, gatr2::kResultUnsupportedOp);
    EXPECT_EQ(op.op, 9u);

    EXPECT_EQ(f.one(getStateRequest(session, 0)).result, gatr2::kResultInvalidArgument);
}

// ---- dedupe ------------------------------------------------------------------

TEST(BrainLinkDedupe, DuplicatesAnsweredNeverReapplied) {
    LinkHarness    f;
    const uint32_t session = f.open();

    EXPECT_EQ(f.one(selectRequest(session, 2, 1, true)).result, gatr2::kResultOk);
    EXPECT_EQ(f.command.object_sequence, 2u);   // HELLO edge plus the select

    const gatr2::BrainReply again = f.one(selectRequest(session, 2, 1, true));
    EXPECT_EQ(again.result, gatr2::kResultOk);
    EXPECT_EQ(again.landmark_id, 1u);
    EXPECT_EQ(again.select_flags, gatr2::kSelectFlagSelected);
    EXPECT_EQ(f.command.object_sequence, 2u);
    EXPECT_EQ(f.stats().duplicates, 1u);

    // same id, different body: not a retry
    EXPECT_EQ(f.one(selectRequest(session, 2, 1, false)).result,
              gatr2::kResultInvalidArgument);
    EXPECT_EQ(f.one(getStateRequest(session, 2)).result, gatr2::kResultInvalidArgument);
    EXPECT_TRUE(f.command.object_requested);

    // SET_POSE retries answer from the record under the same sequence
    const gatr2::BrainReply pose = f.one(setPoseRequest(session, 3, 610, 457, 9000));
    EXPECT_EQ(pose.result, gatr2::kResultPending);   // nothing here applies placements
    EXPECT_EQ(f.command.init_sequence, 1u);
    EXPECT_EQ(f.command.init_session, session);
    EXPECT_NEAR(f.command.init_pose.x_m, 0.610, 1e-12);
    EXPECT_NEAR(f.command.init_pose.heading_rad, kPi / 2.0, 1e-9);
    EXPECT_EQ(f.one(getStateRequest(session, 4)).result, gatr2::kResultOk);
    EXPECT_EQ(f.one(setPoseRequest(session, 3, 610, 457, 9000)).result, gatr2::kResultPending);
    EXPECT_EQ(f.command.init_sequence, 1u);
    EXPECT_EQ(f.one(setPoseRequest(session, 3, 611, 457, 9000)).result,
              gatr2::kResultInvalidArgument);
}

TEST(BrainLinkDedupe, OlderIdIsStaleAndIdsWrap) {
    LinkHarness    f;
    const uint32_t session = f.open(65534);

    EXPECT_EQ(f.one(getStateRequest(session, 65535)).result, gatr2::kResultOk);
    EXPECT_EQ(f.one(getStateRequest(session, 1)).result, gatr2::kResultOk);   // wrapped
    EXPECT_EQ(f.one(selectRequest(session, 2, 1, true)).result, gatr2::kResultOk);
    EXPECT_EQ(f.one(getStateRequest(session, 65535)).result, gatr2::kResultStale);
    EXPECT_EQ(f.one(selectRequest(session, 1, 1, false)).result, gatr2::kResultStale);
    EXPECT_TRUE(f.command.object_requested);   // the stale release never applied
    EXPECT_EQ(f.one(getStateRequest(session, 2)).result, gatr2::kResultInvalidArgument);
    EXPECT_EQ(f.one(getStateRequest(session, 3)).result, gatr2::kResultOk);
    EXPECT_EQ(f.one(getStateRequest(session, 3)).result, gatr2::kResultOk);   // newest again
}

TEST(BrainLinkDedupe, DedupeIsPerSession) {
    LinkHarness    f;
    const uint32_t a = f.open(1, 0x1);
    f.one(setPoseRequest(a, 2, 100, 200, 0));
    EXPECT_EQ(f.command.init_sequence, 1u);

    // rebooted brain: same ids and body, new session, applies again
    const uint32_t b = f.open(1, 0x2);
    EXPECT_EQ(f.one(setPoseRequest(b, 2, 100, 200, 0)).result, gatr2::kResultPending);
    EXPECT_EQ(f.command.init_sequence, 2u);
    EXPECT_EQ(f.command.init_session, b);

    // a delayed session-A request cannot touch session B
    EXPECT_EQ(f.one(setPoseRequest(a, 3, 999, 999, 0)).result, gatr2::kResultUnknownSession);
    EXPECT_EQ(f.command.init_sequence, 2u);
}

// ---- bus rules -----------------------------------------------------------------

TEST(BrainLinkBus, NoOutputWithoutARequest) {
    LinkHarness f;
    for (int i = 0; i < 20; ++i) {
        EXPECT_TRUE(f.cycle().empty());
    }
    EXPECT_TRUE(f.brain->output().takeAll().empty());
}

TEST(BrainLinkBus, FirstDrainAfterStartGetsNoReplyButApplies) {
    LinkHarness f(false, false);
    f.brain->input().feed(requestBytes(helloRequest(1, 0x77)));
    EXPECT_TRUE(f.cycle().empty());
    EXPECT_NE(f.command.session, 0u);   // processed
    EXPECT_EQ(f.stats().unanswered, 1u);

    const gatr2::BrainReply retry = f.one(helloRequest(1, 0x77));   // the brain retries
    EXPECT_EQ(retry.result, gatr2::kResultOk);
    EXPECT_EQ(retry.session, f.command.session);
}

TEST(BrainLinkBus, WindowFromThePreviousDrain) {
    LinkHarness    f;
    const uint32_t session = f.open();

    // request completes 50 ms after the previous drain: too late to answer
    f.brain->input().feed(requestBytes(selectRequest(session, 2, 1, true)));
    EXPECT_TRUE(f.cycle(50000).empty());
    EXPECT_EQ(f.stats().expired, 1u);
    EXPECT_TRUE(f.command.object_requested);   // applied; the retry is a duplicate

    const gatr2::BrainReply retry = f.one(selectRequest(session, 2, 1, true));
    EXPECT_EQ(retry.result, gatr2::kResultOk);
    EXPECT_EQ(f.stats().duplicates, 1u);

    // the window the commands slot hands to the link
    f.brain->input().feed(requestBytes(getStateRequest(session, 3)));
    const int64_t previous_drain = f.clock_us;
    f.cycle();
    EXPECT_EQ(f.command.reply.window.deadline_us, previous_drain + 40000);
    EXPECT_EQ(f.command.reply.window.not_before_us, f.clock_us + 1000);
}

TEST(BrainLinkBus, TrailingBytesAndPendingInputSuppressTheReply) {
    LinkHarness    f;
    const uint32_t session = f.open();

    std::vector<uint8_t> bytes = requestBytes(getStateRequest(session, 2));
    bytes.push_back(gatr2::kSync0);   // the brain may already be transmitting
    f.brain->input().feed(bytes);
    EXPECT_TRUE(f.cycle().empty());
    EXPECT_EQ(f.stats().unanswered, 1u);

    f.cycle();   // empty drain discards the partial frame
    f.brain->input().feed(requestBytes(getStateRequest(session, 3)));
    EXPECT_TRUE(f.cycle(5000, [&] { f.brain->input().feed({0x00}); }).empty());
    EXPECT_EQ(f.stats().input_pending, 1u);
    f.cycle();
    EXPECT_EQ(f.one(getStateRequest(session, 4)).result, gatr2::kResultOk);
}

TEST(BrainLinkBus, NewestRequestWins) {
    LinkHarness    f;
    const uint32_t session = f.open();

    std::vector<uint8_t> bytes = requestBytes(selectRequest(session, 2, 1, true));
    const std::vector<uint8_t> newer = requestBytes(selectRequest(session, 3, 7, true));
    bytes.insert(bytes.end(), newer.begin(), newer.end());
    f.brain->input().feed(bytes);
    const std::vector<gatr2::BrainReply> replies = f.cycle();
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].request_id, 3u);
    EXPECT_EQ(f.command.object_wire_id, 7u);
    EXPECT_EQ(f.command.object_sequence, 2u);   // HELLO plus one select, never rid 2
    EXPECT_EQ(f.stats().superseded, 1u);
    EXPECT_EQ(f.one(selectRequest(session, 2, 1, true)).result, gatr2::kResultStale);
}

TEST(BrainLinkBus, SplitRequestAndDiscardedPartialFrame) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> bytes   = requestBytes(getStateRequest(session, 2));

    f.brain->input().feed({bytes.begin(), bytes.begin() + 5});
    EXPECT_TRUE(f.cycle().empty());
    f.brain->input().feed({bytes.begin() + 5, bytes.end()});
    const std::vector<gatr2::BrainReply> replies = f.cycle();
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].request_id, 2u);

    // a drain with no bytes drops the partial: the tail alone completes nothing
    const std::vector<uint8_t> next = requestBytes(getStateRequest(session, 3));
    f.brain->input().feed({next.begin(), next.begin() + 5});
    f.cycle();
    f.cycle();
    f.brain->input().feed({next.begin() + 5, next.end()});
    EXPECT_TRUE(f.cycle().empty());
    EXPECT_EQ(f.one(getStateRequest(session, 4)).result, gatr2::kResultOk);
}

TEST(BrainLinkBus, CorruptCrcChangesNothing) {
    LinkHarness          f;
    const uint32_t       session = f.open();
    std::vector<uint8_t> bytes   = requestBytes(setPoseRequest(session, 2, 610, 457, 9000));
    bytes[8] ^= 0x01;
    f.brain->input().feed(bytes);
    EXPECT_TRUE(f.cycle().empty());
    EXPECT_EQ(f.command.init_sequence, 0u);
    EXPECT_EQ(f.one(setPoseRequest(session, 2, 610, 457, 9000)).result, gatr2::kResultPending);
}

// ---- reply bodies ------------------------------------------------------------

TEST(BrainLinkState, SelectResultsAndLandmarkSources) {
    LinkHarness    f;
    const uint32_t session = f.open();

    EXPECT_EQ(f.one(selectRequest(session, 2, 1, true)).result, gatr2::kResultOk);
    gatr2::BrainReply s = f.one(getStateRequest(session, 3));
    EXPECT_EQ(s.state.landmark_id, 1u);
    EXPECT_EQ(s.state.landmark_source, gatr2::kLandmarkSourceNominal);
    EXPECT_EQ(s.state.lm_x_mm, 1800);
    EXPECT_EQ(s.state.lm_y_mm, 1800);
    EXPECT_EQ(s.state.landmark_age_ms, 0u);

    // unmapped id: recorded, reported with source none
    EXPECT_EQ(f.one(selectRequest(session, 4, 9, true)).result, gatr2::kResultUnknownLandmark);
    s = f.one(getStateRequest(session, 5));
    EXPECT_EQ(s.state.landmark_id, 9u);
    EXPECT_EQ(s.state.landmark_source, gatr2::kLandmarkSourceNone);

    EXPECT_EQ(f.one(selectRequest(session, 6, 9, false)).result, gatr2::kResultOk);   // release
    s = f.one(getStateRequest(session, 7));
    EXPECT_EQ(s.state.landmark_id, 0u);
    EXPECT_EQ(s.state.landmark_source, gatr2::kLandmarkSourceNone);
}

TEST(BrainLinkState, WorldEstimationNoopIsUnsupported) {
    LinkHarness    f(true);
    const uint32_t session = f.open();
    EXPECT_EQ(f.one(selectRequest(session, 2, 1, true)).result,
              gatr2::kResultLandmarkUnsupported);
    EXPECT_TRUE(f.command.object_requested);   // recorded either way
    const gatr2::BrainReply s = f.one(getStateRequest(session, 3));
    EXPECT_EQ(s.state.landmark_id, 1u);
    EXPECT_EQ(s.state.landmark_source, gatr2::kLandmarkSourceNone);
    EXPECT_EQ(f.one(selectRequest(session, 4, 1, false)).result, gatr2::kResultOk);
}

TEST(BrainLinkState, ObservedLandmarkUsesTheRobotAnchor) {
    LinkHarness    f;
    const uint32_t session = f.open();
    f.one(selectRequest(session, 2, 1, true));

    f.robot.field_from_odom = Pose2D{1.0, 0.0, kPi / 2.0};
    f.robot.odometry_epoch  = 3;
    FieldObjectState& goal  = f.field.objects[FieldObjectId{"center_goal"}];
    goal.source             = EstimateSource::kObserved;
    goal.T_odom_object      = Pose2D{0.5, 0.0, 0.0};
    goal.odometry_epoch     = 3;
    goal.lastObservedAt     = hostTime(f.now_ms + 5 - 30);   // 30 ms before the next cycle

    gatr2::BrainReply s = f.one(getStateRequest(session, 3));
    EXPECT_EQ(s.state.landmark_source, gatr2::kLandmarkSourceObserved);
    EXPECT_EQ(s.state.lm_x_mm, 1000);   // physical landmark, field frame
    EXPECT_EQ(s.state.lm_y_mm, 500);
    EXPECT_EQ(s.state.lm_heading_cdeg, 9000);
    EXPECT_EQ(s.state.landmark_age_ms, 30u);

    goal.odometry_epoch = 2;   // measured in another odometry frame
    s                   = f.one(getStateRequest(session, 4));
    EXPECT_EQ(s.state.landmark_source, gatr2::kLandmarkSourceNone);
    EXPECT_EQ(s.state.lm_x_mm, 0);
}

TEST(BrainLinkState, RobotUnitsAgeAnchorBitsAndHealth) {
    LinkHarness    f;
    const uint32_t session = f.open();

    f.robot.odom_pose       = Pose2D{1.5004, -0.2502, kPi / 2.0};
    f.robot.valid           = true;
    f.robot.initialized     = true;
    f.robot.odometry_epoch  = (uint64_t{1} << 32) + 5;   // low 32 bits on the wire
    f.robot.anchor_revision = 2;
    f.results[SensorId{"enc_a"}].state = SourceState::kValid;
    f.results[SensorId{"imu"}].state   = SourceState::kValid;
    StoredSample fresh;
    fresh.receivedAt                    = hostTime(f.now_ms);
    f.results[SensorId{"enc_a"}].latest = fresh;
    f.results[SensorId{"imu"}].latest   = fresh;
    f.localization.functions = {
        ObservationFunctionStatus{"tracking_motion", "tracking_wheel_motion", true, ""}};

    gatr2::BrainReply s = f.one(getStateRequest(session, 2));
    EXPECT_EQ(s.state.x_mm, 1500);
    EXPECT_EQ(s.state.y_mm, -250);
    EXPECT_EQ(s.state.heading_cdeg, 9000);
    EXPECT_EQ(s.state.odometry_epoch, 5u);
    EXPECT_EQ(s.state.anchor_revision, 2u);
    EXPECT_EQ(s.state.robot_flags, gatr2::kRobotPoseValid | gatr2::kRobotLocalized);
    EXPECT_EQ(s.state.robot_age_ms, 0u);   // age unknown without a host time
    EXPECT_EQ(s.state.health, gatr2::kHealthEncodersFresh | gatr2::kHealthGyroFresh |
                                  gatr2::kHealthBiasCalibrated);

    f.robot.measuredAtHost   = hostTime(f.now_ms + 5 - 20);
    f.robot.placement_origin = "command";
    s                        = f.one(getStateRequest(session, 3));
    EXPECT_TRUE(s.state.robot_flags & gatr2::kRobotAgeKnown);
    EXPECT_TRUE(s.state.robot_flags & gatr2::kRobotAnchorCommand);
    EXPECT_FALSE(s.state.robot_flags & gatr2::kRobotAnchorConfigured);
    EXPECT_EQ(s.state.robot_age_ms, 20u);

    f.robot.measuredAtHost   = hostTime(f.now_ms - 100000);
    f.robot.placement_origin = "configuration";
    f.results[SensorId{"enc_a"}].latest->receivedAt = hostTime(f.now_ms - 1000);
    s = f.one(getStateRequest(session, 4));
    EXPECT_EQ(s.state.robot_age_ms, 65535u);   // clamped
    EXPECT_FALSE(s.state.robot_flags & gatr2::kRobotAnchorCommand);
    EXPECT_TRUE(s.state.robot_flags & gatr2::kRobotAnchorConfigured);
    EXPECT_FALSE(s.state.health & gatr2::kHealthEncodersFresh);   // stale encoder
}

TEST(BrainLinkState, WireHeadingStaysInsideItsRange) {
    // (-18000, 18000]: -180 deg and anything rounding to it is sent as +180
    EXPECT_EQ(radToCdeg(kPi), 18000);
    EXPECT_EQ(radToCdeg(-kPi), 18000);
    EXPECT_EQ(radToCdeg(-kPi + 1e-5), 18000);
    EXPECT_EQ(radToCdeg(-kPi + 1e-3), -17994);
    EXPECT_EQ(radToCdeg(0.0), 0);
    EXPECT_EQ(radToCdeg(-kPi / 2.0), -9000);
}

TEST(BrainLinkState, SetPoseOkOnlyForTheAppliedPlacement) {
    LinkHarness    f;
    const uint32_t session = f.open();
    f.one(setPoseRequest(session, 2, 610, 457, 9000));

    f.robot.placement_origin   = "command";
    f.robot.placement_session  = session;
    f.robot.placement_sequence = f.command.init_sequence;
    f.robot.anchor_revision    = 4;
    const gatr2::BrainReply ok = f.one(setPoseRequest(session, 2, 610, 457, 9000));
    EXPECT_EQ(ok.result, gatr2::kResultOk);
    EXPECT_EQ(ok.anchor_revision, 4u);

    f.robot.placement_session = session + 1;   // same sequence, another session
    EXPECT_EQ(f.one(setPoseRequest(session, 2, 610, 457, 9000)).result, gatr2::kResultPending);
}

TEST(BrainLinkConfig, PublishingPairingAndMappingErrors) {
    LinkHarness f;
    std::string err;
    f.context.commands_type = FunctionKey{"noop"};
    EXPECT_EQ(f.make<PublishingMakeFunction>(kPublishingXml, "brain_link", err), nullptr);
    EXPECT_NE(err.find("brain_link commands"), std::string::npos) << err;

    f.context.commands_type   = FunctionKey{"brain_link"};
    f.context.commands_serial = ResourceId{"other_uart"};
    EXPECT_EQ(f.make<PublishingMakeFunction>(kPublishingXml, "brain_link", err), nullptr);

    f.context.commands_serial = ResourceId{"brain_uart"};
    EXPECT_EQ(f.make<PublishingMakeFunction>(R"(<Publishing type="brain_link">
                    <Serial resource_id="brain_uart"/>
                    <FieldObject object_id="a" wire_id="1"/>
                    <FieldObject object_id="b" wire_id="1"/></Publishing>)",
                                             "brain_link", err),
              nullptr);
    EXPECT_NE(err.find("duplicate"), std::string::npos);
    EXPECT_EQ(f.make<PublishingMakeFunction>(R"(<Publishing type="brain_link">
                    <Serial resource_id="brain_uart"/>
                    <FieldObject object_id="a" wire_id="0"/></Publishing>)",
                                             "brain_link", err),
              nullptr);
    EXPECT_EQ(f.make<PublishingMakeFunction>(R"(<Publishing type="brain_link">
                    <Serial resource_id="brain_uart"/>
                    <Health><BiasCal function_id="ghost_model"/></Health></Publishing>)",
                                             "brain_link", err),
              nullptr);
    EXPECT_NE(err.find("ghost_model"), std::string::npos);
    EXPECT_EQ(f.make<CommandsMakeFunction>(R"(<CommandCollection type="brain_link">
                    <Serial resource_id="brain_uart"/>
                    <Reply window_ms="40" turnaround_guard_us="40000"/></CommandCollection>)",
                                           "brain_link", err),
              nullptr);
}

// ---- whole Pi ----------------------------------------------------------------

TEST(BrainLinkSystem, SetPoseAppliesOnceAndIsAcknowledgedWhenApplied) {
    PiRig rig(piXml());
    ASSERT_NE(rig.system, nullptr);
    const uint32_t session = rig.open();

    const gatr2::BrainReply ok = rig.one(setPoseRequest(session, 2, 610, 457, 9000));
    EXPECT_EQ(ok.result, gatr2::kResultOk);   // localization applied it this cycle
    EXPECT_EQ(ok.anchor_revision, 1u);
    const RobotState& robot = rig.system->robot();
    EXPECT_EQ(robot.anchor_revision, 1u);
    EXPECT_EQ(robot.placement_origin, "command");
    EXPECT_EQ(robot.placement_session, session);
    EXPECT_NEAR(robot.fieldPose().x_m, 0.610, 1e-9);

    // SET_POSE, GET_STATE, then the SET_POSE retry: applied once
    const gatr2::BrainReply s = rig.one(getStateRequest(session, 3));
    EXPECT_EQ(s.state.anchor_revision, 1u);
    EXPECT_TRUE(s.state.robot_flags & gatr2::kRobotAnchorCommand);
    EXPECT_EQ(rig.one(setPoseRequest(session, 2, 610, 457, 9000)).result, gatr2::kResultOk);
    EXPECT_EQ(rig.system->robot().anchor_revision, 1u);
    EXPECT_EQ(rig.system->command().init_sequence, 1u);
}

TEST(BrainLinkSystem, PendingUntilLocalizationApplies) {
    PiRig rig(piXml("noop"));   // an estimator that never places the robot
    ASSERT_NE(rig.system, nullptr);
    const uint32_t session = rig.open();
    EXPECT_EQ(rig.one(setPoseRequest(session, 2, 610, 457, 9000)).result,
              gatr2::kResultPending);
    EXPECT_EQ(rig.one(setPoseRequest(session, 2, 610, 457, 9000)).result,
              gatr2::kResultPending);
    EXPECT_EQ(rig.system->command().init_sequence, 1u);
}

TEST(BrainLinkSystem, NewSessionKeepsTheAnchorAndReplacementApplies) {
    PiRig rig(piXml());
    ASSERT_NE(rig.system, nullptr);
    const uint32_t a = rig.open(1, 0xA);
    rig.one(setPoseRequest(a, 2, 610, 457, 9000));
    const Pose2D placed = rig.system->robot().fieldPose();

    // brain reboot: HELLO alone never relocates the robot
    const uint32_t b = rig.open(1, 0xB);
    EXPECT_NE(a, b);
    EXPECT_EQ(rig.system->robot().anchor_revision, 1u);
    EXPECT_NEAR(rig.system->robot().fieldPose().x_m, placed.x_m, 1e-12);
    EXPECT_EQ(rig.system->robot().placement_session, a);

    // a delayed session-A placement is rejected
    EXPECT_EQ(rig.one(setPoseRequest(a, 3, 0, 0, 0)).result, gatr2::kResultUnknownSession);
    EXPECT_EQ(rig.system->robot().anchor_revision, 1u);

    // the same rid and pose from the new boot is a genuine new placement
    EXPECT_EQ(rig.one(setPoseRequest(b, 2, 610, 457, 9000)).result, gatr2::kResultOk);
    EXPECT_EQ(rig.system->robot().anchor_revision, 2u);
    EXPECT_EQ(rig.system->robot().placement_session, b);
    EXPECT_EQ(rig.system->robot().placement_sequence, 2u);
}

TEST(BrainLinkSystem, NewSessionReleasesTheTargetLatch) {
    PiRig rig(piXml());
    ASSERT_NE(rig.system, nullptr);
    const uint32_t a = rig.open(1, 0xA);
    rig.one(setPoseRequest(a, 2, 610, 457, 9000));   // a valid robot for activation

    // world estimation is noop: unsupported for the wire, still recorded
    EXPECT_EQ(rig.one(selectRequest(a, 3, 2, true)).result, gatr2::kResultLandmarkUnsupported);
    EXPECT_TRUE(rig.system->target().active);
    EXPECT_TRUE(rig.system->target().latched);

    rig.open(1, 0xB);
    EXPECT_FALSE(rig.system->target().active);
    EXPECT_FALSE(rig.system->command().object_requested);
}

TEST(BrainLinkSystem, NoopWorldEstimationReportsNoLandmark) {
    PiRig rig(piXml());
    ASSERT_NE(rig.system, nullptr);
    const uint32_t session = rig.open();
    EXPECT_EQ(rig.one(selectRequest(session, 2, 1, true)).result,
              gatr2::kResultLandmarkUnsupported);
    const gatr2::BrainReply s = rig.one(getStateRequest(session, 3));
    EXPECT_EQ(s.state.landmark_id, 1u);
    EXPECT_EQ(s.state.landmark_source, gatr2::kLandmarkSourceNone);
}

TEST(BrainLinkSystem, PiInstanceIsNewPerSystemAndReset) {
    PiRig first(piXml());
    PiRig second(piXml());
    ASSERT_NE(first.system, nullptr);
    ASSERT_NE(second.system, nullptr);
    const gatr2::BrainReply a = first.one(helloRequest(1, 0x1));
    const gatr2::BrainReply b = second.one(helloRequest(1, 0x1));
    EXPECT_NE(a.pi_instance, 0u);
    EXPECT_NE(a.pi_instance, b.pi_instance);

    first.system->reset();
    EXPECT_TRUE(first.step().empty());   // first drain after reset: no reply
    const gatr2::BrainReply old = first.one(getStateRequest(a.session, 2));
    EXPECT_EQ(old.result, gatr2::kResultUnknownSession);
    EXPECT_NE(old.pi_instance, a.pi_instance);
    EXPECT_EQ(first.system->command().init_sequence, 0u);
}

TEST(BrainLinkSystem, PairingAndLoopRateBuildErrors) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;

    EXPECT_EQ(System::buildFromString(
                  piXml("planar_motion_integrator", "200", R"(<Publishing type="noop"/>)")
                      .c_str(),
                  functions, err),
              nullptr);
    EXPECT_NE(err.find("brain_link Publishing"), std::string::npos) << err;

    EXPECT_EQ(System::buildFromString(
                  piXml("planar_motion_integrator", "200", R"(<Publishing type="brain_link">
                          <Serial resource_id="brain_uart"/></Publishing>)",
                        R"(<CommandCollection type="noop"/>)")
                      .c_str(),
                  functions, err),
              nullptr);
    EXPECT_NE(err.find("brain_link commands"), std::string::npos) << err;

    EXPECT_EQ(System::buildFromString(
                  piXml("planar_motion_integrator", "200", R"(<Publishing type="brain_link">
                          <Serial resource_id="spare_uart"/></Publishing>)")
                      .c_str(),
                  functions, err),
              nullptr);
    EXPECT_NE(err.find("same Serial resource"), std::string::npos) << err;

    // 25 ms loop period against a 40 ms window
    EXPECT_EQ(System::buildFromString(piXml("planar_motion_integrator", "40").c_str(),
                                      functions, err),
              nullptr);
    EXPECT_NE(err.find("half the reply window"), std::string::npos) << err;
    EXPECT_NE(System::buildFromString(piXml("planar_motion_integrator", "50").c_str(),
                                      functions, err),
              nullptr)
        << err;
}

TEST(BrainLinkSystem, InspectionShowsTheSessionAndLinkCounters) {
    PiRig rig(piXml());
    ASSERT_NE(rig.system, nullptr);
    const uint32_t session = rig.open();
    rig.one(getStateRequest(session, 2));

    InspectionServiceStats service;
    const std::string      snap = snapshotDocument(*rig.system, service, hostTime(rig.now_ms));
    EXPECT_NE(snap.find("\"command\":{\"session\":" + std::to_string(session) +
                        ",\"init_sequence\":0,\"init_session\":0,"),
              std::string::npos)
        << snap;
    EXPECT_NE(snap.find("\"id\":\"brain_uart\",\"bytes\":"), std::string::npos);
    EXPECT_NE(snap.find("\"requests\":2,\"duplicates\":0,"), std::string::npos) << snap;
    EXPECT_NE(snap.find("\"unanswered\":0,\"replies\":2,\"expired\":0,"), std::string::npos);
}
