// brain_io_gtest.cpp
// The brain link v4 slots. LinkHarness drives brain_link commands and
// publishing directly over one memory link with a fake link clock and
// hand-built robot and command state: sessions, dedupe, the reply window,
// reply bodies, profile staging and apply, path reports. PiRig builds a
// whole System with real localization for placement acknowledgements,
// pi_instance and build checks.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "translaGATR/frame_codec.h"
#include "translaGATR/link_documents.h"
#include "contracts/brain_profile.h"
#include "diagnostics/hub.h"
#include "diagnostics/instrumentation.h"
#include "impl/resources/serial_links.h"
#include "inspection/json_writer.h"
#include "resources/brain_imu_bench.h"
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

std::vector<uint8_t> requestBytes(const translagatr::BrainRequest& r) {
    std::vector<uint8_t> buf(translagatr::kMaxFrameLen);
    buf.resize(translagatr::encodeBrainRequest(r, buf.data(), translagatr::kMaxFrameLen));
    EXPECT_FALSE(buf.empty());
    return buf;
}

translagatr::BrainRequest request(uint8_t op, uint32_t session, uint16_t rid) {
    translagatr::BrainRequest r;
    r.op         = op;
    r.session    = session;
    r.request_id = rid;
    return r;
}

translagatr::BrainRequest helloRequest(uint16_t rid, uint32_t nonce) {
    translagatr::BrainRequest r = request(translagatr::kOpHello, 0, rid);
    r.nonce               = nonce;
    return r;
}

translagatr::BrainRequest setPoseRequest(uint32_t session, uint16_t rid, int32_t x_mm, int32_t y_mm,
                                   int32_t heading_cdeg) {
    translagatr::BrainRequest r = request(translagatr::kOpSetPose, session, rid);
    r.x_mm                = x_mm;
    r.y_mm                = y_mm;
    r.heading_cdeg        = heading_cdeg;
    return r;
}

translagatr::BrainRequest getStateRequest(uint32_t session, uint16_t rid) {
    return request(translagatr::kOpGetState, session, rid);
}

translagatr::BrainRequest pathRequest(uint32_t session, uint16_t rid, uint32_t command_id,
                                uint8_t mode, const std::vector<translagatr::PathPoint>& points) {
    translagatr::BrainRequest r = request(translagatr::kOpPathReport, session, rid);
    r.command_id          = command_id;
    r.path_mode           = mode;
    r.point_count         = static_cast<uint8_t>(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        r.points[i] = points[i];
    }
    return r;
}

translagatr::BrainRequest readDocRequest(uint32_t session, uint16_t rid, uint8_t kind) {
    translagatr::BrainRequest r = request(translagatr::kOpReadDoc, session, rid);
    r.doc_kind            = kind;
    r.max_len             = translagatr::kDocChunkMax;
    return r;
}

translagatr::BrainRequest controlRequest(uint32_t session, uint16_t rid, uint8_t action) {
    translagatr::BrainRequest r = request(translagatr::kOpControl, session, rid);
    r.action              = action;
    return r;
}

translagatr::BrainRequest writeRequest(uint32_t session, uint16_t rid, uint32_t profile_id,
                                 const std::vector<uint8_t>& doc, uint16_t offset,
                                 uint16_t length) {
    translagatr::BrainRequest r = request(translagatr::kOpProfileWrite, session, rid);
    r.profile_id          = profile_id;
    r.total_len           = static_cast<uint16_t>(doc.size());
    r.offset              = offset;
    r.data_len            = static_cast<uint8_t>(length);
    std::memcpy(r.data, doc.data() + offset, length);
    return r;
}

translagatr::BrainRequest applyRequest(uint32_t session, uint16_t rid, uint32_t profile_id,
                                 uint16_t total_len) {
    translagatr::BrainRequest r = request(translagatr::kOpProfileApply, session, rid);
    r.profile_id          = profile_id;
    r.total_len           = total_len;
    return r;
}

// Two perpendicular wheels and the Brain VEX IMU. Test values only.
translagatr::RobotProfileDoc benchProfile() {
    translagatr::RobotProfileDoc p;
    p.topology           = translagatr::kTopologyTwoWheelImu;
    p.wheel_count        = 2;
    p.imu_source         = translagatr::kImuSourceBrainVex;
    p.vex_smart_port     = 1;
    p.footprint_front_um = 200000;
    p.footprint_back_um  = 200000;
    p.footprint_left_um  = 200000;
    p.footprint_right_um = 200000;
    p.wheels[0]          = {0, 0, 4000, 24000, 0, 150000, 0};
    p.wheels[1]          = {1, 0, 4000, 24000, 100000, 0, 90000};
    return p;
}

// Profile document lengths for two and three wheels, no cameras.
constexpr uint16_t kTwoWheelLen   = translagatr::kProfileHeaderLen + 2 * translagatr::kProfileWheelLen;
constexpr uint16_t kThreeWheelLen = translagatr::kProfileHeaderLen + 3 * translagatr::kProfileWheelLen;

std::vector<uint8_t> profileBytes(const translagatr::RobotProfileDoc& p) {
    std::vector<uint8_t> bytes(translagatr::kProfileMaxLen);
    bytes.resize(translagatr::encodeRobotProfile(p, bytes.data(), translagatr::kProfileMaxLen));
    EXPECT_FALSE(bytes.empty());
    return bytes;
}

uint32_t profileId(const std::vector<uint8_t>& doc) {
    return translagatr::crc32(doc.data(), static_cast<uint32_t>(doc.size()));
}

// Every reply frame the Pi wrote since the last call.
std::vector<translagatr::BrainReply> takeReplies(MemoryLink& link) {
    std::vector<translagatr::BrainReply> out;
    translagatr::FrameReader             reader;
    for (uint8_t b : link.output().takeAll()) {
        if (!reader.push(b)) {
            continue;
        }
        do {
            translagatr::BrainReply reply;
            EXPECT_EQ(reader.frameType(), translagatr::kFrameBrainReply);
            EXPECT_TRUE(translagatr::decodeBrainReply(reader.frame(), reader.frameLen(), reply));
            out.push_back(reply);
        } while (reader.next());
    }
    return out;
}

// Stands in for the System: counts calls, accepts or refuses.
struct FakeProfileHost : BrainProfileHost {
    int      calls   = 0;
    uint32_t last_id = 0;
    bool     accept  = true;
    uint8_t  reason  = translagatr::kProfileReasonNone;
    uint8_t  detail  = 0;

    bool prepare(const translagatr::RobotProfileDoc&, uint32_t profile_id, uint8_t& r,
                 uint8_t& d) override {
        ++calls;
        last_id = profile_id;
        if (!accept) {
            r = reason;
            d = detail;
        }
        return accept;
    }

    std::shared_ptr<const ProfileBinding> applied() const override { return nullptr; }

    uint8_t control(uint8_t, uint8_t, MonotonicTime, uint8_t& d) override {
        ++controls;
        d = translagatr::kControlDetailNone;
        return control_result;
    }

    uint8_t controlProgress(uint8_t, uint8_t, MonotonicTime, uint8_t& d) override {
        ++progress_calls;
        d = progress_detail;
        return progress_result;
    }

    uint8_t readWheels(MonotonicTime, uint8_t& count, translagatr::WheelReading* wheels) override {
        ++wheel_reads;
        count = 0;
        if (wheels_result == translagatr::kResultOk) {
            translagatr::WheelReading r;
            r.port      = 1;
            r.flags     = translagatr::kWheelFresh | translagatr::kWheelValid;
            r.counts    = 1234;
            r.travel_um = -5678;
            wheels[count++] = r;
        }
        return wheels_result;
    }

    int     controls        = 0;
    uint8_t control_result  = translagatr::kResultNotReady;
    int     progress_calls  = 0;
    uint8_t progress_result = translagatr::kResultPending;
    uint8_t progress_detail = translagatr::kControlDetailNone;
    int     wheel_reads     = 0;
    uint8_t wheels_result   = translagatr::kResultNotReady;
};

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
</Publishing>)";

// With a profile host the health references follow the profile.
const char* kHostedPublishingXml = R"(<Publishing type="brain_link">
    <Serial resource_id="brain_uart"/>
    <Health fresh_ms="150"/>
</Publishing>)";

const char* kBenchCommandsXml = R"(<CommandCollection type="brain_link">
    <Serial resource_id="brain_uart"/><BenchImu resource_id="brain_imu"/></CommandCollection>)";

struct LinkHarness {
    tinyxml2::XMLDocument     doc;
    FunctionRegistry          functions;
    std::vector<std::string>  warnings;
    DiagnosticsHub            hub;   // outlives the slots, which keep a pointer
    ResourceStore             store;
    SensorCatalog             catalog;
    SlotInitializationContext context;
    MemoryLink*               brain    = nullptr;
    std::shared_ptr<BrainImuBench> bench;
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

    explicit LinkHarness(BrainProfileHost* host = nullptr, bool idle_first = true,
                         bool with_bench = false) {
        register_resources(functions);
        register_commands(functions);
        register_publishers(functions);

        tinyxml2::XMLDocument resources_doc;
        EXPECT_EQ(resources_doc.Parse(R"(
<Resources><Resource id="brain_uart" type="memory_link"/>
           <Resource id="brain_imu" type="brain_imu_bench"/></Resources>)"),
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
        context.brain_profile         = host;
        context.diagnostics           = &hub;
        bench = store.require<BrainImuBench>(ResourceId{"brain_imu"}, err);
        EXPECT_NE(bench, nullptr) << err;

        commands = make<CommandsMakeFunction>(with_bench ? kBenchCommandsXml : kCommandsXml,
                                              "brain_link", err);
        EXPECT_NE(commands, nullptr) << err;
        publisher = make<PublishingMakeFunction>(host == nullptr ? kPublishingXml
                                                                 : kHostedPublishingXml,
                                                 "brain_link", err);
        EXPECT_NE(publisher, nullptr) << err;

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
    std::vector<translagatr::BrainReply> cycle(int64_t advance_us = 5000,
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

    std::vector<translagatr::BrainReply> exchange(const translagatr::BrainRequest& r) {
        brain->input().feed(requestBytes(r));
        return cycle();
    }

    translagatr::BrainReply one(const translagatr::BrainRequest& r) {
        const std::vector<translagatr::BrainReply> replies = exchange(r);
        EXPECT_EQ(replies.size(), 1u);
        return replies.empty() ? translagatr::BrainReply{} : replies.front();
    }

    uint32_t open(uint16_t rid = 1, uint32_t nonce = 0x12345678) {
        const translagatr::BrainReply reply = one(helloRequest(rid, nonce));
        EXPECT_EQ(reply.result, translagatr::kResultOk);
        EXPECT_NE(reply.session, 0u);
        return reply.session;
    }

    // Whole document in chunks of at most chunk bytes; the last reply.
    translagatr::BrainReply stage(uint32_t session, uint16_t& rid, const std::vector<uint8_t>& bytes,
                            uint16_t chunk = translagatr::kProfileChunkMax) {
        const uint32_t    id = profileId(bytes);
        translagatr::BrainReply reply;
        for (std::size_t offset = 0; offset < bytes.size(); offset += chunk) {
            const std::size_t n = std::min<std::size_t>(chunk, bytes.size() - offset);
            reply = one(writeRequest(session, rid++, id, bytes, static_cast<uint16_t>(offset),
                                     static_cast<uint16_t>(n)));
            EXPECT_EQ(reply.result, translagatr::kResultOk);
        }
        return reply;
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

    std::vector<translagatr::BrainReply> step() {
        clock_us += 5000;
        now_ms += 5;
        system->step(hostTime(now_ms));
        return takeReplies(*brain);
    }

    translagatr::BrainReply one(const translagatr::BrainRequest& r) {
        brain->input().feed(requestBytes(r));
        const std::vector<translagatr::BrainReply> replies = step();
        EXPECT_EQ(replies.size(), 1u);
        return replies.empty() ? translagatr::BrainReply{} : replies.front();
    }

    uint32_t open(uint16_t rid = 1, uint32_t nonce = 0x12345678) {
        const translagatr::BrainReply reply = one(helloRequest(rid, nonce));
        EXPECT_EQ(reply.result, translagatr::kResultOk);
        return reply.session;
    }

    const LinkStats& stats() { return system->diagnostics().links["brain_uart"]; }
};

} // namespace

// ---- sessions ----------------------------------------------------------------

TEST(BrainLinkSession, HelloOpensASessionAndKeepsThePiSideState) {
    LinkHarness f;
    f.command.path.mode          = translagatr::kPathDirect;   // left over from an earlier session
    f.command.profile.state      = translagatr::kProfileApplied;
    f.command.profile.id         = 0x1234;
    f.command.profile.applied_id = 0x1234;

    const translagatr::BrainReply reply = f.one(helloRequest(1, 0xCAFE));
    EXPECT_EQ(reply.version, translagatr::kBrainLinkVersion);
    EXPECT_EQ(reply.op, translagatr::kOpHello);
    EXPECT_EQ(reply.request_id, 1u);
    EXPECT_EQ(reply.result, translagatr::kResultOk);
    EXPECT_EQ(reply.nonce, 0xCAFEu);
    EXPECT_NE(reply.session, 0u);
    EXPECT_NE(reply.pi_instance, 0u);
    EXPECT_EQ(f.command.session, reply.session);
    EXPECT_EQ(f.command.path.mode, translagatr::kPathNone);   // the old path is gone
    EXPECT_EQ(f.command.init_sequence, 0u);             // never touches placement
    EXPECT_EQ(f.command.profile.applied_id, 0x1234u);   // nor the profile
    EXPECT_EQ(f.command.profile.state, translagatr::kProfileApplied);
}

namespace
{

// A link a test can close, as a pulled USB cable closes the Brain device.
struct ClosableLink : SerialLink {
    MemoryLink inner;
    bool       closed = false;

    SerialReadResult readAvailable(MutableByteSpan destination) override {
        return closed ? SerialReadResult{0, true} : inner.readAvailable(destination);
    }
    SerialWriteResult write(ByteSpan source) override { return inner.write(source); }
    SerialWriteResult write(ByteSpan source, const TransmitWindow& window) override {
        return inner.write(source, window);
    }
    bool    inputPending() override { return inner.inputPending(); }
    int64_t nowUs() override { return inner.nowUs(); }
};

} // namespace

TEST(BrainLinkSession, LinkLivenessGoesToTheCommandStateEveryCycle) {
    FunctionRegistry functions;
    register_commands(functions);
    auto link = std::make_shared<ClosableLink>();
    ASSERT_TRUE(functions.add<ResourceMakeFunction>(
        FunctionKey{"closable_link"},
        [link](const ConfigNode&, ResourceInitializationContext&, std::string&) {
            return ResourceInstance::asContract<SerialLink>(link);
        }));
    tinyxml2::XMLDocument resources;
    ASSERT_EQ(resources.Parse(R"(<Resource id="brain_uart" type="closable_link"/>)"),
              tinyxml2::XML_SUCCESS);
    std::vector<std::string> warnings;
    ResourceStoreBuilder     builder(functions, &warnings);
    std::string              err;
    ASSERT_TRUE(builder.index(ConfigNode{resources.RootElement()}, err)) << err;
    ASSERT_TRUE(builder.buildAll(err)) << err;
    ResourceStore             store = builder.take();
    SensorCatalog             catalog;
    SlotInitializationContext context;
    context.resources    = &store;
    context.sensors      = &catalog;
    context.functions    = &functions;
    context.loop_rate_hz = 100.0;
    tinyxml2::XMLDocument node;
    ASSERT_EQ(node.Parse(kCommandsXml), tinyxml2::XML_SUCCESS);
    const CommandsMakeFunction* make =
        functions.find<CommandsMakeFunction>(FunctionKey{"brain_link"}, err);
    ASSERT_NE(make, nullptr) << err;
    std::unique_ptr<Commands> commands = (*make)(ConfigNode{node.RootElement()}, context, err);
    ASSERT_NE(commands, nullptr) << err;

    CommandState state;
    const auto   run = [&](int64_t ms) {
        state = commands->run({state, hostTime(ms), nullptr}).command;
    };
    run(1000);
    EXPECT_NE(state.pi_instance, 0u);   // known before any request
    EXPECT_TRUE(state.link_open);
    EXPECT_FALSE(state.last_request.isSet());

    link->inner.input().feed(requestBytes(helloRequest(1, 0xCAFE)));
    run(1020);
    EXPECT_EQ(state.last_request.ms, 1020);
    run(1040);   // quiet: the newest request time stays
    EXPECT_EQ(state.last_request.ms, 1020);
    EXPECT_TRUE(state.link_open);

    link->closed = true;
    run(1060);
    EXPECT_FALSE(state.link_open);
    EXPECT_EQ(state.last_request.ms, 1020);
    EXPECT_NE(state.session, 0u);   // a closed link keeps the session
    link->closed = false;
    run(1080);
    EXPECT_TRUE(state.link_open);
}

TEST(BrainLinkSession, HelloRetryIsIdempotentUntilTheSessionIsUsed) {
    LinkHarness    f;
    const uint32_t session = f.open(1, 0xAB);
    f.command.path.mode    = translagatr::kPathDirect;

    const translagatr::BrainReply retry = f.one(helloRequest(1, 0xAB));
    EXPECT_EQ(retry.result, translagatr::kResultOk);
    EXPECT_EQ(retry.session, session);
    EXPECT_EQ(f.command.path.mode, translagatr::kPathDirect);   // nothing changed

    EXPECT_EQ(f.one(getStateRequest(session, 2)).result, translagatr::kResultOk);
    const translagatr::BrainReply late = f.one(helloRequest(1, 0xAB));   // used: no longer a retry
    EXPECT_EQ(late.result, translagatr::kResultStale);
    EXPECT_EQ(late.nonce, 0xABu);
    EXPECT_EQ(f.command.session, session);
}

TEST(BrainLinkSession, HelloWithARecentNonceIsStale) {
    LinkHarness    f;
    const uint32_t a = f.open(1, 0x1111);
    const uint32_t b = f.open(1, 0x2222);   // brain reboot: counters restart
    EXPECT_NE(a, b);

    const translagatr::BrainReply reply = f.one(helloRequest(7, 0x1111));
    EXPECT_EQ(reply.result, translagatr::kResultStale);
    EXPECT_EQ(f.command.session, b);
    EXPECT_EQ(f.stats().stale, 1u);

    // the ring holds four nonces: the oldest falls out
    f.open(1, 0x3333);
    f.open(1, 0x4444);
    f.open(1, 0x5555);
    EXPECT_EQ(f.one(helloRequest(2, 0x2222)).result, translagatr::kResultStale);
    EXPECT_EQ(f.one(helloRequest(2, 0x1111)).result, translagatr::kResultOk);
}

TEST(BrainLinkSession, UnknownSessionChangesNothing) {
    LinkHarness f;
    EXPECT_EQ(f.one(getStateRequest(0, 1)).result, translagatr::kResultUnknownSession);   // none open

    const uint32_t          session = f.open();
    const translagatr::BrainReply reply =
        f.one(pathRequest(session + 1, 2, 5, translagatr::kPathDirect, {{0, 0}, {100, 0}}));
    EXPECT_EQ(reply.result, translagatr::kResultUnknownSession);
    EXPECT_EQ(reply.session, session + 1);   // echo
    EXPECT_EQ(f.command.path.mode, translagatr::kPathNone);
    EXPECT_EQ(f.stats().unknown_session, 2u);
}

TEST(BrainLinkSession, VersionAndRetiredOpErrors) {
    LinkHarness    f;
    const uint32_t session = f.open();

    translagatr::BrainRequest old   = getStateRequest(session, 2);
    old.version               = 3;
    const translagatr::BrainReply v = f.one(old);
    EXPECT_EQ(v.result, translagatr::kResultUnsupportedVersion);
    EXPECT_EQ(v.version, translagatr::kBrainLinkVersion);   // the Pi's version

    // 3 and 5 are the retired v3 landmark select and IMU state ops; 13 is not defined
    for (uint8_t op : {uint8_t{3}, uint8_t{5}, uint8_t{13}}) {
        translagatr::BrainRequest unknown = getStateRequest(session, 3);
        unknown.op                  = op;
        const translagatr::BrainReply r   = f.one(unknown);
        EXPECT_EQ(r.result, translagatr::kResultUnsupportedOp);
        EXPECT_EQ(r.op, op);
    }

    EXPECT_EQ(f.one(getStateRequest(session, 0)).result, translagatr::kResultInvalidArgument);
}

TEST(BrainLinkSession, MalformedBodiesAreInvalidAndConsumeNothing) {
    LinkHarness                      f;
    const uint32_t                   session = f.open();
    const std::vector<uint8_t>       doc     = profileBytes(benchProfile());
    std::vector<translagatr::BrainRequest> bad;

    translagatr::BrainRequest state = getStateRequest(session, 2);
    state.imu_flags           = 0x02;   // unknown bit
    bad.push_back(state);
    bad.push_back(readDocRequest(session, 2, 3));   // unknown kind
    translagatr::BrainRequest empty = readDocRequest(session, 2, translagatr::kDocFieldMap);
    empty.max_len             = 0;
    bad.push_back(empty);
    bad.push_back(controlRequest(session, 2, 0));
    bad.push_back(controlRequest(session, 2, 5));
    bad.push_back(pathRequest(session, 2, 1, 3, {}));
    translagatr::BrainRequest past_end = writeRequest(session, 2, profileId(doc), doc, 40, 32);
    past_end.total_len           = 60;   // offset + length over total_len
    bad.push_back(past_end);
    translagatr::BrainRequest too_long = writeRequest(session, 2, profileId(doc), doc, 0, 40);
    too_long.total_len           = translagatr::kProfileMaxLen + 1;
    bad.push_back(too_long);
    bad.push_back(applyRequest(session, 2, 1, translagatr::kProfileHeaderLen - 1));

    for (const translagatr::BrainRequest& r : bad) {
        SCOPED_TRACE(static_cast<int>(r.op));
        EXPECT_EQ(f.one(r).result, translagatr::kResultInvalidArgument);
    }
    // none of them was recorded as the newest request
    EXPECT_EQ(f.one(getStateRequest(session, 2)).result, translagatr::kResultOk);
}

TEST(BrainLinkSession, WheelReadingsAndPicoControlWaitForTheProfileBoundary) {
    LinkHarness    plain;   // no profile host: this configuration never serves readings
    const uint32_t a = plain.open();
    EXPECT_EQ(plain.one(request(translagatr::kOpReadWheels, a, 2)).result, translagatr::kResultUnavailable);
    EXPECT_EQ(plain.one(request(translagatr::kOpReadWheels, a, 2)).result,
              translagatr::kResultUnavailable);   // read-only: the newest id is answered again

    FakeProfileHost host;
    LinkHarness     hosted(&host);
    const uint32_t  b   = hosted.open();
    uint16_t        rid = 2;
    EXPECT_EQ(hosted.one(request(translagatr::kOpReadWheels, b, rid++)).result,
              translagatr::kResultNotReady);
    for (uint8_t action : {translagatr::kControlReinitImu, translagatr::kControlRestartAcquisition}) {
        const translagatr::BrainReply r = hosted.one(controlRequest(b, rid++, action));
        EXPECT_EQ(r.result, translagatr::kResultNotReady);   // header only, no action echo
        EXPECT_EQ(r.op, translagatr::kOpControl);
    }
}

// ---- dedupe ------------------------------------------------------------------

TEST(BrainLinkDedupe, SetPoseDuplicatesAnsweredNeverReapplied) {
    LinkHarness    f;
    const uint32_t session = f.open();

    const translagatr::BrainReply pose = f.one(setPoseRequest(session, 3, 610, 457, 9000));
    EXPECT_EQ(pose.result, translagatr::kResultPending);   // nothing here applies placements
    EXPECT_EQ(f.command.init_sequence, 1u);
    EXPECT_EQ(f.command.init_session, session);
    EXPECT_NEAR(f.command.init_pose.x_m, 0.610, 1e-12);
    EXPECT_NEAR(f.command.init_pose.heading_rad, kPi / 2.0, 1e-9);

    // a retry after newer state polls answers from the record
    EXPECT_EQ(f.one(getStateRequest(session, 4)).result, translagatr::kResultOk);
    EXPECT_EQ(f.one(setPoseRequest(session, 3, 610, 457, 9000)).result, translagatr::kResultPending);
    EXPECT_EQ(f.command.init_sequence, 1u);
    EXPECT_EQ(f.stats().duplicates, 1u);
    EXPECT_EQ(f.one(setPoseRequest(session, 3, 611, 457, 9000)).result,
              translagatr::kResultInvalidArgument);
    EXPECT_EQ(f.one(getStateRequest(session, 3)).result, translagatr::kResultInvalidArgument);
    EXPECT_EQ(f.command.init_sequence, 1u);
}

TEST(BrainLinkDedupe, ControlIsAnsweredFromItsRecord) {
    LinkHarness    f;
    const uint32_t session = f.open();

    const translagatr::BrainReply first = f.one(controlRequest(session, 2, translagatr::kControlRecalibrate));
    EXPECT_EQ(first.result, translagatr::kResultNotReady);   // no calibration control yet
    EXPECT_EQ(f.one(getStateRequest(session, 3)).result, translagatr::kResultOk);
    EXPECT_EQ(f.one(controlRequest(session, 2, translagatr::kControlRecalibrate)).result,
              translagatr::kResultNotReady);
    EXPECT_EQ(f.stats().duplicates, 1u);
    EXPECT_EQ(f.one(controlRequest(session, 2, translagatr::kControlReinitialize)).result,
              translagatr::kResultInvalidArgument);
}

TEST(BrainLinkDedupe, NewestIdempotentRequestIsAnsweredAgain) {
    LinkHarness    f;
    const uint32_t session = f.open();

    f.robot.odom_pose = Pose2D{1.0, 0.0, 0.0};
    EXPECT_EQ(f.one(getStateRequest(session, 2)).state.x_mm, 1000);
    f.robot.odom_pose = Pose2D{1.5, 0.0, 0.0};
    const translagatr::BrainReply again = f.one(getStateRequest(session, 2));
    EXPECT_EQ(again.result, translagatr::kResultOk);
    EXPECT_EQ(again.state.x_mm, 1500);   // fresh state for the resend

    translagatr::BrainRequest other_body = getStateRequest(session, 2);
    other_body.imu_flags           = translagatr::kBenchImuValid;
    EXPECT_EQ(f.one(other_body).result, translagatr::kResultInvalidArgument);
    EXPECT_EQ(f.one(setPoseRequest(session, 2, 0, 0, 0)).result,
              translagatr::kResultInvalidArgument);
    EXPECT_EQ(f.command.init_sequence, 0u);

    const translagatr::BrainRequest path = pathRequest(session, 3, 9, translagatr::kPathDirect, {{1, 2}});
    EXPECT_EQ(f.one(path).result, translagatr::kResultOk);
    f.command.path = PathReport{};
    EXPECT_EQ(f.one(path).result, translagatr::kResultOk);   // stored again, same content
    EXPECT_EQ(f.command.path.command_id, 9u);
    EXPECT_EQ(f.one(pathRequest(session, 3, 9, translagatr::kPathDirect, {{1, 3}})).result,
              translagatr::kResultInvalidArgument);
    EXPECT_DOUBLE_EQ(f.command.path.points[0].y_m, 0.002);
    EXPECT_EQ(f.stats().duplicates, 0u);   // resends of the newest are not records
}

TEST(BrainLinkDedupe, OlderIdIsStaleAndIdsWrap) {
    LinkHarness    f;
    const uint32_t session = f.open(65534);

    EXPECT_EQ(f.one(getStateRequest(session, 65535)).result, translagatr::kResultOk);
    EXPECT_EQ(f.one(getStateRequest(session, 1)).result, translagatr::kResultOk);   // wrapped
    EXPECT_EQ(f.one(pathRequest(session, 2, 1, translagatr::kPathDirect, {{0, 0}})).result,
              translagatr::kResultOk);
    EXPECT_EQ(f.one(getStateRequest(session, 65535)).result, translagatr::kResultStale);
    EXPECT_EQ(f.one(pathRequest(session, 1, 1, translagatr::kPathNone, {})).result,
              translagatr::kResultStale);
    EXPECT_EQ(f.command.path.mode, translagatr::kPathDirect);   // the stale clear never applied
    EXPECT_EQ(f.one(getStateRequest(session, 2)).result, translagatr::kResultInvalidArgument);
    EXPECT_EQ(f.one(getStateRequest(session, 3)).result, translagatr::kResultOk);
    EXPECT_EQ(f.one(getStateRequest(session, 3)).result, translagatr::kResultOk);   // newest again
}

TEST(BrainLinkDedupe, DedupeIsPerSession) {
    LinkHarness    f;
    const uint32_t a = f.open(1, 0x1);
    f.one(setPoseRequest(a, 2, 100, 200, 0));
    EXPECT_EQ(f.command.init_sequence, 1u);

    // rebooted brain: same ids and body, new session, applies again
    const uint32_t b = f.open(1, 0x2);
    EXPECT_EQ(f.one(setPoseRequest(b, 2, 100, 200, 0)).result, translagatr::kResultPending);
    EXPECT_EQ(f.command.init_sequence, 2u);
    EXPECT_EQ(f.command.init_session, b);

    // a delayed session-A request cannot touch session B
    EXPECT_EQ(f.one(setPoseRequest(a, 3, 999, 999, 0)).result, translagatr::kResultUnknownSession);
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
    LinkHarness f(nullptr, false);
    f.brain->input().feed(requestBytes(helloRequest(1, 0x77)));
    EXPECT_TRUE(f.cycle().empty());
    EXPECT_NE(f.command.session, 0u);   // processed
    EXPECT_EQ(f.stats().unanswered, 1u);

    const translagatr::BrainReply retry = f.one(helloRequest(1, 0x77));   // the brain retries
    EXPECT_EQ(retry.result, translagatr::kResultOk);
    EXPECT_EQ(retry.session, f.command.session);
}

TEST(BrainLinkBus, WindowFromThePreviousDrain) {
    LinkHarness    f;
    const uint32_t session = f.open();

    // request completes 50 ms after the previous drain: too late to answer
    f.brain->input().feed(requestBytes(setPoseRequest(session, 2, 610, 457, 9000)));
    EXPECT_TRUE(f.cycle(50000).empty());
    EXPECT_EQ(f.stats().expired, 1u);
    EXPECT_EQ(f.command.init_sequence, 1u);   // applied; the retry is a duplicate

    const translagatr::BrainReply retry = f.one(setPoseRequest(session, 2, 610, 457, 9000));
    EXPECT_EQ(retry.result, translagatr::kResultPending);
    EXPECT_EQ(f.stats().duplicates, 1u);
    EXPECT_EQ(f.command.init_sequence, 1u);

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
    bytes.push_back(translagatr::kSync0);   // the brain may already be transmitting
    f.brain->input().feed(bytes);
    EXPECT_TRUE(f.cycle().empty());
    EXPECT_EQ(f.stats().unanswered, 1u);

    f.cycle();   // empty drain discards the partial frame
    f.brain->input().feed(requestBytes(getStateRequest(session, 3)));
    EXPECT_TRUE(f.cycle(5000, [&] { f.brain->input().feed({0x00}); }).empty());
    EXPECT_EQ(f.stats().input_pending, 1u);
    f.cycle();
    EXPECT_EQ(f.one(getStateRequest(session, 4)).result, translagatr::kResultOk);
}

TEST(BrainLinkBus, NewestRequestWins) {
    LinkHarness    f;
    const uint32_t session = f.open();

    std::vector<uint8_t> bytes =
        requestBytes(pathRequest(session, 2, 20, translagatr::kPathDirect, {{0, 0}}));
    const std::vector<uint8_t> newer =
        requestBytes(pathRequest(session, 3, 30, translagatr::kPathAvoiding, {{0, 0}, {5, 5}}));
    bytes.insert(bytes.end(), newer.begin(), newer.end());
    f.brain->input().feed(bytes);
    const std::vector<translagatr::BrainReply> replies = f.cycle();
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].request_id, 3u);
    EXPECT_EQ(f.command.path.command_id, 30u);   // never rid 2
    EXPECT_EQ(f.stats().superseded, 1u);
    EXPECT_EQ(f.one(pathRequest(session, 2, 20, translagatr::kPathDirect, {{0, 0}})).result,
              translagatr::kResultStale);
    EXPECT_EQ(f.command.path.command_id, 30u);
}

TEST(BrainLinkBus, SplitRequestAndDiscardedPartialFrame) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> bytes   = requestBytes(getStateRequest(session, 2));

    f.brain->input().feed({bytes.begin(), bytes.begin() + 5});
    EXPECT_TRUE(f.cycle().empty());
    f.brain->input().feed({bytes.begin() + 5, bytes.end()});
    const std::vector<translagatr::BrainReply> replies = f.cycle();
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].request_id, 2u);

    // a drain with no bytes drops the partial: the tail alone completes nothing
    const std::vector<uint8_t> next = requestBytes(getStateRequest(session, 3));
    f.brain->input().feed({next.begin(), next.begin() + 5});
    f.cycle();
    f.cycle();
    f.brain->input().feed({next.begin() + 5, next.end()});
    EXPECT_TRUE(f.cycle().empty());
    EXPECT_EQ(f.one(getStateRequest(session, 4)).result, translagatr::kResultOk);
}

TEST(BrainLinkBus, CorruptCrcChangesNothing) {
    LinkHarness          f;
    const uint32_t       session = f.open();
    std::vector<uint8_t> bytes   = requestBytes(setPoseRequest(session, 2, 610, 457, 9000));
    bytes[8] ^= 0x01;
    f.brain->input().feed(bytes);
    EXPECT_TRUE(f.cycle().empty());
    EXPECT_EQ(f.command.init_sequence, 0u);
    EXPECT_EQ(f.one(setPoseRequest(session, 2, 610, 457, 9000)).result, translagatr::kResultPending);
}

TEST(BrainLinkBus, LargestRequestAndReplyFitTheFrame) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> path =
        requestBytes(pathRequest(session, 2, 1, translagatr::kPathAvoiding,
                                 std::vector<translagatr::PathPoint>(translagatr::kPathReportMaxPoints)));
    EXPECT_EQ(path.size(), 6u + 8u + 6u + 8u * translagatr::kPathReportMaxPoints);
    f.brain->input().feed(path);
    ASSERT_EQ(f.cycle().size(), 1u);
    EXPECT_EQ(f.command.path.count, translagatr::kPathReportMaxPoints);

    std::vector<uint8_t> big(translagatr::kProfileMaxLen, 0);
    big[0] = translagatr::kProfileFormat;
    const translagatr::BrainRequest write =
        writeRequest(session, 3, profileId(big), big, 0, translagatr::kProfileChunkMax);
    EXPECT_EQ(requestBytes(write).size(), translagatr::kMaxFrameLen);
    const translagatr::BrainReply   staged = f.one(write);
    EXPECT_EQ(staged.result, translagatr::kResultOk);
    EXPECT_EQ(staged.received, translagatr::kProfileChunkMax);
}

// ---- reply bodies ------------------------------------------------------------

TEST(BrainLinkState, ProfileStatusAndNoFieldDocuments) {
    LinkHarness    f;
    const uint32_t session = f.open();

    translagatr::BrainReply s = f.one(getStateRequest(session, 2));
    EXPECT_EQ(s.state.profile_state, translagatr::kProfileNone);
    EXPECT_EQ(s.state.profile_id, 0u);
    EXPECT_EQ(s.state.map_id, 0u);
    EXPECT_EQ(s.state.estimate_id, 0u);
    EXPECT_EQ(s.state.calibration, translagatr::kCalibrationNone);

    f.command.profile = ProfileStatus{translagatr::kProfileRejected, translagatr::kProfileReasonEncoderPort,
                                      1, 0xABCD, 0x1234};
    s = f.one(getStateRequest(session, 3));
    EXPECT_EQ(s.state.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(s.state.profile_reason, translagatr::kProfileReasonEncoderPort);
    EXPECT_EQ(s.state.profile_detail, 1u);
    EXPECT_EQ(s.state.profile_id, 0xABCDu);   // the refused id, not the running one

    EXPECT_EQ(f.one(readDocRequest(session, 4, translagatr::kDocFieldMap)).result,
              translagatr::kResultUnavailable);
    EXPECT_EQ(f.one(readDocRequest(session, 5, translagatr::kDocFieldEstimate)).result,
              translagatr::kResultUnavailable);
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
    ObservationFunctionStatus tracking;
    tracking.id              = "tracking_motion";
    tracking.type            = "tracking_wheel_motion";
    tracking.ready           = true;
    f.localization.functions = {tracking};

    translagatr::BrainReply s = f.one(getStateRequest(session, 2));
    EXPECT_EQ(s.state.x_mm, 1500);
    EXPECT_EQ(s.state.y_mm, -250);
    EXPECT_EQ(s.state.heading_cdeg, 9000);
    EXPECT_EQ(s.state.odometry_epoch, 5u);
    EXPECT_EQ(s.state.anchor_revision, 2u);
    EXPECT_EQ(s.state.robot_flags, translagatr::kRobotPoseValid | translagatr::kRobotLocalized);
    EXPECT_EQ(s.state.robot_age_ms, 0u);   // age unknown without a host time
    EXPECT_EQ(s.state.health, translagatr::kHealthEncodersFresh | translagatr::kHealthGyroFresh |
                                  translagatr::kHealthBiasCalibrated);

    f.robot.measuredAtHost   = hostTime(f.now_ms + 5 - 20);
    f.robot.placement_origin = "command";
    s                        = f.one(getStateRequest(session, 3));
    EXPECT_TRUE(s.state.robot_flags & translagatr::kRobotAgeKnown);
    EXPECT_TRUE(s.state.robot_flags & translagatr::kRobotAnchorCommand);
    EXPECT_FALSE(s.state.robot_flags & translagatr::kRobotAnchorConfigured);
    EXPECT_EQ(s.state.robot_age_ms, 20u);

    f.robot.measuredAtHost   = hostTime(f.now_ms - 100000);
    f.robot.placement_origin = "configuration";
    f.results[SensorId{"enc_a"}].latest->receivedAt = hostTime(f.now_ms - 1000);
    s = f.one(getStateRequest(session, 4));
    EXPECT_EQ(s.state.robot_age_ms, 65535u);   // clamped
    EXPECT_FALSE(s.state.robot_flags & translagatr::kRobotAnchorCommand);
    EXPECT_TRUE(s.state.robot_flags & translagatr::kRobotAnchorConfigured);
    EXPECT_FALSE(s.state.health & translagatr::kHealthEncodersFresh);   // stale encoder
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
    const translagatr::BrainReply ok = f.one(setPoseRequest(session, 2, 610, 457, 9000));
    EXPECT_EQ(ok.result, translagatr::kResultOk);
    EXPECT_EQ(ok.anchor_revision, 4u);

    f.robot.placement_session = session + 1;   // same sequence, another session
    EXPECT_EQ(f.one(setPoseRequest(session, 2, 610, 457, 9000)).result, translagatr::kResultPending);
}

TEST(BrainLinkState, PathReportIsKeptForInspectionUntilClearedOrANewSession) {
    LinkHarness    f;
    const uint32_t session = f.open();

    std::vector<translagatr::PathPoint> points;
    for (int32_t i = 0; i < translagatr::kPathReportMaxPoints; ++i) {
        points.push_back({i * 100, -i * 50});
    }
    f.one(pathRequest(session, 2, 77, translagatr::kPathAvoiding, points));
    const PathReport& path = f.command.path;
    EXPECT_EQ(path.session, session);
    EXPECT_EQ(path.command_id, 77u);
    EXPECT_EQ(path.mode, translagatr::kPathAvoiding);
    ASSERT_EQ(path.count, translagatr::kPathReportMaxPoints);
    EXPECT_DOUBLE_EQ(path.points[12].x_m, 1.2);
    EXPECT_DOUBLE_EQ(path.points[12].y_m, -0.6);
    EXPECT_EQ(path.received.ms, f.now_ms);

    f.one(pathRequest(session, 3, 77, translagatr::kPathNone, {}));
    EXPECT_EQ(f.command.path.mode, translagatr::kPathNone);
    EXPECT_EQ(f.command.path.count, 0u);

    f.one(pathRequest(session, 4, 78, translagatr::kPathDirect, {{1, 1}}));
    f.open(1, 0xBEEF);
    EXPECT_EQ(f.command.path.mode, translagatr::kPathNone);
}

// ---- robot profile -----------------------------------------------------------

TEST(BrainLinkProfile, WritesStageContiguousBytesAndResendsAreIdempotent) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    ASSERT_EQ(doc.size(), kTwoWheelLen);
    uint16_t rid = 2;

    translagatr::BrainReply r = f.one(writeRequest(session, rid++, id, doc, 0, 40));
    EXPECT_EQ(r.result, translagatr::kResultOk);
    EXPECT_EQ(r.profile_id, id);
    EXPECT_EQ(r.received, 40u);
    EXPECT_EQ(f.one(writeRequest(session, rid++, id, doc, 0, 40)).received, 40u);   // resend
    EXPECT_EQ(f.one(writeRequest(session, rid++, id, doc, 20, 40)).received, 60u);   // overlap

    EXPECT_EQ(f.one(writeRequest(session, rid++, id, doc, 70, 2)).result,
              translagatr::kResultInvalidArgument);   // a gap
    std::vector<uint8_t> other = doc;
    other[10] ^= 0xFF;
    EXPECT_EQ(f.one(writeRequest(session, rid++, id, other, 0, 20)).result,
              translagatr::kResultInvalidArgument);   // a resend with other bytes
    EXPECT_EQ(f.one(applyRequest(session, rid++, id, kTwoWheelLen)).result,
              translagatr::kResultInvalidArgument);   // incomplete

    // staging survives a new session
    const uint32_t next = f.open(1, 0xFEED);
    rid                 = 2;
    r                   = f.one(writeRequest(next, rid++, id, doc, 60, kTwoWheelLen - 60));
    EXPECT_EQ(r.received, kTwoWheelLen);
    EXPECT_EQ(f.one(applyRequest(next, rid++, id, kTwoWheelLen)).result, translagatr::kResultProfileRejected);

    // another id restarts staging, but only from offset 0
    translagatr::RobotProfileDoc second = benchProfile();
    second.wheels[0].counts_per_rev = 8192;
    const std::vector<uint8_t> doc2 = profileBytes(second);
    EXPECT_EQ(f.one(writeRequest(next, rid++, profileId(doc2), doc2, 40, 32)).result,
              translagatr::kResultInvalidArgument);
    EXPECT_EQ(f.one(applyRequest(next, rid++, id, kTwoWheelLen)).result,
              translagatr::kResultProfileRejected);   // the first is still staged
    EXPECT_EQ(f.one(writeRequest(next, rid++, profileId(doc2), doc2, 0, 32)).received, 32u);
    EXPECT_EQ(f.one(applyRequest(next, rid++, id, kTwoWheelLen)).result, translagatr::kResultInvalidArgument);
}

TEST(BrainLinkProfile, ApplyNeedsTheWholeDocumentUnderItsCrc) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    uint16_t                   rid     = 2;

    EXPECT_EQ(f.one(applyRequest(session, rid++, id, kTwoWheelLen)).result,
              translagatr::kResultInvalidArgument);   // nothing staged
    f.stage(session, rid, doc);
    EXPECT_EQ(f.one(applyRequest(session, rid++, id, kThreeWheelLen)).result,
              translagatr::kResultInvalidArgument);   // another length
    EXPECT_EQ(f.one(applyRequest(session, rid++, id + 1, kTwoWheelLen)).result,
              translagatr::kResultInvalidArgument);   // another id

    // bytes staged under an id that is not their crc
    const uint32_t wrong = id ^ 0x5A5A5A5A;
    EXPECT_EQ(f.one(writeRequest(session, rid++, wrong, doc, 0, kTwoWheelLen)).result, translagatr::kResultOk);
    EXPECT_EQ(f.one(applyRequest(session, rid++, wrong, kTwoWheelLen)).result,
              translagatr::kResultInvalidArgument);
    EXPECT_EQ(f.command.profile.state, translagatr::kProfileNone);
}

TEST(BrainLinkProfile, AConfigurationWithoutAProfileHostRefusesEveryProfile) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    uint16_t                   rid     = 2;
    f.stage(session, rid, doc, 30);

    const translagatr::BrainReply r = f.one(applyRequest(session, rid++, id, kTwoWheelLen));
    EXPECT_EQ(r.result, translagatr::kResultProfileRejected);
    EXPECT_EQ(r.profile_id, id);
    EXPECT_EQ(r.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(r.profile_reason, translagatr::kProfileReasonNotAccepted);

    const translagatr::BrainReply s = f.one(getStateRequest(session, rid++));
    EXPECT_EQ(s.state.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(s.state.profile_reason, translagatr::kProfileReasonNotAccepted);
    EXPECT_EQ(s.state.profile_id, id);

    // the XML localization keeps accepting placements
    EXPECT_EQ(f.one(setPoseRequest(session, rid++, 1, 2, 3)).result, translagatr::kResultPending);
    EXPECT_EQ(f.command.init_sequence, 1u);
}

TEST(BrainLinkProfile, HostedProfileAppliesAtTheBoundaryAndGatesPlacement) {
    FakeProfileHost            host;
    LinkHarness                f(&host);
    const uint32_t             session = f.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    uint16_t                   rid     = 2;

    // no odometry to anchor yet
    EXPECT_EQ(f.one(setPoseRequest(session, rid, 610, 457, 0)).result, translagatr::kResultNotReady);
    EXPECT_EQ(f.one(setPoseRequest(session, rid++, 610, 457, 0)).result, translagatr::kResultNotReady);
    EXPECT_EQ(f.command.init_sequence, 0u);

    f.stage(session, rid, doc);
    translagatr::BrainReply r = f.one(applyRequest(session, rid++, id, kTwoWheelLen));
    EXPECT_EQ(r.result, translagatr::kResultPending);
    EXPECT_EQ(r.profile_state, translagatr::kProfileApplying);
    EXPECT_EQ(host.calls, 1);
    EXPECT_EQ(host.last_id, id);
    EXPECT_EQ(f.one(getStateRequest(session, rid++)).state.profile_state,
              translagatr::kProfileApplying);
    EXPECT_EQ(f.one(applyRequest(session, rid++, id, kTwoWheelLen)).result, translagatr::kResultPending);
    EXPECT_EQ(host.calls, 1);   // built once
    EXPECT_EQ(f.one(setPoseRequest(session, rid++, 610, 457, 0)).result,
              translagatr::kResultNotReady);

    // the System swaps it in at its boundary
    f.command.profile.state      = translagatr::kProfileApplied;
    f.command.profile.applied_id = id;
    r                            = f.one(applyRequest(session, rid++, id, kTwoWheelLen));
    EXPECT_EQ(r.result, translagatr::kResultOk);
    EXPECT_EQ(r.profile_state, translagatr::kProfileApplied);
    EXPECT_EQ(host.calls, 1);   // idempotent: nothing rebuilt or reset
    EXPECT_EQ(f.one(setPoseRequest(session, rid++, 610, 457, 0)).result, translagatr::kResultPending);
    EXPECT_EQ(f.command.init_sequence, 1u);

    // a new Brain session keeps the applied profile and its placement gate open
    const uint32_t next = f.open(1, 0xB007);
    EXPECT_EQ(f.one(setPoseRequest(next, 2, 610, 457, 0)).result, translagatr::kResultPending);
}

TEST(BrainLinkProfile, RejectionsCarryReasonsAndAreRemembered) {
    FakeProfileHost host;
    LinkHarness     f(&host);
    const uint32_t  session = f.open();
    uint16_t        rid     = 2;

    // the shared semantic check runs before the host
    translagatr::RobotProfileDoc three_vex = benchProfile();
    three_vex.topology               = translagatr::kTopologyThreeWheel;
    three_vex.wheel_count            = 3;
    three_vex.wheels[2]              = {2, 0, 4000, 24000, -100000, 0, 90000};
    const std::vector<uint8_t> a     = profileBytes(three_vex);
    f.stage(session, rid, a);
    translagatr::BrainReply r = f.one(applyRequest(session, rid++, profileId(a), kThreeWheelLen));
    EXPECT_EQ(r.result, translagatr::kResultProfileRejected);
    EXPECT_EQ(r.profile_reason, translagatr::kProfileReasonImuCombination);
    EXPECT_EQ(host.calls, 0);

    // undecodable bytes
    std::vector<uint8_t> junk(translagatr::kProfileHeaderLen, 0);
    junk[0] = 9;
    f.stage(session, rid, junk);
    r = f.one(applyRequest(session, rid++, profileId(junk), translagatr::kProfileHeaderLen));
    EXPECT_EQ(r.profile_reason, translagatr::kProfileReasonFormat);

    // this Pi's capability check, remembered for the id
    host.accept                    = false;
    host.reason                    = translagatr::kProfileReasonEncoderPort;
    host.detail                    = 1;
    translagatr::RobotProfileDoc unwired = benchProfile();
    unwired.wheels[1].encoder_port = 2;
    const std::vector<uint8_t> b   = profileBytes(unwired);
    f.stage(session, rid, b);
    r = f.one(applyRequest(session, rid++, profileId(b), kTwoWheelLen));
    EXPECT_EQ(r.profile_reason, translagatr::kProfileReasonEncoderPort);
    EXPECT_EQ(r.profile_detail, 1u);
    EXPECT_EQ(host.calls, 1);
    host.accept = true;
    r           = f.one(applyRequest(session, rid++, profileId(b), kTwoWheelLen));
    EXPECT_EQ(r.result, translagatr::kResultProfileRejected);
    EXPECT_EQ(r.profile_reason, translagatr::kProfileReasonEncoderPort);
    EXPECT_EQ(host.calls, 1);   // no retry storm

    const translagatr::BrainReply s = f.one(getStateRequest(session, rid++));
    EXPECT_EQ(s.state.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(s.state.profile_id, profileId(b));
    EXPECT_EQ(s.state.profile_detail, 1u);
    EXPECT_EQ(f.command.profile.applied_id, 0u);

    // a good profile still applies afterwards
    const std::vector<uint8_t> good = profileBytes(benchProfile());
    f.stage(session, rid, good);
    EXPECT_EQ(f.one(applyRequest(session, rid++, profileId(good), kTwoWheelLen)).result,
              translagatr::kResultPending);
    EXPECT_EQ(host.calls, 2);
}

TEST(BrainLinkConfig, PublishingPairingAndChildErrors) {
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
                    <FieldObject object_id="a" wire_id="1"/></Publishing>)",
                                             "brain_link", err),
              nullptr);
    EXPECT_NE(err.find("removed with brain link v4"), std::string::npos) << err;
    EXPECT_EQ(f.make<PublishingMakeFunction>(R"(<Publishing type="brain_link">
                    <Serial resource_id="brain_uart"/><Healt/></Publishing>)",
                                             "brain_link", err),
              nullptr);
    EXPECT_NE(err.find("unknown element Healt"), std::string::npos) << err;
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

    const translagatr::BrainReply ok = rig.one(setPoseRequest(session, 2, 610, 457, 9000));
    EXPECT_EQ(ok.result, translagatr::kResultOk);   // localization applied it this cycle
    EXPECT_EQ(ok.anchor_revision, 1u);
    const RobotState& robot = rig.system->robot();
    EXPECT_EQ(robot.anchor_revision, 1u);
    EXPECT_EQ(robot.placement_origin, "command");
    EXPECT_EQ(robot.placement_session, session);
    EXPECT_NEAR(robot.fieldPose().x_m, 0.610, 1e-9);

    // SET_POSE, GET_STATE, then the SET_POSE retry: applied once
    const translagatr::BrainReply s = rig.one(getStateRequest(session, 3));
    EXPECT_EQ(s.state.anchor_revision, 1u);
    EXPECT_TRUE(s.state.robot_flags & translagatr::kRobotAnchorCommand);
    EXPECT_EQ(rig.one(setPoseRequest(session, 2, 610, 457, 9000)).result, translagatr::kResultOk);
    EXPECT_EQ(rig.system->robot().anchor_revision, 1u);
    EXPECT_EQ(rig.system->command().init_sequence, 1u);
}

TEST(BrainLinkSystem, PendingUntilLocalizationApplies) {
    PiRig rig(piXml("noop"));   // an estimator that never places the robot
    ASSERT_NE(rig.system, nullptr);
    const uint32_t session = rig.open();
    EXPECT_EQ(rig.one(setPoseRequest(session, 2, 610, 457, 9000)).result,
              translagatr::kResultPending);
    EXPECT_EQ(rig.one(setPoseRequest(session, 2, 610, 457, 9000)).result,
              translagatr::kResultPending);
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
    EXPECT_EQ(rig.one(setPoseRequest(a, 3, 0, 0, 0)).result, translagatr::kResultUnknownSession);
    EXPECT_EQ(rig.system->robot().anchor_revision, 1u);

    // the same rid and pose from the new boot is a genuine new placement
    EXPECT_EQ(rig.one(setPoseRequest(b, 2, 610, 457, 9000)).result, translagatr::kResultOk);
    EXPECT_EQ(rig.system->robot().anchor_revision, 2u);
    EXPECT_EQ(rig.system->robot().placement_session, b);
    EXPECT_EQ(rig.system->robot().placement_sequence, 2u);
}

TEST(BrainLinkSystem, XmlConfiguredPiRefusesAProfileAndKeepsLocalizing) {
    PiRig rig(piXml());
    ASSERT_NE(rig.system, nullptr);
    const uint32_t             session = rig.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    uint16_t                   rid     = 2;
    EXPECT_EQ(rig.one(writeRequest(session, rid++, id, doc, 0, kTwoWheelLen)).received, kTwoWheelLen);

    const translagatr::BrainReply refused = rig.one(applyRequest(session, rid++, id, kTwoWheelLen));
    EXPECT_EQ(refused.result, translagatr::kResultProfileRejected);
    EXPECT_EQ(refused.profile_reason, translagatr::kProfileReasonNotAccepted);
    EXPECT_EQ(rig.one(setPoseRequest(session, rid++, 610, 457, 9000)).result,
              translagatr::kResultOk);
    const translagatr::BrainReply s = rig.one(getStateRequest(session, rid++));
    EXPECT_TRUE(s.state.robot_flags & translagatr::kRobotLocalized);
    EXPECT_EQ(s.state.profile_state, translagatr::kProfileRejected);
    EXPECT_EQ(s.state.map_id, 0u);
    EXPECT_EQ(rig.one(readDocRequest(session, rid++, translagatr::kDocFieldMap)).result,
              translagatr::kResultUnavailable);
    EXPECT_FALSE(rig.system->target().active);   // no Brain op selects a target
}

TEST(BrainLinkSystem, PiInstanceIsNewPerSystemAndReset) {
    PiRig first(piXml());
    PiRig second(piXml());
    ASSERT_NE(first.system, nullptr);
    ASSERT_NE(second.system, nullptr);
    const translagatr::BrainReply a = first.one(helloRequest(1, 0x1));
    const translagatr::BrainReply b = second.one(helloRequest(1, 0x1));
    EXPECT_NE(a.pi_instance, 0u);
    EXPECT_NE(a.pi_instance, b.pi_instance);

    const std::vector<uint8_t> doc = profileBytes(benchProfile());
    EXPECT_EQ(first.one(writeRequest(a.session, 2, profileId(doc), doc, 0, kTwoWheelLen)).received, kTwoWheelLen);

    first.system->reset();
    EXPECT_TRUE(first.step().empty());   // first drain after reset: no reply
    const translagatr::BrainReply old = first.one(getStateRequest(a.session, 3));
    EXPECT_EQ(old.result, translagatr::kResultUnknownSession);
    EXPECT_NE(old.pi_instance, a.pi_instance);
    EXPECT_EQ(first.system->command().init_sequence, 0u);

    // power-on state: staging is gone too
    const uint32_t session = first.open(1, 0x2);
    EXPECT_EQ(first.one(applyRequest(session, 2, profileId(doc), kTwoWheelLen)).result,
              translagatr::kResultInvalidArgument);
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

// ---- TELEMETRY -----------------------------------------------------------------

namespace
{

translagatr::BrainRequest telemetryRequest(uint32_t session, uint16_t rid, uint32_t stamp_ms,
                                           uint8_t flags = translagatr::kTelemetryAttitude |
                                                           translagatr::kTelemetryMotion |
                                                           translagatr::kTelemetryWheels) {
    translagatr::BrainRequest r   = request(translagatr::kOpTelemetry, session, rid);
    translagatr::BrainTelemetry& t = r.telemetry;
    t.flags               = flags;
    t.stamp_ms            = stamp_ms;
    t.roll_cdeg           = 250;    // 2.5 degrees, left side up
    t.pitch_cdeg          = -400;   // 4 degrees nose up
    t.command_id          = 7;
    t.motion_state        = 2;
    t.target_x_mm         = 1200;
    t.target_y_mm         = -300;
    t.target_heading_cdeg = 9000;
    t.cmd_vx_mm_s         = 500;
    t.wheel_count         = 2;
    t.wheel_rpm_x10[0]    = 1000;
    t.wheel_rpm_x10[1]    = -1000;
    return r;
}

uint64_t posted(const DiagnosticsHub& hub, DiagKind k) {
    return hub.stats().posted[static_cast<std::size_t>(k)];
}

LinkMonitorSnapshot brainMonitor(DiagnosticsHub& hub) {
    for (const auto& m : hub.links().all()) {
        if (m->id() == "brain_uart") {
            return m->snapshot();
        }
    }
    ADD_FAILURE() << "no brain_uart monitor";
    return LinkMonitorSnapshot{};
}

} // namespace

TEST(BrainLinkTelemetry, AnsweredOkHeaderOnlyAndRecordedWithoutTouchingState) {
    LinkHarness    f;
    const uint32_t session = f.open();
    const CommandState before = f.command;

    const translagatr::BrainRequest t = telemetryRequest(session, 2, 5000);
    const uint64_t sent_before = brainMonitor(f.hub).tx_accepted;
    const translagatr::BrainReply reply = f.one(t);
    // header only: envelope and the 13 byte reply header
    EXPECT_EQ(brainMonitor(f.hub).tx_accepted - sent_before,
              translagatr::kLinkEnvelopeLen + translagatr::kBrainReplyHeaderLen);
    EXPECT_EQ(reply.op, translagatr::kOpTelemetry);
    EXPECT_EQ(reply.result, translagatr::kResultOk);
    EXPECT_EQ(reply.request_id, 2);

    // display and capture only: no placement, path, profile or session change
    EXPECT_EQ(f.command.session, before.session);
    EXPECT_EQ(f.command.init_sequence, before.init_sequence);
    EXPECT_EQ(f.command.path.mode, before.path.mode);
    EXPECT_EQ(f.command.profile.state, before.profile.state);

    DiagRecord record;
    ASSERT_TRUE(f.hub.latest(DiagKind::kBrainTelemetry, record));
    const auto* body = std::get_if<DiagBrainTelemetry>(&record.payload);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(body->session, session);
    translagatr::BrainTelemetry decoded;
    ASSERT_TRUE(translagatr::decodeTelemetryBody(body->body, body->len, decoded));
    EXPECT_EQ(decoded.stamp_ms, 5000u);
    EXPECT_EQ(decoded.roll_cdeg, 250);
    EXPECT_EQ(decoded.target_y_mm, -300);
    EXPECT_EQ(decoded.wheel_rpm_x10[1], -1000);
}

TEST(BrainLinkTelemetry, ResendOfTheNewestIdIsAnsweredAgainButRecordedOnce) {
    LinkHarness    f;
    const uint32_t session = f.open();
    const translagatr::BrainRequest t = telemetryRequest(session, 2, 5000);
    EXPECT_EQ(f.one(t).result, translagatr::kResultOk);
    EXPECT_EQ(f.one(t).result, translagatr::kResultOk);   // a lost reply
    EXPECT_EQ(posted(f.hub, DiagKind::kBrainTelemetry), 1u);
    DiagRecord record;
    ASSERT_TRUE(f.hub.latest(DiagKind::kBrainRequest, record));
    EXPECT_TRUE(std::get<DiagBrainRequest>(record.payload).duplicate);

    // the same id with another body is not a resend
    translagatr::BrainRequest other = t;
    other.telemetry.stamp_ms        = 6000;
    EXPECT_EQ(f.one(other).result, translagatr::kResultInvalidArgument);
    EXPECT_EQ(posted(f.hub, DiagKind::kBrainTelemetry), 1u);
}

TEST(BrainLinkTelemetry, UnknownSessionIsRefusedAndNotRecorded) {
    LinkHarness    f(nullptr, true, true);
    const uint32_t session = f.open();
    EXPECT_EQ(f.one(telemetryRequest(session + 1, 2, 5000)).result,
              translagatr::kResultUnknownSession);
    EXPECT_EQ(posted(f.hub, DiagKind::kBrainTelemetry), 0u);
    EXPECT_FALSE(f.bench->attitude_valid);
}

TEST(BrainLinkTelemetry, AttitudeGoesToTheBenchMailboxOnly) {
    LinkHarness    f(nullptr, true, true);
    const uint32_t session = f.open();
    EXPECT_FALSE(f.bench->attitude_valid);

    EXPECT_EQ(f.one(telemetryRequest(session, 2, 5000)).result, translagatr::kResultOk);
    EXPECT_TRUE(f.bench->attitude_valid);
    EXPECT_EQ(f.bench->roll_cdeg, 250);
    EXPECT_EQ(f.bench->pitch_cdeg, -400);
    EXPECT_EQ(f.bench->attitude_received, hostTime(f.now_ms));   // Pi arrival time
    EXPECT_EQ(f.bench->attitude_sequence, 1u);
    EXPECT_FALSE(f.bench->valid);   // the rotation mailbox is untouched

    // a repeated Brain stamp is not a new sample
    EXPECT_EQ(f.one(telemetryRequest(session, 3, 5000)).result, translagatr::kResultOk);
    EXPECT_EQ(f.bench->attitude_sequence, 1u);
    EXPECT_EQ(f.one(telemetryRequest(session, 4, 5100)).result, translagatr::kResultOk);
    EXPECT_EQ(f.bench->attitude_sequence, 2u);

    // without the attitude group the tilt is unavailable
    EXPECT_EQ(f.one(telemetryRequest(session, 5, 5200, translagatr::kTelemetryMotion)).result,
              translagatr::kResultOk);
    EXPECT_FALSE(f.bench->attitude_valid);

    // a new session starts without attitude
    EXPECT_EQ(f.one(telemetryRequest(session, 6, 5300)).result, translagatr::kResultOk);
    EXPECT_TRUE(f.bench->attitude_valid);
    f.open(7, 0x0BADCAFE);
    EXPECT_FALSE(f.bench->attitude_valid);
}

TEST(BrainLinkTelemetry, OlderPiAnswersUnsupportedOpWhichTheBrainMustTolerate) {
    // The wire contract the Brain relies on: an older Pi without op 12 answers
    // UnsupportedOp, header only. Checked here on the reply codec.
    translagatr::BrainReply r;
    r.op          = translagatr::kOpTelemetry;
    r.session     = 5;
    r.request_id  = 9;
    r.result      = translagatr::kResultUnsupportedOp;
    r.pi_instance = 1;
    uint8_t        buf[translagatr::kMaxFrameLen];
    const uint16_t n = translagatr::encodeBrainReply(r, buf, sizeof(buf));
    ASSERT_EQ(n, translagatr::kLinkEnvelopeLen + translagatr::kBrainReplyHeaderLen);
    translagatr::BrainReply back;
    ASSERT_TRUE(translagatr::decodeBrainReply(buf, n, back));
    EXPECT_EQ(back.result, translagatr::kResultUnsupportedOp);
}

// ---- instrumentation ---------------------------------------------------------

TEST(BrainLinkInstrumentation, MonitorCountsFramesRepliesAndRejections) {
    LinkHarness f;
    f.hub.links().setDecoded(true);
    f.hub.links().setRaw(true);
    const uint32_t session = f.open();
    const std::vector<uint8_t> get = requestBytes(getStateRequest(session, 2));
    f.brain->input().feed(get);
    const std::vector<translagatr::BrainReply> replies = f.cycle();
    ASSERT_EQ(replies.size(), 1u);

    LinkMonitorSnapshot s = brainMonitor(f.hub);
    EXPECT_EQ(s.kind, "brain_serial");
    EXPECT_EQ(s.rx_frames, 2u);   // HELLO, GET_STATE
    EXPECT_EQ(s.tx_frames, 2u);
    EXPECT_EQ(s.tx_attempted, s.tx_accepted);
    EXPECT_EQ(s.reader.frames, 2u);
    EXPECT_EQ(s.rx_bytes, s.reader.bytes);
    ASSERT_GE(s.decoded.size(), 4u);
    EXPECT_EQ(s.decoded[s.decoded.size() - 2].name, "GET_STATE");
    EXPECT_TRUE(s.decoded[s.decoded.size() - 2].rx);
    EXPECT_EQ(s.decoded.back().name, "GET_STATE");
    EXPECT_FALSE(s.decoded.back().rx);
    EXPECT_NE(s.decoded.back().fields.find("result=Ok"), std::string::npos) << s.decoded.back().fields;
    // raw rx holds the request bytes exactly, direction marked
    bool found = false;
    for (const LinkRawChunk& c : s.raw) {
        found = found || (c.dir == DiagDirection::kRx && c.bytes == get);
    }
    EXPECT_TRUE(found);

    // a corrupted request: one CRC rejection, nothing decoded or answered
    std::vector<uint8_t> bad = requestBytes(getStateRequest(session, 3));
    bad[8] ^= 0x01;
    f.brain->input().feed(bad);
    EXPECT_TRUE(f.cycle().empty());
    // a frame that is not a request on the Brain link
    translagatr::PicoStatus status;
    uint8_t                 buf[translagatr::kMaxFrameLen];
    const uint16_t          n = translagatr::encodePicoStatus(status, buf, sizeof(buf));
    f.brain->input().feed(std::vector<uint8_t>(buf, buf + n));
    EXPECT_TRUE(f.cycle().empty());

    s = brainMonitor(f.hub);
    EXPECT_EQ(s.reader.check_errors, 1u);
    EXPECT_EQ(s.rejected, 1u);
    ASSERT_FALSE(s.errors.empty());
    EXPECT_EQ(s.errors.back().reason, "not a brain request frame");
}

TEST(BrainLinkInstrumentation, RequestRecordsCarryWhatWasActuallySent) {
    LinkHarness    f;
    const uint32_t session = f.open();
    f.one(getStateRequest(session, 2));
    DiagRecord record;
    ASSERT_TRUE(f.hub.latest(DiagKind::kBrainRequest, record));
    DiagBrainRequest r = std::get<DiagBrainRequest>(record.payload);
    EXPECT_EQ(r.op, translagatr::kOpGetState);
    EXPECT_EQ(r.request_id, 2);
    EXPECT_EQ(r.result, translagatr::kResultOk);
    EXPECT_EQ(r.request_len, translagatr::kLinkEnvelopeLen + 17);
    EXPECT_EQ(r.reply_len,
              translagatr::kLinkEnvelopeLen + translagatr::kBrainReplyHeaderLen + translagatr::kBrainStateLen);
    EXPECT_FALSE(r.duplicate);

    // the reply window closes before the publisher writes: attempted, not sent
    f.brain->input().feed(requestBytes(getStateRequest(session, 3)));
    const std::vector<translagatr::BrainReply> none =
        f.cycle(5000, [&f] { f.clock_us += 200000; });
    EXPECT_TRUE(none.empty());
    ASSERT_TRUE(f.hub.latest(DiagKind::kBrainRequest, record));
    r = std::get<DiagBrainRequest>(record.payload);
    EXPECT_EQ(r.request_id, 3);
    EXPECT_EQ(r.reply_len, 0);
    const LinkMonitorSnapshot s = brainMonitor(f.hub);
    EXPECT_GT(s.tx_attempted, s.tx_accepted);
    EXPECT_EQ(f.stats().expired, 1u);

    // a request followed by more bytes in the same drain is applied, not answered
    std::vector<uint8_t> trailing = requestBytes(getStateRequest(session, 4));
    trailing.push_back(0xAA);
    f.brain->input().feed(trailing);
    EXPECT_TRUE(f.cycle().empty());
    ASSERT_TRUE(f.hub.latest(DiagKind::kBrainRequest, record));
    r = std::get<DiagBrainRequest>(record.payload);
    EXPECT_EQ(r.request_id, 4);
    EXPECT_EQ(r.reply_len, 0);
}

TEST(BrainLinkInstrumentation, VexImuAndPathReportsArePosted) {
    LinkHarness    f(nullptr, true, true);
    const uint32_t session = f.open();
    translagatr::BrainRequest get = getStateRequest(session, 2);
    get.imu_flags         = translagatr::kBenchImuValid;
    get.imu_stamp_ms      = 777;
    get.imu_rotation_mdeg = -12345;
    f.one(get);
    DiagRecord record;
    ASSERT_TRUE(f.hub.latest(DiagKind::kVexImu, record));
    const DiagVexImu v = std::get<DiagVexImu>(record.payload);
    EXPECT_TRUE(v.accepted);
    EXPECT_EQ(v.stamp_ms, 777u);
    EXPECT_EQ(v.rotation_mdeg, -12345);
    get.request_id = 3;   // same stamp: not a new sample
    f.one(get);
    ASSERT_TRUE(f.hub.latest(DiagKind::kVexImu, record));
    EXPECT_FALSE(std::get<DiagVexImu>(record.payload).accepted);

    f.one(pathRequest(session, 4, 99, translagatr::kPathAvoiding, {{0, 0}, {500, -250}}));
    ASSERT_TRUE(f.hub.latest(DiagKind::kPath, record));
    const DiagPath p = std::get<DiagPath>(record.payload);
    EXPECT_EQ(p.command_id, 99u);
    EXPECT_EQ(p.mode, translagatr::kPathAvoiding);
    ASSERT_EQ(p.count, 2);
    EXPECT_EQ(p.points[1].y_mm, -250);
}

TEST(BrainLinkInstrumentation, WholePiWritesTheInstrumentationObject) {
    PiRig rig(piXml());
    ASSERT_NE(rig.system, nullptr);
    rig.system->diagHub().links().setDecoded(true);
    const uint32_t session = rig.open();
    translagatr::BrainRequest t = request(translagatr::kOpTelemetry, session, 2);
    t.telemetry.flags      = translagatr::kTelemetryAttitude;
    t.telemetry.stamp_ms   = 42;
    t.telemetry.roll_cdeg  = -150;
    t.telemetry.pitch_cdeg = 75;
    EXPECT_EQ(rig.one(t).result, translagatr::kResultOk);
    rig.one(getStateRequest(session, 3));

    JsonWriter w;
    writeInstrumentation(w, *rig.system, InstrumentationOptions{});
    const std::string doc = w.str();
    EXPECT_NE(doc.find("\"id\":\"brain_uart\",\"kind\":\"brain_serial\""), std::string::npos) << doc;
    EXPECT_NE(doc.find("\"id\":\"pico_uart\",\"kind\":\"pico_uart\""), std::string::npos) << doc;
    EXPECT_NE(doc.find("\"telemetry_supported\":true"), std::string::npos) << doc;
    EXPECT_NE(doc.find("\"attitude\":{\"roll_deg\":-1.5,\"pitch_deg\":0.75}"), std::string::npos)
        << doc;
    // no Diagnostics element: never asked, nothing claimed
    EXPECT_NE(doc.find("\"available\":false,\"reason\":\"not requested\""), std::string::npos)
        << doc;
    EXPECT_NE(doc.find("\"diag\":null"), std::string::npos) << doc;
    // raw bytes only when asked for
    EXPECT_NE(doc.find("\"raw\":[]"), std::string::npos);
    EXPECT_NE(doc.find("\"hub\":{\"posted\":{"), std::string::npos);
}
