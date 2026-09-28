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

#include "common/frame_codec.h"
#include "common/link_documents.h"
#include "contracts/brain_profile.h"
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

gatr2::BrainRequest request(uint8_t op, uint32_t session, uint16_t rid) {
    gatr2::BrainRequest r;
    r.op         = op;
    r.session    = session;
    r.request_id = rid;
    return r;
}

gatr2::BrainRequest helloRequest(uint16_t rid, uint32_t nonce) {
    gatr2::BrainRequest r = request(gatr2::kOpHello, 0, rid);
    r.nonce               = nonce;
    return r;
}

gatr2::BrainRequest setPoseRequest(uint32_t session, uint16_t rid, int32_t x_mm, int32_t y_mm,
                                   int32_t heading_cdeg) {
    gatr2::BrainRequest r = request(gatr2::kOpSetPose, session, rid);
    r.x_mm                = x_mm;
    r.y_mm                = y_mm;
    r.heading_cdeg        = heading_cdeg;
    return r;
}

gatr2::BrainRequest getStateRequest(uint32_t session, uint16_t rid) {
    return request(gatr2::kOpGetState, session, rid);
}

gatr2::BrainRequest pathRequest(uint32_t session, uint16_t rid, uint32_t command_id,
                                uint8_t mode, const std::vector<gatr2::PathPoint>& points) {
    gatr2::BrainRequest r = request(gatr2::kOpPathReport, session, rid);
    r.command_id          = command_id;
    r.path_mode           = mode;
    r.point_count         = static_cast<uint8_t>(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        r.points[i] = points[i];
    }
    return r;
}

gatr2::BrainRequest readDocRequest(uint32_t session, uint16_t rid, uint8_t kind) {
    gatr2::BrainRequest r = request(gatr2::kOpReadDoc, session, rid);
    r.doc_kind            = kind;
    r.max_len             = gatr2::kDocChunkMax;
    return r;
}

gatr2::BrainRequest controlRequest(uint32_t session, uint16_t rid, uint8_t action) {
    gatr2::BrainRequest r = request(gatr2::kOpControl, session, rid);
    r.action              = action;
    return r;
}

gatr2::BrainRequest writeRequest(uint32_t session, uint16_t rid, uint32_t profile_id,
                                 const std::vector<uint8_t>& doc, uint16_t offset,
                                 uint16_t length) {
    gatr2::BrainRequest r = request(gatr2::kOpProfileWrite, session, rid);
    r.profile_id          = profile_id;
    r.total_len           = static_cast<uint16_t>(doc.size());
    r.offset              = offset;
    r.data_len            = static_cast<uint8_t>(length);
    std::memcpy(r.data, doc.data() + offset, length);
    return r;
}

gatr2::BrainRequest applyRequest(uint32_t session, uint16_t rid, uint32_t profile_id,
                                 uint16_t total_len) {
    gatr2::BrainRequest r = request(gatr2::kOpProfileApply, session, rid);
    r.profile_id          = profile_id;
    r.total_len           = total_len;
    return r;
}

// Two perpendicular wheels and the Brain VEX IMU. Test values only.
gatr2::RobotProfileDoc benchProfile() {
    gatr2::RobotProfileDoc p;
    p.topology           = gatr2::kTopologyTwoWheelImu;
    p.wheel_count        = 2;
    p.imu_source         = gatr2::kImuSourceBrainVex;
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
constexpr uint16_t kTwoWheelLen   = gatr2::kProfileHeaderLen + 2 * gatr2::kProfileWheelLen;
constexpr uint16_t kThreeWheelLen = gatr2::kProfileHeaderLen + 3 * gatr2::kProfileWheelLen;

std::vector<uint8_t> profileBytes(const gatr2::RobotProfileDoc& p) {
    std::vector<uint8_t> bytes(gatr2::kProfileMaxLen);
    bytes.resize(gatr2::encodeRobotProfile(p, bytes.data(), gatr2::kProfileMaxLen));
    EXPECT_FALSE(bytes.empty());
    return bytes;
}

uint32_t profileId(const std::vector<uint8_t>& doc) {
    return gatr2::crc32(doc.data(), static_cast<uint32_t>(doc.size()));
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

// Stands in for the System: counts calls, accepts or refuses.
struct FakeProfileHost : BrainProfileHost {
    int      calls   = 0;
    uint32_t last_id = 0;
    bool     accept  = true;
    uint8_t  reason  = gatr2::kProfileReasonNone;
    uint8_t  detail  = 0;

    bool prepare(const gatr2::RobotProfileDoc&, uint32_t profile_id, uint8_t& r,
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
        d = gatr2::kControlDetailNone;
        return gatr2::kResultNotReady;
    }

    int controls = 0;
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

    explicit LinkHarness(BrainProfileHost* host = nullptr, bool idle_first = true) {
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
        context.brain_profile         = host;

        commands = make<CommandsMakeFunction>(kCommandsXml, "brain_link", err);
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

    // Whole document in chunks of at most chunk bytes; the last reply.
    gatr2::BrainReply stage(uint32_t session, uint16_t& rid, const std::vector<uint8_t>& bytes,
                            uint16_t chunk = gatr2::kProfileChunkMax) {
        const uint32_t    id = profileId(bytes);
        gatr2::BrainReply reply;
        for (std::size_t offset = 0; offset < bytes.size(); offset += chunk) {
            const std::size_t n = std::min<std::size_t>(chunk, bytes.size() - offset);
            reply = one(writeRequest(session, rid++, id, bytes, static_cast<uint16_t>(offset),
                                     static_cast<uint16_t>(n)));
            EXPECT_EQ(reply.result, gatr2::kResultOk);
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

TEST(BrainLinkSession, HelloOpensASessionAndKeepsThePiSideState) {
    LinkHarness f;
    f.command.path.mode          = gatr2::kPathDirect;   // left over from an earlier session
    f.command.profile.state      = gatr2::kProfileApplied;
    f.command.profile.id         = 0x1234;
    f.command.profile.applied_id = 0x1234;

    const gatr2::BrainReply reply = f.one(helloRequest(1, 0xCAFE));
    EXPECT_EQ(reply.version, gatr2::kBrainLinkVersion);
    EXPECT_EQ(reply.op, gatr2::kOpHello);
    EXPECT_EQ(reply.request_id, 1u);
    EXPECT_EQ(reply.result, gatr2::kResultOk);
    EXPECT_EQ(reply.nonce, 0xCAFEu);
    EXPECT_NE(reply.session, 0u);
    EXPECT_NE(reply.pi_instance, 0u);
    EXPECT_EQ(f.command.session, reply.session);
    EXPECT_EQ(f.command.path.mode, gatr2::kPathNone);   // the old path is gone
    EXPECT_EQ(f.command.init_sequence, 0u);             // never touches placement
    EXPECT_EQ(f.command.profile.applied_id, 0x1234u);   // nor the profile
    EXPECT_EQ(f.command.profile.state, gatr2::kProfileApplied);
}

TEST(BrainLinkSession, HelloRetryIsIdempotentUntilTheSessionIsUsed) {
    LinkHarness    f;
    const uint32_t session = f.open(1, 0xAB);
    f.command.path.mode    = gatr2::kPathDirect;

    const gatr2::BrainReply retry = f.one(helloRequest(1, 0xAB));
    EXPECT_EQ(retry.result, gatr2::kResultOk);
    EXPECT_EQ(retry.session, session);
    EXPECT_EQ(f.command.path.mode, gatr2::kPathDirect);   // nothing changed

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
    const gatr2::BrainReply reply =
        f.one(pathRequest(session + 1, 2, 5, gatr2::kPathDirect, {{0, 0}, {100, 0}}));
    EXPECT_EQ(reply.result, gatr2::kResultUnknownSession);
    EXPECT_EQ(reply.session, session + 1);   // echo
    EXPECT_EQ(f.command.path.mode, gatr2::kPathNone);
    EXPECT_EQ(f.stats().unknown_session, 2u);
}

TEST(BrainLinkSession, VersionAndRetiredOpErrors) {
    LinkHarness    f;
    const uint32_t session = f.open();

    gatr2::BrainRequest old   = getStateRequest(session, 2);
    old.version               = 3;
    const gatr2::BrainReply v = f.one(old);
    EXPECT_EQ(v.result, gatr2::kResultUnsupportedVersion);
    EXPECT_EQ(v.version, gatr2::kBrainLinkVersion);   // the Pi's version

    // 3 and 5 are the retired v3 landmark select and IMU state ops; 12 is not defined
    for (uint8_t op : {uint8_t{3}, uint8_t{5}, uint8_t{12}}) {
        gatr2::BrainRequest unknown = getStateRequest(session, 3);
        unknown.op                  = op;
        const gatr2::BrainReply r   = f.one(unknown);
        EXPECT_EQ(r.result, gatr2::kResultUnsupportedOp);
        EXPECT_EQ(r.op, op);
    }

    EXPECT_EQ(f.one(getStateRequest(session, 0)).result, gatr2::kResultInvalidArgument);
}

TEST(BrainLinkSession, MalformedBodiesAreInvalidAndConsumeNothing) {
    LinkHarness                      f;
    const uint32_t                   session = f.open();
    const std::vector<uint8_t>       doc     = profileBytes(benchProfile());
    std::vector<gatr2::BrainRequest> bad;

    gatr2::BrainRequest state = getStateRequest(session, 2);
    state.imu_flags           = 0x02;   // unknown bit
    bad.push_back(state);
    bad.push_back(readDocRequest(session, 2, 3));   // unknown kind
    gatr2::BrainRequest empty = readDocRequest(session, 2, gatr2::kDocFieldMap);
    empty.max_len             = 0;
    bad.push_back(empty);
    bad.push_back(controlRequest(session, 2, 0));
    bad.push_back(controlRequest(session, 2, 5));
    bad.push_back(pathRequest(session, 2, 1, 3, {}));
    gatr2::BrainRequest past_end = writeRequest(session, 2, profileId(doc), doc, 40, 32);
    past_end.total_len           = 60;   // offset + length over total_len
    bad.push_back(past_end);
    gatr2::BrainRequest too_long = writeRequest(session, 2, profileId(doc), doc, 0, 40);
    too_long.total_len           = gatr2::kProfileMaxLen + 1;
    bad.push_back(too_long);
    bad.push_back(applyRequest(session, 2, 1, gatr2::kProfileHeaderLen - 1));

    for (const gatr2::BrainRequest& r : bad) {
        SCOPED_TRACE(static_cast<int>(r.op));
        EXPECT_EQ(f.one(r).result, gatr2::kResultInvalidArgument);
    }
    // none of them was recorded as the newest request
    EXPECT_EQ(f.one(getStateRequest(session, 2)).result, gatr2::kResultOk);
}

TEST(BrainLinkSession, WheelReadingsAndPicoControlWaitForTheProfileBoundary) {
    LinkHarness    plain;   // no profile host: this configuration never serves readings
    const uint32_t a = plain.open();
    EXPECT_EQ(plain.one(request(gatr2::kOpReadWheels, a, 2)).result, gatr2::kResultUnavailable);
    EXPECT_EQ(plain.one(request(gatr2::kOpReadWheels, a, 2)).result,
              gatr2::kResultUnavailable);   // read-only: the newest id is answered again

    FakeProfileHost host;
    LinkHarness     hosted(&host);
    const uint32_t  b   = hosted.open();
    uint16_t        rid = 2;
    EXPECT_EQ(hosted.one(request(gatr2::kOpReadWheels, b, rid++)).result,
              gatr2::kResultNotReady);
    for (uint8_t action : {gatr2::kControlReinitImu, gatr2::kControlRestartAcquisition}) {
        const gatr2::BrainReply r = hosted.one(controlRequest(b, rid++, action));
        EXPECT_EQ(r.result, gatr2::kResultNotReady);   // header only, no action echo
        EXPECT_EQ(r.op, gatr2::kOpControl);
    }
}

// ---- dedupe ------------------------------------------------------------------

TEST(BrainLinkDedupe, SetPoseDuplicatesAnsweredNeverReapplied) {
    LinkHarness    f;
    const uint32_t session = f.open();

    const gatr2::BrainReply pose = f.one(setPoseRequest(session, 3, 610, 457, 9000));
    EXPECT_EQ(pose.result, gatr2::kResultPending);   // nothing here applies placements
    EXPECT_EQ(f.command.init_sequence, 1u);
    EXPECT_EQ(f.command.init_session, session);
    EXPECT_NEAR(f.command.init_pose.x_m, 0.610, 1e-12);
    EXPECT_NEAR(f.command.init_pose.heading_rad, kPi / 2.0, 1e-9);

    // a retry after newer state polls answers from the record
    EXPECT_EQ(f.one(getStateRequest(session, 4)).result, gatr2::kResultOk);
    EXPECT_EQ(f.one(setPoseRequest(session, 3, 610, 457, 9000)).result, gatr2::kResultPending);
    EXPECT_EQ(f.command.init_sequence, 1u);
    EXPECT_EQ(f.stats().duplicates, 1u);
    EXPECT_EQ(f.one(setPoseRequest(session, 3, 611, 457, 9000)).result,
              gatr2::kResultInvalidArgument);
    EXPECT_EQ(f.one(getStateRequest(session, 3)).result, gatr2::kResultInvalidArgument);
    EXPECT_EQ(f.command.init_sequence, 1u);
}

TEST(BrainLinkDedupe, ControlIsAnsweredFromItsRecord) {
    LinkHarness    f;
    const uint32_t session = f.open();

    const gatr2::BrainReply first = f.one(controlRequest(session, 2, gatr2::kControlRecalibrate));
    EXPECT_EQ(first.result, gatr2::kResultNotReady);   // no calibration control yet
    EXPECT_EQ(f.one(getStateRequest(session, 3)).result, gatr2::kResultOk);
    EXPECT_EQ(f.one(controlRequest(session, 2, gatr2::kControlRecalibrate)).result,
              gatr2::kResultNotReady);
    EXPECT_EQ(f.stats().duplicates, 1u);
    EXPECT_EQ(f.one(controlRequest(session, 2, gatr2::kControlReinitialize)).result,
              gatr2::kResultInvalidArgument);
}

TEST(BrainLinkDedupe, NewestIdempotentRequestIsAnsweredAgain) {
    LinkHarness    f;
    const uint32_t session = f.open();

    f.robot.odom_pose = Pose2D{1.0, 0.0, 0.0};
    EXPECT_EQ(f.one(getStateRequest(session, 2)).state.x_mm, 1000);
    f.robot.odom_pose = Pose2D{1.5, 0.0, 0.0};
    const gatr2::BrainReply again = f.one(getStateRequest(session, 2));
    EXPECT_EQ(again.result, gatr2::kResultOk);
    EXPECT_EQ(again.state.x_mm, 1500);   // fresh state for the resend

    gatr2::BrainRequest other_body = getStateRequest(session, 2);
    other_body.imu_flags           = gatr2::kBenchImuValid;
    EXPECT_EQ(f.one(other_body).result, gatr2::kResultInvalidArgument);
    EXPECT_EQ(f.one(setPoseRequest(session, 2, 0, 0, 0)).result,
              gatr2::kResultInvalidArgument);
    EXPECT_EQ(f.command.init_sequence, 0u);

    const gatr2::BrainRequest path = pathRequest(session, 3, 9, gatr2::kPathDirect, {{1, 2}});
    EXPECT_EQ(f.one(path).result, gatr2::kResultOk);
    f.command.path = PathReport{};
    EXPECT_EQ(f.one(path).result, gatr2::kResultOk);   // stored again, same content
    EXPECT_EQ(f.command.path.command_id, 9u);
    EXPECT_EQ(f.one(pathRequest(session, 3, 9, gatr2::kPathDirect, {{1, 3}})).result,
              gatr2::kResultInvalidArgument);
    EXPECT_DOUBLE_EQ(f.command.path.points[0].y_m, 0.002);
    EXPECT_EQ(f.stats().duplicates, 0u);   // resends of the newest are not records
}

TEST(BrainLinkDedupe, OlderIdIsStaleAndIdsWrap) {
    LinkHarness    f;
    const uint32_t session = f.open(65534);

    EXPECT_EQ(f.one(getStateRequest(session, 65535)).result, gatr2::kResultOk);
    EXPECT_EQ(f.one(getStateRequest(session, 1)).result, gatr2::kResultOk);   // wrapped
    EXPECT_EQ(f.one(pathRequest(session, 2, 1, gatr2::kPathDirect, {{0, 0}})).result,
              gatr2::kResultOk);
    EXPECT_EQ(f.one(getStateRequest(session, 65535)).result, gatr2::kResultStale);
    EXPECT_EQ(f.one(pathRequest(session, 1, 1, gatr2::kPathNone, {})).result,
              gatr2::kResultStale);
    EXPECT_EQ(f.command.path.mode, gatr2::kPathDirect);   // the stale clear never applied
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
    LinkHarness f(nullptr, false);
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
    f.brain->input().feed(requestBytes(setPoseRequest(session, 2, 610, 457, 9000)));
    EXPECT_TRUE(f.cycle(50000).empty());
    EXPECT_EQ(f.stats().expired, 1u);
    EXPECT_EQ(f.command.init_sequence, 1u);   // applied; the retry is a duplicate

    const gatr2::BrainReply retry = f.one(setPoseRequest(session, 2, 610, 457, 9000));
    EXPECT_EQ(retry.result, gatr2::kResultPending);
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

    std::vector<uint8_t> bytes =
        requestBytes(pathRequest(session, 2, 20, gatr2::kPathDirect, {{0, 0}}));
    const std::vector<uint8_t> newer =
        requestBytes(pathRequest(session, 3, 30, gatr2::kPathAvoiding, {{0, 0}, {5, 5}}));
    bytes.insert(bytes.end(), newer.begin(), newer.end());
    f.brain->input().feed(bytes);
    const std::vector<gatr2::BrainReply> replies = f.cycle();
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].request_id, 3u);
    EXPECT_EQ(f.command.path.command_id, 30u);   // never rid 2
    EXPECT_EQ(f.stats().superseded, 1u);
    EXPECT_EQ(f.one(pathRequest(session, 2, 20, gatr2::kPathDirect, {{0, 0}})).result,
              gatr2::kResultStale);
    EXPECT_EQ(f.command.path.command_id, 30u);
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

TEST(BrainLinkBus, LargestRequestAndReplyFitTheFrame) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> path =
        requestBytes(pathRequest(session, 2, 1, gatr2::kPathAvoiding,
                                 std::vector<gatr2::PathPoint>(gatr2::kPathReportMaxPoints)));
    EXPECT_EQ(path.size(), 6u + 8u + 6u + 8u * gatr2::kPathReportMaxPoints);
    f.brain->input().feed(path);
    ASSERT_EQ(f.cycle().size(), 1u);
    EXPECT_EQ(f.command.path.count, gatr2::kPathReportMaxPoints);

    std::vector<uint8_t> big(gatr2::kProfileMaxLen, 0);
    big[0] = gatr2::kProfileFormat;
    const gatr2::BrainRequest write =
        writeRequest(session, 3, profileId(big), big, 0, gatr2::kProfileChunkMax);
    EXPECT_EQ(requestBytes(write).size(), gatr2::kMaxFrameLen);
    const gatr2::BrainReply   staged = f.one(write);
    EXPECT_EQ(staged.result, gatr2::kResultOk);
    EXPECT_EQ(staged.received, gatr2::kProfileChunkMax);
}

// ---- reply bodies ------------------------------------------------------------

TEST(BrainLinkState, ProfileStatusAndNoFieldDocuments) {
    LinkHarness    f;
    const uint32_t session = f.open();

    gatr2::BrainReply s = f.one(getStateRequest(session, 2));
    EXPECT_EQ(s.state.profile_state, gatr2::kProfileNone);
    EXPECT_EQ(s.state.profile_id, 0u);
    EXPECT_EQ(s.state.map_id, 0u);
    EXPECT_EQ(s.state.estimate_id, 0u);
    EXPECT_EQ(s.state.calibration, gatr2::kCalibrationNone);

    f.command.profile = ProfileStatus{gatr2::kProfileRejected, gatr2::kProfileReasonEncoderPort,
                                      1, 0xABCD, 0x1234};
    s = f.one(getStateRequest(session, 3));
    EXPECT_EQ(s.state.profile_state, gatr2::kProfileRejected);
    EXPECT_EQ(s.state.profile_reason, gatr2::kProfileReasonEncoderPort);
    EXPECT_EQ(s.state.profile_detail, 1u);
    EXPECT_EQ(s.state.profile_id, 0xABCDu);   // the refused id, not the running one

    EXPECT_EQ(f.one(readDocRequest(session, 4, gatr2::kDocFieldMap)).result,
              gatr2::kResultUnavailable);
    EXPECT_EQ(f.one(readDocRequest(session, 5, gatr2::kDocFieldEstimate)).result,
              gatr2::kResultUnavailable);
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

TEST(BrainLinkState, PathReportIsKeptForInspectionUntilClearedOrANewSession) {
    LinkHarness    f;
    const uint32_t session = f.open();

    std::vector<gatr2::PathPoint> points;
    for (int32_t i = 0; i < gatr2::kPathReportMaxPoints; ++i) {
        points.push_back({i * 100, -i * 50});
    }
    f.one(pathRequest(session, 2, 77, gatr2::kPathAvoiding, points));
    const PathReport& path = f.command.path;
    EXPECT_EQ(path.session, session);
    EXPECT_EQ(path.command_id, 77u);
    EXPECT_EQ(path.mode, gatr2::kPathAvoiding);
    ASSERT_EQ(path.count, gatr2::kPathReportMaxPoints);
    EXPECT_DOUBLE_EQ(path.points[12].x_m, 1.2);
    EXPECT_DOUBLE_EQ(path.points[12].y_m, -0.6);
    EXPECT_EQ(path.received.ms, f.now_ms);

    f.one(pathRequest(session, 3, 77, gatr2::kPathNone, {}));
    EXPECT_EQ(f.command.path.mode, gatr2::kPathNone);
    EXPECT_EQ(f.command.path.count, 0u);

    f.one(pathRequest(session, 4, 78, gatr2::kPathDirect, {{1, 1}}));
    f.open(1, 0xBEEF);
    EXPECT_EQ(f.command.path.mode, gatr2::kPathNone);
}

// ---- robot profile -----------------------------------------------------------

TEST(BrainLinkProfile, WritesStageContiguousBytesAndResendsAreIdempotent) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    ASSERT_EQ(doc.size(), kTwoWheelLen);
    uint16_t rid = 2;

    gatr2::BrainReply r = f.one(writeRequest(session, rid++, id, doc, 0, 40));
    EXPECT_EQ(r.result, gatr2::kResultOk);
    EXPECT_EQ(r.profile_id, id);
    EXPECT_EQ(r.received, 40u);
    EXPECT_EQ(f.one(writeRequest(session, rid++, id, doc, 0, 40)).received, 40u);   // resend
    EXPECT_EQ(f.one(writeRequest(session, rid++, id, doc, 20, 40)).received, 60u);   // overlap

    EXPECT_EQ(f.one(writeRequest(session, rid++, id, doc, 70, 2)).result,
              gatr2::kResultInvalidArgument);   // a gap
    std::vector<uint8_t> other = doc;
    other[10] ^= 0xFF;
    EXPECT_EQ(f.one(writeRequest(session, rid++, id, other, 0, 20)).result,
              gatr2::kResultInvalidArgument);   // a resend with other bytes
    EXPECT_EQ(f.one(applyRequest(session, rid++, id, kTwoWheelLen)).result,
              gatr2::kResultInvalidArgument);   // incomplete

    // staging survives a new session
    const uint32_t next = f.open(1, 0xFEED);
    rid                 = 2;
    r                   = f.one(writeRequest(next, rid++, id, doc, 60, kTwoWheelLen - 60));
    EXPECT_EQ(r.received, kTwoWheelLen);
    EXPECT_EQ(f.one(applyRequest(next, rid++, id, kTwoWheelLen)).result, gatr2::kResultProfileRejected);

    // another id restarts staging, but only from offset 0
    gatr2::RobotProfileDoc second = benchProfile();
    second.wheels[0].counts_per_rev = 8192;
    const std::vector<uint8_t> doc2 = profileBytes(second);
    EXPECT_EQ(f.one(writeRequest(next, rid++, profileId(doc2), doc2, 40, 32)).result,
              gatr2::kResultInvalidArgument);
    EXPECT_EQ(f.one(applyRequest(next, rid++, id, kTwoWheelLen)).result,
              gatr2::kResultProfileRejected);   // the first is still staged
    EXPECT_EQ(f.one(writeRequest(next, rid++, profileId(doc2), doc2, 0, 32)).received, 32u);
    EXPECT_EQ(f.one(applyRequest(next, rid++, id, kTwoWheelLen)).result, gatr2::kResultInvalidArgument);
}

TEST(BrainLinkProfile, ApplyNeedsTheWholeDocumentUnderItsCrc) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    uint16_t                   rid     = 2;

    EXPECT_EQ(f.one(applyRequest(session, rid++, id, kTwoWheelLen)).result,
              gatr2::kResultInvalidArgument);   // nothing staged
    f.stage(session, rid, doc);
    EXPECT_EQ(f.one(applyRequest(session, rid++, id, kThreeWheelLen)).result,
              gatr2::kResultInvalidArgument);   // another length
    EXPECT_EQ(f.one(applyRequest(session, rid++, id + 1, kTwoWheelLen)).result,
              gatr2::kResultInvalidArgument);   // another id

    // bytes staged under an id that is not their crc
    const uint32_t wrong = id ^ 0x5A5A5A5A;
    EXPECT_EQ(f.one(writeRequest(session, rid++, wrong, doc, 0, kTwoWheelLen)).result, gatr2::kResultOk);
    EXPECT_EQ(f.one(applyRequest(session, rid++, wrong, kTwoWheelLen)).result,
              gatr2::kResultInvalidArgument);
    EXPECT_EQ(f.command.profile.state, gatr2::kProfileNone);
}

TEST(BrainLinkProfile, AConfigurationWithoutAProfileHostRefusesEveryProfile) {
    LinkHarness                f;
    const uint32_t             session = f.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    uint16_t                   rid     = 2;
    f.stage(session, rid, doc, 30);

    const gatr2::BrainReply r = f.one(applyRequest(session, rid++, id, kTwoWheelLen));
    EXPECT_EQ(r.result, gatr2::kResultProfileRejected);
    EXPECT_EQ(r.profile_id, id);
    EXPECT_EQ(r.profile_state, gatr2::kProfileRejected);
    EXPECT_EQ(r.profile_reason, gatr2::kProfileReasonNotAccepted);

    const gatr2::BrainReply s = f.one(getStateRequest(session, rid++));
    EXPECT_EQ(s.state.profile_state, gatr2::kProfileRejected);
    EXPECT_EQ(s.state.profile_reason, gatr2::kProfileReasonNotAccepted);
    EXPECT_EQ(s.state.profile_id, id);

    // the XML localization keeps accepting placements
    EXPECT_EQ(f.one(setPoseRequest(session, rid++, 1, 2, 3)).result, gatr2::kResultPending);
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
    EXPECT_EQ(f.one(setPoseRequest(session, rid, 610, 457, 0)).result, gatr2::kResultNotReady);
    EXPECT_EQ(f.one(setPoseRequest(session, rid++, 610, 457, 0)).result, gatr2::kResultNotReady);
    EXPECT_EQ(f.command.init_sequence, 0u);

    f.stage(session, rid, doc);
    gatr2::BrainReply r = f.one(applyRequest(session, rid++, id, kTwoWheelLen));
    EXPECT_EQ(r.result, gatr2::kResultPending);
    EXPECT_EQ(r.profile_state, gatr2::kProfileApplying);
    EXPECT_EQ(host.calls, 1);
    EXPECT_EQ(host.last_id, id);
    EXPECT_EQ(f.one(getStateRequest(session, rid++)).state.profile_state,
              gatr2::kProfileApplying);
    EXPECT_EQ(f.one(applyRequest(session, rid++, id, kTwoWheelLen)).result, gatr2::kResultPending);
    EXPECT_EQ(host.calls, 1);   // built once
    EXPECT_EQ(f.one(setPoseRequest(session, rid++, 610, 457, 0)).result,
              gatr2::kResultNotReady);

    // the System swaps it in at its boundary
    f.command.profile.state      = gatr2::kProfileApplied;
    f.command.profile.applied_id = id;
    r                            = f.one(applyRequest(session, rid++, id, kTwoWheelLen));
    EXPECT_EQ(r.result, gatr2::kResultOk);
    EXPECT_EQ(r.profile_state, gatr2::kProfileApplied);
    EXPECT_EQ(host.calls, 1);   // idempotent: nothing rebuilt or reset
    EXPECT_EQ(f.one(setPoseRequest(session, rid++, 610, 457, 0)).result, gatr2::kResultPending);
    EXPECT_EQ(f.command.init_sequence, 1u);

    // a new Brain session keeps the applied profile and its placement gate open
    const uint32_t next = f.open(1, 0xB007);
    EXPECT_EQ(f.one(setPoseRequest(next, 2, 610, 457, 0)).result, gatr2::kResultPending);
}

TEST(BrainLinkProfile, RejectionsCarryReasonsAndAreRemembered) {
    FakeProfileHost host;
    LinkHarness     f(&host);
    const uint32_t  session = f.open();
    uint16_t        rid     = 2;

    // the shared semantic check runs before the host
    gatr2::RobotProfileDoc three_vex = benchProfile();
    three_vex.topology               = gatr2::kTopologyThreeWheel;
    three_vex.wheel_count            = 3;
    three_vex.wheels[2]              = {2, 0, 4000, 24000, -100000, 0, 90000};
    const std::vector<uint8_t> a     = profileBytes(three_vex);
    f.stage(session, rid, a);
    gatr2::BrainReply r = f.one(applyRequest(session, rid++, profileId(a), kThreeWheelLen));
    EXPECT_EQ(r.result, gatr2::kResultProfileRejected);
    EXPECT_EQ(r.profile_reason, gatr2::kProfileReasonImuCombination);
    EXPECT_EQ(host.calls, 0);

    // undecodable bytes
    std::vector<uint8_t> junk(gatr2::kProfileHeaderLen, 0);
    junk[0] = 9;
    f.stage(session, rid, junk);
    r = f.one(applyRequest(session, rid++, profileId(junk), gatr2::kProfileHeaderLen));
    EXPECT_EQ(r.profile_reason, gatr2::kProfileReasonFormat);

    // this Pi's capability check, remembered for the id
    host.accept                    = false;
    host.reason                    = gatr2::kProfileReasonEncoderPort;
    host.detail                    = 1;
    gatr2::RobotProfileDoc unwired = benchProfile();
    unwired.wheels[1].encoder_port = 2;
    const std::vector<uint8_t> b   = profileBytes(unwired);
    f.stage(session, rid, b);
    r = f.one(applyRequest(session, rid++, profileId(b), kTwoWheelLen));
    EXPECT_EQ(r.profile_reason, gatr2::kProfileReasonEncoderPort);
    EXPECT_EQ(r.profile_detail, 1u);
    EXPECT_EQ(host.calls, 1);
    host.accept = true;
    r           = f.one(applyRequest(session, rid++, profileId(b), kTwoWheelLen));
    EXPECT_EQ(r.result, gatr2::kResultProfileRejected);
    EXPECT_EQ(r.profile_reason, gatr2::kProfileReasonEncoderPort);
    EXPECT_EQ(host.calls, 1);   // no retry storm

    const gatr2::BrainReply s = f.one(getStateRequest(session, rid++));
    EXPECT_EQ(s.state.profile_state, gatr2::kProfileRejected);
    EXPECT_EQ(s.state.profile_id, profileId(b));
    EXPECT_EQ(s.state.profile_detail, 1u);
    EXPECT_EQ(f.command.profile.applied_id, 0u);

    // a good profile still applies afterwards
    const std::vector<uint8_t> good = profileBytes(benchProfile());
    f.stage(session, rid, good);
    EXPECT_EQ(f.one(applyRequest(session, rid++, profileId(good), kTwoWheelLen)).result,
              gatr2::kResultPending);
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

TEST(BrainLinkSystem, XmlConfiguredPiRefusesAProfileAndKeepsLocalizing) {
    PiRig rig(piXml());
    ASSERT_NE(rig.system, nullptr);
    const uint32_t             session = rig.open();
    const std::vector<uint8_t> doc     = profileBytes(benchProfile());
    const uint32_t             id      = profileId(doc);
    uint16_t                   rid     = 2;
    EXPECT_EQ(rig.one(writeRequest(session, rid++, id, doc, 0, kTwoWheelLen)).received, kTwoWheelLen);

    const gatr2::BrainReply refused = rig.one(applyRequest(session, rid++, id, kTwoWheelLen));
    EXPECT_EQ(refused.result, gatr2::kResultProfileRejected);
    EXPECT_EQ(refused.profile_reason, gatr2::kProfileReasonNotAccepted);
    EXPECT_EQ(rig.one(setPoseRequest(session, rid++, 610, 457, 9000)).result,
              gatr2::kResultOk);
    const gatr2::BrainReply s = rig.one(getStateRequest(session, rid++));
    EXPECT_TRUE(s.state.robot_flags & gatr2::kRobotLocalized);
    EXPECT_EQ(s.state.profile_state, gatr2::kProfileRejected);
    EXPECT_EQ(s.state.map_id, 0u);
    EXPECT_EQ(rig.one(readDocRequest(session, rid++, gatr2::kDocFieldMap)).result,
              gatr2::kResultUnavailable);
    EXPECT_FALSE(rig.system->target().active);   // no Brain op selects a target
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

    const std::vector<uint8_t> doc = profileBytes(benchProfile());
    EXPECT_EQ(first.one(writeRequest(a.session, 2, profileId(doc), doc, 0, kTwoWheelLen)).received, kTwoWheelLen);

    first.system->reset();
    EXPECT_TRUE(first.step().empty());   // first drain after reset: no reply
    const gatr2::BrainReply old = first.one(getStateRequest(a.session, 3));
    EXPECT_EQ(old.result, gatr2::kResultUnknownSession);
    EXPECT_NE(old.pi_instance, a.pi_instance);
    EXPECT_EQ(first.system->command().init_sequence, 0u);

    // power-on state: staging is gone too
    const uint32_t session = first.open(1, 0x2);
    EXPECT_EQ(first.one(applyRequest(session, 2, profileId(doc), kTwoWheelLen)).result,
              gatr2::kResultInvalidArgument);
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
