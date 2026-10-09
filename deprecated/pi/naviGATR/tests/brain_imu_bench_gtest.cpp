#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include "translaGATR/frame_codec.h"
#include "config/composition.h"
#include "impl/resources/serial_links.h"
#include "inspection/inspection_document.h"
#include "math/angles.h"
#include "resources/brain_imu_bench.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;
namespace {
struct BenchWheel {
    double x, y, angle;
    const char* direction = "positive";
};
using BenchGeometry = std::array<BenchWheel, 2>;
constexpr BenchGeometry kParallelGeometry{{{0, 0.15, 0}, {0, -0.15, 0}}};
constexpr BenchGeometry kPerpendicularGeometry{{{0, 0.15, 0}, {0.10, 0, 90}}};

struct BenchRig {
    FunctionRegistry functions;
    std::unique_ptr<System> system;
    MemoryLink *brain = nullptr, *pico = nullptr;
    uint32_t session = 0;
    uint16_t rid = 1;
    uint8_t seq = 0;
    int64_t now = 1000;
    std::shared_ptr<BrainImuBench> imu;
    bool attitude = false;   // fold the TELEMETRY tilt as vex_attitude

    std::string configuration(bool enabled, const BenchGeometry& geometry,
                              const char* observation_type) {
        ResolvedConfiguration c;
        std::string err;
        if (!resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) +
            "/override/diagnostics/bench_vex_imu.xml", c, err)) { ADD_FAILURE() << err; return {}; }
        for (std::size_t p = 0; (p = c.xml.find("linux_serial_link", p)) != std::string::npos;)
            c.xml.replace(p, 17, "memory_link");
        if (!enabled) {
            const auto start = c.xml.find("<BenchImu");
            const auto end = c.xml.find("/>", start);
            c.xml.erase(start, end + 2 - start);
        }
        // The bench profile is user-editable. Keep the wire/configuration path,
        // but give this test fixed geometry and encoder calibration.
        tinyxml2::XMLDocument doc;
        if (doc.Parse(c.xml.c_str()) != tinyxml2::XML_SUCCESS) return {};
        auto* resources = doc.RootElement()->FirstChildElement("Resources");
        auto* sensors = doc.RootElement()->FirstChildElement("Sensors");
        if (!resources || !sensors) return {};
        for (auto* resource = resources->FirstChildElement("Resource"); resource;
             resource = resource->NextSiblingElement("Resource")) {
            if (ConfigNode{resource}.attr("id") != "wheel_geometry") continue;
            for (auto* wheel = resource->FirstChildElement("Wheel"); wheel;
                 wheel = wheel->NextSiblingElement("Wheel")) {
                const auto& spec = geometry[ConfigNode{wheel}.attr("id") == "forward_wheel_a" ? 0 : 1];
                wheel->SetAttribute("radius_m", 0.0254);
                wheel->SetAttribute("position_x_m", spec.x);
                wheel->SetAttribute("position_y_m", spec.y);
                wheel->SetAttribute("measurement_angle_deg", spec.angle);
                wheel->SetAttribute("direction", spec.direction);
            }
        }
        for (auto* sensor = sensors->FirstChildElement("Sensor"); sensor;
             sensor = sensor->NextSiblingElement("Sensor")) {
            if (ConfigNode{sensor}.attr("type") != "pico_encoder_channel") continue;
            auto* calibration = sensor->FirstChildElement("Calibration");
            if (!calibration) return {};
            calibration->SetAttribute("counts_per_revolution", 4000);
            calibration->SetAttribute("invert", "false");
        }
        auto* pipeline = doc.RootElement()->FirstChildElement("Pipeline");
        auto* localization = pipeline ? pipeline->FirstChildElement("Localization") : nullptr;
        auto* observation = localization ? localization->FirstChildElement("Observation") : nullptr;
        if (!observation) return {};
        observation->SetAttribute("type", observation_type);
        if (attitude) {
            auto* output = doc.NewElement("Attitude");
            output->SetAttribute("observation_id", "vex_attitude");
            observation->InsertEndChild(output);
            auto* estimator = localization->FirstChildElement("Estimator");
            if (!estimator) return {};
            auto* fold = doc.NewElement("Attitude");
            fold->SetAttribute("observation_id", "vex_attitude");
            fold->SetAttribute("max_age_ms", 250);
            estimator->InsertEndChild(fold);
        }
        tinyxml2::XMLPrinter printer;
        doc.Print(&printer);
        return printer.CStr();
    }
    bool build(bool enabled = true, const BenchGeometry& geometry = kParallelGeometry,
               const char* observation_type = "brain_imu_parallel_bench") {
        registerAll(functions);
        const std::string xml = configuration(enabled, geometry, observation_type);
        if (xml.empty()) return false;
        std::string err;
        system = System::buildFromString(xml.c_str(), functions, err);
        if (!system) { ADD_FAILURE() << err; return false; }
        auto b = system->resources().require<SerialLink>(ResourceId{"brain_uart"}, err);
        auto p = system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
        brain = dynamic_cast<MemoryLink*>(b.get());
        pico = dynamic_cast<MemoryLink*>(p.get());
        imu = system->resources().require<BrainImuBench>(ResourceId{"brain_imu"}, err);
        if (!brain || !pico || !imu) return false;
        brain->setClock([this] { return now * 1000; });
        system->step(hostTime(now));
        translagatr::BrainRequest hello;
        hello.op = translagatr::kOpHello;
        hello.nonce = 12345;
        hello.request_id = rid++;
        session = request(hello).session;
        return session != 0;
    }
    void wheels(int a, int b) {
        translagatr::SensorSample f{};
        f.seq = seq++;
        f.stamp_ms = static_cast<uint32_t>(now);
        f.mask = translagatr::kSensorEnc0 | translagatr::kSensorEnc1;
        f.enc[0] = a; f.enc[1] = b;
        std::vector<uint8_t> bytes(translagatr::kMaxFrameLen);
        bytes.resize(translagatr::encodeSensorFrame(f, bytes.data(), translagatr::kMaxFrameLen));
        pico->input().feed(bytes);
    }
    translagatr::BrainReply request(translagatr::BrainRequest r) {
        std::vector<uint8_t> bytes(translagatr::kMaxFrameLen);
        bytes.resize(translagatr::encodeBrainRequest(r, bytes.data(), translagatr::kMaxFrameLen));
        brain->input().feed(bytes);
        now += 20;
        system->step(hostTime(now));
        translagatr::FrameReader reader;
        translagatr::BrainReply reply;
        bool got = false;
        for (uint8_t byte : brain->output().takeAll()) {
            if (reader.push(byte)) got = translagatr::decodeBrainReply(reader.frame(), reader.frameLen(), reply);
        }
        EXPECT_TRUE(got);
        return reply;
    }
    translagatr::BrainRequest imuRequest(uint32_t stamp, int32_t angle = 0, bool valid = true) {
        translagatr::BrainRequest r;
        r.op = translagatr::kOpGetState;
        r.session = session;
        r.request_id = rid++;
        r.imu_flags = valid ? translagatr::kBenchImuValid : 0;
        r.imu_stamp_ms = stamp;
        r.imu_rotation_mdeg = angle;
        return r;
    }
};
}

TEST(BrainImuBench, WirePathPlacementStraightTurnAndInspection) {
    BenchRig r;
    ASSERT_TRUE(r.build());
    r.wheels(0, 0);
    EXPECT_EQ(r.request(r.imuRequest(100)).result, translagatr::kResultOk);
    translagatr::BrainRequest place;
    place.op = translagatr::kOpSetPose; place.session = r.session; place.request_id = r.rid++;
    place.x_mm = 1000;
    r.request(place);
    r.wheels(0, 0);
    r.request(r.imuRequest(120));
    // A pending SET_POSE may be retried after newer IMU/state polling.
    const auto revision = r.system->robot().anchor_revision;
    EXPECT_EQ(r.request(place).result, translagatr::kResultOk);
    EXPECT_EQ(r.system->robot().anchor_revision, revision);
    r.wheels(4000, 4000);
    const auto state = r.request(r.imuRequest(160));
    EXPECT_EQ(state.op, translagatr::kOpGetState);
    EXPECT_NE(state.state.robot_flags & translagatr::kRobotPoseValid, 0);
    EXPECT_NEAR(state.state.x_mm, 1000 + 2 * kPi * 0.0254 * 1000, 2);
    const auto x = r.system->robot().odom_pose.x_m;
    // Symmetric pure turn: left negative/right positive, IMU +5 degrees.
    const int turn = static_cast<int>(std::round(0.15 * degToRad(5) / (2*kPi*0.0254) * 4000));
    r.wheels(4000-turn, 4000+turn);
    r.request(r.imuRequest(180, 5000));
    EXPECT_NEAR(r.system->robot().odom_pose.heading_rad, degToRad(5), 1e-9);
    EXPECT_NEAR(r.system->robot().odom_pose.x_m, x, 1e-9);
    EXPECT_TRUE(r.system->localization().feed()->status().clock_mapped);
    r.now += 10;
    r.system->step(hostTime(r.now)); // retained samples, no new motion
    EXPECT_TRUE(r.system->localization().feed()->status().clock_mapped);
    const auto snapshot = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    EXPECT_NE(snapshot.find("arrival-time"), std::string::npos);
}

TEST(BrainImuBench, DuplicateInvalidStaleAndRestartDoNotBridgeTravel) {
    BenchRig r;
    ASSERT_TRUE(r.build());
    r.wheels(0,0); r.request(r.imuRequest(100));
    r.wheels(100,100); r.request(r.imuRequest(120));
    const double x = r.system->robot().odom_pose.x_m;
    const auto receipt = r.imu->received.ms;
    r.wheels(200,200); r.request(r.imuRequest(120));
    EXPECT_EQ(r.imu->received.ms, receipt);
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.x_m, x);
    r.request(r.imuRequest(140,0,false));
    r.wheels(1000,1000); r.request(r.imuRequest(160));
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.x_m, x);
    r.wheels(1100,1100); r.request(r.imuRequest(180));
    EXPECT_GT(r.system->robot().odom_pose.x_m, x);
    const double resumed = r.system->robot().odom_pose.x_m;
    r.wheels(2000,2000); r.request(r.imuRequest(1)); // Brain stamp reset
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.x_m, resumed);
    r.now += 250; r.system->step(hostTime(r.now));
    EXPECT_FALSE(r.system->localization().functionStatus().front().ready);
    // Prime previous drain again after outage so the next reply has a window.
    r.wheels(3000,3000); r.request(r.imuRequest(20));
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.x_m, resumed);
    const uint32_t old_session = r.session;
    translagatr::BrainRequest hello;
    hello.op = translagatr::kOpHello; hello.nonce = 98765; hello.request_id = r.rid++;
    r.session = r.request(hello).session;
    EXPECT_NE(r.session, old_session);
    EXPECT_FALSE(r.imu->valid);
    auto old = r.imuRequest(50);
    old.session = old_session;
    EXPECT_EQ(r.request(old).result, translagatr::kResultUnknownSession);
    r.wheels(4000,4000); r.request(r.imuRequest(60));
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.x_m, resumed);
}

TEST(BrainImuBench, SamplesNeedExplicitConfiguration) {
    BenchRig r;
    ASSERT_TRUE(r.build(false));
    const auto reply = r.request(r.imuRequest(100));
    EXPECT_EQ(reply.result, translagatr::kResultOk);   // the state is served, the sample ignored
    EXPECT_FALSE(r.imu->valid);
    EXPECT_EQ(r.imu->sequence, 0u);
}

TEST(BrainImuBench, UnknownSampleFlagsAreRefusedAndChangeNothing) {
    BenchRig r;
    ASSERT_TRUE(r.build());
    auto bad = r.imuRequest(100);
    bad.imu_flags |= 0x80;
    EXPECT_EQ(r.request(bad).result, translagatr::kResultInvalidArgument);
    EXPECT_FALSE(r.imu->valid);
    EXPECT_EQ(r.request(r.imuRequest(100)).result, translagatr::kResultOk);
    EXPECT_TRUE(r.imu->valid);
    EXPECT_EQ(r.imu->stamp_ms, 100u);
}

TEST(BrainImuBench, PerpendicularWheelsMeasureForwardAndSidewaysIndependently) {
    BenchRig r;
    ASSERT_TRUE(r.build(true, kPerpendicularGeometry, "brain_imu_planar_bench"));
    r.wheels(0, 0);
    r.request(r.imuRequest(100));

    const double revolution_m = 2 * kPi * 0.0254;
    r.wheels(4000, 0);
    r.request(r.imuRequest(120));
    EXPECT_NEAR(r.system->robot().odom_pose.x_m, revolution_m, 1e-9);
    EXPECT_NEAR(r.system->robot().odom_pose.y_m, 0, 1e-9);

    r.wheels(4000, 4000);
    r.request(r.imuRequest(140));
    EXPECT_NEAR(r.system->robot().odom_pose.x_m, revolution_m, 1e-9);
    EXPECT_NEAR(r.system->robot().odom_pose.y_m, revolution_m, 1e-9);
    EXPECT_NEAR(r.system->robot().odom_pose.heading_rad, 0, 1e-9);
}

TEST(BrainImuBench, PerpendicularWheelsCompensateBothLeverArmsDuringRotation) {
    BenchRig r;
    ASSERT_TRUE(r.build(true, kPerpendicularGeometry, "brain_imu_planar_bench"));
    r.wheels(0, 0);
    r.request(r.imuRequest(100));

    // A pure CCW turn moves the forward wheel backwards at y=+0.15,
    // and the sideways wheel leftwards at x=+0.10. Neither is translation
    // of the chassis origin. Allow one encoder count of quantization.
    const double radians = degToRad(5);
    const double meters_per_count = 2 * kPi * 0.0254 / 4000;
    r.wheels(static_cast<int>(std::round(-0.15 * radians / meters_per_count)),
             static_cast<int>(std::round(0.10 * radians / meters_per_count)));
    r.request(r.imuRequest(120, 5000));
    EXPECT_NEAR(r.system->robot().odom_pose.x_m, 0, meters_per_count);
    EXPECT_NEAR(r.system->robot().odom_pose.y_m, 0, meters_per_count);
    EXPECT_NEAR(r.system->robot().odom_pose.heading_rad, radians, 1e-9);
}

TEST(BrainImuBench, PerpendicularWheelDirectionCanReverseEncoderPolarity) {
    BenchRig r;
    BenchGeometry geometry = kPerpendicularGeometry;
    geometry[1].direction = "negative";
    ASSERT_TRUE(r.build(true, geometry, "brain_imu_planar_bench"));
    r.wheels(0, 0);
    r.request(r.imuRequest(100));
    r.wheels(0, -4000);
    r.request(r.imuRequest(120));
    EXPECT_NEAR(r.system->robot().odom_pose.x_m, 0, 1e-9);
    EXPECT_NEAR(r.system->robot().odom_pose.y_m, 2 * kPi * 0.0254, 1e-9);
}

TEST(BrainImuBench, PlanarWheelsAllowIndependentNonRightAngleDirections) {
    BenchRig r;
    BenchGeometry geometry = kPerpendicularGeometry;
    geometry[1].angle = 45;
    ASSERT_TRUE(r.build(true, geometry, "brain_imu_planar_bench"));
    r.wheels(0, 0);
    r.request(r.imuRequest(100));
    r.wheels(4000, 4000);
    r.request(r.imuRequest(120));
    const double revolution_m = 2 * kPi * 0.0254;
    EXPECT_NEAR(r.system->robot().odom_pose.x_m, revolution_m, 1e-9);
    EXPECT_NEAR(r.system->robot().odom_pose.y_m, revolution_m * (std::sqrt(2.0) - 1), 1e-9);
}

TEST(BrainImuBench, PlanarWheelsRejectCollinearAndNearlyCollinearDirections) {
    BenchRig r;
    registerAll(r.functions);
    for (double angle : {0.0, 180.0, 0.001}) {
        SCOPED_TRACE(angle);
        BenchGeometry geometry = kPerpendicularGeometry;
        geometry[1].angle = angle;
        const std::string xml = r.configuration(true, geometry, "brain_imu_planar_bench");
        ASSERT_FALSE(xml.empty());
        std::string err;
        EXPECT_EQ(System::buildFromString(xml.c_str(), r.functions, err), nullptr);
        EXPECT_NE(err.find("nonparallel"), std::string::npos) << err;
    }
}

TEST(BrainImuBench, ParallelProfileStillRejectsSidewaysWheelConfiguration) {
    BenchRig r;
    registerAll(r.functions);
    const std::string xml = r.configuration(true, kPerpendicularGeometry,
                                            "brain_imu_parallel_bench");
    ASSERT_FALSE(xml.empty());
    std::string err;
    EXPECT_EQ(System::buildFromString(xml.c_str(), r.functions, err), nullptr);
    EXPECT_NE(err.find("forward"), std::string::npos) << err;
}

TEST(BrainImuBench, PerpendicularMotionUsesSameFieldAxesForBrainAndInspection) {
    BenchRig r;
    ASSERT_TRUE(r.build(true, kPerpendicularGeometry, "brain_imu_planar_bench"));
    r.wheels(0, 0);
    r.request(r.imuRequest(100));
    translagatr::BrainRequest place;
    place.op = translagatr::kOpSetPose;
    place.session = r.session;
    place.request_id = r.rid++;
    place.x_mm = 1000;
    place.y_mm = 2000;
    place.heading_cdeg = 9000;
    r.request(place);
    r.wheels(0, 0);
    r.request(r.imuRequest(140));
    ASSERT_EQ(r.request(place).result, translagatr::kResultOk);

    // Body-forward travel is field +y after a 90 degree placement.
    r.wheels(4000, 0);
    const auto reply = r.request(r.imuRequest(180));
    ASSERT_NE(reply.state.robot_flags & translagatr::kRobotPoseValid, 0);
    EXPECT_NEAR(reply.state.x_mm, 1000, 1);
    EXPECT_NEAR(reply.state.y_mm, 2000 + 2 * kPi * 0.0254 * 1000, 1);
    const auto field_pose = r.system->robot().fieldPose();
    EXPECT_NEAR(field_pose.x_m, 1.0, 1e-9);
    EXPECT_NEAR(field_pose.y_m, 2.0 + 2 * kPi * 0.0254, 1e-9);

    const std::string snapshot = snapshotDocument(*r.system, InspectionServiceStats{}, hostTime(r.now));
    const auto robot = snapshot.find("\"robot\":{");
    ASSERT_NE(robot, std::string::npos);
    const auto field = snapshot.find("\"field\":{", robot);
    ASSERT_NE(field, std::string::npos);
    const auto x = snapshot.find("\"x_m\":", field);
    const auto y = snapshot.find("\"y_m\":", field);
    ASSERT_NE(x, std::string::npos);
    ASSERT_NE(y, std::string::npos);
    EXPECT_NEAR(std::stod(snapshot.substr(x + 6)) * 1000, reply.state.x_mm, 1);
    EXPECT_NEAR(std::stod(snapshot.substr(y + 6)) * 1000, reply.state.y_mm, 1);
}

namespace {
translagatr::BrainRequest telemetry(BenchRig& r, uint32_t stamp, int16_t roll_cdeg,
                                    int16_t pitch_cdeg, bool with_attitude = true) {
    translagatr::BrainRequest t;
    t.op                   = translagatr::kOpTelemetry;
    t.session              = r.session;
    t.request_id           = r.rid++;
    t.telemetry.flags      = with_attitude ? translagatr::kTelemetryAttitude : 0;
    t.telemetry.stamp_ms   = stamp;
    t.telemetry.roll_cdeg  = roll_cdeg;
    t.telemetry.pitch_cdeg = pitch_cdeg;
    return t;
}
} // namespace

TEST(BrainImuBench, TelemetryAttitudeIsMeasuredWhileFreshThenStale) {
    BenchRig r;
    r.attitude = true;
    ASSERT_TRUE(r.build());
    r.wheels(0, 0);
    r.request(r.imuRequest(100));

    // configured, nothing measured yet: named source, no measurement time
    Attitude a = r.system->robot().attitude;
    EXPECT_FALSE(a.valid);
    EXPECT_TRUE(a.assumed_level);
    EXPECT_EQ(a.source, "vex_attitude");
    EXPECT_FALSE(a.measuredAt.isSet());

    const Pose2D before = r.system->robot().odom_pose;
    EXPECT_EQ(r.request(telemetry(r, 110, 300, -200)).result, translagatr::kResultOk);
    a = r.system->robot().attitude;
    ASSERT_TRUE(a.valid);
    EXPECT_FALSE(a.assumed_level);
    EXPECT_EQ(a.source, "brain_vex_imu");
    EXPECT_EQ(a.measuredAt, r.imu->attitude_received);   // Pi arrival time
    double roll = 0, pitch = 0, yaw = 0;
    attitudeEuler(a, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), 3.0, 1e-9);
    EXPECT_NEAR(radToDeg(pitch), -2.0, 1e-9);
    EXPECT_NEAR(yaw, r.system->robot().odom_pose.heading_rad, 1e-12);
    // tilt never moves the pose
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.x_m, before.x_m);
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.y_m, before.y_m);
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.heading_rad, before.heading_rad);

    // still fresh at 240 ms
    r.now += 220;
    r.system->step(hostTime(r.now));
    EXPECT_TRUE(r.system->robot().attitude.valid);
    // stale after 250 ms: explicitly level, the old measurement time kept
    r.now += 40;
    r.system->step(hostTime(r.now));
    a = r.system->robot().attitude;
    EXPECT_FALSE(a.valid);
    EXPECT_TRUE(a.assumed_level);
    EXPECT_TRUE(a.measuredAt.isSet());
    attitudeEuler(a, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0, 1e-12);
    EXPECT_NEAR(pitch, 0.0, 1e-12);

    // fresh again with the next report
    EXPECT_EQ(r.request(telemetry(r, 400, -100, 50)).result, translagatr::kResultOk);
    a = r.system->robot().attitude;
    ASSERT_TRUE(a.valid);
    attitudeEuler(a, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), -1.0, 1e-9);
    EXPECT_NEAR(radToDeg(pitch), 0.5, 1e-9);
}

TEST(BrainImuBench, TelemetryWithoutAttitudePublishesNoTilt) {
    BenchRig r;
    r.attitude = true;
    ASSERT_TRUE(r.build());
    r.wheels(0, 0);
    r.request(r.imuRequest(100));
    EXPECT_EQ(r.request(telemetry(r, 110, 300, -200, false)).result, translagatr::kResultOk);
    EXPECT_FALSE(r.imu->attitude_valid);
    EXPECT_FALSE(r.system->robot().attitude.valid);
    EXPECT_FALSE(r.system->robot().attitude.measuredAt.isSet());
}

// Documented in brain_profile.md: reports that drop the attitude group
// (VEX IMU calibrating or failing) age the last tilt out like missed reports.
TEST(BrainImuBench, TelemetryDroppingTheAttitudeGroupAgesTheLastTiltOut) {
    BenchRig r;
    r.attitude = true;
    ASSERT_TRUE(r.build());
    r.wheels(0, 0);
    r.request(r.imuRequest(100));
    EXPECT_EQ(r.request(telemetry(r, 110, 300, -200)).result, translagatr::kResultOk);
    const MonotonicTime measured = r.imu->attitude_received;
    ASSERT_TRUE(r.system->robot().attitude.valid);
    // the Pi drains every 10 ms; a reply is due within the window after the
    // previous drain
    const auto idle = [&](int ms) {
        for (int t = 0; t < ms; t += 10) {
            r.now += 10;
            r.system->step(hostTime(r.now));
        }
    };

    idle(100);   // 120 ms after the tilt
    EXPECT_EQ(r.request(telemetry(r, 210, 0, 0, false)).result, translagatr::kResultOk);
    EXPECT_FALSE(r.imu->attitude_valid);
    Attitude a = r.system->robot().attitude;
    EXPECT_TRUE(a.valid);
    EXPECT_EQ(a.measuredAt, measured);
    idle(100);   // 240 ms
    EXPECT_EQ(r.request(telemetry(r, 310, 0, 0, false)).result, translagatr::kResultOk);
    EXPECT_TRUE(r.system->robot().attitude.valid);

    idle(20);    // 280 ms: stale, level, the old measurement time kept
    EXPECT_EQ(r.request(telemetry(r, 330, 0, 0, false)).result, translagatr::kResultOk);
    a = r.system->robot().attitude;
    EXPECT_FALSE(a.valid);
    EXPECT_TRUE(a.assumed_level);
    EXPECT_EQ(a.measuredAt, measured);
    EXPECT_EQ(a.source, "brain_vex_imu");
    double roll = 0, pitch = 0, yaw = 0;
    attitudeEuler(a, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0, 1e-12);
    EXPECT_NEAR(pitch, 0.0, 1e-12);

    // the group returns: measured again
    EXPECT_EQ(r.request(telemetry(r, 430, 150, 0)).result, translagatr::kResultOk);
    a = r.system->robot().attitude;
    ASSERT_TRUE(a.valid);
    attitudeEuler(a, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), 1.5, 1e-9);
}

TEST(BrainImuBench, AttitudeOutputNeedsItsOwnObservationId) {
    BenchRig r;
    r.attitude = true;
    registerAll(r.functions);
    std::string xml = r.configuration(true, kParallelGeometry, "brain_imu_parallel_bench");
    ASSERT_FALSE(xml.empty());
    const std::string output = R"(<Output observation_id=")";
    const auto        at     = xml.find(output);
    ASSERT_NE(at, std::string::npos);
    const auto id_end = xml.find('"', at + output.size());
    const std::string motion_id = xml.substr(at + output.size(), id_end - at - output.size());
    const std::string wanted = R"(<Attitude observation_id="vex_attitude"/>)";
    const auto attitude_at = xml.find(wanted);
    ASSERT_NE(attitude_at, std::string::npos);
    xml.replace(attitude_at, wanted.size(),
                R"(<Attitude observation_id=")" + motion_id + R"("/>)");
    std::string err;
    EXPECT_EQ(System::buildFromString(xml.c_str(), r.functions, err), nullptr);
    EXPECT_NE(err.find("Attitude needs its own observation_id"), std::string::npos) << err;
}
