// parallel_wheels_gtest.cpp
// Two forward-measuring tracking wheels with the gyro heading and lateral
// motion assumed zero: refusal without the explicit assumption, lever-arm
// corrected forward travel, wheel signs, gyro outages, and the full Pico
// wire path through planar_motion_integrator; the checked-in parallel-wheel
// profiles: shared localization, no camera or third wheel in profile A,
// the ordinary noop world estimation driven by a synthetic rig, and Brain
// placement plus the unavailable field map answer over the real brain link.

#include <gtest/gtest.h>

#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "common/frame_codec.h"
#include "common/link_documents.h"
#include "config/composition.h"
#include "impl/localization/tracking_wheel_motion.h"
#include "impl/resources/serial_links.h"
#include "impl/resources/synthetic_rig.h"
#include "inspection/inspection_document.h"
#include "math/angles.h"
#include "payloads/robot_observations.h"
#include "runtime/register_all.h"
#include "runtime/sensor_catalog.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

constexpr double kRadius = 0.0254;
constexpr double kLeftY  = 0.15;    // unequal offsets on purpose
constexpr double kRightY = -0.08;

struct Fixture {
    tinyxml2::XMLDocument doc;
    SensorCatalog         catalog;
    SensorMap             sensors;
    Diagnostics           diagnostics;
    int64_t               now_ms = 1;

    Fixture() {
        for (const char* id : {"enc_a", "enc_b", "enc_c"}) {
            catalog.add(SensorId{id},
                        PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        }
        catalog.add(SensorId{"imu"}, PayloadDescriptor::of<ImuSample>(payload_names::kImuSample));
    }

    std::unique_ptr<RobotObservationFunction> make(const std::string& xml, std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml.c_str()), tinyxml2::XML_SUCCESS);
        RobotObservationInitializationContext c;
        c.sensors = &catalog;
        return TrackingWheelMotion::create(ConfigNode{doc.RootElement()}, c, err);
    }

    void put(const char* id, TypedPayload payload, int64_t stamp_ms, uint64_t sequence) {
        MeasurementRecord record;
        record.state = SourceState::kValid;
        StoredSample stored;
        stored.measuredAt      = deviceTime(stamp_ms);
        stored.receivedAt      = hostTime(stamp_ms + 3);
        stored.sequence        = sequence;
        stored.upstream.clock  = "pico";
        stored.upstream.source = id;
        stored.payload         = std::move(payload);
        record.latest          = std::move(stored);
        sensors[SensorId{id}]  = std::move(record);
    }

    void putEncoder(const char* id, double angle_rad, int64_t stamp_ms, uint64_t sequence) {
        EncoderSample s;
        s.angle_rad = angle_rad;
        put(id, TypedPayload::store(s, payload_names::kEncoderSample), stamp_ms, sequence);
    }

    void putImu(double accum_rad, int64_t stamp_ms, uint64_t sequence) {
        ImuSample s;
        s.has_accumulated       = true;
        s.accumulated_angle_rad = accum_rad;
        put("imu", TypedPayload::store(s, payload_names::kImuSample), stamp_ms, sequence);
    }

    RobotObservationMap run(RobotObservationFunction& fn) {
        RobotObservationMap out;
        ExecutionContext    context{hostTime(now_ms++), 1, &diagnostics};
        fn.run({sensors, context}, out);
        for (const auto& record : out) {
            fn.settle(record.first, true);
        }
        return out;
    }

    static const BodyMotionIncrement* motion(const RobotObservationMap& out) {
        const auto it = out.find(ObservationId{"motion"});
        return it == out.end() ? nullptr : it->second.payload.get<BodyMotionIncrement>();
    }
};

struct WheelSpec {
    const char* sensor;
    double      y_m;
    double      angle_deg;
    bool        positive;
};

const std::vector<WheelSpec> kParallel = {{"enc_a", kLeftY, 0, true},
                                          {"enc_b", kRightY, 0, true}};

std::string modelXml(const std::vector<WheelSpec>& wheels, bool heading, bool lateral,
                     const char* assume = "zero") {
    std::string xml = R"(<Observation id="m" type="tracking_wheel_motion">)";
    for (const WheelSpec& w : wheels) {
        xml += std::string(R"(<TrackingWheel sensor_id=")") + w.sensor +
               R"(" radius_m="0.0254" position_x_m="0.05" position_y_m=")" +
               std::to_string(w.y_m) + R"(" measurement_angle_deg=")" +
               std::to_string(w.angle_deg) + R"(" direction=")" +
               (w.positive ? "positive" : "negative") + R"("/>)";
    }
    if (heading) {
        xml += R"(<HeadingConstraint sensor_id="imu" bias_samples="0"/>)";
    }
    if (lateral) {
        xml += std::string(R"(<LateralMotion assume=")") + assume + R"("/>)";
    }
    xml += R"(<Output observation_id="motion"/></Observation>)";
    return xml;
}

// Encoder angle a wheel reports for rigid body motion (s forward, dtheta).
double wheelAngle(const WheelSpec& w, double s, double dtheta) {
    const double ux     = std::cos(degToRad(w.angle_deg));
    const double travel = ux * (s - w.y_m * dtheta);   // k = -y * ux when uy = 0
    return travel / kRadius * (w.positive ? 1.0 : -1.0);
}

// Seeds every source at 1000 ms, then one window to 1010 ms.
BodyMotionIncrement oneWindow(Fixture& f, RobotObservationFunction& fn,
                              const std::vector<WheelSpec>& wheels, double s, double dtheta) {
    for (const WheelSpec& w : wheels) {
        f.putEncoder(w.sensor, 0.0, 1000, 1);
    }
    f.putImu(0.0, 1000, 1);
    EXPECT_TRUE(f.run(fn).empty());
    for (const WheelSpec& w : wheels) {
        f.putEncoder(w.sensor, wheelAngle(w, s, dtheta), 1010, 2);
    }
    f.putImu(dtheta, 1010, 2);
    const RobotObservationMap  out   = f.run(fn);
    const BodyMotionIncrement* delta = Fixture::motion(out);
    EXPECT_NE(delta, nullptr);
    return delta != nullptr ? *delta : BodyMotionIncrement{};
}

} // namespace

TEST(ParallelWheels, DefaultModelStillRejectsParallelWheels) {
    Fixture     f;
    std::string err;
    EXPECT_EQ(f.make(modelXml(kParallel, true, false), err), nullptr);
    EXPECT_NE(err.find("collinear"), std::string::npos) << err;
    EXPECT_NE(err.find("LateralMotion"), std::string::npos) << err;

    err.clear();
    EXPECT_EQ(f.make(modelXml(kParallel, false, false), err), nullptr);
    EXPECT_NE(err.find("HeadingConstraint"), std::string::npos) << err;
}

TEST(ParallelWheels, ZeroLateralAssumptionIsExplicitAndValidated) {
    Fixture     f;
    std::string err;
    EXPECT_EQ(f.make(modelXml(kParallel, false, true), err), nullptr);
    EXPECT_NE(err.find("needs a HeadingConstraint"), std::string::npos) << err;

    err.clear();
    const std::vector<WheelSpec> with_lateral = {{"enc_a", kLeftY, 0, true},
                                                 {"enc_c", 0.0, 90, true}};
    EXPECT_EQ(f.make(modelXml(with_lateral, true, true), err), nullptr);
    EXPECT_NE(err.find("measures sideways"), std::string::npos) << err;

    err.clear();
    EXPECT_EQ(f.make(modelXml(kParallel, true, true, "small"), err), nullptr);
    EXPECT_NE(err.find("assume must be zero"), std::string::npos) << err;

    err.clear();
    std::string twice = modelXml(kParallel, true, true);
    twice.insert(twice.find("<Output"), R"(<LateralMotion assume="zero"/>)");
    EXPECT_EQ(f.make(twice, err), nullptr);
    EXPECT_NE(err.find("at most one LateralMotion"), std::string::npos) << err;

    err.clear();
    EXPECT_NE(f.make(modelXml(kParallel, true, true), err), nullptr) << err;
}

TEST(ParallelWheels, StraightTravel) {
    Fixture     f;
    std::string err;
    auto        fn = f.make(modelXml(kParallel, true, true), err);
    ASSERT_NE(fn, nullptr) << err;
    const auto delta = oneWindow(f, *fn, kParallel, 0.1, 0.0);
    EXPECT_NEAR(delta.dx_m, 0.1, 1e-9);
    EXPECT_EQ(delta.dy_m, 0.0);
    EXPECT_NEAR(delta.dtheta_rad, 0.0, 1e-12);
    EXPECT_TRUE(delta.has_rotation);
}

TEST(ParallelWheels, TurnInPlaceWithUnequalOffsetsHasNoTranslation) {
    Fixture     f;
    std::string err;
    auto        fn = f.make(modelXml(kParallel, true, true), err);
    ASSERT_NE(fn, nullptr) << err;
    const double dtheta = 0.3;
    // the plain wheel mean would be (-0.15 + 0.08) / 2 * 0.3 = -0.0105 m
    const auto delta = oneWindow(f, *fn, kParallel, 0.0, dtheta);
    EXPECT_NEAR(delta.dx_m, 0.0, 1e-9);
    EXPECT_EQ(delta.dy_m, 0.0);
    EXPECT_NEAR(delta.dtheta_rad, dtheta, 1e-12);
}

TEST(ParallelWheels, ArcUsesLeverArmCorrectedForwardTravel) {
    Fixture     f;
    std::string err;
    auto        fn = f.make(modelXml(kParallel, true, true), err);
    ASSERT_NE(fn, nullptr) << err;
    const auto delta = oneWindow(f, *fn, kParallel, 0.2, 0.25);
    EXPECT_NEAR(delta.dx_m, 0.2, 1e-9);   // arc length; the estimator applies the chord
    EXPECT_EQ(delta.dy_m, 0.0);
    EXPECT_NEAR(delta.dtheta_rad, 0.25, 1e-12);
}

TEST(ParallelWheels, WheelSignsAndReversedMounting) {
    // left encoder counts backwards, right wheel mounted facing -x
    const std::vector<WheelSpec> wheels = {{"enc_a", kLeftY, 0, false},
                                           {"enc_b", kRightY, 180, true}};
    Fixture     f;
    std::string err;
    auto        fn = f.make(modelXml(wheels, true, true), err);
    ASSERT_NE(fn, nullptr) << err;
    const auto delta = oneWindow(f, *fn, wheels, 0.2, -0.4);
    EXPECT_NEAR(delta.dx_m, 0.2, 1e-9);
    EXPECT_NEAR(delta.dtheta_rad, -0.4, 1e-12);

    // the same angles read with the geometry's signs flipped disagree
    const std::vector<WheelSpec> wrong = {{"enc_a", kLeftY, 0, true},
                                          {"enc_b", kRightY, 0, true}};
    Fixture f2;
    auto    fn2 = f2.make(modelXml(wrong, true, true), err);
    ASSERT_NE(fn2, nullptr) << err;
    for (const WheelSpec& w : wheels) {
        f2.putEncoder(w.sensor, 0.0, 1000, 1);
    }
    f2.putImu(0.0, 1000, 1);
    f2.run(*fn2);
    for (const WheelSpec& w : wheels) {
        f2.putEncoder(w.sensor, wheelAngle(w, 0.2, -0.4), 1010, 2);
    }
    f2.putImu(-0.4, 1010, 2);
    const auto  wrong_out   = f2.run(*fn2);
    const auto* wrong_delta = Fixture::motion(wrong_out);
    ASSERT_NE(wrong_delta, nullptr);
    EXPECT_GT(std::fabs(wrong_delta->dx_m - 0.2), 0.1);
}

TEST(ParallelWheels, RotationCouplingIsTheMeanLateralOffset) {
    Fixture     f;
    std::string err;
    auto        fn = f.make(modelXml(kParallel, true, true), err);
    ASSERT_NE(fn, nullptr) << err;
    const auto delta = oneWindow(f, *fn, kParallel, 0.05, 0.1);
    EXPECT_TRUE(delta.has_rotation_coupling);
    EXPECT_NEAR(delta.dx_per_dtheta_m_rad, 0.5 * (kLeftY + kRightY), 1e-9);
    EXPECT_EQ(delta.dy_per_dtheta_m_rad, 0.0);
}

TEST(ParallelWheels, ImuIsPartOfTheLineage) {
    Fixture     f;
    std::string err;
    auto        fn = f.make(modelXml(kParallel, true, true), err);
    ASSERT_NE(fn, nullptr) << err;
    const auto delta = oneWindow(f, *fn, kParallel, 0.05, 0.1);
    ASSERT_EQ(delta.sources.size(), 3u);
    EXPECT_EQ(delta.sources[2].source, "imu");
}

TEST(ParallelWheels, MissingGyroNeverFabricatesHeadingFromWheels) {
    Fixture     f;
    std::string err;
    auto        fn = f.make(modelXml(kParallel, true, true), err);
    ASSERT_NE(fn, nullptr) << err;
    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putImu(0.0, 1000, 1);
    f.run(*fn);

    // wheels disagree like a turn, the gyro says nothing new: no motion
    f.putEncoder("enc_a", wheelAngle(kParallel[0], 0.0, 0.3), 1010, 2);
    f.putEncoder("enc_b", wheelAngle(kParallel[1], 0.0, 0.3), 1010, 2);
    EXPECT_TRUE(f.run(*fn).empty());

    // the gyro returns after an outage longer than max_gap_ms: dropped, not bridged
    f.putEncoder("enc_a", wheelAngle(kParallel[0], 0.0, 0.6), 1300, 3);
    f.putEncoder("enc_b", wheelAngle(kParallel[1], 0.0, 0.6), 1300, 3);
    f.putImu(0.6, 1300, 2);
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_NE(fn->readiness().note.find("gyro gap"), std::string::npos) << fn->readiness().note;

    // fresh baselines, then motion resumes
    f.putEncoder("enc_a", wheelAngle(kParallel[0], 0.0, 0.6), 1310, 4);
    f.putEncoder("enc_b", wheelAngle(kParallel[1], 0.0, 0.6), 1310, 4);
    f.putImu(0.6, 1310, 3);
    f.run(*fn);
    f.putEncoder("enc_a", wheelAngle(kParallel[0], 0.1, 0.6), 1320, 5);
    f.putEncoder("enc_b", wheelAngle(kParallel[1], 0.1, 0.6), 1320, 5);
    f.putImu(0.6, 1320, 4);
    const auto  out   = f.run(*fn);
    const auto* delta = Fixture::motion(out);
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->dx_m, 0.1, 1e-9);
    EXPECT_NEAR(delta->dtheta_rad, 0.0, 1e-12);
}

namespace
{

// Real Pico sensor frames into two forward wheels on channels 0 and 1.
std::string wirePathConfig(bool right_inverted) {
    return std::string(R"(
<System>
    <Loop rate_hz="200"/>
    <Resources>
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="imu" channel="imu"/>
        </Resource>
        <Resource id="wheel_geometry" type="wheel_geometry">
            <Wheel id="left_wheel" sensor_id="tracking_encoder_a" radius_m="0.0254"
                   position_x_m="0.05" position_y_m="0.15" measurement_angle_deg="0"
                   direction="positive"/>
            <Wheel id="right_wheel" sensor_id="tracking_encoder_b" radius_m="0.0254"
                   position_x_m="-0.02" position_y_m="-0.08" measurement_angle_deg="0"
                   direction="positive"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="tracking_encoder_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000" invert="false"/>
        </Sensor>
        <Sensor id="tracking_encoder_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000" invert=")") +
           (right_inverted ? "true" : "false") + R"("/>
        </Sensor>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/>
            <Calibration invert="false"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">
                <Wheels resource_id="wheel_geometry">
                    <Use wheel_id="left_wheel"/>
                    <Use wheel_id="right_wheel"/>
                </Wheels>
                <HeadingConstraint sensor_id="robot_imu" bias_samples="200"
                                   max_calibration_travel_m="0.005" max_gap_ms="250"/>
                <LateralMotion assume="zero"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
            </Estimator>
            <History retention_s="5"/>
        </Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
}

std::vector<uint8_t> sensorPacket(uint8_t seq, uint32_t stamp, int32_t enc0, int32_t enc1,
                                  int32_t gyro_mdps) {
    gatr2::SensorSample s{};
    s.seq      = seq;
    s.stamp_ms = stamp;
    s.mask     = gatr2::kSensorEnc0 | gatr2::kSensorEnc1 | gatr2::kSensorGyroZ;
    s.enc[0]   = enc0;
    s.enc[1]   = enc1;
    s.gyro_z   = gyro_mdps;
    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeSensorFrame(s, buf.data(), gatr2::kMaxFrameLen));
    return buf;
}

struct Drive {
    FunctionRegistry        functions;
    std::unique_ptr<System> system;
    MemoryLink*             pico = nullptr;

    bool     right_inverted = false;
    double   travel_left = 0.0, travel_right = 0.0;
    uint32_t stamp = 1000;
    uint8_t  seq   = 0;
    int      cycle = 0;
    Pose2D   truth;   // odometry frame

    explicit Drive(bool inverted) : right_inverted(inverted) {
        registerAll(functions);
        std::string err;
        system = System::buildFromString(wirePathConfig(inverted).c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system == nullptr) {
            return;
        }
        auto link = system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
        pico      = dynamic_cast<MemoryLink*>(link.get());
    }

    // One 5 ms frame of constant-twist body motion: s forward, dtheta_deg turn.
    void step(double s, double dtheta_deg) {
        constexpr double kCountsPerMeter = 4000.0 / (2.0 * kPi * kRadius);
        constexpr int    kBiasMdps       = 1500;
        const double     dtheta          = degToRad(dtheta_deg);
        travel_left += s - kLeftY * dtheta;
        travel_right += s - kRightY * dtheta;
        const double right_counts = travel_right * kCountsPerMeter * (right_inverted ? -1 : 1);

        // exact arc for the truth
        if (std::fabs(dtheta) < 1e-12) {
            truth.x_m += s * std::cos(truth.heading_rad);
            truth.y_m += s * std::sin(truth.heading_rad);
        } else {
            const double r = s / dtheta;
            truth.x_m += r * (std::sin(truth.heading_rad + dtheta) - std::sin(truth.heading_rad));
            truth.y_m += r * (std::cos(truth.heading_rad) - std::cos(truth.heading_rad + dtheta));
        }
        truth.heading_rad = wrapAngle(truth.heading_rad + dtheta);

        stamp += 5;
        ++cycle;
        const int32_t rate =
            kBiasMdps + static_cast<int32_t>(std::llround(dtheta_deg / 0.005 * 1000.0));
        pico->input().feed(sensorPacket(
            seq++, stamp, static_cast<int32_t>(std::llround(travel_left * kCountsPerMeter)),
            static_cast<int32_t>(std::llround(right_counts)), rate));
        system->step(hostTime(cycle));
    }

    void expectAtTruth(const char* where) const {
        const RobotState& robot = system->robot();
        EXPECT_TRUE(robot.valid) << where;
        EXPECT_NEAR(robot.odom_pose.x_m, truth.x_m, 0.004) << where;
        EXPECT_NEAR(robot.odom_pose.y_m, truth.y_m, 0.004) << where;
        EXPECT_NEAR(radToDeg(wrapAngle(robot.odom_pose.heading_rad - truth.heading_rad)), 0.0,
                    0.1)
            << where;
    }
};

void driveStraightTurnArc(bool right_inverted) {
    Drive d(right_inverted);
    ASSERT_NE(d.system, nullptr);
    ASSERT_NE(d.pico, nullptr);
    for (int i = 0; i < 205; ++i) {
        d.step(0.0, 0.0);   // stationary gyro bias calibration
    }
    for (int i = 0; i < 200; ++i) {
        d.step(0.005, 0.0);   // 1 m straight
    }
    d.step(0.0, 0.0);
    d.expectAtTruth("after straight");
    for (int i = 0; i < 100; ++i) {
        d.step(0.0, 0.9);   // +90 degrees in place
    }
    d.step(0.0, 0.0);
    d.expectAtTruth("after turn in place");
    EXPECT_NEAR(d.system->robot().odom_pose.x_m, 1.0, 0.002);   // no drift from wheel offsets
    for (int i = 0; i < 100; ++i) {
        d.step(0.005, 0.45);   // 0.5 m arc through +45 degrees
    }
    for (int i = 0; i < 100; ++i) {
        d.step(0.004, -0.6);   // tighter arc back the other way
    }
    for (int i = 0; i < 5; ++i) {
        d.step(0.0, 0.0);
    }
    d.expectAtTruth("after arcs");
    EXPECT_EQ(d.system->diagnostics().links.at("pico_uart").decode_errors, 0u);
}

} // namespace

TEST(ParallelWheels, WirePathStraightTurnAndArcsTrackTheTruth) { driveStraightTurnArc(false); }

TEST(ParallelWheels, InvertedEncoderChannelIsCorrectedByCalibration) {
    driveStraightTurnArc(true);
}

namespace
{

const std::string kConfigDir = NAVIGATR_CONFIG_DIR;

std::string readFile(const std::filesystem::path& path) {
    std::ifstream     in(path, std::ios::binary);
    std::stringstream s;
    s << in.rdbuf();
    return s.str();
}

// Replaces every @TOKEN@ (upper case, digits, underscores) with its test
// value; a token the test does not know fails it.
std::string fillTemplate(const std::string&                        text,
                         const std::map<std::string, std::string>& values) {
    std::string out;
    std::size_t i = 0;
    while (i < text.size()) {
        const std::size_t a = text.find('@', i);
        const std::size_t b = a == std::string::npos ? a : text.find('@', a + 1);
        if (b == std::string::npos) {
            out += text.substr(i);
            break;
        }
        const std::string token    = text.substr(a + 1, b - a - 1);
        bool              is_token = !token.empty();
        for (char c : token) {
            is_token = is_token && (std::isupper(static_cast<unsigned char>(c)) ||
                                    std::isdigit(static_cast<unsigned char>(c)) || c == '_');
        }
        if (!is_token) {
            out += text.substr(i, a + 1 - i);
            i = a + 1;
            continue;
        }
        const auto it = values.find(token);
        if (it == values.end()) {
            ADD_FAILURE() << "template token without a test value: " << token;
        }
        out += text.substr(i, a - i) + (it == values.end() ? "0" : it->second);
        i = b + 1;
    }
    return out;
}

// test stand-ins for measurements, never checked in
const std::map<std::string, std::string> kRobotValues = {
    {"MEASURE_WHEEL_A_LOADED_RADIUS_M", "0.0254"}, {"MEASURE_WHEEL_A_X_M", "0.05"},
    {"MEASURE_WHEEL_A_Y_M", "0.15"},               {"CONFIRM_WHEEL_A_DIRECTION", "positive"},
    {"MEASURE_WHEEL_B_LOADED_RADIUS_M", "0.0254"}, {"MEASURE_WHEEL_B_X_M", "-0.02"},
    {"MEASURE_WHEEL_B_Y_M", "-0.08"},              {"CONFIRM_WHEEL_B_DIRECTION", "positive"},
    {"VERIFY_ENCODER_A_CPR", "4000"},              {"VERIFY_ENCODER_B_CPR", "4000"},
    {"CONFIRM_IMU_CCW_POSITIVE_INVERT", "false"},
};

const std::map<std::string, std::string> kCameraValues = {
    {"MEASURE_CAMERA_FORWARD_POSITION_M", "0.18"},
    {"MEASURE_CAMERA_LEFT_POSITION_M", "0"},
    {"MEASURE_CAMERA_HEIGHT_M", "0.22"},
    {"MEASURE_CAMERA_ROLL_DEG", "0"},
    {"MEASURE_CAMERA_DOWNWARD_PITCH_DEG", "8"},
    {"MEASURE_CAMERA_LEFTWARD_YAW_DEG", "0"},
    {"SELECT_CAMERA_INDEX", "0"},
    {"SELECT_CAPTURE_WIDTH_PX", "1280"},
    {"SELECT_CAPTURE_HEIGHT_PX", "960"},
    {"SELECT_CAPTURE_RATE_HZ", "30"},
    {"CALIBRATION_RUN_ID", "test"},
    {"CALIBRATED_WIDTH_PX", "1280"},
    {"CALIBRATED_HEIGHT_PX", "960"},
    {"CALIBRATED_FX_PX", "1000"},
    {"CALIBRATED_FY_PX", "1000"},
    {"CALIBRATED_CX_PX", "640"},
    {"CALIBRATED_CY_PX", "480"},
    {"CALIBRATED_K1", "0"},
    {"CALIBRATED_K2", "0"},
    {"CALIBRATED_P1", "0"},
    {"CALIBRATED_P2", "0"},
    {"CALIBRATED_K3", "0"},
    {"CALIBRATION_RMS_PX", "0.3"},
    {"MEASURE_DETECTOR_CORNER_EDGE_SIZE_M", "0.01761272"},
};

// The checked-in parallel-wheel files copied with their layout, so every
// relative reference resolves exactly as in the repository.
struct ProfileTree {
    std::filesystem::path root;

    ProfileTree() {
        // one directory per test: ctest runs them in parallel
        root = std::filesystem::temp_directory_path() /
               (std::string("navigatr_parallel_wheels_gtest_") +
                ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::remove_all(root);
        for (const char* d : {"override/diagnostics", "shared/pipelines", "shared/robots"}) {
            std::filesystem::create_directories(root / d);
        }
        for (const char* f : {"override/diagnostics/parallel_wheels_bno08x.xml",
                              "override/diagnostics/parallel_wheels_bno08x_camera.xml",
                              "override/field.xml",
                              "shared/pipelines/parallel_wheels_bno08x_localization.xml",
                              "shared/pipelines/parallel_wheels_bno08x_no_camera.xml",
                              "shared/pipelines/parallel_wheels_bno08x_camera.xml"}) {
            std::filesystem::copy_file(kConfigDir + "/" + f, root / f);
        }
    }
    ~ProfileTree() {
        std::error_code ec;
        std::filesystem::remove_all(root, ec);
    }

    void fill(const char* templ, const char* out, const std::map<std::string, std::string>& v) {
        std::ofstream f(root / out, std::ios::binary);
        f << fillTemplate(readFile(kConfigDir + "/" + templ), v);
    }
    void write(const char* out, const std::string& text) {
        std::ofstream f(root / out, std::ios::binary);
        f << text;
    }
    std::string path(const char* rel) const { return (root / rel).string(); }
};

std::string localizationSection(const std::string& xml) {
    const std::size_t a = xml.find("<Localization");
    const std::size_t b = xml.find("</Localization>");
    return a == std::string::npos || b == std::string::npos ? std::string()
                                                            : xml.substr(a, b - a);
}

} // namespace

TEST(ParallelWheelProfiles, CheckedInProfilesNameTheMissingMeasuredRobot) {
    for (const char* profile : {"/override/diagnostics/parallel_wheels_bno08x.xml",
                                "/override/diagnostics/parallel_wheels_bno08x_camera.xml"}) {
        ResolvedConfiguration resolved;
        std::string           err;
        EXPECT_FALSE(resolveConfiguration(kConfigDir + profile, resolved, err)) << profile;
        EXPECT_NE(err.find("gatr2_parallel_wheels_bno08x.xml"), std::string::npos) << err;
    }
    // the measurements are still tokens, and there is no third wheel
    const std::string robot =
        readFile(kConfigDir + "/shared/robots/gatr2_parallel_wheels_bno08x.xml.in");
    EXPECT_NE(robot.find("@MEASURE_WHEEL_A_Y_M@"), std::string::npos);
    EXPECT_EQ(robot.find("encoder_c\""), std::string::npos);
    EXPECT_EQ(robot.find("channel=\"2\""), std::string::npos);
}

TEST(ParallelWheelProfiles, LocalizationOnlyNeedsNoCameraAndNoThirdWheel) {
    ProfileTree tree;
    tree.fill("shared/robots/gatr2_parallel_wheels_bno08x.xml.in",
              "shared/robots/gatr2_parallel_wheels_bno08x.xml", kRobotValues);

    ResolvedConfiguration a;
    std::string           err;
    ASSERT_TRUE(resolveConfiguration(tree.path("override/diagnostics/parallel_wheels_bno08x.xml"),
                                     a, err))
        << err;
    EXPECT_EQ(a.id, "override_parallel_wheels_bno08x");
    EXPECT_EQ(a.files.size(), 5u);   // profile, robot, field, pipeline, localization
    for (const char* absent : {"libcamera_camera", "camera_frame", "apriltag", "robot_frame_map",
                               "encoder_c\"", "channel=\"2\"", "InitialPlacement"}) {
        EXPECT_EQ(a.xml.find(absent), std::string::npos) << absent;
    }
    EXPECT_NE(a.xml.find("LateralMotion"), std::string::npos);
    EXPECT_NE(a.xml.find("type=\"field_map\""), std::string::npos);   // map for display

    // the host has no Pico or Brain UART: dead links, still a normal build
    FunctionRegistry functions;
    registerAll(functions);
    auto system = System::buildFromString(a.xml.c_str(), functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->worldEstimation().estimatorType(), "noop");
    EXPECT_EQ(system->localization().estimatorType(), "planar_motion_integrator");
    int64_t now = 0;
    for (int i = 0; i < 20; ++i) {
        now += 10;
        system->step(hostTime(now));
    }
    EXPECT_FALSE(system->robot().valid);
    EXPECT_FALSE(system->robot().initialized);
    EXPECT_TRUE(system->detectionFrames().empty());

    const std::string hello = helloDocument(*system, hostTime(now));
    EXPECT_NE(hello.find("\"camera_sensors\":[]"), std::string::npos);
    EXPECT_NE(hello.find("\"landmarks\":[{"), std::string::npos);   // nominal map for display
    const std::string snap = snapshotDocument(*system, InspectionServiceStats{}, hostTime(now));
    EXPECT_NE(snap.find("\"detection_frames\":[]"), std::string::npos);
    EXPECT_NE(snap.find("\"field_objects\":[]"), std::string::npos);
}

TEST(ParallelWheelProfiles, CameraProfileSharesTheExactLocalization) {
    ProfileTree tree;
    tree.fill("shared/robots/gatr2_parallel_wheels_bno08x.xml.in",
              "shared/robots/gatr2_parallel_wheels_bno08x.xml", kRobotValues);
    tree.fill("shared/robots/gatr2_front_camera.xml.in", "shared/robots/gatr2_front_camera.xml",
              kCameraValues);

    ResolvedConfiguration a, b;
    std::string           err;
    ASSERT_TRUE(resolveConfiguration(tree.path("override/diagnostics/parallel_wheels_bno08x.xml"),
                                     a, err))
        << err;
    ASSERT_TRUE(resolveConfiguration(
        tree.path("override/diagnostics/parallel_wheels_bno08x_camera.xml"), b, err))
        << err;
    const std::string loc_a = localizationSection(a.xml);
    ASSERT_FALSE(loc_a.empty());
    EXPECT_EQ(loc_a, localizationSection(b.xml));
    EXPECT_NE(b.xml.find("libcamera_camera"), std::string::npos);
    EXPECT_NE(b.xml.find("type=\"apriltag\""), std::string::npos);
    EXPECT_NE(b.xml.find("assume_level"), std::string::npos);
    EXPECT_EQ(b.xml.find("encoder_c\""), std::string::npos);

    FunctionRegistry functions;
    registerAll(functions);
#if NAVIGATR_HAVE_LIBCAMERA
    EXPECT_NE(System::buildFromString(b.xml.c_str(), functions, err), nullptr) << err;
#else
    EXPECT_EQ(System::buildFromString(b.xml.c_str(), functions, err), nullptr);
    EXPECT_NE(err.find("NAVIGATR_WITH_LIBCAMERA"), std::string::npos) << err;
#endif
}

namespace
{

// A synthetic stand-in for the measured robot: the same ids as the real
// description, two forward wheels at unequal offsets, driving forward along
// a circle so the body never moves sideways.
const char* kRigRobot = R"(<Robot>
    <Resources>
        <Resource id="brain_uart" type="memory_link"/>
        <Resource id="wheel_geometry" type="wheel_geometry">
            <Wheel id="forward_wheel_a" sensor_id="tracking_encoder_a" radius_m="0.0254"
                   position_x_m="0.05" position_y_m="0.15" measurement_angle_deg="0"
                   direction="positive"/>
            <Wheel id="forward_wheel_b" sensor_id="tracking_encoder_b" radius_m="0.0254"
                   position_x_m="-0.02" position_y_m="-0.08" measurement_angle_deg="0"
                   direction="positive"/>
        </Resource>
        <Resource id="rig" type="synthetic_rig">
            <Field resource_id="override_field"/>
            <Wheels resource_id="wheel_geometry" counts_per_revolution="4000"/>
            <Trajectory center_x_m="1.7832" center_y_m="1.7832" radius_m="0.8"
                        period_s="30" facing="tangent" start_deg="180" hold_s="5"/>
            <Telemetry tick_hz="50" device_offset_ms="5000" gyro_bias_mdps="800"/>
            <Attitude mode="unavailable"/>
            <Output id="encoder_a" wheel_id="forward_wheel_a"/>
            <Output id="encoder_b" wheel_id="forward_wheel_b"/>
            <Output id="imu" channel="imu"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="tracking_encoder_a" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000" invert="false"/>
        </Sensor>
        <Sensor id="tracking_encoder_b" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000" invert="false"/>
        </Sensor>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="rig" output_id="imu"/>
            <Calibration invert="false"/>
        </Sensor>
    </Sensors>
</Robot>)";

// Profile A's checked-in pipeline and field with the rig in place of the
// measured robot.
std::string writeRigProfile(ProfileTree& tree) {
    tree.write("shared/robots/rig_robot.xml", kRigRobot);
    tree.write("override/diagnostics/rig_profile.xml",
               R"(<Configuration id="parallel_wheels_rig">
    <Loop rate_hz="100"/>
    <Robot file="../../shared/robots/rig_robot.xml"/>
    <Field file="../field.xml"/>
    <Pipeline file="../../shared/pipelines/parallel_wheels_bno08x_no_camera.xml"/>
</Configuration>)");
    return tree.path("override/diagnostics/rig_profile.xml");
}

} // namespace

TEST(ParallelWheelProfiles, NoCameraPipelineTracksARigThroughTheOrdinaryExecutors) {
    ProfileTree      tree;
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromFile(writeRigProfile(tree), functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->worldEstimation().estimatorType(), "noop");
    const auto rig = system->resources().require<const SyntheticRig>(ResourceId{"rig"}, err);
    ASSERT_NE(rig, nullptr) << err;

    int64_t now = 0;
    for (int i = 0; i < 1500; ++i) {   // 5 s still for the gyro bias, then 10 s of arc
        now += 10;
        system->step(hostTime(now));
    }
    const RobotState& robot = system->robot();
    ASSERT_TRUE(robot.valid);
    EXPECT_FALSE(robot.initialized);   // odometry only until the Brain places it
    ASSERT_TRUE(robot.measuredAtHost.isSet());

    // odometry starts where the rig sat still; compare motion since then
    const Pose2D start = rig->truthAt(hostTime(1)).pose;
    const Pose2D truth = rig->truthAt(robot.measuredAtHost).pose;
    const Pose2D moved = compose(inverse(start), truth);
    const Pose2D odom  = robot.odom_pose;
    EXPECT_GT(std::hypot(moved.x_m, moved.y_m), 1.0);
    EXPECT_NEAR(odom.x_m, moved.x_m, 0.03);
    EXPECT_NEAR(odom.y_m, moved.y_m, 0.03);
    EXPECT_NEAR(radToDeg(wrapAngle(odom.heading_rad - moved.heading_rad)), 0.0, 2.0);

    // world estimation stays empty; the map is only configured geometry
    EXPECT_TRUE(system->field().objects.empty());
    EXPECT_TRUE(system->detectionFrames().empty());
    const std::string snap = snapshotDocument(*system, InspectionServiceStats{}, hostTime(now));
    EXPECT_NE(snap.find("\"initialized\":false"), std::string::npos);
    EXPECT_NE(snap.find("\"trail\":[{"), std::string::npos);
    EXPECT_NE(snap.find("\"field_objects\":[]"), std::string::npos);
    system.reset();
}

namespace
{

std::vector<uint8_t> requestBytes(const gatr2::BrainRequest& r) {
    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeBrainRequest(r, buf.data(), gatr2::kMaxFrameLen));
    EXPECT_FALSE(buf.empty());
    return buf;
}

// Every brain reply the Pi wrote since the last call.
std::vector<gatr2::BrainReply> takeReplies(MemoryLink& link) {
    std::vector<gatr2::BrainReply> out;
    gatr2::FrameReader             reader;
    for (uint8_t b : link.output().takeAll()) {
        if (!reader.push(b)) {
            continue;
        }
        do {
            gatr2::BrainReply reply;
            EXPECT_TRUE(gatr2::decodeBrainReply(reader.frame(), reader.frameLen(), reply));
            out.push_back(reply);
        } while (reader.next());
    }
    return out;
}

} // namespace

TEST(ParallelWheelProfiles, BrainPlacesTheRobotAndReadsTheField) {
    ProfileTree      tree;
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromFile(writeRigProfile(tree), functions, err);
    ASSERT_NE(system, nullptr) << err;
    const auto rig = system->resources().require<const SyntheticRig>(ResourceId{"rig"}, err);
    ASSERT_NE(rig, nullptr) << err;
    auto        link  = system->resources().require<SerialLink>(ResourceId{"brain_uart"}, err);
    MemoryLink* brain = dynamic_cast<MemoryLink*>(link.get());
    ASSERT_NE(brain, nullptr);

    int64_t clock_us = 1000000;
    int64_t now_ms   = 0;
    brain->setClock([&] { return clock_us; });
    const auto step = [&] {
        clock_us += 5000;
        now_ms += 5;
        system->step(hostTime(now_ms));
        return takeReplies(*brain);
    };
    const auto ask = [&](const gatr2::BrainRequest& r) {
        brain->input().feed(requestBytes(r));
        const std::vector<gatr2::BrainReply> replies = step();
        EXPECT_EQ(replies.size(), 1u);
        return replies.empty() ? gatr2::BrainReply{} : replies.front();
    };
    step();   // the first drain after start never replies

    // gyro bias while the rig holds still, then two seconds of odometry only
    for (int i = 0; i < 1400; ++i) {
        step();
    }
    ASSERT_TRUE(system->robot().valid);
    EXPECT_FALSE(system->robot().initialized);

    gatr2::BrainRequest hello;
    hello.op                      = gatr2::kOpHello;
    hello.request_id              = 1;
    hello.nonce                   = 0x5eed;
    const gatr2::BrainReply opened = ask(hello);
    ASSERT_EQ(opened.result, gatr2::kResultOk);
    const uint32_t session = opened.session;

    gatr2::BrainRequest get;
    get.op         = gatr2::kOpGetState;
    get.session    = session;
    get.request_id = 2;
    gatr2::BrainReply state = ask(get);
    ASSERT_EQ(state.result, gatr2::kResultOk);
    EXPECT_TRUE(state.state.robot_flags & gatr2::kRobotPoseValid);
    EXPECT_FALSE(state.state.robot_flags & gatr2::kRobotLocalized);   // odometry only

    // the Brain supplies the starting field pose; here the rig's truth
    const Pose2D        start = rig->truthAt(system->robot().measuredAtHost).pose;
    gatr2::BrainRequest place;
    place.op           = gatr2::kOpSetPose;
    place.session      = session;
    place.request_id   = 3;
    place.x_mm         = static_cast<int32_t>(std::llround(start.x_m * 1000.0));
    place.y_mm         = static_cast<int32_t>(std::llround(start.y_m * 1000.0));
    place.heading_cdeg = static_cast<int32_t>(std::llround(radToDeg(start.heading_rad) * 100.0));
    gatr2::BrainReply placed = ask(place);
    for (int i = 0; i < 20 && placed.result == gatr2::kResultPending; ++i) {
        placed = ask(place);   // same request id: a retry, never a second placement
    }
    ASSERT_EQ(placed.result, gatr2::kResultOk);
    EXPECT_TRUE(system->robot().initialized);
    EXPECT_EQ(system->robot().placement_origin, "command");
    EXPECT_EQ(system->robot().placement_session, session);
    EXPECT_EQ(system->robot().anchor_revision, 1u);

    // the field pose now follows the rig
    for (int i = 0; i < 400; ++i) {
        step();
    }
    const Pose2D truth = rig->truthAt(system->robot().measuredAtHost).pose;
    const Pose2D field = system->robot().fieldPose();
    EXPECT_NEAR(field.x_m, truth.x_m, 0.03);
    EXPECT_NEAR(field.y_m, truth.y_m, 0.03);
    EXPECT_NEAR(radToDeg(wrapAngle(field.heading_rad - truth.heading_rad)), 0.0, 2.0);

    // the profile serves the Override field: the map, then an estimate in the
    // placed frame, every object nominal with noop world estimation
    get.request_id = 4;
    state          = ask(get);
    ASSERT_EQ(state.result, gatr2::kResultOk);
    EXPECT_NE(state.state.map_id, 0u);
    EXPECT_NE(state.state.estimate_id, 0u);
    uint16_t   rid       = 5;
    const auto readWhole = [&](uint8_t kind, uint32_t doc_id) {
        std::vector<uint8_t> bytes;
        gatr2::BrainRequest  read;
        read.op       = gatr2::kOpReadDoc;
        read.session  = session;
        read.doc_kind = kind;
        read.doc_id   = doc_id;
        read.max_len  = gatr2::kDocChunkMax;
        for (;;) {
            read.request_id             = rid++;
            read.doc_offset             = static_cast<uint16_t>(bytes.size());
            const gatr2::BrainReply got = ask(read);
            EXPECT_EQ(got.result, gatr2::kResultOk);
            if (got.result != gatr2::kResultOk || got.data_len == 0) {
                return bytes;
            }
            bytes.insert(bytes.end(), got.data, got.data + got.data_len);
            if (bytes.size() >= got.doc_total_len) {
                EXPECT_EQ(gatr2::crc32(bytes.data(), static_cast<uint32_t>(bytes.size())),
                          got.doc_crc32);
                return bytes;
            }
        }
    };
    const std::vector<uint8_t> map = readWhole(gatr2::kDocFieldMap, state.state.map_id);
    EXPECT_EQ(map.size(), gatr2::fieldMapLen(17));
    EXPECT_EQ(gatr2::crc32(map.data(), static_cast<uint32_t>(map.size())), state.state.map_id);
    const std::vector<uint8_t> estimate =
        readWhole(gatr2::kDocFieldEstimate, state.state.estimate_id);
    ASSERT_EQ(gatr2::validateFieldEstimate(estimate.data(), static_cast<uint16_t>(estimate.size()),
                                           map.data(), static_cast<uint16_t>(map.size()),
                                           state.state.map_id),
              gatr2::DocError::kNone);
    gatr2::FieldEstimateHeader header;
    ASSERT_TRUE(gatr2::decodeFieldEstimateHeader(estimate.data(),
                                                 static_cast<uint16_t>(estimate.size()), header));
    EXPECT_EQ(header.anchor_revision, 1u);
    EXPECT_EQ(header.odometry_epoch, state.state.odometry_epoch);
    for (uint16_t i = 0; i < header.object_count; ++i) {
        gatr2::FieldEstimateRecord r;
        ASSERT_TRUE(gatr2::decodeFieldEstimateRecord(estimate.data(),
                                                     static_cast<uint16_t>(estimate.size()), i, r));
        EXPECT_EQ(r.source, gatr2::kEstimateSourceNominal);
    }
    get.request_id = rid++;
    state          = ask(get);
    ASSERT_EQ(state.result, gatr2::kResultOk);
    EXPECT_EQ(state.state.profile_state, gatr2::kProfileNone);   // XML robot, no profile
    EXPECT_TRUE(state.state.robot_flags & gatr2::kRobotLocalized);
    EXPECT_TRUE(state.state.robot_flags & gatr2::kRobotAnchorCommand);
    EXPECT_EQ(system->robot().anchor_revision, 1u);   // nothing moved the anchor again

    const std::string snap = snapshotDocument(*system, InspectionServiceStats{}, hostTime(now_ms));
    EXPECT_NE(snap.find("\"initialized\":true"), std::string::npos);
    EXPECT_NE(snap.find("\"placement_origin\":\"command\""), std::string::npos);
    system.reset();
}
