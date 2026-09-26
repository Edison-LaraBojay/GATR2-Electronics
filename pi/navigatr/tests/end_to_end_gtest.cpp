// end_to_end_gtest.cpp
// The whole system from one XML string: Pico telemetry into three
// individually configured encoder channels, geometry-owned odometry, chord
// prediction, map-anchored world, and the brain link. A 1 m square must close
// on the commanded start, every request gets exactly one decodable reply, and
// nothing is sent without a request.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "common/frame_codec.h"
#include "impl/resources/cameras.h"
#include "impl/resources/serial_links.h"
#include "math/angles.h"
#include "resources/camera.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;

namespace
{

// reference three wheel layout: left (0, 0.13), right (0, -0.13), rear
// (-0.12, 0) measuring at 90 degrees; radius 0.0254 m, 4000 counts per rev
const char* kFullConfig = R"(
<System>
    <Loop rate_hz="200"/>
    <Resources>
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="encoder_c" channel="2"/>
            <Output id="imu" channel="imu"/>
        </Resource>
        <Resource id="brain_uart" type="memory_link"/>
        <Resource id="override_field" type="field_map">
            <Landmark id="center_goal">
                <NominalPose calibration_status="verified"
                             x_m="1.8" y_m="1.8" heading_deg="0"/>
            </Landmark>
        </Resource>
        <Resource id="camera_device" type="idle_camera"/>
        <Resource id="tag_detector" type="apriltag_detector">
            <Family name="tag36h11" detection_size_m="0.06"/>
            <Detector quad_decimate="1.0" nthreads="1"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="front_camera" type="camera_frame">
            <Source resource_id="camera_device" output_id="frame"/>
        </Sensor>
        <Sensor id="tracking_encoder_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="tracking_encoder_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="tracking_encoder_c" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_c"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="brain_link">
            <Serial resource_id="brain_uart"/>
        </CommandCollection>
        <Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">
                <TrackingWheel sensor_id="tracking_encoder_a" label="left"
                               radius_m="0.0254" position_x_m="0"
                               position_y_m="0.13" measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="tracking_encoder_b" label="right"
                               radius_m="0.0254" position_x_m="0"
                               position_y_m="-0.13" measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="tracking_encoder_c" label="rear"
                               radius_m="0.0254" position_x_m="-0.12"
                               position_y_m="0" measurement_angle_deg="90" direction="positive"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Observation id="imu_heading" type="imu_heading_increment">
                <Input sensor_id="robot_imu"/>
                <Calibration bias_samples="200"/>
                <Output observation_id="imu_heading"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
            </Estimator>
        </Localization>
        <WorldEstimation>
            <Estimator id="goals" type="apriltag">
                <FieldMap resource_id="override_field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="tag_detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <LandmarkEstimation commit="never"/>
            </Estimator>
        </WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="brain_link">
            <Serial resource_id="brain_uart"/>
            <Health fresh_ms="150">
                <Encoder sensor_id="tracking_encoder_a"/>
                <Encoder sensor_id="tracking_encoder_b"/>
                <Encoder sensor_id="tracking_encoder_c"/>
                <Gyro sensor_id="robot_imu"/>
                <BiasCal function_id="imu_heading"/>
            </Health>
            <FieldObject object_id="center_goal" wire_id="1"/>
        </Publishing>
    </Pipeline>
</System>
)";

std::vector<uint8_t> sensorPacket(uint8_t seq, uint32_t stamp, int32_t enc0, int32_t enc1,
                                  int32_t enc2, int32_t gyro) {
    gatr2::SensorSample s{};
    s.seq      = seq;
    s.stamp_ms = stamp;
    s.mask     = gatr2::kSensorEnc0 | gatr2::kSensorEnc1 | gatr2::kSensorEnc2 |
             gatr2::kSensorGyroZ;
    s.enc[0] = enc0;
    s.enc[1] = enc1;
    s.enc[2] = enc2;
    s.gyro_z = gyro;

    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeSensorFrame(s, buf.data(), gatr2::kMaxFrameLen));
    return buf;
}

std::vector<uint8_t> requestBytes(const gatr2::BrainRequest& r) {
    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeBrainRequest(r, buf.data(), gatr2::kMaxFrameLen));
    return buf;
}

// Every reply frame in the link output; each must decode.
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

// A camera that is alive but never delivers a frame: the world estimator
// seeds the map and observes nothing, so the published object is valid
// from the map and never observed.
class IdleCamera : public CameraDevice
{
public:
    bool                    alive() const override { return true; }
    const CameraIntrinsics* intrinsics() const override { return nullptr; }
    FrameId engineeringFrame() const override { return FrameId{"front_camera_engineering"}; }
    std::optional<CameraFrameData> latestFrame(uint64_t, uint32_t) override {
        return std::nullopt;
    }
};

// The brain link clock is the test's: every step advances it with the host
// clock, so reply windows are deterministic.
struct Rig {
    FunctionRegistry        functions;
    std::unique_ptr<System> system;
    MemoryLink*             pico     = nullptr;
    MemoryLink*             brain    = nullptr;
    int64_t                 clock_us = 0;

    explicit Rig(const char* xml) {
        registerAll(functions);
        functions.add(FunctionKey{"idle_camera"},
                      ResourceMakeFunction([](const ConfigNode&, ResourceInitializationContext&,
                                              std::string&) {
                          return cameraResource(std::make_shared<IdleCamera>(),
                                                OutputId{"frame"});
                      }));
        std::string err;
        system = System::buildFromString(xml, functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system == nullptr) {
            return;
        }
        auto pico_link =
            system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
        pico = dynamic_cast<MemoryLink*>(pico_link.get());
        auto brain_link =
            system->resources().require<SerialLink>(ResourceId{"brain_uart"}, err);
        brain = dynamic_cast<MemoryLink*>(brain_link.get());
        if (brain != nullptr) {
            brain->setClock([this] { return clock_us; });
        }
    }

    // One inline cycle at host time ms; the link clock follows it.
    void step(int64_t ms) {
        clock_us = ms * 1000;
        system->step(hostTime(ms));
    }
};

} // namespace

TEST(EndToEnd, SquarePathClosesOnCommandedStart) {
    Rig rig(kFullConfig);
    ASSERT_NE(rig.system, nullptr);
    ASSERT_NE(rig.pico, nullptr);
    ASSERT_NE(rig.brain, nullptr);

    // simulated truth in the body frame, wheel travel from the rigid model
    const double kA = -0.13, kB = 0.13, kC = -0.12;   // x*uy - y*ux per wheel
    const double kCountsPerMeter = 4000.0 / (2.0 * kPi * 0.0254);
    const int    kBiasMdps       = 2000;
    const int    kCal = 205, kDrive = 200, kTurn = 100, kTail = 10;

    double   travel_a = 0.0, travel_b = 0.0, travel_c = 0.0;
    uint32_t stamp = 1000;
    uint8_t  seq   = 0;
    int      cycle = 0;

    // brain: open a session, start at (0.61, 0.457) facing +y, poll state
    // every 4 cycles, select landmark wire id 1 at cycle 300
    uint32_t          session = 0;
    uint16_t          rid     = 0;
    int               sent    = 0;
    gatr2::BrainReply last_state;
    bool              placed = false, selected = false;

    auto step = [&](bool driving, bool turning) {
        ++cycle;
        stamp += 5;
        const double dtheta = turning ? degToRad(0.9) : 0.0;   // 180 dps at 5 ms
        const double dx     = driving ? 0.005 : 0.0;

        travel_a += dx + kA * dtheta;
        travel_b += dx + kB * dtheta;
        travel_c += kC * dtheta;

        const int32_t gyro =
            kBiasMdps + (turning ? 180000 : 0);   // wire millidegrees per second
        rig.pico->input().feed(sensorPacket(
            seq++, stamp, static_cast<int32_t>(std::llround(travel_a * kCountsPerMeter)),
            static_cast<int32_t>(std::llround(travel_b * kCountsPerMeter)),
            static_cast<int32_t>(std::llround(travel_c * kCountsPerMeter)), gyro));

        gatr2::BrainRequest request;
        request.session    = session;
        request.request_id = static_cast<uint16_t>(rid + 1);
        if (cycle == 2) {
            request.op    = gatr2::kOpHello;
            request.nonce = 0x5EED;
        } else if (cycle == 3) {
            request.op           = gatr2::kOpSetPose;
            request.x_mm         = 610;
            request.y_mm         = 457;
            request.heading_cdeg = 9000;
        } else if (cycle == 300) {
            request.op           = gatr2::kOpSelectLandmark;
            request.landmark_id  = 1;
            request.select_flags = gatr2::kSelectFlagSelected;
        } else if (cycle > 3 && cycle % 4 == 0) {
            request.op = gatr2::kOpGetState;
        }
        if (request.op != 0) {
            rig.brain->input().feed(requestBytes(request));
            rid = request.request_id;
            ++sent;
        }

        rig.step(cycle);

        const std::vector<gatr2::BrainReply> replies = takeReplies(*rig.brain);
        ASSERT_EQ(replies.size(), request.op != 0 ? 1u : 0u) << "cycle " << cycle;
        if (replies.empty()) {
            return;
        }
        const gatr2::BrainReply& reply = replies.front();
        EXPECT_EQ(reply.request_id, request.request_id);
        EXPECT_EQ(reply.result, gatr2::kResultOk) << "cycle " << cycle;
        if (reply.op == gatr2::kOpHello) {
            session = reply.session;
        } else if (reply.op == gatr2::kOpSetPose) {
            placed = reply.result == gatr2::kResultOk;
        } else if (reply.op == gatr2::kOpSelectLandmark) {
            selected = reply.result == gatr2::kResultOk;
        } else {
            last_state = reply;
        }
    };

    for (int i = 0; i < kCal; ++i) {
        step(false, false);
    }
    for (int leg = 0; leg < 4; ++leg) {
        for (int i = 0; i < kDrive; ++i) {
            step(true, false);
        }
        for (int i = 0; i < kTurn; ++i) {
            step(false, true);
        }
    }
    for (int i = 0; i < kTail; ++i) {
        step(false, false);
    }

    const RobotState& robot = rig.system->robot();
    EXPECT_TRUE(robot.valid);
    EXPECT_TRUE(robot.initialized);
    EXPECT_NEAR(robot.fieldPose().x_m, 0.610, 0.003);
    EXPECT_NEAR(robot.fieldPose().y_m, 0.457, 0.003);
    EXPECT_NEAR(radToDeg(wrapAngle(robot.fieldPose().heading_rad - kPi / 2.0)), 0.0, 0.05);

    EXPECT_EQ(rig.system->diagnostics().links.at("pico_uart").seq_gaps, 0u);
    EXPECT_EQ(rig.system->diagnostics().links.at("pico_uart").decode_errors, 0u);
    const LinkStats& link = rig.system->diagnostics().links.at("brain_uart");
    EXPECT_EQ(link.requests, static_cast<uint32_t>(sent));
    EXPECT_EQ(link.replies, static_cast<uint32_t>(sent));

    EXPECT_TRUE(placed);
    EXPECT_TRUE(selected);
    const gatr2::BrainState& s = last_state.state;
    EXPECT_EQ(last_state.session, session);
    EXPECT_TRUE(s.robot_flags & gatr2::kRobotPoseValid);
    EXPECT_TRUE(s.robot_flags & gatr2::kRobotLocalized);
    EXPECT_TRUE(s.robot_flags & gatr2::kRobotAnchorCommand);
    EXPECT_FALSE(s.robot_flags & gatr2::kRobotAnchorConfigured);
    EXPECT_EQ(s.anchor_revision, 1u);
    EXPECT_TRUE(s.health & gatr2::kHealthEncodersFresh);
    EXPECT_TRUE(s.health & gatr2::kHealthGyroFresh);
    EXPECT_TRUE(s.health & gatr2::kHealthBiasCalibrated);
    EXPECT_FALSE(s.health & gatr2::kHealthVisionAlive);

    // wire id 1 maps to center_goal: the map nominal, never observed
    EXPECT_EQ(s.landmark_id, 1u);
    EXPECT_EQ(s.landmark_source, gatr2::kLandmarkSourceNominal);
    EXPECT_EQ(s.lm_x_mm, 1800);
    EXPECT_EQ(s.lm_y_mm, 1800);
    EXPECT_EQ(s.landmark_age_ms, 0u);

    EXPECT_NEAR(s.x_mm, 610, 3);
    EXPECT_NEAR(s.y_mm, 457, 3);
    EXPECT_NEAR(s.heading_cdeg, 9000, 5);
}

TEST(EndToEnd, NothingAttachedStillRunsAndSendsNothingUnasked) {
    Rig rig(kFullConfig);
    ASSERT_NE(rig.system, nullptr);

    for (int i = 0; i < 20; ++i) {
        rig.step(i + 1);
    }

    EXPECT_EQ(rig.system->cycle(), 20u);
    EXPECT_FALSE(rig.system->robot().valid);
    EXPECT_EQ(rig.system->sensorMap().at(SensorId{"tracking_encoder_a"}).state,
              SourceState::kNoDataYet);
    EXPECT_TRUE(rig.brain->output().takeAll().empty());   // the brain initiates

    // asked, it reports an invalid pose and stale health
    gatr2::BrainRequest hello;
    hello.op         = gatr2::kOpHello;
    hello.request_id = 1;
    hello.nonce      = 0xAB;
    rig.brain->input().feed(requestBytes(hello));
    rig.step(21);
    std::vector<gatr2::BrainReply> replies = takeReplies(*rig.brain);
    ASSERT_EQ(replies.size(), 1u);

    gatr2::BrainRequest poll;
    poll.op         = gatr2::kOpGetState;
    poll.session    = replies[0].session;
    poll.request_id = 2;
    rig.brain->input().feed(requestBytes(poll));
    rig.step(22);
    replies = takeReplies(*rig.brain);
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].result, gatr2::kResultOk);
    EXPECT_FALSE(replies[0].state.robot_flags & gatr2::kRobotPoseValid);
    EXPECT_FALSE(replies[0].state.robot_flags & gatr2::kRobotAgeKnown);
    EXPECT_FALSE(replies[0].state.health & gatr2::kHealthEncodersFresh);
    EXPECT_EQ(replies[0].state.landmark_source, gatr2::kLandmarkSourceNone);

    // the map-seeded world exists with no data at all
    EXPECT_EQ(rig.system->field().objects.count(FieldObjectId{"center_goal"}), 1u);
}

namespace
{

// Fused odometry rigs: wheels plus the IMU heading constraint, gyro in the
// solve. Only the wheel declarations differ between the two- and
// three-wheel variants.
std::string fusedConfig(bool three_wheel) {
    std::string wheels;
    if (three_wheel) {
        wheels = R"(
                <TrackingWheel sensor_id="enc_a" label="left" radius_m="0.0254"
                               position_x_m="0" position_y_m="0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_b" label="right" radius_m="0.0254"
                               position_x_m="0" position_y_m="-0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_c" label="rear" radius_m="0.0254"
                               position_x_m="-0.12" position_y_m="0"
                               measurement_angle_deg="90" direction="positive"/>)";
    } else {
        wheels = R"(
                <TrackingWheel sensor_id="enc_a" label="forward" radius_m="0.0254"
                               position_x_m="0" position_y_m="0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_b" label="lateral" radius_m="0.0254"
                               position_x_m="-0.12" position_y_m="0"
                               measurement_angle_deg="90" direction="positive"/>)";
    }
    std::string sensors = R"(
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>)";
    if (three_wheel) {
        sensors += R"(
        <Sensor id="enc_c" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_c"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>)";
    }
    return std::string(R"(
<System>
    <Loop rate_hz="200"/>
    <Resources>
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="encoder_c" channel="2"/>
            <Output id="imu" channel="imu"/>
        </Resource>
    </Resources>
    <Sensors>)") +
           sensors + R"(
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">)" +
           wheels + R"(
                <HeadingConstraint sensor_id="robot_imu" bias_samples="200"
                                   max_calibration_travel_m="0.005"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
            </Estimator>
        </Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
}

// Drives calibration, a 1 m square with in-place 90 degree turns, and a
// lateral out-and-back through real Pico packets, and expects the fused
// odometry to close on the start.
void driveFusedClosure(bool three_wheel) {
    Rig rig(fusedConfig(three_wheel).c_str());
    ASSERT_NE(rig.system, nullptr);
    ASSERT_NE(rig.pico, nullptr);

    const double kA = -0.13;               // forward wheel lever arm
    const double kB = three_wheel ? 0.13 : -0.12;
    const double kC = -0.12;               // three-wheel lateral lever arm
    const double kCountsPerMeter = 4000.0 / (2.0 * kPi * 0.0254);
    const int    kBiasMdps       = 2000;

    double   travel_a = 0.0, travel_b = 0.0, travel_c = 0.0;
    uint32_t stamp = 1000;
    uint8_t  seq   = 0;
    int      cycle = 0;

    const auto step = [&](double dx, double dy, double dtheta_deg) {
        ++cycle;
        stamp += 5;
        const double dtheta = degToRad(dtheta_deg);
        if (three_wheel) {
            travel_a += dx + kA * dtheta;
            travel_b += dx + kB * dtheta;
            travel_c += dy + kC * dtheta;
        } else {
            travel_a += dx + kA * dtheta;   // forward wheel
            travel_b += dy + kB * dtheta;   // lateral wheel
        }
        const auto rate_mdps =
            kBiasMdps + static_cast<int>(std::llround(dtheta_deg / 0.005 * 1000.0));
        rig.pico->input().feed(sensorPacket(
            seq++, stamp,
            static_cast<int32_t>(std::llround(travel_a * kCountsPerMeter)),
            static_cast<int32_t>(std::llround(travel_b * kCountsPerMeter)),
            static_cast<int32_t>(std::llround(travel_c * kCountsPerMeter)), rate_mdps));
        rig.system->step(hostTime(cycle));
    };

    for (int i = 0; i < 205; ++i) {
        step(0.0, 0.0, 0.0);   // stationary gyro bias calibration
    }
    for (int leg = 0; leg < 4; ++leg) {
        for (int i = 0; i < 200; ++i) {
            step(0.005, 0.0, 0.0);   // forward 1 m
        }
        for (int i = 0; i < 100; ++i) {
            step(0.0, 0.0, 0.9);   // in-place +90 degrees at 180 dps
        }
    }
    for (int i = 0; i < 100; ++i) {
        step(0.0, 0.005, 0.0);   // strafe left 0.5 m
    }
    for (int i = 0; i < 100; ++i) {
        step(0.0, -0.005, 0.0);   // and back
    }
    for (int i = 0; i < 10; ++i) {
        step(0.0, 0.0, 0.0);
    }

    const RobotState& robot = rig.system->robot();
    EXPECT_TRUE(robot.valid);
    EXPECT_NEAR(robot.odom_pose.x_m, 0.0, 0.004);
    EXPECT_NEAR(robot.odom_pose.y_m, 0.0, 0.004);
    EXPECT_NEAR(radToDeg(wrapAngle(robot.odom_pose.heading_rad)), 0.0, 0.1);
    EXPECT_EQ(rig.system->diagnostics().links.at("pico_uart").decode_errors, 0u);
}

} // namespace

TEST(EndToEnd, ThreeWheelImuFusedMotionCloses) { driveFusedClosure(true); }

TEST(EndToEnd, TwoWheelImuFusedMotionCloses) { driveFusedClosure(false); }
