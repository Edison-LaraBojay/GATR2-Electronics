// end_to_end_gtest.cpp
// The whole system from one XML string: Pico telemetry into three
// individually configured encoder channels, geometry-owned odometry, chord
// prediction, map-anchored world, and the brain wire out. A 1 m square must
// close on the commanded start and every published packet must decode.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "common/frame_codec.h"
#include "impl/resources/serial_links.h"
#include "math/angles.h"
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
        </Resource>
        <Resource id="brain_uart" type="memory_link"/>
        <Resource id="override_field" type="field_map">
            <Landmark id="center_goal">
                <NominalPose x_m="1.8" y_m="1.8" heading_deg="0"/>
            </Landmark>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="tracking_encoder_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" channel="0"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="tracking_encoder_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" channel="1"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="tracking_encoder_c" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" channel="2"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" channel="imu"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="vex_brain_serial">
            <Serial resource_id="brain_uart"/>
        </CommandCollection>
        <Preprocessing type="configured_collection">
            <Preprocessor id="tracking_motion"
                          type="tracking_wheel_odometry">
                <TrackingWheel sensor_id="tracking_encoder_a" label="left"
                               radius_m="0.0254" position_x_m="0"
                               position_y_m="0.13" measurement_angle_deg="0"/>
                <TrackingWheel sensor_id="tracking_encoder_b" label="right"
                               radius_m="0.0254" position_x_m="0"
                               position_y_m="-0.13" measurement_angle_deg="0"/>
                <TrackingWheel sensor_id="tracking_encoder_c" label="rear"
                               radius_m="0.0254" position_x_m="-0.12"
                               position_y_m="0" measurement_angle_deg="90"/>
                <Output artifact_id="tracking_motion_delta"/>
            </Preprocessor>
            <Preprocessor id="imu_normalization"
                          type="imu_normalization">
                <Input sensor_id="robot_imu"/>
                <Calibration bias_samples="200"/>
                <Output artifact_id="imu_orientation"/>
            </Preprocessor>
        </Preprocessing>
        <LocalizationPrediction type="wheel_imu_prediction">
            <Motion artifact_id="tracking_motion_delta"/>
        </LocalizationPrediction>
        <Perception type="noop"/>
        <Association type="noop"/>
        <PoseCorrection type="noop"/>
        <WorldPrediction type="landmark_map">
            <FieldMap resource_id="override_field"/>
        </WorldPrediction>
        <Publishing type="vex_brain">
            <Serial resource_id="brain_uart"/>
            <Health fresh_ms="150">
                <Encoder sensor_id="tracking_encoder_a"/>
                <Encoder sensor_id="tracking_encoder_b"/>
                <Encoder sensor_id="tracking_encoder_c"/>
                <Gyro sensor_id="robot_imu"/>
                <BiasCal artifact_id="imu_orientation"/>
            </Health>
            <WorldObject object_id="center_goal" wire_id="1"/>
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

std::vector<uint8_t> commandBytes(const gatr2::CommandFrame& c) {
    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeCommandFrame(c, buf.data(), gatr2::kMaxFrameLen));
    return buf;
}

struct Rig {
    FunctionRegistry        functions;
    std::unique_ptr<System> system;
    MemoryLink*             pico  = nullptr;
    MemoryLink*             brain = nullptr;

    explicit Rig(const char* xml) {
        registerAll(functions);
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
    }
};

} // namespace

TEST(EndToEnd, SquarePathClosesOnCommandedStart) {
    Rig rig(kFullConfig);
    ASSERT_NE(rig.system, nullptr);
    ASSERT_NE(rig.pico, nullptr);
    ASSERT_NE(rig.brain, nullptr);

    // brain: start at (0.61, 0.457) facing +y, later track wire object 1
    gatr2::CommandFrame init{};
    init.seq          = 1;
    init.command      = gatr2::kCmdInitPose;
    init.x_mm         = 610;
    init.y_mm         = 457;
    init.heading_cdeg = 9000;
    rig.brain->input().feed(commandBytes(init));

    // simulated truth in the body frame, wheel travel from the rigid model
    const double kA = -0.13, kB = 0.13, kC = -0.12;   // x*uy - y*ux per wheel
    const double kCountsPerMeter = 4000.0 / (2.0 * kPi * 0.0254);
    const int    kBiasMdps       = 2000;
    const int    kCal = 205, kDrive = 200, kTurn = 100, kTail = 10;

    double   travel_a = 0.0, travel_b = 0.0, travel_c = 0.0;
    uint32_t stamp = 1000;
    uint8_t  seq   = 0;
    int      cycle = 0;

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

        if (cycle == 300) {
            gatr2::CommandFrame select{};
            select.seq       = 2;
            select.command   = gatr2::kCmdSelectObject;
            select.object_id = 1;
            select.flags     = gatr2::kCmdFlagObjectRequested;
            rig.brain->input().feed(commandBytes(select));
        }

        rig.system->step(hostTime(cycle));
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
    EXPECT_NEAR(robot.pose.x_m, 0.610, 0.003);
    EXPECT_NEAR(robot.pose.y_m, 0.457, 0.003);
    EXPECT_NEAR(radToDeg(wrapAngle(robot.pose.heading_rad - kPi / 2.0)), 0.0, 0.05);

    EXPECT_EQ(rig.system->diagnostics().links.at("pico_uart").seq_gaps, 0u);
    EXPECT_EQ(rig.system->diagnostics().links.at("pico_uart").decode_errors, 0u);

    // every cycle published one pose packet and all of them decode
    gatr2::FrameReader reader;
    gatr2::PoseFrame   last{};
    int                packets = 0;
    for (uint8_t b : rig.brain->output().takeAll()) {
        if (reader.push(b)) {
            ASSERT_TRUE(gatr2::decodePoseFrame(reader.frame(), reader.frameLen(), last));
            ++packets;
        }
    }
    EXPECT_EQ(packets, cycle);

    EXPECT_TRUE(last.status & gatr2::kStatusPoseValid);
    EXPECT_TRUE(last.status & gatr2::kStatusLocInit);
    EXPECT_TRUE(last.status & gatr2::kStatusEncHealthy);
    EXPECT_TRUE(last.status & gatr2::kStatusGyroHealthy);
    EXPECT_TRUE(last.status & gatr2::kStatusBiasCal);
    EXPECT_FALSE(last.status & gatr2::kStatusVisionAlive);

    // wire object 1 maps to center_goal, valid from the map, never observed
    EXPECT_TRUE(last.status & gatr2::kStatusObjRequested);
    EXPECT_TRUE(last.status & gatr2::kStatusObjValid);
    EXPECT_FALSE(last.status & gatr2::kStatusObjObserved);
    EXPECT_EQ(last.object_id, 1);
    EXPECT_EQ(last.obj_x_mm, 1800);
    EXPECT_EQ(last.obj_y_mm, 1800);

    EXPECT_NEAR(last.x_mm, 610, 3);
    EXPECT_NEAR(last.y_mm, 457, 3);
    EXPECT_NEAR(last.heading_cdeg, 9000, 5);
}

TEST(EndToEnd, NothingAttachedStillRunsAndPublishesHealth) {
    Rig rig(kFullConfig);
    ASSERT_NE(rig.system, nullptr);

    for (int i = 0; i < 20; ++i) {
        rig.system->step(hostTime(i + 1));
    }

    EXPECT_EQ(rig.system->cycle(), 20u);
    EXPECT_FALSE(rig.system->robot().valid);
    EXPECT_EQ(rig.system->sensorResults().at(SensorId{"tracking_encoder_a"}).state,
              SensorState::kNoDataYet);

    gatr2::FrameReader reader;
    gatr2::PoseFrame   last{};
    int                packets = 0;
    for (uint8_t b : rig.brain->output().takeAll()) {
        if (reader.push(b)) {
            ASSERT_TRUE(gatr2::decodePoseFrame(reader.frame(), reader.frameLen(), last));
            ++packets;
        }
    }
    EXPECT_EQ(packets, 20);
    EXPECT_FALSE(last.status & gatr2::kStatusPoseValid);
    EXPECT_FALSE(last.status & gatr2::kStatusEncHealthy);

    // the map-seeded world exists with no data at all
    EXPECT_EQ(rig.system->world().objects.count(WorldObjectId{"center_goal"}), 1u);
}
