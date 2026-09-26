#include <gtest/gtest.h>
#include <cmath>
#include "common/frame_codec.h"
#include "config/composition.h"
#include "impl/resources/serial_links.h"
#include "inspection/inspection_document.h"
#include "math/angles.h"
#include "resources/brain_imu_bench.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;
namespace {
struct BenchRig {
    FunctionRegistry functions;
    std::unique_ptr<System> system;
    MemoryLink *brain = nullptr, *pico = nullptr;
    uint32_t session = 0;
    uint16_t rid = 1;
    uint8_t seq = 0;
    int64_t now = 1000;
    std::shared_ptr<BrainImuBench> imu;

    bool build(bool enabled = true) {
        registerAll(functions);
        ResolvedConfiguration c;
        std::string err;
        if (!resolveConfiguration(std::string(NAVIGATR_CONFIG_DIR) +
            "/override/diagnostics/bench_vex_imu.xml", c, err)) { ADD_FAILURE() << err; return false; }
        for (std::size_t p = 0; (p = c.xml.find("linux_serial_link", p)) != std::string::npos;)
            c.xml.replace(p, 17, "memory_link");
        if (!enabled) {
            const auto start = c.xml.find("<BenchImu");
            const auto end = c.xml.find("/>", start);
            c.xml.erase(start, end + 2 - start);
        }
        system = System::buildFromString(c.xml.c_str(), functions, err);
        if (!system) { ADD_FAILURE() << err; return false; }
        auto b = system->resources().require<SerialLink>(ResourceId{"brain_uart"}, err);
        auto p = system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
        brain = dynamic_cast<MemoryLink*>(b.get());
        pico = dynamic_cast<MemoryLink*>(p.get());
        imu = system->resources().require<BrainImuBench>(ResourceId{"brain_imu"}, err);
        if (!brain || !pico || !imu) return false;
        brain->setClock([this] { return now * 1000; });
        system->step(hostTime(now));
        gatr2::BrainRequest hello;
        hello.op = gatr2::kOpHello;
        hello.nonce = 12345;
        hello.request_id = rid++;
        session = request(hello).session;
        return session != 0;
    }
    void wheels(int a, int b) {
        gatr2::SensorSample f{};
        f.seq = seq++;
        f.stamp_ms = static_cast<uint32_t>(now);
        f.mask = gatr2::kSensorEnc0 | gatr2::kSensorEnc1;
        f.enc[0] = a; f.enc[1] = b;
        std::vector<uint8_t> bytes(gatr2::kMaxFrameLen);
        bytes.resize(gatr2::encodeSensorFrame(f, bytes.data(), gatr2::kMaxFrameLen));
        pico->input().feed(bytes);
    }
    gatr2::BrainReply request(gatr2::BrainRequest r) {
        std::vector<uint8_t> bytes(gatr2::kMaxFrameLen);
        bytes.resize(gatr2::encodeBrainRequest(r, bytes.data(), gatr2::kMaxFrameLen));
        brain->input().feed(bytes);
        now += 20;
        system->step(hostTime(now));
        gatr2::FrameReader reader;
        gatr2::BrainReply reply;
        bool got = false;
        for (uint8_t byte : brain->output().takeAll()) {
            if (reader.push(byte)) got = gatr2::decodeBrainReply(reader.frame(), reader.frameLen(), reply);
        }
        EXPECT_TRUE(got);
        return reply;
    }
    gatr2::BrainRequest imuRequest(uint32_t stamp, int32_t angle = 0, bool valid = true) {
        gatr2::BrainRequest r;
        r.op = gatr2::kOpGetStateWithImu;
        r.session = session;
        r.request_id = rid++;
        r.imu_flags = valid ? gatr2::kBenchImuValid : 0;
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
    EXPECT_EQ(r.request(r.imuRequest(100)).result, gatr2::kResultOk);
    gatr2::BrainRequest place;
    place.op = gatr2::kOpSetPose; place.session = r.session; place.request_id = r.rid++;
    place.x_mm = 1000;
    r.request(place);
    r.wheels(0, 0);
    r.request(r.imuRequest(120));
    // A pending SET_POSE may be retried after newer IMU/state polling.
    const auto revision = r.system->robot().anchor_revision;
    EXPECT_EQ(r.request(place).result, gatr2::kResultOk);
    EXPECT_EQ(r.system->robot().anchor_revision, revision);
    r.wheels(4000, 4000);
    const auto state = r.request(r.imuRequest(160));
    EXPECT_EQ(state.op, gatr2::kOpGetStateWithImu);
    EXPECT_NE(state.state.robot_flags & gatr2::kRobotPoseValid, 0);
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
    gatr2::BrainRequest hello;
    hello.op = gatr2::kOpHello; hello.nonce = 98765; hello.request_id = r.rid++;
    r.session = r.request(hello).session;
    EXPECT_NE(r.session, old_session);
    EXPECT_FALSE(r.imu->valid);
    auto old = r.imuRequest(50);
    old.session = old_session;
    EXPECT_EQ(r.request(old).result, gatr2::kResultUnknownSession);
    r.wheels(4000,4000); r.request(r.imuRequest(60));
    EXPECT_DOUBLE_EQ(r.system->robot().odom_pose.x_m, resumed);
}

TEST(BrainImuBench, OpRequiresExplicitConfiguration) {
    BenchRig r;
    ASSERT_TRUE(r.build(false));
    EXPECT_EQ(r.request(r.imuRequest(100)).result, gatr2::kResultUnsupportedOp);
    EXPECT_FALSE(r.imu->valid);
}
