// pico_link_gtest.cpp
// The Pi side of the Pico link: v2 identity and the v1 fallback in
// pico_telemetry; Pico reboot, acquisition restart and IMU restart through
// the real telemetry, sensor and localization path with no false
// displacement; PicoControl against a fake Pico built from the common
// codec (resend, duplicates, completion only from reports, failures,
// timeouts, reboots); serial reopening.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "common/frame_codec.h"
#include "config/config_node.h"
#include "core/diagnostics.h"
#include "impl/resources/pico_telemetry.h"
#include "impl/resources/serial_links.h"
#include "math/angles.h"
#include "payloads/sensor_samples.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#endif

using namespace navigatr;

namespace
{

// ---------------------------------------------------------------------------
// Fake Pico. Frames out and commands in through the common codec. Command
// handling follows the firmware (pico/src/pico_commands.cpp): a wrong
// target fails unrecorded, a known (id, op, body) is a duplicate that never
// runs again, the last four are recorded, last_* names the newest frame.
// Status frames every 200 ms, and 20 ms after an answered command.
// ---------------------------------------------------------------------------

class FakePico
{
public:
    explicit FakePico(MemoryLink* link) : link(link) {}

    MemoryLink* link; // null: frames are lost (cable out)

    bool     v2       = true;
    uint16_t boot     = 0x51C3;
    uint8_t  acq      = 0;
    uint8_t  imu      = 0;
    uint32_t stamp_ms = 1000;
    uint8_t  seq      = 0;
    int32_t  counts[3] = {0, 0, 0};
    int32_t  gyro_mdps = 0;
    bool     gyro      = true;
    bool     imu_enabled = true;
    uint8_t  imu_state   = gatr2::kPicoImuReady;
    int      lose_status = 0;     // status frames dropped in transit
    bool     deaf        = false; // commands never arrive (Pi TX wire open)

    std::vector<gatr2::PicoCommand> received; // every decoded command frame
    std::vector<int64_t>            received_at;
    std::map<uint8_t, int>          runs;     // executions per op

    void reboot(uint16_t new_boot) {
        boot     = new_boot;
        acq      = 0;
        imu      = 0;
        stamp_ms = 30;
        seq      = 0;
        for (int32_t& c : counts) {
            c = 0;
        }
        records_.clear();
        last_     = Record{};
        answered_ = false;
    }

    void sensorFrame() {
        gatr2::SensorSample s{};
        s.seq      = seq++;
        s.stamp_ms = stamp_ms;
        s.mask     = gatr2::kSensorEnc0 | gatr2::kSensorEnc1 | gatr2::kSensorEnc2 |
                 (gyro ? gatr2::kSensorGyroZ : 0);
        for (int i = 0; i < 3; ++i) {
            s.enc[i] = counts[i];
        }
        s.gyro_z    = gyro_mdps;
        s.identity  = v2;
        s.boot_id   = boot;
        s.acq_epoch = acq;
        s.imu_epoch = imu;
        uint8_t        buf[gatr2::kMaxFrameLen];
        const uint16_t n = gatr2::encodeSensorFrame(s, buf, sizeof(buf));
        EXPECT_GT(n, 0u);
        feed(buf, n);
        stamp_ms += 10;
    }

    void statusFrame(int64_t now_ms) {
        last_status_ms_ = now_ms;
        answered_       = false;
        if (lose_status > 0) {
            --lose_status;
            return;
        }
        gatr2::PicoStatus st;
        st.boot_id         = boot;
        st.acq_epoch       = acq;
        st.imu_epoch       = imu;
        st.uptime_ms       = stamp_ms;
        st.imu_state       = imu_state;
        st.flags           = imu_enabled ? gatr2::kPicoImuEnabled : 0;
        st.last_request_id = last_.rid;
        st.last_op         = last_.op;
        st.last_status     = last_.status;
        st.last_detail     = last_.detail;
        st.firmware        = gatr2::kPicoFirmwareBno08x;
        uint8_t        buf[gatr2::kMaxFrameLen];
        const uint16_t n = gatr2::encodePicoStatus(st, buf, sizeof(buf));
        EXPECT_GT(n, 0u);
        feed(buf, n);
    }

    // One 10 ms tick of the firmware loop.
    void tick(int64_t now_ms) {
        sensorFrame();
        const bool periodic = now_ms - last_status_ms_ >= 200;
        const bool prompt   = answered_ && now_ms - last_status_ms_ >= 20;
        if (v2 && (periodic || prompt)) {
            statusFrame(now_ms);
        }
    }

    // Reads every command the Pi wrote since the last call.
    void serve(int64_t now_ms) {
        if (link == nullptr) {
            return;
        }
        const std::vector<uint8_t> bytes = link->output().takeAll();
        if (deaf) {
            return;
        }
        for (uint8_t b : bytes) {
            if (!reader_.push(b)) {
                continue;
            }
            do {
                gatr2::PicoCommand c;
                if (reader_.frameType() == gatr2::kFramePicoCommand &&
                    gatr2::decodePicoCommand(reader_.frame(), reader_.frameLen(), c)) {
                    received.push_back(c);
                    received_at.push_back(now_ms);
                    if (v2) {
                        handle(c);   // v1 firmware never reads its RX
                    }
                }
            } while (reader_.next());
        }
    }

    // The running IMU reinit ends.
    void settleReinit(bool ok) {
        for (Record& r : records_) {
            if (r.op == gatr2::kPicoOpReinitImu && r.status == gatr2::kPicoCommandRunning) {
                r.status = ok ? gatr2::kPicoCommandCompleted : gatr2::kPicoCommandFailed;
                r.detail = ok ? gatr2::kPicoDetailNone : gatr2::kPicoDetailImuAbsent;
                if (last_.rid == r.rid && last_.op == r.op) {
                    last_ = r;
                }
            }
        }
        imu_state = ok ? gatr2::kPicoImuReady : gatr2::kPicoImuFailed;
        gyro      = ok;
        answered_ = true;
    }

private:
    struct Record {
        uint16_t rid    = 0;
        uint8_t  op     = 0;
        uint8_t  body   = 0;
        uint8_t  status = gatr2::kPicoCommandNone;
        uint8_t  detail = gatr2::kPicoDetailNone;
    };

    void feed(const uint8_t* buf, uint16_t n) {
        if (link != nullptr) {
            link->input().feed(std::vector<uint8_t>(buf, buf + n));
        }
    }

    void handle(const gatr2::PicoCommand& c) {
        if (c.request_id == 0) {
            return;
        }
        answered_          = true;
        const uint8_t body = c.op == gatr2::kPicoOpConfigure  ? c.imu_enabled
                             : c.op == gatr2::kPicoOpReinitImu ? c.imu_port
                                                               : 0;
        if (c.target_boot_id != boot) {
            last_ = Record{c.request_id, c.op, body, gatr2::kPicoCommandFailed,
                           gatr2::kPicoDetailWrongTarget};
            return;
        }
        for (const Record& r : records_) {
            if (r.rid == c.request_id && r.op == c.op && r.body == body) {
                last_ = r;   // duplicate: reported, never run again
                return;
            }
        }
        Record r{c.request_id, c.op, body, gatr2::kPicoCommandCompleted, gatr2::kPicoDetailNone};
        ++runs[c.op];
        if (c.op == gatr2::kPicoOpConfigure) {
            imu_enabled = c.imu_enabled != 0;
        } else if (c.op == gatr2::kPicoOpReinitImu) {
            if (c.imu_port != 0) {
                r.status = gatr2::kPicoCommandFailed;
                r.detail = gatr2::kPicoDetailNoSuchPort;
            } else {
                r.status  = gatr2::kPicoCommandRunning;
                imu_state = gatr2::kPicoImuInitializing;
                gyro      = false;
                ++imu;
            }
        } else if (c.op == gatr2::kPicoOpRestartAcquisition) {
            for (int32_t& count : counts) {
                count = 0;
            }
            ++acq;
        }
        for (auto it = records_.begin(); it != records_.end(); ++it) {
            if (it->rid == r.rid) {
                records_.erase(it);
                break;
            }
        }
        records_.push_back(r);
        if (records_.size() > 4) {
            records_.pop_front();
        }
        last_ = r;
    }

    gatr2::FrameReader reader_;
    std::deque<Record> records_;
    Record             last_;
    bool               answered_       = false;
    int64_t            last_status_ms_ = -1000000;
};

// PicoTelemetry on a memory link, driven cycle by cycle.
struct ControlRig {
    std::shared_ptr<MemoryLink> link = std::make_shared<MemoryLink>();
    PicoTelemetry               telemetry;
    FakePico                    pico;
    Diagnostics                 diagnostics;
    int64_t                     now_ms = 50000;
    uint64_t                    cycle  = 0;

    explicit ControlRig(uint16_t first_request_id = 0x7000)
        : telemetry(link, "pico_uart", first_request_id), pico(link.get()) {}

    MonotonicTime now() const { return hostTime(now_ms); }

    // 10 ms cycles: Pico frames, Pi refresh (decode, then send), Pico reads.
    void step(int cycles = 1, bool pico_sends = true) {
        for (int i = 0; i < cycles; ++i) {
            if (pico_sends) {
                pico.tick(now_ms);
            }
            telemetry.refresh(ExecutionContext{hostTime(now_ms), ++cycle, &diagnostics});
            pico.serve(now_ms);
            now_ms += 10;
        }
    }

    PicoRequestState state(uint32_t handle) const { return telemetry.request(handle).state; }

    // Steps until the request settles; false after the bound.
    bool settle(uint32_t handle, int max_cycles = 300) {
        for (int i = 0; i < max_cycles; ++i) {
            const PicoRequestState s = state(handle);
            if (s == PicoRequestState::kCompleted || s == PicoRequestState::kFailed) {
                return true;
            }
            step();
        }
        return false;
    }
};

} // namespace

// ---------------------------------------------------------------------------
// Identity in pico_telemetry
// ---------------------------------------------------------------------------

TEST(PicoLinkIdentity, EachRestartMovesOnlyItsOwnEpoch) {
    ControlRig rig;
    rig.step(5);
    PicoLinkState l = rig.telemetry.link();
    ASSERT_TRUE(l.identity);
    EXPECT_TRUE(l.frames_fresh);
    EXPECT_EQ(l.boot_id, 0x51C3);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 0u);
    EXPECT_EQ(rig.telemetry.imuEpoch(), 0u);

    // IMU reinitialized: only the IMU output restarts, and its accumulator
    const uint64_t accum_epoch = rig.telemetry.gyroAccumulatedEpoch();
    ++rig.pico.imu;
    rig.step(3);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 0u);
    EXPECT_EQ(rig.telemetry.imuEpoch(), 1u);
    EXPECT_GT(rig.telemetry.gyroAccumulatedEpoch(), accum_epoch);
    EXPECT_EQ(rig.telemetry.link().imu_restarts, 1u);

    // acquisition restarted: only the encoders
    ++rig.pico.acq;
    rig.step(3);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 1u);
    EXPECT_EQ(rig.telemetry.imuEpoch(), 1u);
    EXPECT_EQ(rig.telemetry.link().restarts, 1u);

    // reboot: both
    rig.pico.reboot(0x0BAD);
    rig.step(3);
    l = rig.telemetry.link();
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 2u);
    EXPECT_EQ(rig.telemetry.imuEpoch(), 2u);
    EXPECT_EQ(l.reboots, 1u);
    EXPECT_EQ(l.boot_id, 0x0BAD);
    EXPECT_EQ(l.acq_epoch, 0u);

    // nothing changes while the identity holds
    rig.step(20);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 2u);
    EXPECT_EQ(rig.telemetry.imuEpoch(), 2u);
    EXPECT_EQ(rig.diagnostics.links.at("pico_uart").decode_errors, 0u);
}

TEST(PicoLinkIdentity, NewBootIdIsARebootWithoutClockRegression) {
    ControlRig rig;
    rig.step(3);
    // a short-lived previous boot: the new clock is already past the old one
    rig.pico.boot     = 0x0BAD;
    rig.pico.stamp_ms = 5000;
    rig.step(2);
    EXPECT_EQ(rig.telemetry.link().reboots, 1u);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 1u);
    EXPECT_EQ(rig.telemetry.imuEpoch(), 1u);
}

TEST(PicoLinkIdentity, RepeatedBootIdIsCaughtByClockRegression) {
    ControlRig rig;
    rig.step(20);
    const uint16_t boot = rig.pico.boot;
    rig.pico.reboot(boot);   // 1 in 65535 reboots draws the same id
    rig.step(2);
    EXPECT_EQ(rig.telemetry.link().reboots, 1u);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 1u);
}

TEST(PicoLinkIdentity, V1FramesHaveNoIdentityAndRebootsAreFoundByClockRegression) {
    ControlRig rig;
    rig.pico.v2 = false;
    rig.step(10);
    PicoLinkState l = rig.telemetry.link();
    EXPECT_FALSE(l.identity);
    EXPECT_TRUE(l.frames_fresh);
    EXPECT_EQ(l.boot_id, 0u);
    EXPECT_FALSE(rig.telemetry.identity());
    EXPECT_EQ(rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 1.0), 0u);

    rig.pico.stamp_ms = 20;   // v1 reboot: only the clock tells
    rig.step(2);
    EXPECT_EQ(rig.telemetry.link().reboots, 1u);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 1u);
    EXPECT_EQ(rig.telemetry.imuEpoch(), 1u);

    // new firmware sending v2 is a reboot too
    rig.pico.v2 = true;
    rig.step(2);
    EXPECT_TRUE(rig.telemetry.link().identity);
    EXPECT_EQ(rig.telemetry.link().reboots, 2u);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 2u);

    // and back to v1 firmware, even with a clock that did not run backwards;
    // commands of the v2 boot fail
    rig.step(25);
    rig.pico.lose_status = 1000;
    const uint32_t handle = rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 5.0);
    ASSERT_NE(handle, 0u);
    rig.pico.v2 = false;
    rig.step(2);
    EXPECT_FALSE(rig.telemetry.link().identity);
    EXPECT_EQ(rig.telemetry.link().reboots, 3u);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 3u);
    EXPECT_EQ(rig.state(handle), PicoRequestState::kFailed);
    EXPECT_EQ(rig.telemetry.request(handle).detail, gatr2::kControlDetailPicoLink);
}

TEST(PicoLinkIdentity, StatusOfTheCurrentBootIsPublished) {
    ControlRig rig;
    rig.step(25);
    PicoLinkState l = rig.telemetry.link();
    ASSERT_TRUE(l.status_known);
    EXPECT_EQ(l.status.boot_id, rig.pico.boot);
    EXPECT_EQ(l.status.imu_state, gatr2::kPicoImuReady);
    EXPECT_EQ(l.status.firmware, gatr2::kPicoFirmwareBno08x);
    EXPECT_TRUE(l.last_status.isSet());
    EXPECT_EQ(l.last_frame.domain, ClockDomain::kHost);
    EXPECT_EQ(l.last_frame.ms, rig.now_ms - 10);

    // a reboot hides the old boot's status until the new one reports
    rig.pico.reboot(0x0BAD);
    rig.pico.lose_status = 1;
    rig.step(1);
    EXPECT_FALSE(rig.telemetry.link().status_known);
    rig.step(45);   // the lost one plus the next periodic status
    l = rig.telemetry.link();
    EXPECT_TRUE(l.status_known);
    EXPECT_EQ(l.status.boot_id, 0x0BAD);

    // frames stop: stale after 250 ms on the host clock
    rig.step(26, false);
    EXPECT_FALSE(rig.telemetry.link().frames_fresh);
}

// ---------------------------------------------------------------------------
// Restarts through the real telemetry, sensor and localization path
// ---------------------------------------------------------------------------

namespace
{

constexpr double kRadius        = 0.024;
constexpr double kCountsPerRev  = 4000.0;
constexpr double kCountsPerMeter = kCountsPerRev / (2.0 * kPi * kRadius);

// Three wheels with no IMU, so every pose change comes from the encoders;
// the Pico IMU output is bound too, to watch its restarts.
std::string threeWheelConfig(const char* link_type) {
    return std::string(R"(
<System>
    <Loop rate_hz="100"/>
    <Resources>
        <Resource id="pico_uart" type=")") +
           link_type + R"("/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_0" channel="0"/>
            <Output id="encoder_1" channel="1"/>
            <Output id="encoder_2" channel="2"/>
            <Output id="imu_0" channel="imu"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="left" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_0"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="right" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_1"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="rear" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_2"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="gyro" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu_0"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization>
            <Observation id="wheels" type="tracking_wheel_motion">
                <TrackingWheel sensor_id="left" radius_m="0.024" position_x_m="0"
                               position_y_m="0.15" measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="right" radius_m="0.024" position_x_m="0"
                               position_y_m="-0.15" measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="rear" radius_m="0.024" position_x_m="-0.1"
                               position_y_m="0" measurement_angle_deg="90" direction="positive"/>
                <Output observation_id="wheels"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="wheels"/>
            </Estimator>
            <History retention_s="5"/>
        </Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
}

// A MemoryLink that can report closed, for cable pulls.
class ClosableLink : public MemoryLink
{
public:
    using MemoryLink::write;
    bool closed = false;

    SerialReadResult readAvailable(MutableByteSpan destination) override {
        if (closed) {
            return SerialReadResult{0, true};
        }
        return MemoryLink::readAvailable(destination);
    }
    SerialWriteResult write(ByteSpan source) override {
        if (closed) {
            return SerialWriteResult{};
        }
        return MemoryLink::write(source);
    }
};

// A cable that can be pulled: the opener hands out a fresh device while
// plugged in, nothing while out.
struct Cable {
    bool                          plugged = true;
    int64_t                       now_us  = 0;
    std::shared_ptr<ClosableLink> device;
    ReopeningLink*                link  = nullptr;
    int                           opens = 0;

    ReopeningLink::Opener opener() {
        return [this](std::string& err) -> std::shared_ptr<SerialLink> {
            if (!plugged) {
                err = "unplugged";
                return nullptr;
            }
            ++opens;
            device = std::make_shared<ClosableLink>();
            return device;
        };
    }

    void pull() {
        plugged = false;
        if (device != nullptr) {
            device->closed = true;
        }
    }
};

struct Robot {
    FunctionRegistry        functions;
    std::unique_ptr<System> system;
    MemoryLink*             uart = nullptr;
    FakePico                pico{nullptr};
    Cable                   cable;
    int64_t                 now_ms = 1000;

    double wheel_m[3] = {0.0, 0.0, 0.0};   // travel since the Pico counters were zeroed
    double truth_x = 0.0, truth_y = 0.0;

    explicit Robot(bool reopening = false) {
        registerAll(functions);
        functions.add<ResourceMakeFunction>(
            FunctionKey{"cable_link"},
            [this](const ConfigNode&, ResourceInitializationContext&,
                   std::string&) -> ResourceInstance {
                std::string err;
                auto        first = cable.opener()(err);
                auto        link  = std::make_shared<ReopeningLink>(cable.opener(), first);
                link->setClock([this] { return cable.now_us; });
                cable.link = link.get();
                return ResourceInstance::asContract<SerialLink>(link);
            });
        std::string err;
        system = System::buildFromString(
            threeWheelConfig(reopening ? "cable_link" : "memory_link").c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system == nullptr) {
            return;
        }
        if (!reopening) {
            auto link = system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
            uart      = dynamic_cast<MemoryLink*>(link.get());
            pico.link = uart;
        }
    }

    PicoTelemetry& telemetry() {
        std::string err;
        auto t = system->resources().require<PicoTelemetry>(ResourceId{"pico_telemetry"}, err);
        EXPECT_NE(t, nullptr) << err;
        return *t;
    }

    // One 10 ms step of body translation (dx forward, dy left), heading 0.
    // frames false: the Pico sends nothing (rebooting, or cable out).
    void step(double dx = 0.0, double dy = 0.0, bool frames = true) {
        cable.now_us = now_ms * 1000;
        wheel_m[0] += dx;
        wheel_m[1] += dx;
        wheel_m[2] += dy;
        truth_x += dx;
        truth_y += dy;
        for (int i = 0; i < 3; ++i) {
            pico.counts[i] = static_cast<int32_t>(std::llround(wheel_m[i] * kCountsPerMeter));
        }
        if (cable.link != nullptr) {
            pico.link = cable.plugged && cable.device != nullptr && !cable.device->closed
                            ? cable.device.get()
                            : nullptr;
        }
        if (frames) {
            pico.sensorFrame();
        } else {
            pico.stamp_ms += 10;   // the Pico clock runs on
        }
        system->step(hostTime(now_ms));
        now_ms += 10;
    }

    void zeroCounters() {
        for (double& w : wheel_m) {
            w = 0.0;
        }
    }

    const EncoderSample& encoder(const char* id) const {
        const MeasurementRecord& r = system->sensorMap().at(SensorId{id});
        const EncoderSample*     s = r.latest->payload.get<EncoderSample>();
        return *s;
    }

    const ImuSample* imu() const {
        const MeasurementRecord& r = system->sensorMap().at(SensorId{"gyro"});
        return r.latest.has_value() ? r.latest->payload.get<ImuSample>() : nullptr;
    }

    void expectAtTruth(const char* where, double tolerance = 0.002) const {
        const RobotState& robot = system->robot();
        EXPECT_NEAR(robot.odom_pose.x_m, truth_x, tolerance) << where;
        EXPECT_NEAR(robot.odom_pose.y_m, truth_y, tolerance) << where;
        EXPECT_NEAR(robot.odom_pose.heading_rad, 0.0, 1e-3) << where;
    }

    // Stand still, then 0.3 m forward and 0.1 m left.
    void driveFirstLeg() {
        for (int i = 0; i < 20; ++i) {
            step();
        }
        for (int i = 0; i < 60; ++i) {
            step(0.005, 0.1 / 60.0);
        }
        for (int i = 0; i < 10; ++i) {
            step();
        }
    }
};

void rebootWithoutDisplacement(bool v2) {
    Robot robot;
    ASSERT_NE(robot.system, nullptr);
    robot.pico.v2 = v2;
    robot.driveFirstLeg();
    robot.expectAtTruth("before the reboot");
    const uint64_t left_epoch = robot.encoder("left").discontinuity_epoch;
    const double   left_angle = robot.encoder("left").angle_rad;

    // Pico reboot: 300 ms of silence, then counts from zero on a new clock
    for (int i = 0; i < 30; ++i) {
        robot.step(0.0, 0.0, false);
    }
    robot.pico.reboot(0x0BAD);
    robot.zeroCounters();
    for (int i = 0; i < 20; ++i) {
        robot.step();
    }
    robot.expectAtTruth("after the reboot");
    EXPECT_EQ(robot.encoder("left").discontinuity_epoch, left_epoch + 1);
    EXPECT_NEAR(robot.encoder("left").angle_rad, left_angle, 1e-9);   // rebased, not dropped to 0
    EXPECT_EQ(robot.telemetry().link().reboots, 1u);

    // the new counts measure motion again
    for (int i = 0; i < 40; ++i) {
        robot.step(0.005, -0.05 / 40.0);
    }
    for (int i = 0; i < 10; ++i) {
        robot.step();
    }
    robot.expectAtTruth("after driving on the new boot");
    EXPECT_NEAR(robot.truth_x, 0.5, 1e-9);
}

} // namespace

TEST(PicoLinkRestarts, V2RebootWithCountsBackToZeroGivesNoDisplacement) {
    rebootWithoutDisplacement(true);
}

TEST(PicoLinkRestarts, V1RebootFoundByClockRegressionGivesNoDisplacement) {
    rebootWithoutDisplacement(false);
}

TEST(PicoLinkRestarts, V1FramesAreFlaggedOnTheResource) {
    Robot robot;
    ASSERT_NE(robot.system, nullptr);
    robot.pico.v2 = false;
    for (int i = 0; i < 5; ++i) {
        robot.step();
    }
    const ResourceRecord& record = robot.system->resourceMap().at(ResourceId{"pico_telemetry"});
    EXPECT_EQ(record.state, SourceState::kValid);
    EXPECT_NE(record.diagnostic.find("v1 sensor frames"), std::string::npos) << record.diagnostic;

    robot.pico.v2 = true;
    for (int i = 0; i < 5; ++i) {
        robot.step();
    }
    EXPECT_TRUE(robot.system->resourceMap().at(ResourceId{"pico_telemetry"}).diagnostic.empty());
}

TEST(PicoLinkRestarts, AcquisitionRestartZeroesCountsWithNoDisplacement) {
    Robot robot;
    ASSERT_NE(robot.system, nullptr);
    robot.driveFirstLeg();
    const uint64_t left_epoch = robot.encoder("left").discontinuity_epoch;
    const uint64_t rear_epoch = robot.encoder("rear").discontinuity_epoch;
    const uint64_t imu_epoch  = robot.telemetry().imuEpoch();

    // same boot and clock, counters zeroed under the next acq_epoch
    ++robot.pico.acq;
    robot.zeroCounters();
    for (int i = 0; i < 10; ++i) {
        robot.step();
    }
    robot.expectAtTruth("after the acquisition restart");
    EXPECT_EQ(robot.encoder("left").discontinuity_epoch, left_epoch + 1);
    EXPECT_EQ(robot.encoder("rear").discontinuity_epoch, rear_epoch + 1);
    EXPECT_EQ(robot.telemetry().imuEpoch(), imu_epoch);   // the IMU did not restart

    for (int i = 0; i < 40; ++i) {
        robot.step(0.005, 0.0);
    }
    robot.step();
    robot.expectAtTruth("after driving on");
}

TEST(PicoLinkRestarts, ImuRestartKeepsTheEncodersContinuous) {
    Robot robot;
    ASSERT_NE(robot.system, nullptr);
    robot.driveFirstLeg();
    const uint64_t left_epoch  = robot.encoder("left").discontinuity_epoch;
    const uint64_t rear_epoch  = robot.encoder("rear").discontinuity_epoch;
    ASSERT_NE(robot.imu(), nullptr);
    const uint64_t accum_epoch = robot.imu()->accumulated_epoch;

    // the IMU reinitializes while the robot drives: no gyro for 500 ms, then
    // gyro samples under the next imu_epoch
    ++robot.pico.imu;
    robot.pico.gyro = false;
    for (int i = 0; i < 50; ++i) {
        robot.step(0.004, 0.001);
    }
    robot.pico.gyro = true;
    for (int i = 0; i < 20; ++i) {
        robot.step(0.004, 0.0);
    }
    robot.step();
    robot.expectAtTruth("through the IMU restart");
    EXPECT_EQ(robot.encoder("left").discontinuity_epoch, left_epoch);
    EXPECT_EQ(robot.encoder("rear").discontinuity_epoch, rear_epoch);
    ASSERT_NE(robot.imu(), nullptr);
    EXPECT_NE(robot.imu()->accumulated_epoch, accum_epoch);   // the gyro did restart
    EXPECT_EQ(robot.telemetry().link().imu_restarts, 1u);
    EXPECT_EQ(robot.telemetry().link().reboots, 0u);
}

TEST(PicoLinkRestarts, PulledCableReopensWithTheSameIdentityAndNoRebase) {
    Robot robot(true);
    ASSERT_NE(robot.system, nullptr);
    ASSERT_NE(robot.cable.link, nullptr);
    robot.driveFirstLeg();
    robot.expectAtTruth("before the pull");
    const uint64_t left_epoch = robot.encoder("left").discontinuity_epoch;
    EXPECT_EQ(robot.cable.opens, 1);

    robot.cable.pull();
    robot.step();
    EXPECT_TRUE(robot.telemetry().linkDead());
    EXPECT_FALSE(robot.telemetry().link().frames_fresh);
    EXPECT_EQ(robot.system->resourceMap().at(ResourceId{"pico_telemetry"}).state,
              SourceState::kFault);
    EXPECT_EQ(robot.telemetry().submit(gatr2::kPicoOpConfigure, 1, hostTime(robot.now_ms), 1.0),
              0u);

    // out for 400 ms: one retry right after the close fails, the next
    // waits a full second
    for (int i = 0; i < 40; ++i) {
        robot.step();
    }
    EXPECT_EQ(robot.cable.link->attempts(), 1u);
    robot.cable.plugged = true;
    for (int i = 0; i < 100 && !robot.cable.link->isOpen(); ++i) {
        robot.step();
    }
    ASSERT_TRUE(robot.cable.link->isOpen());
    EXPECT_EQ(robot.cable.link->attempts(), 2u);
    EXPECT_EQ(robot.cable.opens, 2);

    for (int i = 0; i < 10; ++i) {
        robot.step();
    }
    EXPECT_TRUE(robot.telemetry().link().frames_fresh);
    EXPECT_EQ(robot.encoder("left").discontinuity_epoch, left_epoch);   // same Pico, no rebase
    EXPECT_EQ(robot.telemetry().link().reboots, 0u);
    for (int i = 0; i < 40; ++i) {
        robot.step(0.005, 0.0);
    }
    robot.step();
    robot.expectAtTruth("after the reconnect");
}

// ---------------------------------------------------------------------------
// PicoControl
// ---------------------------------------------------------------------------

TEST(PicoControl, SubmitNeedsAV2IdentityAndFreshFrames) {
    ControlRig rig;
    EXPECT_EQ(rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 1.0), 0u);
    rig.step(5);
    EXPECT_EQ(rig.telemetry.submit(0, 1, rig.now(), 1.0), 0u);    // unknown op
    EXPECT_EQ(rig.telemetry.submit(9, 1, rig.now(), 1.0), 0u);
    EXPECT_EQ(rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 0.0), 0u);
    EXPECT_EQ(rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(),
                                   std::numeric_limits<double>::quiet_NaN()),
              0u);
    EXPECT_NE(rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(),
                                   std::numeric_limits<double>::infinity()),
              0u);   // clamped, not overflowed
    const uint32_t handle = rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 1.0);
    EXPECT_NE(handle, 0u);
    EXPECT_EQ(rig.state(handle), PicoRequestState::kSending);
    EXPECT_EQ(rig.state(handle + 1), PicoRequestState::kUnknown);

    rig.step(26, false);   // silence beyond 250 ms
    EXPECT_FALSE(rig.telemetry.link().frames_fresh);
    EXPECT_EQ(rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 1.0), 0u);
}

TEST(PicoControl, CommandsCarryTheirIdTargetAndBody) {
    ControlRig rig(0xFFFF);
    rig.step(5);
    const uint32_t a = rig.telemetry.submit(gatr2::kPicoOpConfigure, 0, rig.now(), 2.0);
    const uint32_t b = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 0, rig.now(), 2.0);
    const uint32_t c = rig.telemetry.submit(gatr2::kPicoOpRestartAcquisition, 0, rig.now(), 2.0);
    ASSERT_NE(a, 0u);
    ASSERT_NE(b, 0u);
    ASSERT_NE(c, 0u);
    rig.step(30);
    ASSERT_GE(rig.pico.received.size(), 3u);
    const gatr2::PicoCommand& first = rig.pico.received[0];
    EXPECT_EQ(first.op, gatr2::kPicoOpConfigure);
    EXPECT_EQ(first.request_id, 0xFFFF);
    EXPECT_EQ(first.target_boot_id, rig.pico.boot);
    EXPECT_EQ(first.imu_enabled, 0);
    EXPECT_EQ(rig.pico.received[1].op, gatr2::kPicoOpReinitImu);
    EXPECT_EQ(rig.pico.received[1].request_id, 1);   // 0 is skipped
    EXPECT_EQ(rig.pico.received[1].imu_port, 0);
    EXPECT_EQ(rig.pico.received[2].request_id, 2);
    // one command frame per 100 ms at most
    EXPECT_GE(rig.pico.received_at[1] - rig.pico.received_at[0], PicoTelemetry::kCommandGapMs);
    EXPECT_GE(rig.pico.received_at[2] - rig.pico.received_at[1], PicoTelemetry::kCommandGapMs);
}

TEST(PicoControl, RequestIdsStartAtARandomBase) {
    std::vector<uint16_t> firsts;
    for (int i = 0; i < 4; ++i) {
        ControlRig rig(0);
        rig.step(5);
        ASSERT_NE(rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 1.0), 0u);
        rig.step(1);
        ASSERT_EQ(rig.pico.received.size(), 1u);
        EXPECT_NE(rig.pico.received[0].request_id, 0);
        firsts.push_back(rig.pico.received[0].request_id);
    }
    // four draws all equal would take a broken generator
    EXPECT_FALSE(firsts[0] == firsts[1] && firsts[1] == firsts[2] && firsts[2] == firsts[3]);
}

TEST(PicoControl, ResentEvery200MsUnderOneIdUntilReported) {
    ControlRig rig;
    rig.step(25);
    rig.pico.lose_status = 1000;   // every acknowledgement lost
    const uint32_t handle = rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 5.0);
    ASSERT_NE(handle, 0u);
    rig.step(100);
    ASSERT_EQ(rig.pico.received.size(), 5u);
    for (std::size_t i = 0; i < rig.pico.received.size(); ++i) {
        EXPECT_EQ(rig.pico.received[i].request_id, rig.pico.received[0].request_id);
        if (i > 0) {
            EXPECT_EQ(rig.pico.received_at[i] - rig.pico.received_at[i - 1],
                      PicoTelemetry::kResendMs);
        }
    }
    EXPECT_EQ(rig.pico.runs[gatr2::kPicoOpConfigure], 1);   // duplicates never run
    EXPECT_EQ(rig.state(handle), PicoRequestState::kSending);

    rig.pico.lose_status = 0;
    ASSERT_TRUE(rig.settle(handle));
    EXPECT_EQ(rig.state(handle), PicoRequestState::kCompleted);
    const std::size_t sent = rig.pico.received.size();
    rig.step(100);
    EXPECT_EQ(rig.pico.received.size(), sent);   // reported: never sent again
    EXPECT_EQ(rig.pico.runs[gatr2::kPicoOpConfigure], 1);
}

TEST(PicoControl, UnreportedCommandsTakeTurns) {
    ControlRig rig;
    rig.step(25);
    rig.pico.lose_status = 1000;
    const uint32_t a = rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 5.0);
    const uint32_t b = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 0, rig.now(), 5.0);
    const uint32_t c = rig.telemetry.submit(gatr2::kPicoOpRestartAcquisition, 0, rig.now(), 5.0);
    ASSERT_NE(a, 0u);
    ASSERT_NE(b, 0u);
    ASSERT_NE(c, 0u);
    rig.step(60);
    // one frame per 100 ms, oldest first: a b c a b c
    ASSERT_EQ(rig.pico.received.size(), 6u);
    for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(rig.pico.received[i].request_id, rig.pico.received[i % 3].request_id) << i;
    }
    EXPECT_EQ(rig.pico.received[0].op, gatr2::kPicoOpConfigure);
    EXPECT_EQ(rig.pico.received[1].op, gatr2::kPicoOpReinitImu);
    EXPECT_EQ(rig.pico.received[2].op, gatr2::kPicoOpRestartAcquisition);
    EXPECT_EQ(rig.pico.runs[gatr2::kPicoOpRestartAcquisition], 1);
}

TEST(PicoControl, AStatusNamingAnotherCommandSettlesNothing) {
    ControlRig rig;
    rig.step(25);
    const uint32_t first = rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 2.0);
    ASSERT_TRUE(rig.settle(first));
    ASSERT_EQ(rig.state(first), PicoRequestState::kCompleted);

    // the Pico stops hearing the Pi; its statuses keep naming the first
    rig.pico.deaf         = true;
    const uint32_t second = rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 1.0);
    rig.step(50);
    EXPECT_EQ(rig.state(second), PicoRequestState::kSending);
    rig.step(60);
    EXPECT_EQ(rig.state(second), PicoRequestState::kFailed);
    EXPECT_EQ(rig.telemetry.request(second).detail, gatr2::kControlDetailPicoLink);
    EXPECT_EQ(rig.state(first), PicoRequestState::kCompleted);
}

TEST(PicoControl, LostAcknowledgementsNeverRestartAcquisitionTwice) {
    ControlRig rig;
    rig.pico.counts[0] = 1000;
    rig.pico.counts[1] = -2000;
    rig.step(25);
    rig.pico.lose_status = 3;
    const uint32_t handle =
        rig.telemetry.submit(gatr2::kPicoOpRestartAcquisition, 0, rig.now(), 5.0);
    ASSERT_NE(handle, 0u);
    ASSERT_TRUE(rig.settle(handle));
    EXPECT_EQ(rig.state(handle), PicoRequestState::kCompleted);
    EXPECT_GE(rig.pico.received.size(), 2u);   // resent after the lost reports
    EXPECT_EQ(rig.pico.runs[gatr2::kPicoOpRestartAcquisition], 1);
    EXPECT_EQ(rig.pico.acq, 1);
    rig.step(20);
    EXPECT_EQ(rig.telemetry.link().acq_epoch, 1);
    EXPECT_EQ(rig.telemetry.link().restarts, 1u);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 1u);
    EXPECT_EQ(rig.telemetry.encoder(0).value[0], 0);
}

TEST(PicoControl, ReinitRunsUntilThePicoReportsItDone) {
    ControlRig rig;
    rig.step(25);
    const uint32_t handle = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 0, rig.now(), 15.0);
    ASSERT_NE(handle, 0u);
    rig.step(30);
    EXPECT_EQ(rig.state(handle), PicoRequestState::kRunning);
    EXPECT_EQ(rig.pico.received.size(), 1u);   // named by every status: nothing to resend
    EXPECT_EQ(rig.telemetry.link().imu_restarts, 1u);

    // the gyro returning is not completion; only the report is
    rig.pico.gyro = true;
    rig.step(30);
    EXPECT_EQ(rig.state(handle), PicoRequestState::kRunning);

    rig.pico.settleReinit(true);
    rig.step(5);
    EXPECT_EQ(rig.state(handle), PicoRequestState::kCompleted);
    EXPECT_EQ(rig.pico.runs[gatr2::kPicoOpReinitImu], 1);
}

TEST(PicoControl, PicoFailuresCarryAControlDetail) {
    ControlRig rig;
    rig.step(25);
    const uint32_t absent = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 0, rig.now(), 15.0);
    rig.step(10);
    rig.pico.settleReinit(false);
    ASSERT_TRUE(rig.settle(absent));
    EXPECT_EQ(rig.state(absent), PicoRequestState::kFailed);
    EXPECT_EQ(rig.telemetry.request(absent).detail, gatr2::kControlDetailImuAbsent);

    const uint32_t port = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 1, rig.now(), 15.0);
    ASSERT_TRUE(rig.settle(port));
    EXPECT_EQ(rig.state(port), PicoRequestState::kFailed);
    EXPECT_EQ(rig.telemetry.request(port).detail, gatr2::kControlDetailPicoRefused);
}

TEST(PicoControl, TimeoutIsPicoLinkUnreportedAndTimedOutWhileRunning) {
    ControlRig silent;
    silent.step(25);
    silent.pico.lose_status = 1000;
    const uint32_t lost = silent.telemetry.submit(gatr2::kPicoOpConfigure, 1, silent.now(), 0.5);
    silent.step(60);
    EXPECT_EQ(silent.state(lost), PicoRequestState::kFailed);
    EXPECT_EQ(silent.telemetry.request(lost).detail, gatr2::kControlDetailPicoLink);
    const std::size_t sent = silent.pico.received.size();
    silent.step(50);
    EXPECT_EQ(silent.pico.received.size(), sent);   // bounded: no resends after the timeout

    ControlRig stuck;
    stuck.step(25);
    const uint32_t running = stuck.telemetry.submit(gatr2::kPicoOpReinitImu, 0, stuck.now(), 1.0);
    stuck.step(50);
    EXPECT_EQ(stuck.state(running), PicoRequestState::kRunning);
    stuck.step(60);
    EXPECT_EQ(stuck.state(running), PicoRequestState::kFailed);
    EXPECT_EQ(stuck.telemetry.request(running).detail, gatr2::kControlDetailTimedOut);
}

TEST(PicoControl, RebootFailsCommandsOfTheOldBootAndNeverResendsThem) {
    ControlRig rig;
    rig.step(25);
    rig.pico.lose_status = 1000;
    const uint32_t handle = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 0, rig.now(), 15.0);
    rig.step(5);
    ASSERT_EQ(rig.pico.received.size(), 1u);
    rig.pico.reboot(0x0BAD);
    rig.pico.lose_status = 0;
    rig.step(50);
    EXPECT_EQ(rig.state(handle), PicoRequestState::kFailed);
    EXPECT_EQ(rig.telemetry.request(handle).detail, gatr2::kControlDetailPicoLink);
    EXPECT_EQ(rig.pico.received.size(), 1u);   // the new boot never saw it
    EXPECT_EQ(rig.telemetry.link().reboots, 1u);

    // a new command targets the new boot
    const uint32_t next = rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 2.0);
    ASSERT_TRUE(rig.settle(next));
    EXPECT_EQ(rig.state(next), PicoRequestState::kCompleted);
    EXPECT_EQ(rig.pico.received.back().target_boot_id, 0x0BAD);
}

TEST(PicoControl, TwoCommandsInFlightBothSettle) {
    ControlRig rig;
    rig.step(25);
    const uint32_t reinit = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 0, rig.now(), 15.0);
    const uint32_t config = rig.telemetry.submit(gatr2::kPicoOpConfigure, 1, rig.now(), 15.0);
    ASSERT_TRUE(rig.settle(config));
    EXPECT_EQ(rig.state(config), PicoRequestState::kCompleted);
    EXPECT_EQ(rig.state(reinit), PicoRequestState::kRunning);

    // the Pico now names the CONFIGURE; the REINIT result shows once it is
    // asked for again
    rig.pico.settleReinit(true);
    ASSERT_TRUE(rig.settle(reinit));
    EXPECT_EQ(rig.state(reinit), PicoRequestState::kCompleted);
    EXPECT_EQ(rig.pico.runs[gatr2::kPicoOpReinitImu], 1);
    EXPECT_EQ(rig.pico.runs[gatr2::kPicoOpConfigure], 1);
}

TEST(PicoControl, LinkAndRequestsAreReadableFromAnotherThread) {
    ControlRig rig;
    rig.step(25);
    const uint32_t handle = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 0, rig.now(), 15.0);
    std::atomic<bool> stop{false};
    std::atomic<int>  reads{0};
    std::atomic<bool> torn{false};
    std::thread       reader([&] {
        while (!stop.load()) {
            const PicoLinkState l = rig.telemetry.link();
            if (l.identity && l.boot_id != 0x51C3) {
                torn = true;
            }
            (void)rig.telemetry.request(handle);
            ++reads;
        }
    });
    // keep refreshing until the reader has overlapped plenty of cycles
    const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    for (int i = 0; i < 100 || (reads.load() < 1000 && std::chrono::steady_clock::now() < give_up);
         ++i) {
        rig.step();
    }
    rig.pico.settleReinit(true);
    rig.step(50);
    stop = true;
    reader.join();
    EXPECT_FALSE(torn.load());
    EXPECT_GT(reads.load(), 0);
    EXPECT_EQ(rig.state(handle), PicoRequestState::kCompleted);
}

TEST(PicoControl, ResetKeepsIdentityAndCommands) {
    ControlRig rig;
    rig.step(25);
    const uint32_t handle = rig.telemetry.submit(gatr2::kPicoOpReinitImu, 0, rig.now(), 15.0);
    rig.step(10);
    rig.telemetry.reset();
    EXPECT_FALSE(rig.telemetry.anyPacket());
    rig.step(5);
    EXPECT_EQ(rig.telemetry.link().reboots, 0u);
    EXPECT_EQ(rig.telemetry.encoderEpoch(), 0u);
    EXPECT_EQ(rig.state(handle), PicoRequestState::kRunning);
    rig.pico.settleReinit(true);
    ASSERT_TRUE(rig.settle(handle));
    EXPECT_EQ(rig.state(handle), PicoRequestState::kCompleted);
}

// ---------------------------------------------------------------------------
// Serial reopening
// ---------------------------------------------------------------------------

TEST(SerialReopen, FailedOpenIsRetriedOncePerIntervalWithoutWaiting) {
    int64_t now_us = 0;
    int     calls  = 0;
    bool    ready  = false;
    ReopeningLink link(
        [&](std::string& err) -> std::shared_ptr<SerialLink> {
            ++calls;
            if (!ready) {
                err = "cannot open /dev/ttyAMA0";
                return nullptr;
            }
            auto device = std::make_shared<MemoryLink>();
            device->input().feed({0x42});
            return device;
        },
        nullptr);
    link.setClock([&] { return now_us; });

    uint8_t byte = 0;
    for (; now_us < 2'500'000; now_us += 10'000) {
        EXPECT_TRUE(link.readAvailable(MutableByteSpan{&byte, 1}).closed);
        EXPECT_FALSE(link.write(ByteSpan{&byte, 1}).ok);
    }
    EXPECT_EQ(calls, 2);   // at 1 s and 2 s
    EXPECT_EQ(link.attempts(), 2u);
    EXPECT_EQ(link.lastError(), "cannot open /dev/ttyAMA0");
    EXPECT_EQ(link.nowUs(), now_us);

    ready = true;
    for (; now_us < 3'000'000; now_us += 10'000) {
        EXPECT_TRUE(link.readAvailable(MutableByteSpan{&byte, 1}).closed);
    }
    EXPECT_EQ(calls, 2);
    now_us = 3'000'000;
    const SerialReadResult read = link.readAvailable(MutableByteSpan{&byte, 1});
    EXPECT_FALSE(read.closed);
    ASSERT_EQ(read.bytes, 1u);
    EXPECT_EQ(byte, 0x42);
    EXPECT_TRUE(link.isOpen());
    EXPECT_EQ(link.reopens(), 1u);
}

TEST(SerialReopen, ClosedDeviceIsDroppedAndReopened) {
    int64_t                       now_us = 5'000'000;
    std::shared_ptr<ClosableLink> current;
    int                           opened = 0;
    auto                          open   = [&](std::string&) -> std::shared_ptr<SerialLink> {
        ++opened;
        current = std::make_shared<ClosableLink>();
        return current;
    };
    std::string err;
    auto        first = open(err);
    ReopeningLink link(open, first);
    link.setClock([&] { return now_us; });
    std::weak_ptr<ClosableLink> old = current;
    first.reset();
    current.reset();

    const uint8_t out[2] = {1, 2};
    EXPECT_TRUE(link.write(ByteSpan{out, 2}).ok);
    EXPECT_EQ(old.lock()->output().size(), 2u);

    old.lock()->closed = true;
    uint8_t buf[4];
    EXPECT_TRUE(link.readAvailable(MutableByteSpan{buf, 4}).closed);
    EXPECT_FALSE(link.isOpen());
    EXPECT_TRUE(old.expired());   // the descriptor is released at once
    EXPECT_EQ(link.closes(), 1u);
    EXPECT_FALSE(link.write(ByteSpan{out, 2}).ok);
    EXPECT_FALSE(link.inputPending());

    // the first open was at the clock start; 1 s has not passed
    now_us += 500'000;
    EXPECT_TRUE(link.readAvailable(MutableByteSpan{buf, 4}).closed);
    EXPECT_EQ(opened, 1);
    now_us += 500'000;
    const SerialReadResult read = link.readAvailable(MutableByteSpan{buf, 4});
    EXPECT_FALSE(read.closed);
    EXPECT_EQ(opened, 2);
    ASSERT_NE(current, nullptr);
    current->input().feed({7});
    EXPECT_TRUE(link.inputPending());
    EXPECT_EQ(link.readAvailable(MutableByteSpan{buf, 4}).bytes, 1u);
    EXPECT_TRUE(link.write(ByteSpan{out, 2}).ok);
    EXPECT_EQ(current->output().size(), 2u);
}

TEST(SerialReopen, OptionalLinuxSerialLinkWarnsOnceAndKeepsRetrying) {
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(R"(<Resource id="uart" type="linux_serial_link">
        <Device path="/navigatr-test-missing/uart"/></Resource>)"),
              tinyxml2::XML_SUCCESS);
    std::vector<std::string>      warnings;
    ResourceInitializationContext context;
    context.warnings = &warnings;
    std::string      err;
    ResourceInstance instance = make_linux_serial_link(ConfigNode{doc.RootElement()}, context, err);
    ASSERT_TRUE(err.empty()) << err;
    ASSERT_EQ(warnings.size(), 1u);
    auto link = instance.require<SerialLink>(err);
    ASSERT_NE(link, nullptr) << err;
    auto* reopening = dynamic_cast<ReopeningLink*>(link.get());
    ASSERT_NE(reopening, nullptr);

    int64_t now_us = 0;
    reopening->setClock([&] { return now_us; });
    uint8_t byte = 0;
    for (; now_us <= 3'000'000; now_us += 20'000) {
        EXPECT_TRUE(link->readAvailable(MutableByteSpan{&byte, 1}).closed);
    }
    EXPECT_EQ(reopening->attempts(), 3u);
    EXPECT_NE(reopening->lastError().find("/navigatr-test-missing/uart"), std::string::npos);
    EXPECT_EQ(warnings.size(), 1u);   // retries never add build warnings
}

#if defined(__unix__) || defined(__APPLE__)

namespace
{

// A pseudo terminal whose slave stands in for the device node.
struct Pty {
    int         master = -1;
    std::string slave;

    bool open() {
        master = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0) {
            return false;
        }
        const char* name = ::ptsname(master);
        slave            = name != nullptr ? name : "";
        return !slave.empty();
    }
    void close() {
        if (master >= 0) {
            ::close(master);
            master = -1;
        }
    }
    ~Pty() { close(); }
};

} // namespace

TEST(SerialReopen, LinuxSerialLinkReopensAReplacedDevice) {
    namespace fs = std::filesystem;
    const fs::path dir =
        fs::temp_directory_path() / ("navigatr_pico_link_" + std::to_string(::getpid()));
    fs::create_directories(dir);
    const fs::path node = dir / "pico_uart";

    Pty first;
    ASSERT_TRUE(first.open());
    fs::create_symlink(first.slave, node);

    tinyxml2::XMLDocument doc;
    const std::string     xml = "<Resource id=\"uart\" type=\"linux_serial_link\"><Device path=\"" +
                            node.string() + "\" required=\"true\"/></Resource>";
    ASSERT_EQ(doc.Parse(xml.c_str()), tinyxml2::XML_SUCCESS);
    std::vector<std::string>      warnings;
    ResourceInitializationContext context;
    context.warnings = &warnings;
    std::string      err;
    ResourceInstance instance = make_linux_serial_link(ConfigNode{doc.RootElement()}, context, err);
    ASSERT_TRUE(err.empty()) << err;
    auto link      = instance.require<SerialLink>(err);
    auto reopening = dynamic_cast<ReopeningLink*>(link.get());
    ASSERT_NE(reopening, nullptr);
    int64_t now_us = 0;
    reopening->setClock([&] { return now_us; });

    uint8_t buf[8];
    ASSERT_EQ(::write(first.master, "ab", 2), 2);
    std::size_t got = 0;
    for (int i = 0; i < 200 && got < 2; ++i) {
        const SerialReadResult r = link->readAvailable(MutableByteSpan{buf + got, 8 - got});
        ASSERT_FALSE(r.closed);
        got += r.bytes;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_EQ(got, 2u);

    // the device goes away: closed, then a replacement appears at the path
    first.close();
    bool closed = false;
    for (int i = 0; i < 200 && !closed; ++i) {
        closed = link->readAvailable(MutableByteSpan{buf, 8}).closed;
    }
    EXPECT_TRUE(closed);
    Pty second;
    ASSERT_TRUE(second.open());
    fs::remove(node);
    fs::create_symlink(second.slave, node);

    // the reopen flushes pending input, so the byte goes after it
    now_us += ReopeningLink::kRetryUs;
    EXPECT_FALSE(link->readAvailable(MutableByteSpan{buf, 8}).closed);
    ASSERT_EQ(reopening->reopens(), 1u);
    ASSERT_EQ(::write(second.master, "c", 1), 1);
    got = 0;
    for (int i = 0; i < 200 && got == 0; ++i) {
        got = link->readAvailable(MutableByteSpan{buf, 8}).bytes;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_EQ(got, 1u);
    EXPECT_EQ(buf[0], 'c');
    EXPECT_EQ(reopening->reopens(), 1u);
    fs::remove_all(dir);
}

#endif
