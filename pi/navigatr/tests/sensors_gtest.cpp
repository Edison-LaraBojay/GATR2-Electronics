// sensors_gtest.cpp
// Channel sensors through the shared telemetry resource, and the framework's
// record bookkeeping: sequence, retention, health separated from data.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "common/frame_codec.h"
#include "impl/resources/serial_links.h"
#include "math/angles.h"
#include "payloads/sensor_samples.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;

namespace
{

const char* kConfig = R"(
<System>
    <Resources>
        <Resource id="pico_uart" type="resource/memory_link"/>
        <Resource id="pico_telemetry" type="resource/pico_telemetry">
            <Serial resource_id="pico_uart"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="tracking_encoder_a" type="sensor/pico_encoder_channel">
            <Source resource_id="pico_telemetry" channel="0"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="tracking_encoder_b" type="sensor/pico_encoder_channel">
            <Source resource_id="pico_telemetry" channel="1"/>
            <Calibration counts_per_revolution="4000" invert="true"/>
        </Sensor>
        <Sensor id="robot_imu" type="sensor/pico_imu_channel">
            <Source resource_id="pico_telemetry" channel="imu"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="commands/noop"/>
        <Preprocessing type="preprocessing/noop"/>
        <LocalizationPrediction type="localization/noop"/>
        <Perception type="perception/noop"/>
        <Association type="association/noop"/>
        <PoseCorrection type="pose_correction/noop"/>
        <WorldPrediction type="world_prediction/noop"/>
        <Publishing type="publishing/noop"/>
    </Pipeline>
</System>
)";

std::vector<uint8_t> packet(uint8_t seq, uint32_t stamp, int32_t enc0, int32_t enc1,
                            int32_t gyro) {
    gatr2::SensorSample s{};
    s.seq      = seq;
    s.stamp_ms = stamp;
    s.mask     = gatr2::kSensorEnc0 | gatr2::kSensorEnc1 | gatr2::kSensorGyroZ;
    s.enc[0]   = enc0;
    s.enc[1]   = enc1;
    s.gyro_z   = gyro;

    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeSensorFrame(s, buf.data(), gatr2::kMaxFrameLen));
    return buf;
}

struct Fixture {
    FunctionRegistry        functions;
    std::unique_ptr<System> system;
    MemoryLink*             pico = nullptr;

    explicit Fixture(const char* xml = kConfig) {
        registerAll(functions);
        std::string err;
        system = System::buildFromString(xml, functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system != nullptr) {
            auto link =
                system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
            pico = dynamic_cast<MemoryLink*>(link.get());
            EXPECT_NE(pico, nullptr);
        }
    }

    void feed(const std::vector<uint8_t>& bytes) { pico->input().feed(bytes); }
};

} // namespace

TEST(Sensors, OneDrainFeedsEveryChannel) {
    Fixture f;
    f.feed(packet(1, 1000, 4000, -2000, 90000));
    f.system->step(hostTime(1));

    const SensorRecord& a = f.system->sensorResults().at(SensorId{"tracking_encoder_a"});
    ASSERT_EQ(a.state, SensorState::kValid);
    ASSERT_TRUE(a.latest.has_value());
    EXPECT_EQ(a.latest->sequence, 1u);
    EXPECT_EQ(a.latest->measuredAt.ms, 1000);
    EXPECT_EQ(a.latest->receivedAt.ms, 1);
    // first publication seeds the accumulator at zero angle
    EXPECT_NEAR(a.latest->payload.get<EncoderSample>()->angle_rad, 0.0, 1e-12);

    const SensorRecord& imu = f.system->sensorResults().at(SensorId{"robot_imu"});
    ASSERT_TRUE(imu.latest.has_value());
    EXPECT_NEAR(imu.latest->payload.get<ImuSample>()->yaw_rate_rad_s, degToRad(90.0),
                1e-12);

    EXPECT_EQ(f.system->diagnostics().links.at("pico_uart").packets, 1u);
}

TEST(Sensors, CalibrationOwnsScaleAndSign) {
    Fixture f;
    f.feed(packet(1, 1000, 0, 0, 0));
    f.system->step(hostTime(1));
    f.feed(packet(2, 1005, 4000, 4000, 0));   // one full revolution on both
    f.system->step(hostTime(2));

    const auto& a = f.system->sensorResults().at(SensorId{"tracking_encoder_a"});
    const auto& b = f.system->sensorResults().at(SensorId{"tracking_encoder_b"});
    EXPECT_NEAR(a.latest->payload.get<EncoderSample>()->angle_rad, 2.0 * kPi, 1e-12);
    EXPECT_NEAR(b.latest->payload.get<EncoderSample>()->angle_rad, -2.0 * kPi,
                1e-12);   // invert=true
}

TEST(Sensors, NoUpdateKeepsLatestAndSequence) {
    Fixture f;
    f.feed(packet(1, 1000, 100, 0, 0));
    f.system->step(hostTime(1));
    f.system->step(hostTime(2));   // nothing new

    const SensorRecord& a = f.system->sensorResults().at(SensorId{"tracking_encoder_a"});
    EXPECT_EQ(a.state, SensorState::kValid);   // healthy, no new sample
    ASSERT_TRUE(a.latest.has_value());
    EXPECT_EQ(a.latest->sequence, 1u);   // unchanged means no new publication
    EXPECT_EQ(a.latest->receivedAt.ms, 1);
    EXPECT_EQ(a.lastPolledAt.ms, 2);   // it was polled, it just had nothing new
}

TEST(Sensors, SequenceIncrementsPerPublication) {
    Fixture f;
    for (int i = 1; i <= 3; ++i) {
        f.feed(packet(static_cast<uint8_t>(i), 1000 + 5 * i, 100 * i, 0, 0));
        f.system->step(hostTime(i));
    }
    EXPECT_EQ(f.system->sensorResults()
                  .at(SensorId{"tracking_encoder_a"})
                  .latest->sequence,
              3u);
}

TEST(Sensors, LatestPacketWinsAndGapsCounted) {
    Fixture f;
    f.feed(packet(1, 1000, 100, 0, 0));
    f.feed(packet(3, 1010, 200, 0, 0));   // seq 2 lost
    f.system->step(hostTime(1));

    EXPECT_EQ(f.system->diagnostics().links.at("pico_uart").seq_gaps, 1u);
    EXPECT_EQ(f.system->diagnostics().links.at("pico_uart").packets, 2u);
}

TEST(Sensors, PartialMaskOnlyUpdatesPresentChannels) {
    Fixture f;

    gatr2::SensorSample s{};
    s.seq      = 1;
    s.stamp_ms = 100;
    s.mask     = gatr2::kSensorGyroZ;
    s.gyro_z   = 777;
    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeSensorFrame(s, buf.data(), gatr2::kMaxFrameLen));
    f.feed(buf);

    f.system->step(hostTime(1));
    EXPECT_EQ(f.system->sensorResults().at(SensorId{"tracking_encoder_a"}).state,
              SensorState::kNoDataYet);
    EXPECT_EQ(f.system->sensorResults().at(SensorId{"robot_imu"}).state,
              SensorState::kValid);
}

TEST(Sensors, FaultRetainsHistoricalSample) {
    // a link that yields one packet and then reports itself dead
    class ClosingLink : public SerialLink
    {
    public:
        explicit ClosingLink(std::vector<uint8_t> bytes) : bytes_(std::move(bytes)) {}
        SerialReadResult readAvailable(MutableByteSpan destination) override {
            if (at_ >= bytes_.size()) {
                return SerialReadResult{0, true};
            }
            std::size_t n = 0;
            while (n < destination.size && at_ < bytes_.size()) {
                destination.data[n++] = bytes_[at_++];
            }
            return SerialReadResult{n, false};
        }
        SerialWriteResult write(ByteSpan) override { return SerialWriteResult{false}; }

    private:
        std::vector<uint8_t> bytes_;
        std::size_t          at_ = 0;
    };

    FunctionRegistry functions;
    registerAll(functions);
    functions.add<ResourceMakeFunction>(
        FunctionKey{"resource/test_closing_link"},
        [](const ConfigNode&, ResourceInitializationContext&,
           std::string&) -> ResourceValue {
            return ResourceValue::asContract<SerialLink>(
                std::make_shared<ClosingLink>(packet(1, 1000, 500, 0, 0)));
        });

    const char* xml = R"(
<System>
    <Resources>
        <Resource id="pico_uart" type="resource/test_closing_link"/>
        <Resource id="pico_telemetry" type="resource/pico_telemetry">
            <Serial resource_id="pico_uart"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="enc" type="sensor/pico_encoder_channel">
            <Source resource_id="pico_telemetry" channel="0"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="commands/noop"/>
        <Preprocessing type="preprocessing/noop"/>
        <LocalizationPrediction type="localization/noop"/>
        <Perception type="perception/noop"/>
        <Association type="association/noop"/>
        <PoseCorrection type="pose_correction/noop"/>
        <WorldPrediction type="world_prediction/noop"/>
        <Publishing type="publishing/noop"/>
    </Pipeline>
</System>
)";
    std::string err;
    auto        system = System::buildFromString(xml, functions, err);
    ASSERT_NE(system, nullptr) << err;

    system->step(hostTime(1));
    EXPECT_EQ(system->sensorResults().at(SensorId{"enc"}).state, SensorState::kValid);

    system->step(hostTime(2));
    const SensorRecord& record = system->sensorResults().at(SensorId{"enc"});
    EXPECT_EQ(record.state, SensorState::kFault);   // unhealthy now
    ASSERT_TRUE(record.latest.has_value());         // history preserved
    EXPECT_EQ(record.latest->sequence, 1u);
}

TEST(Sensors, BuilderOwnsRoutingFactoryOwnsTheRest) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;

    const auto build = [&](const char* sensors_xml) {
        const std::string xml = std::string(R"(
<System>
    <Resources>
        <Resource id="pico_uart" type="resource/memory_link"/>
        <Resource id="pico_telemetry" type="resource/pico_telemetry">
            <Serial resource_id="pico_uart"/>
        </Resource>
    </Resources>
    <Sensors>)") + sensors_xml + R"(</Sensors>
    <Pipeline>
        <CommandCollection type="commands/noop"/>
        <Preprocessing type="preprocessing/noop"/>
        <LocalizationPrediction type="localization/noop"/>
        <Perception type="perception/noop"/>
        <Association type="association/noop"/>
        <PoseCorrection type="pose_correction/noop"/>
        <WorldPrediction type="world_prediction/noop"/>
        <Publishing type="publishing/noop"/>
    </Pipeline>
</System>
)";
        return System::buildFromString(xml.c_str(), functions, err);
    };

    // missing id
    EXPECT_EQ(build(R"(<Sensor type="sensor/pico_imu_channel">
        <Source resource_id="pico_telemetry" channel="imu"/></Sensor>)"),
              nullptr);
    EXPECT_NE(err.find("id and type"), std::string::npos);

    // missing type
    EXPECT_EQ(build(R"(<Sensor id="x">
        <Source resource_id="pico_telemetry" channel="imu"/></Sensor>)"),
              nullptr);

    // duplicate id
    EXPECT_EQ(build(R"(
        <Sensor id="x" type="sensor/pico_imu_channel">
            <Source resource_id="pico_telemetry" channel="imu"/></Sensor>
        <Sensor id="x" type="sensor/pico_imu_channel">
            <Source resource_id="pico_telemetry" channel="imu"/></Sensor>)"),
              nullptr);
    EXPECT_NE(err.find("duplicate Sensor id"), std::string::npos);

    // unknown type
    EXPECT_EQ(build(R"(<Sensor id="x" type="sensor/quantum"/>)"), nullptr);
    EXPECT_NE(err.find("sensor/quantum"), std::string::npos);

    // factory-owned children pass through the generic builder untouched;
    // unknown extra children are the factory's business
    EXPECT_NE(build(R"(<Sensor id="x" type="sensor/pico_encoder_channel">
        <Source resource_id="pico_telemetry" channel="0"/>
        <Calibration counts_per_revolution="4000"/>
        <VendorSpecificNote anything="goes"/></Sensor>)"),
              nullptr)
        << err;

    // the selected factory rejects its own invalid configuration
    EXPECT_EQ(build(R"(<Sensor id="x" type="sensor/pico_encoder_channel">
        <Source resource_id="pico_telemetry" channel="0"/>
        <Calibration counts_per_revolution="-5"/></Sensor>)"),
              nullptr);
    EXPECT_NE(err.find("counts_per_revolution"), std::string::npos);

    // channel bounds are factory knowledge too
    EXPECT_EQ(build(R"(<Sensor id="x" type="sensor/pico_encoder_channel">
        <Source resource_id="pico_telemetry" channel="9"/>
        <Calibration counts_per_revolution="4000"/></Sensor>)"),
              nullptr);
    EXPECT_NE(err.find("channel"), std::string::npos);
}
