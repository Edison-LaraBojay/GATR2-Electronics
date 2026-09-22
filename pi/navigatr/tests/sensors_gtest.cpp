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
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="encoder_c" channel="2"/>
            <Output id="imu" channel="imu"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="tracking_encoder_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="tracking_encoder_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000" invert="true"/>
        </Sensor>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
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

    const MeasurementRecord& a = f.system->sensorMap().at(SensorId{"tracking_encoder_a"});
    ASSERT_EQ(a.state, SourceState::kValid);
    ASSERT_TRUE(a.latest.has_value());
    EXPECT_EQ(a.latest->sequence, 1u);
    EXPECT_EQ(a.latest->measuredAt.ms, 1000);
    EXPECT_EQ(a.latest->receivedAt.ms, 1);
    // first publication seeds the accumulator at zero angle
    EXPECT_NEAR(a.latest->payload.get<EncoderSample>()->angle_rad, 0.0, 1e-12);

    const MeasurementRecord& imu = f.system->sensorMap().at(SensorId{"robot_imu"});
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

    const auto& a = f.system->sensorMap().at(SensorId{"tracking_encoder_a"});
    const auto& b = f.system->sensorMap().at(SensorId{"tracking_encoder_b"});
    EXPECT_NEAR(a.latest->payload.get<EncoderSample>()->angle_rad, 2.0 * kPi, 1e-12);
    EXPECT_NEAR(b.latest->payload.get<EncoderSample>()->angle_rad, -2.0 * kPi,
                1e-12);   // invert=true
}

TEST(Sensors, NoUpdateKeepsLatestAndSequence) {
    Fixture f;
    f.feed(packet(1, 1000, 100, 0, 0));
    f.system->step(hostTime(1));
    f.system->step(hostTime(2));   // nothing new

    const MeasurementRecord& a = f.system->sensorMap().at(SensorId{"tracking_encoder_a"});
    EXPECT_EQ(a.state, SourceState::kValid);   // healthy, no new sample
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
    EXPECT_EQ(f.system->sensorMap()
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
    EXPECT_EQ(f.system->sensorMap().at(SensorId{"tracking_encoder_a"}).state,
              SourceState::kNoDataYet);
    EXPECT_EQ(f.system->sensorMap().at(SensorId{"robot_imu"}).state,
              SourceState::kValid);
}

TEST(Sensors, BatchedGyroPacketsKeepAccumulatedRotation) {
    Fixture f;
    f.feed(packet(1, 1000, 0, 0, 0));
    f.system->step(hostTime(1));

    // two packets drained in one cycle: 100 deg/s for 10 ms, then zero for
    // 10 ms. A latest-rate snapshot would lose the rotation entirely.
    f.feed(packet(2, 1010, 0, 0, 100000));
    f.feed(packet(3, 1020, 0, 0, 0));
    f.system->step(hostTime(2));

    const MeasurementRecord& imu = f.system->sensorMap().at(SensorId{"robot_imu"});
    ASSERT_TRUE(imu.latest.has_value());
    const ImuSample* sample = imu.latest->payload.get<ImuSample>();
    ASSERT_NE(sample, nullptr);
    ASSERT_TRUE(sample->has_accumulated);
    // trapezoids: 0.5*(0+100)*0.01 + 0.5*(100+0)*0.01 = 1 degree
    EXPECT_NEAR(sample->accumulated_angle_rad, degToRad(1.0), 1e-9);
    EXPECT_NEAR(sample->yaw_rate_rad_s, 0.0, 1e-12);   // the snapshot alone lies
}

TEST(Sensors, GyroAccumulatorMarksDiscontinuitiesWithAnEpoch) {
    Fixture f;
    f.feed(packet(1, 1000, 0, 0, 100000));
    f.system->step(hostTime(1));
    const ImuSample first = *f.system->sensorMap()
                                 .at(SensorId{"robot_imu"})
                                 .latest->payload.get<ImuSample>();

    // a one second silence: the dropped interval must not be integrated,
    // and must not be mistakable for zero rotation
    f.feed(packet(2, 2000, 0, 0, 100000));
    f.system->step(hostTime(2));
    const ImuSample second = *f.system->sensorMap()
                                  .at(SensorId{"robot_imu"})
                                  .latest->payload.get<ImuSample>();
    EXPECT_EQ(second.accumulated_angle_rad, first.accumulated_angle_rad);
    EXPECT_NE(second.accumulated_epoch, first.accumulated_epoch);
}

TEST(Sensors, SilentOpenLinkGoesUnavailableNotValidForever) {
    // freshness policy: an open link that stops publishing must not leave
    // its sensors Valid indefinitely
    const char* xml = R"(
<System>
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
    <Sensors>
        <Sensor id="enc" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
            <Freshness stale_after_ms="50"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>
)";
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromString(xml, functions, err);
    ASSERT_NE(system, nullptr) << err;
    auto link = system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
    auto* pico = dynamic_cast<MemoryLink*>(link.get());
    ASSERT_NE(pico, nullptr);

    pico->input().feed(packet(1, 1000, 500, 0, 0));
    system->step(hostTime(1));
    EXPECT_EQ(system->sensorMap().at(SensorId{"enc"}).state, SourceState::kValid);

    system->step(hostTime(20));   // quiet but within the window
    EXPECT_EQ(system->sensorMap().at(SensorId{"enc"}).state, SourceState::kValid);

    system->step(hostTime(100));   // silent past stale_after_ms
    const MeasurementRecord& stale = system->sensorMap().at(SensorId{"enc"});
    EXPECT_EQ(stale.state, SourceState::kUnavailable);
    ASSERT_TRUE(stale.latest.has_value());   // history retained
    EXPECT_EQ(stale.latest->sequence, 1u);

    pico->input().feed(packet(2, 1200, 600, 0, 0));   // data resumes
    system->step(hostTime(110));
    EXPECT_EQ(system->sensorMap().at(SensorId{"enc"}).state, SourceState::kValid);
    EXPECT_EQ(system->sensorMap().at(SensorId{"enc"}).latest->sequence, 2u);
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
        FunctionKey{"test_closing_link"},
        [](const ConfigNode&, ResourceInitializationContext&,
           std::string&) -> ResourceInstance {
            return ResourceInstance::asContract<SerialLink>(
                std::make_shared<ClosingLink>(packet(1, 1000, 500, 0, 0)));
        });

    const char* xml = R"(
<System>
    <Resources>
        <Resource id="pico_uart" type="test_closing_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="encoder_c" channel="2"/>
            <Output id="imu" channel="imu"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="enc" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>
)";
    std::string err;
    auto        system = System::buildFromString(xml, functions, err);
    ASSERT_NE(system, nullptr) << err;

    system->step(hostTime(1));
    EXPECT_EQ(system->sensorMap().at(SensorId{"enc"}).state, SourceState::kValid);

    system->step(hostTime(2));
    const MeasurementRecord& record = system->sensorMap().at(SensorId{"enc"});
    EXPECT_EQ(record.state, SourceState::kFault);   // unhealthy now
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
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="encoder_c" channel="2"/>
            <Output id="imu" channel="imu"/>
        </Resource>
    </Resources>
    <Sensors>)") + sensors_xml + R"(</Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>
)";
        return System::buildFromString(xml.c_str(), functions, err);
    };

    // missing id
    EXPECT_EQ(build(R"(<Sensor type="pico_imu_channel">
        <Source resource_id="pico_telemetry" output_id="imu"/></Sensor>)"),
              nullptr);
    EXPECT_NE(err.find("id and type"), std::string::npos);

    // missing type
    EXPECT_EQ(build(R"(<Sensor id="x">
        <Source resource_id="pico_telemetry" output_id="imu"/></Sensor>)"),
              nullptr);

    // duplicate id
    EXPECT_EQ(build(R"(
        <Sensor id="x" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/></Sensor>
        <Sensor id="x" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/></Sensor>)"),
              nullptr);
    EXPECT_NE(err.find("duplicate Sensor id"), std::string::npos);

    // unknown type
    EXPECT_EQ(build(R"(<Sensor id="x" type="quantum"/>)"), nullptr);
    EXPECT_NE(err.find("quantum"), std::string::npos);

    // factory-owned children pass through the generic builder untouched;
    // unknown extra children are the factory's business
    EXPECT_NE(build(R"(<Sensor id="x" type="pico_encoder_channel">
        <Source resource_id="pico_telemetry" output_id="encoder_a"/>
        <Calibration counts_per_revolution="4000"/>
        <VendorSpecificNote anything="goes"/></Sensor>)"),
              nullptr)
        << err;

    // the selected factory rejects its own invalid configuration
    EXPECT_EQ(build(R"(<Sensor id="x" type="pico_encoder_channel">
        <Source resource_id="pico_telemetry" output_id="encoder_a"/>
        <Calibration counts_per_revolution="-5"/></Sensor>)"),
              nullptr);
    EXPECT_NE(err.find("counts_per_revolution"), std::string::npos);

    // the source binding is checked against the resource's declared outputs
    EXPECT_EQ(build(R"(<Sensor id="x" type="pico_encoder_channel">
        <Source resource_id="pico_telemetry" output_id="encoder_z"/>
        <Calibration counts_per_revolution="4000"/></Sensor>)"),
              nullptr);
    EXPECT_NE(err.find("encoder_z"), std::string::npos);
}
