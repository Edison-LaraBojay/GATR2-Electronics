// stages_gtest.cpp
// The two aggregate stages through System: make_resources and make_sensors
// capture configured collections once; execute_resources fills the
// ResourceMap with named outputs, execute_sensors reads it into the
// SensorMap. Identity, bindings, captured state, shared acquisition,
// retention, batching, and reset.

#include <gtest/gtest.h>

#include <memory>
#include <limits>
#include <string>
#include <vector>

#include "common/frame_codec.h"
#include "impl/resources/serial_links.h"
#include "impl/sensors/attitude_channel.h"
#include "math/angles.h"
#include "payloads/pico_telemetry_samples.h"
#include "payloads/attitude_samples.h"
#include "payloads/sensor_samples.h"
#include "runtime/register_all.h"
#include "runtime/resource_catalog.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

const char* kNoopPipeline = R"(
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>
        <FieldEstimation type="noop"/>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>)";

std::string systemXml(const std::string& resources, const std::string& sensors,
                      const std::string& pipeline = kNoopPipeline) {
    return "<System><Resources>" + resources + "</Resources><Sensors>" + sensors +
           "</Sensors>" + pipeline + "</System>";
}

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

// Test-only payload for a synthetic single-output resource.
struct CounterSample {
    int n = 0;
};
constexpr const char* kCounterPayload = "test.counter";

// A link that counts how many times it was drained to empty.
struct DrainCount {
    int drains = 0;
};

class CountingLink : public SerialLink
{
public:
    explicit CountingLink(std::shared_ptr<DrainCount> count) : count_(std::move(count)) {}
    SerialReadResult readAvailable(MutableByteSpan destination) override {
        const SerialReadResult r = inner_.readAvailable(destination);
        if (r.bytes == 0) {
            ++count_->drains;
        }
        return r;
    }
    SerialWriteResult write(ByteSpan source) override { return inner_.write(source); }
    MemoryStream&     input() { return inner_.input(); }

private:
    MemoryLink                  inner_;
    std::shared_ptr<DrainCount> count_;
};

struct Fixture {
    FunctionRegistry                          functions;
    std::shared_ptr<std::vector<std::string>> log = std::make_shared<std::vector<std::string>>();
    std::shared_ptr<DrainCount>               drains = std::make_shared<DrainCount>();
    std::unique_ptr<System>                   system;

    Fixture() {
        registerAll(functions);

        // single-output resource with captured state: publishes 1, 2, 3, ...
        auto log_copy = log;
        functions.add<ResourceMakeFunction>(
            FunctionKey{"counter_source"},
            [log_copy](const ConfigNode&, ResourceInitializationContext&,
                       std::string&) -> ResourceInstance {
                auto               count = std::make_shared<int>(0);
                ResourceExecutable executable;
                executable.outputs = {ResourceOutputDecl{
                    OutputId{"value"}, PayloadDescriptor::of<CounterSample>(kCounterPayload)}};
                executable.execute = [count, log_copy](const ExecutionContext&) {
                    ++*count;
                    log_copy->push_back("resource");
                    ResourcePollResult result;
                    result.state = SourceState::kValid;
                    OutputPoll poll;
                    poll.id           = OutputId{"value"};
                    poll.result.state = SourceState::kValid;
                    Publication publication;
                    publication.measuredAt = deviceTime(*count * 10);
                    publication.payload =
                        TypedPayload::store(CounterSample{*count}, kCounterPayload);
                    poll.result.publication = std::move(publication);
                    result.outputs.push_back(std::move(poll));
                    return result;
                };
                executable.reset = [count] { *count = 0; };
                auto instance    = ResourceInstance::asContract<int>(count);
                instance.setExecutable(std::move(executable));
                return instance;
            });

        // forwarding sensor over a counter output
        functions.add<SensorMakeFunction>(
            FunctionKey{"counter_reader"},
            [log_copy](const ConfigNode& node, SensorInitializationContext& context,
                       std::string& err) -> std::optional<SensorExecutable> {
                auto binding = std::make_shared<TypedOutputBinding<CounterSample>>();
                const ConfigNode source = node.child("Source");
                if (!context.outputs->bind<CounterSample>(
                        ResourceId{source.attr("resource_id")},
                        OutputId{source.attr("output_id")}, node.path(), *binding, err)) {
                    return std::nullopt;
                }
                SensorExecutable executable;
                executable.outputPayload = PayloadDescriptor::of<CounterSample>(kCounterPayload);
                executable.execute = [binding, log_copy](const ResourceMap&      resources,
                                                         const ExecutionContext&) {
                    log_copy->push_back("sensor");
                    PollResult          result;
                    const StoredSample* stored = binding->stored(resources);
                    if (stored == nullptr) {
                        return result;
                    }
                    result.state = SourceState::kValid;
                    Publication publication;
                    publication.measuredAt = stored->measuredAt;
                    publication.receivedAt = stored->receivedAt;
                    publication.payload    = stored->payload;
                    result.publication     = std::move(publication);
                    return result;
                };
                return executable;
            });

        auto drains_copy = drains;
        functions.add<ResourceMakeFunction>(
            FunctionKey{"counting_link"},
            [drains_copy](const ConfigNode&, ResourceInitializationContext&,
                          std::string&) -> ResourceInstance {
                return ResourceInstance::asContract<SerialLink>(
                    std::make_shared<CountingLink>(drains_copy));
            });
    }

    bool build(const std::string& xml, std::string& err) {
        system = System::buildFromString(xml.c_str(), functions, err);
        return system != nullptr;
    }

    template <typename Link>
    Link* link(const char* id) {
        std::string err;
        auto        handle = system->resources().require<SerialLink>(ResourceId{id}, err);
        return dynamic_cast<Link*>(handle.get());
    }
};

const char* kPicoResources = R"(
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="imu" channel="imu"/>
        </Resource>)";

const char* kPicoSensors = R"(
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000" invert="true"/>
        </Sensor>
        <Sensor id="imu" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/>
        </Sensor>)";

} // namespace

TEST(Stages, MultipleInstancesOfOneTypeStayIndependent) {
    Fixture     f;
    std::string err;
    ASSERT_TRUE(f.build(systemXml(R"(
        <Resource id="uart_a" type="memory_link"/>
        <Resource id="uart_b" type="memory_link"/>
        <Resource id="pico_a" type="pico_telemetry">
            <Serial resource_id="uart_a"/>
            <Output id="enc" channel="0"/>
            <Output id="imu" channel="imu"/>
        </Resource>
        <Resource id="pico_b" type="pico_telemetry">
            <Serial resource_id="uart_b"/>
            <Output id="enc" channel="0"/>
        </Resource>
        <Resource id="count_a" type="counter_source"/>
        <Resource id="count_b" type="counter_source"/>)",
                                  R"(
        <Sensor id="wheel_a" type="pico_encoder_channel">
            <Source resource_id="pico_a" output_id="enc"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="wheel_b" type="pico_encoder_channel">
            <Source resource_id="pico_b" output_id="enc"/>
            <Calibration counts_per_revolution="1000"/>
        </Sensor>
        <Sensor id="gyro_a" type="pico_imu_channel">
            <Source resource_id="pico_a" output_id="imu"/>
        </Sensor>)"),
                        err))
        << err;

    auto* uart_a = f.link<MemoryLink>("uart_a");
    auto* uart_b = f.link<MemoryLink>("uart_b");
    ASSERT_NE(uart_a, nullptr);
    ASSERT_NE(uart_b, nullptr);

    uart_a->input().feed(packet(1, 1000, 0, 0, 0));
    uart_b->input().feed(packet(1, 2000, 0, 0, 0));
    f.system->step(hostTime(1));
    uart_a->input().feed(packet(2, 1005, 4000, 0, 90000));
    uart_b->input().feed(packet(2, 2005, 1000, 0, 0));
    f.system->step(hostTime(2));

    const ResourceMap& resources = f.system->resourceMap();
    EXPECT_EQ(resources.at(ResourceId{"pico_a"})
                  .outputs.at(OutputId{"enc"})
                  .latest->payload.get<PicoEncoderCounts>()
                  ->counts,
              4000);
    EXPECT_EQ(resources.at(ResourceId{"pico_a"}).outputs.at(OutputId{"enc"}).latest->measuredAt.ms,
              1005);
    EXPECT_EQ(resources.at(ResourceId{"pico_b"})
                  .outputs.at(OutputId{"enc"})
                  .latest->payload.get<PicoEncoderCounts>()
                  ->counts,
              1000);
    EXPECT_EQ(resources.at(ResourceId{"pico_b"}).outputs.at(OutputId{"enc"}).latest->measuredAt.ms,
              2005);
    EXPECT_EQ(resources.at(ResourceId{"pico_b"}).outputs.count(OutputId{"imu"}), 0u);

    // one revolution each, through different calibrations, from different links
    const SensorMap& sensors = f.system->sensorMap();
    EXPECT_NEAR(sensors.at(SensorId{"wheel_a"}).latest->payload.get<EncoderSample>()->angle_rad,
                2.0 * kPi, 1e-12);
    EXPECT_NEAR(sensors.at(SensorId{"wheel_b"}).latest->payload.get<EncoderSample>()->angle_rad,
                2.0 * kPi, 1e-12);
    EXPECT_NEAR(sensors.at(SensorId{"gyro_a"}).latest->payload.get<ImuSample>()->yaw_rate_rad_s,
                degToRad(90.0), 1e-12);

    // two counters, two private states
    EXPECT_EQ(resources.at(ResourceId{"count_a"})
                  .outputs.at(OutputId{"value"})
                  .latest->payload.get<CounterSample>()
                  ->n,
              2);
    EXPECT_EQ(resources.at(ResourceId{"count_b"})
                  .outputs.at(OutputId{"value"})
                  .latest->payload.get<CounterSample>()
                  ->n,
              2);
    EXPECT_EQ(f.system->diagnostics().links.at("uart_a").packets, 2u);
    EXPECT_EQ(f.system->diagnostics().links.at("uart_b").packets, 2u);
}

TEST(Stages, NamedOutputsForMultiChannelAndSingleOutputSources) {
    Fixture     f;
    std::string err;
    ASSERT_TRUE(f.build(systemXml(std::string(kPicoResources) + R"(
        <Resource id="counter" type="counter_source"/>
        <Resource id="geometry" type="wheel_geometry">
            <Wheel id="w" sensor_id="enc_a" calibration_status="verified"
                   radius_m="0.0254" position_x_m="0" position_y_m="0"
                   measurement_angle_deg="0" direction="positive"/>
        </Resource>)",
                                  kPicoSensors),
                        err))
        << err;

    // declared before any data: every output addressable, nothing sampled
    const ResourceMap& before = f.system->resourceMap();
    ASSERT_EQ(before.count(ResourceId{"pico_telemetry"}), 1u);
    EXPECT_EQ(before.at(ResourceId{"pico_telemetry"}).outputs.size(), 3u);
    EXPECT_FALSE(before.at(ResourceId{"pico_telemetry"})
                     .outputs.at(OutputId{"encoder_a"})
                     .latest.has_value());
    EXPECT_EQ(before.at(ResourceId{"counter"}).outputs.size(), 1u);
    // configuration only: bound directly, never executed
    EXPECT_EQ(before.count(ResourceId{"geometry"}), 0u);
    EXPECT_FALSE(f.system->resourceCatalog().has(ResourceId{"geometry"}));
    EXPECT_TRUE(f.system->resourceCatalog().has(ResourceId{"counter"}));

    f.link<MemoryLink>("pico_uart")->input().feed(packet(1, 1000, 4000, -2000, 90000));
    f.system->step(hostTime(1));

    const ResourceRecord& pico = f.system->resourceMap().at(ResourceId{"pico_telemetry"});
    EXPECT_EQ(pico.state, SourceState::kValid);
    EXPECT_EQ(pico.outputs.at(OutputId{"encoder_a"}).latest->payload.get<PicoEncoderCounts>()->counts,
              4000);
    EXPECT_EQ(pico.outputs.at(OutputId{"encoder_b"}).latest->payload.get<PicoEncoderCounts>()->counts,
              -2000);
    EXPECT_EQ(pico.outputs.at(OutputId{"imu"}).latest->payload.get<PicoGyroRate>()->rate_mdps,
              90000);
    // each channel carries its own timing and status
    EXPECT_EQ(pico.outputs.at(OutputId{"imu"}).latest->measuredAt.ms, 1000);
    EXPECT_EQ(pico.outputs.at(OutputId{"imu"}).latest->receivedAt.ms, 1);
    EXPECT_EQ(pico.outputs.at(OutputId{"imu"}).latest->sequence, 1u);
    EXPECT_EQ(pico.outputs.at(OutputId{"imu"}).state, SourceState::kValid);

    // the single-output source uses the same addressing
    const ResourceRecord& counter = f.system->resourceMap().at(ResourceId{"counter"});
    EXPECT_EQ(counter.outputs.at(OutputId{"value"}).latest->payload.get<CounterSample>()->n, 1);

    EXPECT_EQ(f.system->diagnostics().functions.at("Resource/pico_telemetry").ok, 1u);
    EXPECT_EQ(f.system->diagnostics().functions.at("Sensor/enc_a").ok, 1u);
}

TEST(Stages, InvalidBindingsAndDuplicateIdsFailConfiguration) {
    Fixture     f;
    std::string err;

    const auto sensors = [&](const char* sensors_xml) {
        err.clear();
        return f.build(systemXml(kPicoResources, sensors_xml), err);
    };
    const auto resources = [&](const char* resources_xml) {
        err.clear();
        return f.build(systemXml(resources_xml, ""), err);
    };

    // unknown resource
    EXPECT_FALSE(sensors(R"(<Sensor id="x" type="pico_encoder_channel">
        <Source resource_id="ghost" output_id="encoder_a"/>
        <Calibration counts_per_revolution="4000"/></Sensor>)"));
    EXPECT_NE(err.find("ghost"), std::string::npos) << err;

    // unknown output on a known resource
    EXPECT_FALSE(sensors(R"(<Sensor id="x" type="pico_encoder_channel">
        <Source resource_id="pico_telemetry" output_id="encoder_z"/>
        <Calibration counts_per_revolution="4000"/></Sensor>)"));
    EXPECT_NE(err.find("encoder_z"), std::string::npos) << err;
    EXPECT_NE(err.find("does not publish"), std::string::npos) << err;

    // wrong payload: an encoder sensor on the gyro output
    EXPECT_FALSE(sensors(R"(<Sensor id="x" type="pico_encoder_channel">
        <Source resource_id="pico_telemetry" output_id="imu"/>
        <Calibration counts_per_revolution="4000"/></Sensor>)"));
    EXPECT_NE(err.find("different payload"), std::string::npos) << err;
    EXPECT_NE(err.find(payload_names::kPicoGyroRate), std::string::npos) << err;

    // and the other way round
    EXPECT_FALSE(sensors(R"(<Sensor id="x" type="pico_imu_channel">
        <Source resource_id="pico_telemetry" output_id="encoder_a"/></Sensor>)"));
    EXPECT_NE(err.find("different payload"), std::string::npos) << err;

    // a resource without outputs cannot be a source
    EXPECT_FALSE(sensors(R"(<Sensor id="x" type="pico_encoder_channel">
        <Source resource_id="pico_uart" output_id="encoder_a"/>
        <Calibration counts_per_revolution="4000"/></Sensor>)"));
    EXPECT_NE(err.find("publishes no outputs"), std::string::npos) << err;

    // missing output id
    EXPECT_FALSE(sensors(R"(<Sensor id="x" type="pico_encoder_channel">
        <Source resource_id="pico_telemetry"/>
        <Calibration counts_per_revolution="4000"/></Sensor>)"));
    EXPECT_NE(err.find("output_id"), std::string::npos) << err;

    // duplicate sensor id
    EXPECT_FALSE(sensors(R"(
        <Sensor id="x" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/></Sensor>
        <Sensor id="x" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/></Sensor>)"));
    EXPECT_NE(err.find("duplicate Sensor id"), std::string::npos) << err;

    // unknown sensor type
    EXPECT_FALSE(sensors(R"(<Sensor id="x" type="quantum"/>)"));
    EXPECT_NE(err.find("quantum"), std::string::npos) << err;

    // duplicate resource id
    EXPECT_FALSE(resources(R"(
        <Resource id="c" type="counter_source"/>
        <Resource id="c" type="counter_source"/>)"));
    EXPECT_NE(err.find("duplicate Resource id"), std::string::npos) << err;

    // unknown resource type
    EXPECT_FALSE(resources(R"(<Resource id="c" type="carrier_pigeon"/>)"));
    EXPECT_NE(err.find("carrier_pigeon"), std::string::npos) << err;

    // duplicate output id within one resource
    EXPECT_FALSE(resources(R"(
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="same" channel="0"/>
            <Output id="same" channel="1"/>
        </Resource>)"));
    EXPECT_NE(err.find("duplicate Output id"), std::string::npos) << err;

    // channel bounds are the resource's business
    EXPECT_FALSE(resources(R"(
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="enc" channel="9"/>
        </Resource>)"));
    EXPECT_NE(err.find("channel"), std::string::npos) << err;

    // unknown children of the stage sections
    EXPECT_FALSE(f.build("<System><Resources><Sensor id=\"x\"/></Resources>" +
                             std::string(kNoopPipeline) + "</System>",
                         err));
    EXPECT_NE(err.find("unknown element Sensor"), std::string::npos) << err;
}

TEST(Stages, CapturedStatePersistsBetweenCalls) {
    Fixture     f;
    std::string err;
    ASSERT_TRUE(f.build(systemXml(std::string(kPicoResources) +
                                      R"(<Resource id="counter" type="counter_source"/>)",
                                  std::string(kPicoSensors) + R"(
        <Sensor id="reader" type="counter_reader">
            <Source resource_id="counter" output_id="value"/>
        </Sensor>)"),
                        err))
        << err;
    auto* uart = f.link<MemoryLink>("pico_uart");

    for (int i = 1; i <= 3; ++i) {
        uart->input().feed(packet(static_cast<uint8_t>(i), 1000 + 5 * i, 4000 * (i - 1), 0, 0));
        f.system->step(hostTime(i));

        const ResourceMap& resources = f.system->resourceMap();
        const SensorMap&   sensors   = f.system->sensorMap();
        // the counter remembers itself across invocations
        EXPECT_EQ(resources.at(ResourceId{"counter"})
                      .outputs.at(OutputId{"value"})
                      .latest->payload.get<CounterSample>()
                      ->n,
                  i);
        EXPECT_EQ(resources.at(ResourceId{"counter"}).outputs.at(OutputId{"value"}).latest->sequence,
                  static_cast<uint64_t>(i));
        EXPECT_EQ(sensors.at(SensorId{"reader"}).latest->payload.get<CounterSample>()->n, i);
        // the encoder accumulates across cycles, not per poll
        EXPECT_NEAR(sensors.at(SensorId{"enc_a"}).latest->payload.get<EncoderSample>()->angle_rad,
                    2.0 * kPi * (i - 1), 1e-12);
    }

    // resources ran before sensors every cycle
    ASSERT_EQ(f.log->size(), 6u);
    for (std::size_t i = 0; i < f.log->size(); i += 2) {
        EXPECT_EQ((*f.log)[i], "resource");
        EXPECT_EQ((*f.log)[i + 1], "sensor");
    }
}

TEST(Stages, SharedAcquisitionHappensOncePerStageInvocation) {
    Fixture     f;
    std::string err;
    ASSERT_TRUE(f.build(systemXml(R"(
        <Resource id="pico_uart" type="counting_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="imu" channel="imu"/>
        </Resource>)",
                                  kPicoSensors),
                        err))
        << err;
    auto* uart = f.link<CountingLink>("pico_uart");
    ASSERT_NE(uart, nullptr);

    for (int i = 1; i <= 5; ++i) {
        uart->input().feed(packet(static_cast<uint8_t>(i), 1000 + 5 * i, i, i, i));
        f.system->step(hostTime(i));
        // three sensors read the same decode; only the resource drains
        EXPECT_EQ(f.drains->drains, i);
    }
    EXPECT_EQ(f.system->diagnostics().links.at("pico_uart").packets, 5u);
    EXPECT_EQ(f.system->sensorMap().at(SensorId{"enc_a"}).latest->sequence, 5u);
    EXPECT_EQ(f.system->sensorMap().at(SensorId{"imu"}).latest->sequence, 5u);
}

TEST(Stages, QuietPollsRetainSamplesAndReadingTwiceIsOneMeasurement) {
    Fixture     f;
    std::string err;
    ASSERT_TRUE(f.build(systemXml(kPicoResources, kPicoSensors), err)) << err;
    auto* uart = f.link<MemoryLink>("pico_uart");

    uart->input().feed(packet(1, 1000, 4000, 0, 90000));
    f.system->step(hostTime(1));
    f.system->step(hostTime(2));   // nothing new
    f.system->step(hostTime(3));

    const MeasurementRecord& enc =
        f.system->resourceMap().at(ResourceId{"pico_telemetry"}).outputs.at(OutputId{"encoder_a"});
    EXPECT_EQ(enc.state, SourceState::kValid);
    EXPECT_EQ(enc.lastPolledAt.ms, 3);
    ASSERT_TRUE(enc.latest.has_value());
    EXPECT_EQ(enc.latest->sequence, 1u);   // no fabricated publication
    EXPECT_EQ(enc.latest->measuredAt.ms, 1000);
    EXPECT_EQ(enc.latest->receivedAt.ms, 1);
    EXPECT_EQ(enc.latest->payload.get<PicoEncoderCounts>()->counts, 4000);   // not zeroed

    const MeasurementRecord& sensor = f.system->sensorMap().at(SensorId{"enc_a"});
    EXPECT_EQ(sensor.state, SourceState::kValid);
    EXPECT_EQ(sensor.latest->sequence, 1u);   // the retained record was not re-measured
    EXPECT_EQ(sensor.latest->measuredAt.ms, 1000);
    EXPECT_EQ(sensor.lastPolledAt.ms, 3);
    EXPECT_EQ(f.system->diagnostics().functions.at("Sensor/enc_a").ok, 3u);
}

TEST(Stages, BatchedPacketsInOneCycleKeepMotionAndLatestValue) {
    Fixture     f;
    std::string err;
    ASSERT_TRUE(f.build(systemXml(kPicoResources, kPicoSensors), err)) << err;
    auto* uart = f.link<MemoryLink>("pico_uart");

    uart->input().feed(packet(1, 1000, 0, 0, 0));
    f.system->step(hostTime(1));
    // 100 deg/s for 10 ms then zero for 10 ms, drained in one cycle
    uart->input().feed(packet(2, 1010, 100, 0, 100000));
    uart->input().feed(packet(3, 1020, 200, 0, 0));
    f.system->step(hostTime(2));

    const ResourceRecord& pico = f.system->resourceMap().at(ResourceId{"pico_telemetry"});
    const MeasurementRecord& enc = pico.outputs.at(OutputId{"encoder_a"});
    EXPECT_EQ(enc.latest->sequence, 2u);   // one publication per cycle
    EXPECT_EQ(enc.latest->payload.get<PicoEncoderCounts>()->counts, 200);   // latest wins
    EXPECT_EQ(enc.latest->measuredAt.ms, 1020);

    const PicoGyroRate* gyro =
        pico.outputs.at(OutputId{"imu"}).latest->payload.get<PicoGyroRate>();
    ASSERT_NE(gyro, nullptr);
    EXPECT_EQ(gyro->rate_mdps, 0);
    EXPECT_NEAR(gyro->accumulated_mdeg, 1000.0, 1e-9);   // both trapezoids kept

    const ImuSample* imu =
        f.system->sensorMap().at(SensorId{"imu"}).latest->payload.get<ImuSample>();
    ASSERT_NE(imu, nullptr);
    EXPECT_NEAR(imu->accumulated_angle_rad, degToRad(1.0), 1e-9);
    EXPECT_EQ(f.system->diagnostics().links.at("pico_uart").packets, 3u);
}

TEST(Stages, LinkDeathFaultsOutputsButKeepsHistory) {
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

    Fixture f;
    f.functions.add<ResourceMakeFunction>(
        FunctionKey{"closing_link"},
        [](const ConfigNode&, ResourceInitializationContext&,
           std::string&) -> ResourceInstance {
            return ResourceInstance::asContract<SerialLink>(
                std::make_shared<ClosingLink>(packet(1, 1000, 500, 0, 0)));
        });
    std::string err;
    ASSERT_TRUE(f.build(systemXml(R"(
        <Resource id="pico_uart" type="closing_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="imu" channel="imu"/>
        </Resource>)",
                                  kPicoSensors),
                        err))
        << err;

    f.system->step(hostTime(1));
    const ResourceRecord& first = f.system->resourceMap().at(ResourceId{"pico_telemetry"});
    EXPECT_EQ(first.state, SourceState::kFault);   // the link closed during the drain
    EXPECT_EQ(first.outputs.at(OutputId{"encoder_a"}).state, SourceState::kValid);
    EXPECT_EQ(f.system->sensorMap().at(SensorId{"enc_a"}).state, SourceState::kValid);

    f.system->step(hostTime(2));
    const ResourceRecord& second = f.system->resourceMap().at(ResourceId{"pico_telemetry"});
    EXPECT_EQ(second.outputs.at(OutputId{"encoder_a"}).state, SourceState::kFault);
    ASSERT_TRUE(second.outputs.at(OutputId{"encoder_a"}).latest.has_value());
    EXPECT_EQ(second.outputs.at(OutputId{"encoder_a"}).latest->sequence, 1u);
    const MeasurementRecord& sensor = f.system->sensorMap().at(SensorId{"enc_a"});
    EXPECT_EQ(sensor.state, SourceState::kFault);
    EXPECT_NE(sensor.diagnostic.find("link dead"), std::string::npos);
    ASSERT_TRUE(sensor.latest.has_value());
    EXPECT_EQ(sensor.latest->sequence, 1u);
}

TEST(Stages, ResetClearsRecordsRestartsSequencesAndMovesTheEpoch) {
    Fixture     f;
    std::string err;
    ASSERT_TRUE(f.build(systemXml(std::string(kPicoResources) +
                                      R"(<Resource id="counter" type="counter_source"/>)",
                                  kPicoSensors),
                        err))
        << err;
    auto* uart = f.link<MemoryLink>("pico_uart");

    uart->input().feed(packet(1, 1000, 0, 0, 0));
    f.system->step(hostTime(1));
    uart->input().feed(packet(2, 1005, 4000, 0, 0));
    f.system->step(hostTime(2));
    EXPECT_NEAR(f.system->sensorMap().at(SensorId{"enc_a"}).latest->payload.get<EncoderSample>()->angle_rad,
                2.0 * kPi, 1e-12);

    f.system->reset();

    const ResourceRecord& pico = f.system->resourceMap().at(ResourceId{"pico_telemetry"});
    EXPECT_EQ(pico.state, SourceState::kNoDataYet);
    EXPECT_EQ(pico.outputs.at(OutputId{"encoder_a"}).state, SourceState::kNoDataYet);
    EXPECT_FALSE(pico.outputs.at(OutputId{"encoder_a"}).latest.has_value());
    EXPECT_EQ(pico.outputs.at(OutputId{"encoder_a"}).epoch, 1u);
    EXPECT_EQ(f.system->sensorMap().at(SensorId{"enc_a"}).state, SourceState::kNoDataYet);
    EXPECT_FALSE(f.system->sensorMap().at(SensorId{"enc_a"}).latest.has_value());

    // after reset: sequences restart in the next epoch, the shared decoder
    // and the sensor accumulators start over, and a retained pre-reset
    // count is not differenced against the new one
    uart->input().feed(packet(1, 5000, 9000, 0, 0));
    f.system->step(hostTime(3));
    const MeasurementRecord& enc =
        f.system->resourceMap().at(ResourceId{"pico_telemetry"}).outputs.at(OutputId{"encoder_a"});
    EXPECT_EQ(enc.latest->sequence, 1u);
    EXPECT_EQ(enc.latest->epoch, 1u);
    const MeasurementRecord& sensor = f.system->sensorMap().at(SensorId{"enc_a"});
    EXPECT_EQ(sensor.latest->sequence, 1u);
    EXPECT_EQ(sensor.latest->epoch, 1u);
    EXPECT_NEAR(sensor.latest->payload.get<EncoderSample>()->angle_rad, 0.0, 1e-12);

    // the counter's captured state reset once as well
    EXPECT_EQ(f.system->resourceMap()
                  .at(ResourceId{"counter"})
                  .outputs.at(OutputId{"value"})
                  .latest->payload.get<CounterSample>()
                  ->n,
              1);
    EXPECT_EQ(f.system->resourceMap().at(ResourceId{"counter"}).outputs.at(OutputId{"value"}).latest->epoch,
              1u);
}

TEST(Stages, DownstreamConsumersReadTheSensorMap) {
    Fixture     f;
    std::string err;
    ASSERT_TRUE(f.build(systemXml(kPicoResources, kPicoSensors, R"(
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization>
            <Observation id="heading" type="imu_heading_increment">
                <Input sensor_id="imu"/>
                <Calibration bias_samples="0"/>
                <Output observation_id="heading"/>
            </Observation>
            <Estimator type="noop"/>
        </Localization>
        <FieldEstimation type="noop"/>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>)"),
                        err))
        << err;
    auto* uart = f.link<MemoryLink>("pico_uart");

    uart->input().feed(packet(1, 1000, 0, 0, 0));
    f.system->step(hostTime(1));
    uart->input().feed(packet(2, 1005, 0, 0, 100000));
    f.system->step(hostTime(2));
    f.system->step(hostTime(3));   // quiet: nothing new to integrate

    const FunctionStats& stats = f.system->diagnostics().functions.at("Observation/heading");
    EXPECT_EQ(stats.ok, 2u);        // seed, then one integrated delta
    EXPECT_EQ(stats.no_data, 1u);   // the retained sample was not consumed twice
}

TEST(Stages, AttitudeRejectsInvalidQuaternionsAndRecoversOnNewEvidence) {
    ResourceCatalog outputs;
    ASSERT_TRUE(outputs.add(ResourceId{"imu"}, {
        ResourceOutputDecl{OutputId{"attitude"},
                           PayloadDescriptor::of<AttitudeSample>(payload_names::kAttitudeSample)}}));
    SensorInitializationContext init;
    init.outputs = &outputs;
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(R"(
        <Sensor><Source resource_id="imu" output_id="attitude"/>
            <Mounting calibration_status="UNCONFIGURED" roll_deg="0" pitch_deg="0" yaw_deg="90"/>
        </Sensor>)"), tinyxml2::XML_SUCCESS);
    std::string err;
    auto sensor = make_attitude_channel(ConfigNode{doc.RootElement()}, init, err);
    ASSERT_TRUE(sensor.has_value()) << err;

    ResourceMap resources;
    auto& record = resources[ResourceId{"imu"}].outputs[OutputId{"attitude"}];
    record.state = SourceState::kValid;
    uint64_t sequence = 0;
    auto publish = [&](Quaternion q) {
        AttitudeSample value;
        value.q_reference_body = q;
        StoredSample stored;
        stored.sequence = ++sequence;
        stored.measuredAt = hostTime(100 + sequence);
        stored.receivedAt = hostTime(200 + sequence);
        stored.payload = TypedPayload::store(value, payload_names::kAttitudeSample);
        record.latest = std::move(stored);
        return sensor->execute(resources, ExecutionContext{hostTime(300 + sequence), sequence, nullptr});
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const Quaternion invalid : {Quaternion{0, 0, 0, 0}, Quaternion{nan, 0, 0, 0},
                                     Quaternion{1, inf, 0, 0}, Quaternion{1, 0, nan, 0},
                                     Quaternion{1, 0, 0, inf}}) {
        const auto result = publish(invalid);
        EXPECT_EQ(result.state, SourceState::kFault);
        EXPECT_FALSE(result.publication.has_value());
        EXPECT_NE(result.diagnostic.find("quaternion"), std::string::npos);
        // Polling the retained invalid sample must not turn it healthy.
        const auto retained = sensor->execute(resources, ExecutionContext{hostTime(350), 10, nullptr});
        EXPECT_EQ(retained.state, SourceState::kFault);
        EXPECT_FALSE(retained.publication.has_value());
    }
    // Mounting is applied even though its human annotation says UNCONFIGURED.
    const auto good = publish(yawQuaternion(degToRad(90)));
    ASSERT_TRUE(good.publication.has_value());
    EXPECT_EQ(good.state, SourceState::kValid);
    const auto* body = good.publication->payload.get<AttitudeSample>();
    ASSERT_NE(body, nullptr);
    EXPECT_NEAR(angleBetween(body->q_reference_body, Quaternion{}), 0.0, 1e-9);
    EXPECT_EQ(good.publication->measuredAt, record.latest->measuredAt);
    EXPECT_EQ(good.publication->receivedAt, record.latest->receivedAt);
}
