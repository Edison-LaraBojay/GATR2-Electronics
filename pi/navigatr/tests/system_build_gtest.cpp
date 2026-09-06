// system_build_gtest.cpp
// The configuration is the complete declaration of the running system: every
// slot explicitly typed, every reference resolvable, strict values, fixed
// execution order, and no partial startup.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "impl/noop/noops.h"
#include "payloads/sensor_samples.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;

namespace
{

const char* kAllNoop = R"(
<System>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Preprocessing type="noop"/>
        <Localization type="noop"/>
        <FieldEstimation type="noop"/>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>
)";

std::unique_ptr<System> tryBuild(const char* xml, std::string& err) {
    FunctionRegistry functions;
    registerAll(functions);
    return System::buildFromString(xml, functions, err);
}

// replaces one slot line of the all-noop pipeline
std::string withSlot(const char* original_line, const char* replacement) {
    std::string xml(kAllNoop);
    const auto  pos = xml.find(original_line);
    EXPECT_NE(pos, std::string::npos);
    xml.replace(pos, std::string(original_line).size(), replacement);
    return xml;
}

} // namespace

TEST(SystemBuild, AllNoopBuildsStepsAndPersistsCommandState) {
    std::string err;
    auto        system = tryBuild(kAllNoop, err);
    ASSERT_NE(system, nullptr) << err;

    for (int i = 0; i < 5; ++i) {
        system->step(hostTime(i + 1));
    }
    EXPECT_EQ(system->cycle(), 5u);
    EXPECT_FALSE(system->robot().valid);
    EXPECT_TRUE(system->field().objects.empty());
    // no update carries the previous command state forward unchanged
    EXPECT_TRUE(system->command().stream_on);
    EXPECT_EQ(system->command().init_sequence, 0u);
    EXPECT_EQ(system->diagnostics().functions.at("Localization/noop").runs, 5u);
}

TEST(SystemBuild, MissingSlotIsAnError) {
    std::string err;
    EXPECT_EQ(tryBuild(withSlot("<FieldEstimation type=\"noop\"/>", "")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("FieldEstimation"), std::string::npos);
    EXPECT_NE(err.find("noop"), std::string::npos);   // the message names the fix
}

TEST(SystemBuild, MissingTypeIsAnError) {
    std::string err;
    EXPECT_EQ(tryBuild(withSlot("<CommandCollection type=\"noop\"/>",
                                "<CommandCollection/>")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("CommandCollection"), std::string::npos);
}

TEST(SystemBuild, UnknownTypeIsAnError) {
    std::string err;
    EXPECT_EQ(tryBuild(withSlot("<Localization type=\"noop\"/>",
                                "<Localization type=\"quantum\"/>")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("quantum"), std::string::npos);
}

TEST(SystemBuild, WrongCategoryKeyCannotBeConstructed) {
    // a publisher-only name in the localization slot fails on signature,
    // while the shared name noop resolves per category
    std::string err;
    EXPECT_EQ(tryBuild(withSlot("<Localization type=\"noop\"/>",
                                "<Localization type=\"vex_brain\"/>")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("different signature"), std::string::npos);
}

TEST(SystemBuild, DuplicateSlotAndUnknownChildrenRejected) {
    std::string err;
    EXPECT_EQ(tryBuild(withSlot("<FieldEstimation type=\"noop\"/>",
                                "<FieldEstimation type=\"noop\"/>"
                                "<FieldEstimation type=\"noop\"/>")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("more than one FieldEstimation"), std::string::npos);

    EXPECT_EQ(tryBuild(withSlot("<FieldEstimation type=\"noop\"/>",
                                "<FieldEstimation type=\"noop\"/>"
                                "<Perceptron type=\"noop\"/>")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("Perceptron"), std::string::npos);

    EXPECT_EQ(tryBuild(R"(<System><Sensros/><Pipeline/></System>)", err), nullptr);
    EXPECT_NE(err.find("Sensros"), std::string::npos);
}

TEST(SystemBuild, StrictValuesRejectNanInfinityAndJunk) {
    std::string err;

    EXPECT_EQ(tryBuild(withSlot("<Pipeline>", "<Loop rate_hz=\"fast\"/><Pipeline>").c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("invalid value"), std::string::npos);

    EXPECT_EQ(tryBuild(withSlot("<Pipeline>", "<Loop rate_hz=\"nan\"/><Pipeline>").c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("invalid value"), std::string::npos);

    EXPECT_EQ(tryBuild(withSlot("<Pipeline>", "<Loop rate_hz=\"inf\"/><Pipeline>").c_str(),
                       err),
              nullptr);

    EXPECT_EQ(tryBuild(withSlot("<Pipeline>", "<Loop rate_hz=\"0\"/><Pipeline>").c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("positive"), std::string::npos);
}

TEST(SystemBuild, ErrorsCarryPathIdAndType) {
    std::string err;
    EXPECT_EQ(tryBuild(R"(
<System>
    <Resources>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="missing_uart"/>
        </Resource>
    </Resources>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Preprocessing type="noop"/>
        <Localization type="noop"/>
        <FieldEstimation type="noop"/>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>
)",
                       err),
              nullptr);
    EXPECT_NE(err.find("/System/Resources/Resource[@id='pico_telemetry']"),
              std::string::npos);
    EXPECT_NE(err.find("missing_uart"), std::string::npos);
}

TEST(SystemBuild, ReplayMustNameDeclaredResource) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string  err;
    BuildOptions options;
    options.replay["ghost"] = "capture.bin";
    EXPECT_EQ(System::buildFromString(kAllNoop, functions, err, options), nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);
}

TEST(SystemBuild, DeclarationReorderingChangesNothing) {
    // telemetry declared before the uart it depends on; sensors in any order
    const char* xml = R"(
<System>
    <Resources>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
        </Resource>
        <Resource id="pico_uart" type="memory_link"/>
    </Resources>
    <Sensors>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" channel="imu"/>
        </Sensor>
        <Sensor id="enc" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" channel="0"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Preprocessing type="noop"/>
        <Localization type="noop"/>
        <FieldEstimation type="noop"/>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>
)";
    std::string err;
    auto        system = tryBuild(xml, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_NE(system->sensorCatalog().payloadOf(SensorId{"enc"}), nullptr);
    EXPECT_NE(system->sensorCatalog().payloadOf(SensorId{"robot_imu"}), nullptr);
}


TEST(SystemBuild, UndeclaredOrMistypedOutputsNeverEnterStandardMaps) {
    FunctionRegistry functions;
    registerAll(functions);

    // A sensor that declares one payload and publishes another.
    functions.add<SensorMakeFunction>(
        FunctionKey{"liar_sensor"},
        [](const ConfigNode&, SensorInitializationContext&, std::string&) {
            SensorExecutable executable;
            executable.outputPayload =
                PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample);
            executable.execute = [](const SensorExecutionInput&) {
                SensorPollResult result;
                result.state = SensorState::kValid;
                SensorPublication publication;
                publication.measuredAt = deviceTime(1);
                publication.payload =
                    TypedPayload::store(ImuSample{1.0}, payload_names::kImuSample);
                result.publication = std::move(publication);
                return result;
            };
            return std::optional<SensorExecutable>(std::move(executable));
        });

    // A world estimation that emits an observation it never declared.
    struct LiarWorld : FieldEstimation {
        FieldEstimationOutput run(const FieldEstimationInput& in) override {
            FieldEstimationOutput out;
            out.field = in.previousField;
            ObservationRecord record;
            record.payload = TypedPayload::store(ImuSample{1.0},
                                                 payload_names::kImuSample);
            out.observations.emplace(ObservationId{"ghost"}, std::move(record));
            return out;
        }
    };
    functions.add<FieldEstimationMakeFunction>(
        FunctionKey{"liar_field"},
        [](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<LiarWorld>();
        });

    // A capture point downstream: what does target resolution actually see?
    auto seen = std::make_shared<std::size_t>(99);
    struct CaptureTargets : TargetResolution {
        std::shared_ptr<std::size_t> seen;
        explicit CaptureTargets(std::shared_ptr<std::size_t> s) : seen(std::move(s)) {}
        TargetResolutionOutput run(const TargetResolutionInput& in) override {
            *seen = in.observations.size();
            return TargetResolutionOutput{in.previous, FunctionStatus::kOk};
        }
    };
    functions.add<TargetResolutionMakeFunction>(
        FunctionKey{"capture"},
        [seen](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<CaptureTargets>(seen);
        });

    const char* xml = R"(
<System>
    <Resources>
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="liar" type="liar_sensor"/>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Preprocessing type="noop"/>
        <Localization type="noop"/>
        <FieldEstimation type="liar_field"/>
        <TargetResolution type="capture"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
    std::string err;
    auto        system = System::buildFromString(xml, functions, err);
    ASSERT_NE(system, nullptr) << err;
    system->step(hostTime(1));

    // the lying publication never entered the record and reads as a fault
    const auto& record = system->sensorResults().at(SensorId{"liar"});
    EXPECT_EQ(record.state, SensorState::kFault);
    EXPECT_FALSE(record.latest.has_value());
    EXPECT_NE(record.diagnostic.find("declared"), std::string::npos);

    // the undeclared observation was dropped before anyone downstream saw it
    EXPECT_EQ(*seen, 0u);
    EXPECT_EQ(system->diagnostics()
                  .functions.count("FieldEstimation/liar_field/undeclared_output:ghost"),
              1u);
}

TEST(SystemBuild, ExecutionOrderMatchesTheFixedPipeline) {
    // probe implementations log their category; the log must follow the
    // semantic order regardless of anything else
    auto log = std::make_shared<std::vector<std::string>>();

    FunctionRegistry functions;
    registerAll(functions);

    struct ProbeCommands : Commands {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbeCommands(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        CommandsOutput run(const CommandsInput& in) override {
            log->push_back("commands");
            return CommandsOutput{in.previous, FunctionStatus::kOk};
        }
    };
    struct ProbePreprocessing : Preprocessing {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbePreprocessing(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        PreprocessingOutput run(const PreprocessingInput&) override {
            log->push_back("preprocessing");
            return PreprocessingOutput{};
        }
    };
    struct ProbeLocalization : Localization {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbeLocalization(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        LocalizationOutput run(const LocalizationInput& in) override {
            log->push_back("localization");
            return LocalizationOutput{in.previous, FunctionStatus::kOk};
        }
    };
    struct ProbeField : FieldEstimation {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbeField(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        FieldEstimationOutput run(const FieldEstimationInput& in) override {
            log->push_back("field_estimation");
            FieldEstimationOutput out;
            out.field = in.previousField;
            return out;
        }
    };
    struct ProbeTargets : TargetResolution {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbeTargets(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        TargetResolutionOutput run(const TargetResolutionInput& in) override {
            log->push_back("target_resolution");
            return TargetResolutionOutput{in.previous, FunctionStatus::kOk};
        }
    };
    struct ProbePublishing : Publishing {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbePublishing(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        PublishingOutput run(const PublishingInput&) override {
            log->push_back("publishing");
            return PublishingOutput{};
        }
    };

    functions.add<CommandsMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbeCommands>(log);
        });
    functions.add<PreprocessingMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, PreprocessorInitializationContext&, std::string&) {
            return std::make_unique<ProbePreprocessing>(log);
        });
    functions.add<LocalizationMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbeLocalization>(log);
        });
    functions.add<FieldEstimationMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbeField>(log);
        });
    functions.add<TargetResolutionMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbeTargets>(log);
        });
    functions.add<PublishingMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbePublishing>(log);
        });

    // slots deliberately listed out of order in the XML; execution order is
    // the framework's, not the document's
    const char* xml = R"(
<System>
    <Pipeline>
        <Publishing type="probe"/>
        <TargetResolution type="probe"/>
        <CommandCollection type="probe"/>
        <FieldEstimation type="probe"/>
        <Localization type="probe"/>
        <Preprocessing type="probe"/>
    </Pipeline>
</System>
)";
    std::string err;
    auto        system = System::buildFromString(xml, functions, err);
    ASSERT_NE(system, nullptr) << err;

    system->step(hostTime(1));
    EXPECT_EQ(*log, (std::vector<std::string>{"commands", "preprocessing", "localization",
                                              "field_estimation", "target_resolution",
                                              "publishing"}));
}
