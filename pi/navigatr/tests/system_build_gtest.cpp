// system_build_gtest.cpp
// The configuration is the complete declaration of the running system: every
// slot explicitly typed, every reference resolvable, strict values, fixed
// execution order, and no partial startup.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "impl/noop/noops.h"
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
        <LocalizationPrediction type="noop"/>
        <Perception type="noop"/>
        <Association type="noop"/>
        <PoseCorrection type="noop"/>
        <WorldPrediction type="noop"/>
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
    EXPECT_TRUE(system->world().objects.empty());
    // no update carries the previous command state forward unchanged
    EXPECT_TRUE(system->command().stream_on);
    EXPECT_EQ(system->command().init_sequence, 0u);
    EXPECT_EQ(system->diagnostics().functions.at("LocalizationPrediction/noop").runs, 5u);
}

TEST(SystemBuild, MissingSlotIsAnError) {
    std::string err;
    EXPECT_EQ(tryBuild(withSlot("<WorldPrediction type=\"noop\"/>", "")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("WorldPrediction"), std::string::npos);
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
    EXPECT_EQ(tryBuild(withSlot("<LocalizationPrediction type=\"noop\"/>",
                                "<LocalizationPrediction type=\"quantum\"/>")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("quantum"), std::string::npos);
}

TEST(SystemBuild, WrongCategoryKeyCannotBeConstructed) {
    // a publisher-only name in the localization slot fails on signature,
    // while the shared name noop resolves per category
    std::string err;
    EXPECT_EQ(tryBuild(withSlot("<LocalizationPrediction type=\"noop\"/>",
                                "<LocalizationPrediction type=\"vex_brain\"/>")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("different signature"), std::string::npos);
}

TEST(SystemBuild, DuplicateSlotAndUnknownChildrenRejected) {
    std::string err;
    EXPECT_EQ(tryBuild(withSlot("<Perception type=\"noop\"/>",
                                "<Perception type=\"noop\"/>"
                                "<Perception type=\"noop\"/>")
                           .c_str(),
                       err),
              nullptr);
    EXPECT_NE(err.find("more than one Perception"), std::string::npos);

    EXPECT_EQ(tryBuild(withSlot("<Perception type=\"noop\"/>",
                                "<Perception type=\"noop\"/>"
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
        <LocalizationPrediction type="noop"/>
        <Perception type="noop"/>
        <Association type="noop"/>
        <PoseCorrection type="noop"/>
        <WorldPrediction type="noop"/>
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
        <LocalizationPrediction type="noop"/>
        <Perception type="noop"/>
        <Association type="noop"/>
        <PoseCorrection type="noop"/>
        <WorldPrediction type="noop"/>
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
    struct ProbePerception : Perception {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbePerception(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        PerceptionOutput run(const PerceptionInput&) override {
            log->push_back("perception");
            return PerceptionOutput{};
        }
    };
    struct ProbeAssociation : Association {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbeAssociation(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        AssociationOutput run(const AssociationInput&) override {
            log->push_back("association");
            return AssociationOutput{};
        }
    };
    struct ProbeCorrection : PoseCorrection {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbeCorrection(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        PoseCorrectionOutput run(const PoseCorrectionInput& in) override {
            log->push_back("pose_correction");
            return PoseCorrectionOutput{in.predicted, FunctionStatus::kOk};
        }
    };
    struct ProbeWorld : WorldPrediction {
        std::shared_ptr<std::vector<std::string>> log;
        explicit ProbeWorld(std::shared_ptr<std::vector<std::string>> l)
            : log(std::move(l)) {}
        WorldPredictionOutput run(const WorldPredictionInput& in) override {
            log->push_back("world_prediction");
            return WorldPredictionOutput{in.previousWorld, in.previousTarget,
                                         FunctionStatus::kOk};
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
    functions.add<PerceptionMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbePerception>(log);
        });
    functions.add<AssociationMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbeAssociation>(log);
        });
    functions.add<PoseCorrectionMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbeCorrection>(log);
        });
    functions.add<WorldPredictionMakeFunction>(
        FunctionKey{"probe"},
        [log](const ConfigNode&, SlotInitializationContext&, std::string&) {
            return std::make_unique<ProbeWorld>(log);
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
        <Perception type="probe"/>
        <CommandCollection type="probe"/>
        <WorldPrediction type="probe"/>
        <LocalizationPrediction type="probe"/>
        <PoseCorrection type="probe"/>
        <Preprocessing type="probe"/>
        <Association type="probe"/>
    </Pipeline>
</System>
)";
    std::string err;
    auto        system = System::buildFromString(xml, functions, err);
    ASSERT_NE(system, nullptr) << err;

    system->step(hostTime(1));
    EXPECT_EQ(*log, (std::vector<std::string>{"commands", "preprocessing", "localization",
                                              "perception", "association",
                                              "pose_correction", "world_prediction",
                                              "publishing"}));
}
