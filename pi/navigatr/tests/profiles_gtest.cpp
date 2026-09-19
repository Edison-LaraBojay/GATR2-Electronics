// profiles_gtest.cpp
// Multi-file configuration resolution and the robot-description ownership
// of wheel geometry: profiles compose from fragments with strict roots,
// relative paths, duplicate and template rejection, stable identity, and
// the checked-in configuration tree stays honest.

#include <gtest/gtest.h>

#include <cstdint>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>

#include "config/composition.h"
#include "config/field_map.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

const char* kRobotFragment = R"(
<Robot>
  <Resources>
    <Resource id="pico_uart" type="memory_link"/>
    <Resource id="pico_telemetry" type="pico_telemetry">
      <Serial resource_id="pico_uart"/>
      <Output id="encoder_a" channel="0"/>
      <Output id="encoder_b" channel="1"/>
      <Output id="encoder_c" channel="2"/>
      <Output id="imu" channel="imu"/>
    </Resource>
    <Resource id="wheel_geometry" type="wheel_geometry">
      <Wheel id="forward_wheel" sensor_id="enc_a" calibration_status="verified"
             radius_m="0.0254" position_x_m="0" position_y_m="0"
             measurement_angle_deg="0" direction="positive"/>
      <Wheel id="lateral_wheel" sensor_id="enc_b" calibration_status="verified"
             radius_m="0.0254" position_x_m="0" position_y_m="0"
             measurement_angle_deg="90" direction="positive"/>
    </Resource>
  </Resources>
  <Sensors>
    <Sensor id="enc_a" type="pico_encoder_channel">
      <Source resource_id="pico_telemetry" output_id="encoder_a"/>
      <Calibration counts_per_revolution="4000"/>
    </Sensor>
    <Sensor id="enc_b" type="pico_encoder_channel">
      <Source resource_id="pico_telemetry" output_id="encoder_b"/>
      <Calibration counts_per_revolution="4000"/>
    </Sensor>
    <Sensor id="imu" type="pico_imu_channel">
      <Source resource_id="pico_telemetry" output_id="imu"/>
    </Sensor>
  </Sensors>
</Robot>
)";

const char* kPipelineFragment = R"(
<Pipeline>
  <CommandCollection type="noop"/>
  <Localization>
    <Observation id="motion" type="tracking_wheel_motion">
      <Wheels resource_id="wheel_geometry">
        <Use wheel_id="forward_wheel"/>
        <Use wheel_id="lateral_wheel"/>
      </Wheels>
      <HeadingConstraint sensor_id="imu" bias_samples="0"/>
      <Output observation_id="motion"/>
    </Observation>
    <Estimator type="planar_motion_integrator">
      <Motion observation_id="motion"/>
    </Estimator>
  </Localization>
  <FieldEstimation type="noop"/>
  <TargetResolution type="noop"/>
  <Publishing type="noop"/>
</Pipeline>
)";

const char* kFieldFragment = R"(
<Resource id="test_field" type="field_map">
  <Landmark id="center_goal">
    <NominalPose calibration_status="verified" x_m="1.8" y_m="1.8" heading_deg="0"/>
  </Landmark>
</Resource>
)";

const char* kProfile = R"(
<Configuration id="test_profile">
  <ConfigurationName>Test profile</ConfigurationName>
  <Loop rate_hz="50"/>
  <Robot file="robot.xml"/>
  <Field file="field.xml"/>
  <Pipeline file="pipeline.xml"/>
</Configuration>
)";

struct Workspace {
    std::filesystem::path dir;

    Workspace() {
        static std::atomic<uint64_t> sequence{0};
        std::random_device random;
        do {
            const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
            dir = std::filesystem::temp_directory_path() /
                  ("navigatr_profiles_gtest_" + std::to_string(stamp) + "_" +
                   std::to_string(random()) + "_" + std::to_string(sequence++));
        } while (!std::filesystem::create_directory(dir));
        write("robot.xml", kRobotFragment);
        write("pipeline.xml", kPipelineFragment);
        write("field.xml", kFieldFragment);
        write("profile.xml", kProfile);
    }
    ~Workspace() { std::filesystem::remove_all(dir); }

    void write(const char* name, const std::string& content) {
        std::filesystem::create_directories((dir / name).parent_path());
        std::ofstream f(dir / name, std::ios::binary);
        f << content;
    }

    std::string path(const char* name) const { return (dir / name).string(); }
};

std::string xmlOf(const tinyxml2::XMLElement* element) {
    tinyxml2::XMLPrinter printer;
    element->Accept(&printer);
    return printer.CStr();
}

// Replace one section with a reference after saving its current subtree.
void externalize(Workspace& ws, tinyxml2::XMLElement* element,
                 const char* destination, const char* reference) {
    ws.write(destination, xmlOf(element));
    auto* replacement = element->GetDocument()->NewElement(element->Name());
    replacement->SetAttribute("file", reference);
    auto* parent = element->Parent();
    parent->InsertAfterChild(element, replacement);
    parent->DeleteChild(element);
}

} // namespace

TEST(Profiles, PlainSystemSectionsAndIndividualEntriesResolveLikeInlineConfiguration) {
    Workspace ws;
    tinyxml2::XMLDocument robot;
    ASSERT_EQ(robot.Parse(kRobotFragment), tinyxml2::XML_SUCCESS);
    auto* resources = robot.RootElement()->FirstChildElement("Resources");
    auto* sensors = robot.RootElement()->FirstChildElement("Sensors");
    ws.write("inline.xml", "<System><Loop rate_hz=\"50\"/>" + xmlOf(resources) +
             xmlOf(sensors) + kPipelineFragment + "</System>");
    for (auto* resource = resources->FirstChildElement(); resource != nullptr;) {
        auto* next = resource->NextSiblingElement();
        const std::string reference = std::string("devices/") + resource->Attribute("id") + ".xml";
        const std::string destination = "parts/hardware/" + reference;
        externalize(ws, resource, destination.c_str(), reference.c_str());
        resource = next;
    }
    for (auto* sensor = sensors->FirstChildElement(); sensor != nullptr;) {
        auto* next = sensor->NextSiblingElement();
        const std::string reference = std::string("channels/") + sensor->Attribute("id") + ".xml";
        const std::string destination = "parts/sensing/" + reference;
        externalize(ws, sensor, destination.c_str(), reference.c_str());
        sensor = next;
    }
    ws.write("parts/hardware/resources.xml", xmlOf(resources));
    ws.write("parts/sensing/sensors.xml", xmlOf(sensors));
    tinyxml2::XMLDocument pipeline;
    ASSERT_EQ(pipeline.Parse(kPipelineFragment), tinyxml2::XML_SUCCESS);
    auto* localization = pipeline.RootElement()->FirstChildElement("Localization");
    externalize(ws, localization->FirstChildElement("Observation"),
                "parts/localization/models/motion.xml", "models/motion.xml");
    externalize(ws, localization->FirstChildElement("Estimator"),
                "parts/localization/estimator.xml", "estimator.xml");
    externalize(ws, localization, "parts/localization/localization.xml",
                "localization/localization.xml");
    ws.write("parts/pipeline.xml", xmlOf(pipeline.RootElement()));
    ws.write("main.xml", R"(<System><Loop rate_hz="50"/>
        <Resources file="parts/hardware/resources.xml"/>
        <Sensors file="parts/sensing/sensors.xml"/>
        <Pipeline file="parts/pipeline.xml"/></System>)");

    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto inline_system = System::buildFromFile(ws.path("inline.xml"), functions, err);
    ASSERT_NE(inline_system, nullptr) << err;
    auto modular = System::buildFromFile(ws.path("main.xml"), functions, err);
    ASSERT_NE(modular, nullptr) << err;
    EXPECT_EQ(modular->configurationId(), ws.path("main.xml"));
    EXPECT_EQ(modular->loopRateHz(), inline_system->loopRateHz());
    for (const char* id : {"enc_a", "enc_b", "imu"}) {
        ASSERT_NE(modular->sensorCatalog().payloadOf(SensorId{id}), nullptr);
    }
    EXPECT_NE(modular->resources().findValue(ResourceId{"pico_telemetry"}), nullptr);
    EXPECT_EQ(modular->localization().estimatorType(), inline_system->localization().estimatorType());
    EXPECT_EQ(modular->localization().observationOutputs().size(),
              inline_system->localization().observationOutputs().size());
    modular->step(hostTime(10));
    inline_system->step(hostTime(10));
    EXPECT_EQ(modular->robot().valid, inline_system->robot().valid);
    EXPECT_EQ(modular->sensorMap().size(), inline_system->sensorMap().size());
    ResolvedConfiguration resolved;
    ASSERT_TRUE(resolveConfiguration(ws.path("main.xml"), resolved, err)) << err;
    EXPECT_EQ(resolved.files.size(), 13u);
    EXPECT_EQ(resolved.xml.find("file="), std::string::npos);
    EXPECT_EQ(modular->configurationDigest(), resolved.digest);

    const auto old_digest = resolved.digest;
    ws.write("parts/localization/models/motion.xml",
             [&] { tinyxml2::XMLDocument doc; doc.Parse(kPipelineFragment);
                   return xmlOf(doc.RootElement()->FirstChildElement("Localization")
                                    ->FirstChildElement("Observation")) + "<!-- changed -->"; }());
    ASSERT_TRUE(resolveConfiguration(ws.path("main.xml"), resolved, err)) << err;
    EXPECT_NE(resolved.digest, old_digest); // deepest dependency participates
}

TEST(Profiles, NestedStagePipelinesAndAbsoluteReferencesResolve) {
    Workspace ws;
    ws.write("stage/field.xml", R"(<FieldEstimation type="landmark_field">
        <FieldMap resource_id="test_field"/><Pipeline file="inner/pipeline.xml"/>
        </FieldEstimation>)");
    ws.write("stage/inner/pipeline.xml", R"(<Pipeline>
        <ObservationExtraction file="observation.xml"/>
        <Association file="association.xml"/><Estimator file="estimator.xml"/>
        </Pipeline>)");
    ws.write("stage/inner/observation.xml", R"(<ObservationExtraction type="noop"/>)");
    ws.write("stage/inner/association.xml", R"(<Association type="noop"/>)");
    ws.write("stage/inner/estimator.xml", R"(<Estimator type="landmark_estimator" commit="never"/>)");
    tinyxml2::XMLDocument main;
    ASSERT_EQ(main.Parse(R"(<System><Resources><Resource file="field.xml"/></Resources>
        <Sensors/><Pipeline><CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>
        <FieldEstimation/><TargetResolution type="noop"/><Publishing type="noop"/>
        </Pipeline></System>)"), tinyxml2::XML_SUCCESS);
    main.RootElement()->FirstChildElement("Pipeline")->FirstChildElement("FieldEstimation")
        ->SetAttribute("file", ws.path("stage/field.xml").c_str());
    ws.write("nested.xml", xmlOf(main.RootElement()));
    std::string err;
    ResolvedConfiguration resolved;
    ASSERT_TRUE(resolveConfiguration(ws.path("nested.xml"), resolved, err)) << err;
    EXPECT_EQ(resolved.files.size(), 7u);
    tinyxml2::XMLDocument expanded;
    ASSERT_EQ(expanded.Parse(resolved.xml.c_str()), tinyxml2::XML_SUCCESS);
    const auto* field = expanded.RootElement()->FirstChildElement("Pipeline")
                           ->FirstChildElement("FieldEstimation");
    ASSERT_NE(field, nullptr);
    EXPECT_STREQ(field->Attribute("type"), "landmark_field");
    EXPECT_STREQ(field->FirstChildElement("Pipeline")->FirstChildElement("Estimator")
                     ->Attribute("commit"), "never");
    FunctionRegistry functions;
    registerAll(functions);
    EXPECT_NE(System::buildFromFile(ws.path("nested.xml"), functions, err), nullptr) << err;
}

TEST(Profiles, ExistingConfigurationFragmentsCanRecursivelyReferenceSections) {
    Workspace ws;
    tinyxml2::XMLDocument robot;
    ASSERT_EQ(robot.Parse(kRobotFragment), tinyxml2::XML_SUCCESS);
    externalize(ws, robot.RootElement()->FirstChildElement("Resources"),
                "hardware/resources.xml", "hardware/resources.xml");
    externalize(ws, robot.RootElement()->FirstChildElement("Sensors"),
                "hardware/sensors.xml", "hardware/sensors.xml");
    ws.write("robot.xml", xmlOf(robot.RootElement()));
    ws.write("pipeline.xml", R"(<Pipeline file="pipeline/actual.xml"/>)");
    ws.write("pipeline/actual.xml", kPipelineFragment);
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    const auto system = System::buildFromFile(ws.path("profile.xml"), functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->configurationId(), "test_profile");
    EXPECT_NE(system->sensorCatalog().payloadOf(SensorId{"imu"}), nullptr);
}

TEST(Profiles, RecursiveReferenceFailuresAreExplicitAndAtomic) {
    Workspace ws;
    ws.write("resources.xml", "<Resources/>");
    const std::pair<const char*, const char*> conflicts[] = {
        {R"(<Resources file="resources.xml" id="override"/>)", "cannot also specify id"},
        {R"(<Resources file="resources.xml" type="override"/>)", "cannot also specify type"},
        {R"(<Resources file="resources.xml"><Resource id="r" type="memory_link"/></Resources>)", "inline content"},
        {R"(<Resources file="resources.xml">unexpected text</Resources>)", "inline content"},
        {R"(<Resources file=""/>)", "nonempty file"},
        {R"(<Resources file="missing.xml"/>)", "missing.xml"},
        {R"(<Resources file="pipeline.xml"/>)", "must be Resources"},
    };
    ResolvedConfiguration resolved;
    std::string err;
    for (const auto& conflict : conflicts) {
        ws.write("invalid.xml", std::string("<System>") + conflict.first + "</System>");
        resolved.xml = "old result";
        resolved.digest = 42;
        EXPECT_FALSE(resolveConfiguration(ws.path("invalid.xml"), resolved, err)) << conflict.first;
        EXPECT_NE(err.find(conflict.second), std::string::npos) << err;
        EXPECT_NE(err.find("include chain"), std::string::npos) << err;
        EXPECT_TRUE(resolved.xml.empty());
        EXPECT_EQ(resolved.digest, 0u);
    }
    ws.write("main.xml", R"(<System><Resources file="cycle/a.xml"/></System>)");
    ws.write("cycle/a.xml", R"(<Resources file="b.xml"/>)");
    ws.write("cycle/b.xml", R"(<Resources file="../cycle/./a.xml"/>)");
    EXPECT_FALSE(resolveConfiguration(ws.path("main.xml"), resolved, err));
    EXPECT_NE(err.find("cyclic include"), std::string::npos) << err;
    EXPECT_NE(err.find("a.xml"), std::string::npos);
    EXPECT_NE(err.find("b.xml"), std::string::npos);
}

TEST(Profiles, ImplementationFileAttributesAndHumanAnnotationsAreNotIncludesOrGates) {
    Workspace ws;
    ws.write("resources.xml", R"(<Resources calibration_status="@UNMEASURED@">
        <Resource id="device" type="implementation_owned">
            <Device file="not-an-xml-document.raw"/>
            <Options><Pipeline file="also-implementation-owned.raw"/></Options>
        </Resource></Resources>)");
    ws.write("main.xml", R"(<System>
        <Resources file="resources.xml" calibration_status="@READER_NOTE@">
            <!-- comments on references are fine -->
        </Resources><Sensors/><Pipeline><Localization><Estimator type="noop"/>
        </Localization></Pipeline></System>)");
    ResolvedConfiguration resolved;
    std::string err;
    ASSERT_TRUE(resolveConfiguration(ws.path("main.xml"), resolved, err)) << err;
    EXPECT_EQ(resolved.files.size(), 2u);
    EXPECT_NE(resolved.xml.find("not-an-xml-document.raw"), std::string::npos);
    EXPECT_NE(resolved.xml.find("also-implementation-owned.raw"), std::string::npos);
    EXPECT_NE(resolved.xml.find("@UNMEASURED@"), std::string::npos);
    ws.write("resources.xml", R"(<Resources><Resource id="w" type="wheel_geometry">
        <Wheel radius_m="@REAL_VALUE_REQUIRED@"/></Resource></Resources>)");
    EXPECT_FALSE(resolveConfiguration(ws.path("main.xml"), resolved, err));
    EXPECT_NE(err.find("radius_m"), std::string::npos);
}

TEST(Profiles, StringConstructionRejectsUnresolvedSectionReferencesWithoutReadingFiles) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    const char* unresolved[] = {
        R"(<System file="unknown.xml"/>)",
        R"(<System><Resources file="unknown.xml"/></System>)",
        R"(<System><Resources><Resource file="unknown.xml"/></Resources></System>)",
        R"(<System><Sensors><Sensor file="unknown.xml"/></Sensors></System>)",
        R"(<System><Pipeline file="unknown.xml"/></System>)",
        R"(<System><Pipeline><Localization file="unknown.xml"/></Pipeline></System>)",
        R"(<System><Pipeline><Localization><Observation file="unknown.xml"/></Localization></Pipeline></System>)",
        R"(<System><Pipeline><FieldEstimation type="landmark_field"><Pipeline>
            <Association file="unknown.xml"/></Pipeline></FieldEstimation></Pipeline></System>)",
    };
    for (const char* xml : unresolved) {
        EXPECT_EQ(System::buildFromString(xml, functions, err), nullptr);
        EXPECT_NE(err.find("unresolved XML file reference"), std::string::npos) << err;
        EXPECT_NE(err.find("System::buildFromFile"), std::string::npos) << err;
    }
    EXPECT_NE(System::buildFromString(R"(<System><Resources>
        <Resource id="memory" type="memory_link" calibration_status="@HUMAN_NOTE@">
            <Device file="implementation-owned.raw"/>
            <Options><Pipeline file="also-not-an-include.raw"/></Options>
        </Resource></Resources><Sensors/><Pipeline><CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization><FieldEstimation type="noop"/>
        <TargetResolution type="noop"/><Publishing type="noop"/></Pipeline></System>)",
        functions, err), nullptr) << err;
}

TEST(Profiles, ComposedProfileResolvesBuildsAndCarriesIdentity) {
    Workspace        ws;
    FunctionRegistry functions;
    registerAll(functions);

    std::string err;
    auto        system = System::buildFromFile(ws.path("profile.xml"), functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->configurationId(), "test_profile");
    EXPECT_NE(system->configurationDigest(), 0u);
    EXPECT_NEAR(system->loopRateHz(), 50.0, 1e-12);
    EXPECT_NE(system->sensorCatalog().payloadOf(SensorId{"enc_a"}), nullptr);
    system->step(hostTime(1));

    // resolution is deterministic: the same files give the same digest
    auto again = System::buildFromFile(ws.path("profile.xml"), functions, err);
    ASSERT_NE(again, nullptr) << err;
    EXPECT_EQ(again->configurationDigest(), system->configurationDigest());

    // and a changed fragment changes the digest
    ws.write("field.xml", std::string(kFieldFragment) + "<!-- touched -->\n");
    auto touched = System::buildFromFile(ws.path("profile.xml"), functions, err);
    ASSERT_NE(touched, nullptr) << err;
    EXPECT_NE(touched->configurationDigest(), system->configurationDigest());

    // a robot description may be split across fragments (base + add-on)
    ws.write("robot_addon.xml", R"(
<Robot><Resources><Resource id="extra_link" type="memory_link"/></Resources></Robot>)");
    ws.write("profile_split.xml", R"(
<Configuration id="split_profile">
  <Robot file="robot.xml"/>
  <Robot file="robot_addon.xml"/>
  <Pipeline file="pipeline.xml"/>
</Configuration>)");
    auto split = System::buildFromFile(ws.path("profile_split.xml"), functions, err);
    ASSERT_NE(split, nullptr) << err;
    EXPECT_NE(split->resources().findValue(ResourceId{"extra_link"}), nullptr);
}

TEST(Profiles, ResolutionViolationsFailWithTheIncludeChain) {
    Workspace   ws;
    std::string err;

    // missing fragment names the chain
    ws.write("profile_missing.xml", R"(
<Configuration id="p"><Robot file="ghost.xml"/><Pipeline file="pipeline.xml"/></Configuration>)");
    ResolvedConfiguration resolved;
    EXPECT_FALSE(resolveConfiguration(ws.path("profile_missing.xml"), resolved, err));
    EXPECT_NE(err.find("ghost.xml"), std::string::npos);
    EXPECT_NE(err.find("include chain"), std::string::npos);

    // the same file twice is a repeated include, not a merge
    ws.write("profile_dup.xml", R"(
<Configuration id="p"><Robot file="robot.xml"/><Field file="field.xml"/>
<Field file="field.xml"/><Pipeline file="pipeline.xml"/></Configuration>)");
    EXPECT_FALSE(resolveConfiguration(ws.path("profile_dup.xml"), resolved, err));
    EXPECT_NE(err.find("more than once"), std::string::npos);

    // template files cannot run
    ws.write("template.xml.in", kRobotFragment);
    ws.write("profile_template.xml", R"(
<Configuration id="p"><Robot file="template.xml.in"/><Pipeline file="pipeline.xml"/></Configuration>)");
    EXPECT_FALSE(resolveConfiguration(ws.path("profile_template.xml"), resolved, err));
    EXPECT_NE(err.find("template"), std::string::npos);

    // a leftover placeholder token anywhere is fatal, with the location
    ws.write("robot_tokens.xml", R"(
<Robot><Resources>
  <Resource id="wheel_geometry" type="wheel_geometry">
    <Wheel id="w" sensor_id="e" calibration_status="verified"
           radius_m="@MEASURE_ME@" position_x_m="0" position_y_m="0"
           measurement_angle_deg="0" direction="positive"/>
  </Resource>
</Resources></Robot>)");
    ws.write("profile_tokens.xml", R"(
<Configuration id="p"><Robot file="robot_tokens.xml"/><Pipeline file="pipeline.xml"/></Configuration>)");
    EXPECT_FALSE(resolveConfiguration(ws.path("profile_tokens.xml"), resolved, err));
    EXPECT_NE(err.find("@MEASURE_ME@"), std::string::npos);
    EXPECT_NE(err.find("radius_m"), std::string::npos);

    // a resource id declared by two fragments names both files
    ws.write("field_clash.xml", R"(
<Resource id="pico_uart" type="memory_link"/>)");
    ws.write("profile_clash.xml", R"(
<Configuration id="p"><Robot file="robot.xml"/><Field file="field_clash.xml"/>
<Pipeline file="pipeline.xml"/></Configuration>)");
    EXPECT_FALSE(resolveConfiguration(ws.path("profile_clash.xml"), resolved, err));
    EXPECT_NE(err.find("pico_uart"), std::string::npos);
    EXPECT_NE(err.find("robot.xml"), std::string::npos);
    EXPECT_NE(err.find("field_clash.xml"), std::string::npos);

    // unknown profile children and wrong fragment roots fail
    ws.write("profile_unknown.xml", R"(
<Configuration id="p"><Rout file="robot.xml"/><Robot file="robot.xml"/>
<Pipeline file="pipeline.xml"/></Configuration>)");
    EXPECT_FALSE(resolveConfiguration(ws.path("profile_unknown.xml"), resolved, err));
    EXPECT_NE(err.find("Rout"), std::string::npos);

    ws.write("profile_wrong_root.xml", R"(
<Configuration id="p"><Robot file="pipeline.xml"/><Pipeline file="pipeline.xml"/></Configuration>)");
    EXPECT_FALSE(resolveConfiguration(ws.path("profile_wrong_root.xml"), resolved, err));
    EXPECT_NE(err.find("must be Robot"), std::string::npos);
}

TEST(Profiles, WheelGeometryReferenceFormMatchesInlineAndValidates) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;

    // unknown wheel id dies at build with the resource named
    const char* bad_use = R"(
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
    <Resource id="wheel_geometry" type="wheel_geometry">
      <Wheel id="forward_wheel" sensor_id="enc_a" calibration_status="verified"
             radius_m="0.0254" position_x_m="0" position_y_m="0"
             measurement_angle_deg="0" direction="positive"/>
    </Resource>
  </Resources>
  <Sensors>
    <Sensor id="enc_a" type="pico_encoder_channel">
      <Source resource_id="pico_telemetry" output_id="encoder_a"/>
      <Calibration counts_per_revolution="4000"/>
    </Sensor>
  </Sensors>
  <Pipeline>
    <CommandCollection type="noop"/>
    <Localization>
      <Observation id="motion" type="tracking_wheel_motion">
        <Wheels resource_id="wheel_geometry">
          <Use wheel_id="phantom_wheel"/>
        </Wheels>
        <Output observation_id="motion"/>
      </Observation>
      <Estimator type="noop"/>
    </Localization>
    <FieldEstimation type="noop"/>
    <TargetResolution type="noop"/>
    <Publishing type="noop"/>
  </Pipeline>
</System>)";
    EXPECT_EQ(System::buildFromString(bad_use, functions, err), nullptr);
    EXPECT_NE(err.find("phantom_wheel"), std::string::npos);

    // A calibration label is an annotation; valid configured geometry runs.
    const char* provisional = R"(
<System>
  <Resources>
    <Resource id="wheel_geometry" type="wheel_geometry">
      <Wheel id="w" sensor_id="enc_a" calibration_status="provisional"
             radius_m="0.0254" position_x_m="0" position_y_m="0"
             measurement_angle_deg="0" direction="positive"/>
    </Resource>
  </Resources>
  <Pipeline>
    <CommandCollection type="noop"/>
    <Localization><Estimator type="noop"/></Localization>
    <FieldEstimation type="noop"/>
    <TargetResolution type="noop"/>
    <Publishing type="noop"/>
  </Pipeline>
</System>)";
    EXPECT_NE(System::buildFromString(provisional, functions, err), nullptr)
        << err;
}

TEST(Profiles, CheckedInTreeStaysHonest) {
    const std::string config_dir = NAVIGATR_CONFIG_DIR;

    // the diagnostic profiles are templates on purpose: they reference the
    // measured robot description that does not exist yet, and they are
    // .xml.in so the loader refuses them outright
    ResolvedConfiguration resolved;
    std::string           err;
    EXPECT_FALSE(resolveConfiguration(
        config_dir + "/override/diagnostics/three_wheel_imu.xml.in", resolved, err));
    EXPECT_NE(err.find("template"), std::string::npos);

    // the shared pipeline fragments are valid Pipeline documents
    for (const char* name :
         {"/shared/pipelines/two_wheel_imu_no_correction.xml",
          "/shared/pipelines/three_wheel_imu_no_correction.xml",
          "/shared/pipelines/two_wheel_imu_camera_diagnostic.xml",
          "/shared/pipelines/three_wheel_imu_camera_diagnostic.xml"}) {
        tinyxml2::XMLDocument doc;
        ASSERT_EQ(doc.LoadFile((config_dir + name).c_str()), tinyxml2::XML_SUCCESS)
            << name;
        ASSERT_NE(doc.RootElement(), nullptr);
        EXPECT_STREQ(doc.RootElement()->Name(), "Pipeline") << name;
    }

    // the field map resolves to exactly nine landmarks and 36 tag mounts
    tinyxml2::XMLDocument field_doc;
    ASSERT_EQ(field_doc.LoadFile((config_dir + "/override/field.xml").c_str()),
              tinyxml2::XML_SUCCESS);
    FieldMap map;
    ASSERT_TRUE(parseFieldMap(ConfigNode{field_doc.RootElement()}, map, err))
        << err;
    EXPECT_EQ(map.landmarks.size(), 9u);
    std::size_t mounts = 0;
    for (const auto& lm : map.landmarks) {
        mounts += lm.mounts.size();
    }
    EXPECT_EQ(mounts, 36u);
}
