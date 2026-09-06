// profiles_gtest.cpp
// Multi-file configuration resolution and the robot-description ownership
// of wheel geometry: profiles compose from fragments with strict roots,
// relative paths, duplicate and template rejection, stable identity, and
// the checked-in configuration tree stays honest.

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
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
      <Source resource_id="pico_telemetry" channel="0"/>
      <Calibration counts_per_revolution="4000"/>
    </Sensor>
    <Sensor id="enc_b" type="pico_encoder_channel">
      <Source resource_id="pico_telemetry" channel="1"/>
      <Calibration counts_per_revolution="4000"/>
    </Sensor>
    <Sensor id="imu" type="pico_imu_channel">
      <Source resource_id="pico_telemetry" channel="imu"/>
    </Sensor>
  </Sensors>
</Robot>
)";

const char* kPipelineFragment = R"(
<Pipeline>
  <CommandCollection type="noop"/>
  <Preprocessing type="configured_collection">
    <Preprocessor id="motion" type="tracking_wheel_odometry">
      <Wheels resource_id="wheel_geometry">
        <Use wheel_id="forward_wheel"/>
        <Use wheel_id="lateral_wheel"/>
      </Wheels>
      <HeadingConstraint sensor_id="imu" bias_samples="0"/>
      <Output artifact_id="motion_delta"/>
    </Preprocessor>
  </Preprocessing>
  <Localization type="wheel_imu_prediction">
    <Motion artifact_id="motion_delta"/>
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
        dir = std::filesystem::temp_directory_path() / "navigatr_profiles_gtest";
        std::filesystem::remove_all(dir);
        std::filesystem::create_directories(dir);
        write("robot.xml", kRobotFragment);
        write("pipeline.xml", kPipelineFragment);
        write("field.xml", kFieldFragment);
        write("profile.xml", kProfile);
    }
    ~Workspace() { std::filesystem::remove_all(dir); }

    void write(const char* name, const std::string& content) {
        std::ofstream f(dir / name, std::ios::binary);
        f << content;
    }

    std::string path(const char* name) const { return (dir / name).string(); }
};

} // namespace

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
    </Resource>
    <Resource id="wheel_geometry" type="wheel_geometry">
      <Wheel id="forward_wheel" sensor_id="enc_a" calibration_status="verified"
             radius_m="0.0254" position_x_m="0" position_y_m="0"
             measurement_angle_deg="0" direction="positive"/>
    </Resource>
  </Resources>
  <Sensors>
    <Sensor id="enc_a" type="pico_encoder_channel">
      <Source resource_id="pico_telemetry" channel="0"/>
      <Calibration counts_per_revolution="4000"/>
    </Sensor>
  </Sensors>
  <Pipeline>
    <CommandCollection type="noop"/>
    <Preprocessing type="configured_collection">
      <Preprocessor id="motion" type="tracking_wheel_odometry">
        <Wheels resource_id="wheel_geometry">
          <Use wheel_id="phantom_wheel"/>
        </Wheels>
        <Output artifact_id="motion_delta"/>
      </Preprocessor>
    </Preprocessing>
    <Localization type="noop"/>
    <FieldEstimation type="noop"/>
    <TargetResolution type="noop"/>
    <Publishing type="noop"/>
  </Pipeline>
</System>)";
    EXPECT_EQ(System::buildFromString(bad_use, functions, err), nullptr);
    EXPECT_NE(err.find("phantom_wheel"), std::string::npos);

    // an unmeasured wheel cannot exist without the bench escape hatch
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
    <Preprocessing type="noop"/>
    <Localization type="noop"/>
    <FieldEstimation type="noop"/>
    <TargetResolution type="noop"/>
    <Publishing type="noop"/>
  </Pipeline>
</System>)";
    EXPECT_EQ(System::buildFromString(provisional, functions, err), nullptr);
    EXPECT_NE(err.find("provisional"), std::string::npos);

    BuildOptions bench;
    bench.allow_provisional = true;
    EXPECT_NE(System::buildFromString(provisional, functions, err, bench), nullptr)
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
        config_dir + "/override/diagnostics/three_wheel_bno08x.xml.in", resolved, err));
    EXPECT_NE(err.find("template"), std::string::npos);

    // the shared pipeline fragments are valid Pipeline documents
    for (const char* name :
         {"/shared/pipelines/two_wheel_bno08x_no_correction.xml",
          "/shared/pipelines/three_wheel_bno08x_no_correction.xml",
          "/shared/pipelines/two_wheel_bno08x_camera_diagnostic.xml",
          "/shared/pipelines/three_wheel_bno08x_camera_diagnostic.xml"}) {
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
    ASSERT_TRUE(parseFieldMap(ConfigNode{field_doc.RootElement()}, true, map, err))
        << err;
    EXPECT_EQ(map.landmarks.size(), 9u);
    std::size_t mounts = 0;
    for (const auto& lm : map.landmarks) {
        mounts += lm.mounts.size();
    }
    EXPECT_EQ(mounts, 36u);
}
