// geometry_resources_gtest.cpp
// The measured-geometry resources and their strictness: robot frame chains,
// annotation-only calibration labels, camera configuration validation,
// and target set validation. Required numeric values must remain usable
// regardless of a human-readable annotation.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "config/field_map.h"
#include "impl/resources/cameras.h"
#include "math/angles.h"
#include "resources/camera.h"
#include "resources/robot_frames.h"
#include "resources/target_set.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

struct Fixture {
    FunctionRegistry         functions;
    std::vector<std::string> warnings;

    Fixture() { register_resources(functions); }

    // Builds the resources of one <Resources> document; returns the map or
    // nothing with err set.
    bool build(const char* xml, ResourceStore& out, std::string& err) {
        doc_.Clear();
        if (doc_.Parse(xml) != tinyxml2::XML_SUCCESS) {
            err = "parse";
            return false;
        }
        ResourceStoreBuilder builder(functions, &warnings);
        bool ok = true;
        ConfigNode{doc_.RootElement()}.forEach("Resource", [&](const ConfigNode& r) {
            if (ok) {
                ok = builder.index(r, err);
            }
        });
        if (!ok || !builder.buildAll(err)) {
            return false;
        }
        out = builder.take();
        return true;
    }

private:
    tinyxml2::XMLDocument doc_;
};

const char* frameXml(const char* status) {
    static std::string xml;
    xml = std::string(R"(
<Resources>
    <Resource id="robot_geometry" type="robot_frame_map">
        <Frame id="front_contact" parent_frame_id="robot_body"
               calibration_status=")") +
          status + R"(">
            <PoseOfChildInParent x_m="0.35" y_m="0" z_m="0.1"
                roll_deg="0" pitch_deg="0" yaw_deg="0"/>
        </Frame>
    </Resource>
</Resources>)";
    return xml.c_str();
}

} // namespace

TEST(RobotFrameMap, ResolvesChainsToRobotBody) {
    Fixture       f;
    ResourceStore map;
    std::string   err;
    ASSERT_TRUE(f.build(R"(
<Resources>
    <Resource id="robot_geometry" type="robot_frame_map">
        <Frame id="mast" parent_frame_id="robot_body" calibration_status="verified">
            <PoseOfChildInParent x_m="0.1" y_m="0" z_m="0.5"
                roll_deg="0" pitch_deg="0" yaw_deg="90"/>
        </Frame>
        <Frame id="mast_camera" parent_frame_id="mast" calibration_status="verified">
            <PoseOfChildInParent x_m="0.2" y_m="0" z_m="0"
                roll_deg="0" pitch_deg="0" yaw_deg="0"/>
        </Frame>
    </Resource>
</Resources>)",
                        map, err))
        << err;

    auto frames = map.require<const RobotFrameMap>(ResourceId{"robot_geometry"}, err);
    ASSERT_NE(frames, nullptr) << err;
    // declaration order independent chain: mast yaw 90 turns the camera
    // offset onto robot +y
    const Transform3* cam = frames->find(FrameId{"mast_camera"});
    ASSERT_NE(cam, nullptr);
    EXPECT_NEAR(cam->x_m, 0.1, 1e-12);
    EXPECT_NEAR(cam->y_m, 0.2, 1e-12);
    EXPECT_NEAR(cam->z_m, 0.5, 1e-12);
    EXPECT_NE(frames->find(robotBodyFrameId()), nullptr);   // identity root
}

TEST(RobotFrameMap, MissingParentIsAnError) {
    Fixture       f;
    ResourceStore map;
    std::string   err;
    EXPECT_FALSE(f.build(R"(
<Resources>
    <Resource id="robot_geometry" type="robot_frame_map">
        <Frame id="orphan" parent_frame_id="ghost" calibration_status="verified">
            <PoseOfChildInParent x_m="0" y_m="0" z_m="0"
                roll_deg="0" pitch_deg="0" yaw_deg="0"/>
        </Frame>
    </Resource>
</Resources>)",
                         map, err));
    EXPECT_NE(err.find("ghost"), std::string::npos);
}

TEST(CalibrationAnnotations, LabelsDoNotAffectGeometry) {
    Fixture       f;
    ResourceStore map;
    std::string   err;
    for (const char* label : {"UNCONFIGURED", "provisional", "verified", "finished", ""}) {
        ASSERT_TRUE(f.build(frameXml(label), map, err)) << label << ": " << err;
        auto frames = map.require<const RobotFrameMap>(ResourceId{"robot_geometry"}, err);
        ASSERT_NE(frames, nullptr);
        const auto* contact = frames->find(FrameId{"front_contact"});
        ASSERT_NE(contact, nullptr);
        EXPECT_DOUBLE_EQ(contact->x_m, 0.35);
        EXPECT_DOUBLE_EQ(contact->z_m, 0.1);
    }
}

TEST(CalibrationAnnotations, LabelCannotMakeInvalidNumericGeometryUsable) {
    Fixture       f;
    ResourceStore map;
    std::string   err;
    for (const char* label : {"UNCONFIGURED", "provisional", "verified"}) {
        std::string xml = frameXml(label);
        xml.replace(xml.find("x_m=\"0.35\""), 10, "x_m=\"@MEASURE_X@\"");
        EXPECT_FALSE(f.build(xml.c_str(), map, err));
        EXPECT_NE(err.find("x_m"), std::string::npos);
    }
}

TEST(CalibrationAnnotations, MissingLabelIsAllowed) {
    Fixture       f;
    ResourceStore map;
    std::string   err;
    EXPECT_TRUE(f.build(R"(
<Resources>
    <Resource id="robot_geometry" type="robot_frame_map">
        <Frame id="front_contact" parent_frame_id="robot_body">
            <PoseOfChildInParent x_m="0.35" y_m="0" z_m="0.1"
                roll_deg="0" pitch_deg="0" yaw_deg="0"/>
        </Frame>
    </Resource>
</Resources>)",
                         map, err)) << err;
}

TEST(FieldMapStrictness, MissingGeometryAttributesAreErrors) {
    Fixture       f;
    ResourceStore map;
    std::string   err;
    // z_m deliberately missing from the tag surface pose
    EXPECT_FALSE(f.build(R"(
<Resources>
    <Resource id="game_field" type="field_map">
        <Landmark id="goal">
            <NominalPose calibration_status="verified"
                         x_m="1" y_m="1" heading_deg="0"/>
            <TagMount instance_id="t" calibration_status="verified"
                      family="tag36h11" observed_id="3" detection_size_m="0.06">
                <PoseOfTagSurfaceInLandmark x_m="0.1" y_m="0"
                    roll_deg="0" pitch_deg="0" yaw_deg="0"/>
            </TagMount>
        </Landmark>
    </Resource>
</Resources>)",
                         map, err));
    EXPECT_NE(err.find("missing required attribute z_m"), std::string::npos);
}

TEST(CameraResource, CalibrationResolutionMustMatchCapture) {
    Fixture           f;
    ResourceStore     map;
    std::string       err;
    const std::string base = R"(
<Resources>
    <Resource id="cam" type="libcamera_camera">
        <Device index="0"/>
        <Capture width_px="1456" height_px="1088" pixel_format="Y8"
                 frame_rate_hz="30"/>
        <Calibration calibration_status="verified" calibration_id="cam_a">
            <Intrinsics model="brown_conrady"
                calibrated_width_px="WIDTH" calibrated_height_px="1088"
                fx_px="1100" fy_px="1100" cx_px="728" cy_px="544"
                k1="0" k2="0" p1="0" p2="0" k3="0"
                rms_reprojection_px="0.5"/>
            <Extrinsic frame_id="front_camera_engineering"/>
        </Calibration>
    </Resource>
</Resources>)";

    std::string mismatched = base;
    mismatched.replace(mismatched.find("WIDTH"), 5, "640");
    EXPECT_FALSE(f.build(mismatched.c_str(), map, err));
    EXPECT_NE(err.find("does not match capture resolution"), std::string::npos);

    std::string matched = base;
    matched.replace(matched.find("WIDTH"), 5, "1456");
#if NAVIGATR_HAVE_LIBCAMERA
    // with the backend compiled in, an absent device is a warning and a
    // dead camera, never a silent absence
    ASSERT_TRUE(f.build(matched.c_str(), map, err)) << err;
    auto camera = map.require<CameraDevice>(ResourceId{"cam"}, err);
    ASSERT_NE(camera, nullptr) << err;
#else
    // this binary has no libcamera backend: selecting the type is an
    // explicit configuration error naming the missing feature, not a dead
    // device that quietly produces nothing
    EXPECT_FALSE(f.build(matched.c_str(), map, err));
    EXPECT_NE(err.find("NAVIGATR_WITH_LIBCAMERA"), std::string::npos) << err;
#endif
}

TEST(CameraResource, LabelsAreOptionalAndNumericCalibrationControlsMetricAvailability) {
    const std::string base = R"(
        <Resource><Device index="0"/>
            <Capture width_px="640" height_px="480" pixel_format="Y8" frame_rate_hz="30"/>
            <Calibration LABEL calibration_id="test">
                <Intrinsics model="brown_conrady" calibrated_width_px="640" calibrated_height_px="480"
                    fx_px="500" fy_px="500" cx_px="320" cy_px="240"
                    k1="0" k2="0" p1="0" p2="0" k3="0" rms_reprojection_px="0.5"/>
                <Extrinsic frame_id="camera"/>
            </Calibration>
        </Resource>)";
    auto parse = [](const std::string& xml, CameraCaptureConfig& config, std::string& err) {
        tinyxml2::XMLDocument doc;
        if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
            return false;
        }
        return parseCameraConfig(ConfigNode{doc.RootElement()}, config, err);
    };
    for (const char* label : {"", "calibration_status=\"UNCONFIGURED\"",
                              "calibration_status=\"provisional\"", "calibration_status=\"verified\"",
                              "calibration_status=\"notes for the operator\""}) {
        std::string xml = base;
        xml.replace(xml.find("LABEL"), 5, label);
        CameraCaptureConfig config;
        std::string err;
        ASSERT_TRUE(parse(xml, config, err)) << err;
        EXPECT_TRUE(config.calibrated);
        EXPECT_DOUBLE_EQ(config.intrinsics.fx_px, 500.0);

        xml.replace(xml.find("fx_px=\"500\""), 11, "fx_px=\"0\"");
        EXPECT_FALSE(parse(xml, config, err));
        EXPECT_NE(err.find("fx_px"), std::string::npos);
    }
    for (const char* bad : {"-1", "nan", "@MEASURE_RATE@", "1e-100", "2000000"}) {
        std::string xml = R"(<Resource><Device index="0"/>
            <Capture width_px="640" height_px="480" pixel_format="Y8" frame_rate_hz="RATE"/>
        </Resource>)";
        xml.replace(xml.find("RATE"), 4, bad);
        CameraCaptureConfig config;
        std::string err;
        EXPECT_FALSE(parse(xml, config, err));
        EXPECT_NE(err.find("frame_rate_hz"), std::string::npos);
    }
}

TEST(TargetSetValidation, RejectsBrokenDeclarations) {
    Fixture           f;
    ResourceStore     map;
    std::string       err;
    const std::string prelude = R"(
<Resources>
    <Resource id="robot_geometry" type="robot_frame_map">
        <Frame id="rear_contact" parent_frame_id="robot_body"
               calibration_status="verified">
            <PoseOfChildInParent x_m="-0.4" y_m="0" z_m="0"
                roll_deg="0" pitch_deg="0" yaw_deg="180"/>
        </Frame>
    </Resource>
    <Resource id="game_field" type="field_map">
        <Landmark id="goal">
            <NominalPose calibration_status="verified"
                         x_m="1" y_m="1" heading_deg="0"/>
            <ApproachFrame id="face" calibration_status="verified">
                <PoseOfApproachFrameInLandmark x_m="0.2" y_m="0" z_m="0"
                    roll_deg="0" pitch_deg="0" yaw_deg="0"/>
            </ApproachFrame>
            <TagMount instance_id="goal_tag" calibration_status="verified"
                      family="tag36h11" observed_id="3" detection_size_m="0.06">
                <PoseOfTagSurfaceInLandmark x_m="0.2" y_m="0" z_m="0.2"
                    roll_deg="0" pitch_deg="0" yaw_deg="0"/>
            </TagMount>
        </Landmark>
    </Resource>
    <Resource id="targets" type="target_set">
        <FieldMap resource_id="game_field"/>
        <RobotFrames resource_id="robot_geometry"/>
)";
    const std::string epilogue = R"(
    </Resource>
</Resources>)";

    const auto expectError = [&](const char* target_xml, const char* needle) {
        const std::string xml = prelude + target_xml + epilogue;
        EXPECT_FALSE(f.build(xml.c_str(), map, err)) << needle;
        EXPECT_NE(err.find(needle), std::string::npos) << err;
    };

    // continuous is explicitly future work
    expectError(R"(
        <Target id="a" type="landmark_relative" wire_id="1" landmark_id="goal"
                approach_frame_id="face" controlled_frame_id="rear_contact">
            <DesiredControlledFramePose calibration_status="verified"
                x_m="0.1" y_m="0" heading_deg="180"/>
            <VisionCorrection type="continuous"/>
        </Target>)",
                "continuous");

    // unknown landmark
    expectError(R"(
        <Target id="a" type="landmark_relative" wire_id="1" landmark_id="ghost"
                approach_frame_id="face" controlled_frame_id="rear_contact">
            <DesiredControlledFramePose calibration_status="verified"
                x_m="0.1" y_m="0" heading_deg="180"/>
            <VisionCorrection type="none"/>
        </Target>)",
                "ghost");

    // unknown approach frame
    expectError(R"(
        <Target id="a" type="landmark_relative" wire_id="1" landmark_id="goal"
                approach_frame_id="north_side" controlled_frame_id="rear_contact">
            <DesiredControlledFramePose calibration_status="verified"
                x_m="0.1" y_m="0" heading_deg="180"/>
            <VisionCorrection type="none"/>
        </Target>)",
                "north_side");

    // unknown controlled frame
    expectError(R"(
        <Target id="a" type="landmark_relative" wire_id="1" landmark_id="goal"
                approach_frame_id="face" controlled_frame_id="claw">
            <DesiredControlledFramePose calibration_status="verified"
                x_m="0.1" y_m="0" heading_deg="180"/>
            <VisionCorrection type="none"/>
        </Target>)",
                "claw");

    // a robot-relative target must not claim visual correction
    expectError(R"(
        <Target id="a" type="robot_relative" wire_id="1"
                controlled_frame_id="robot_body">
            <Snapshot frame_id="robot_body" delta_axes="robot_at_activation"/>
            <Delta x_m="0.5" y_m="0" heading_deg="0"/>
            <VisionCorrection type="acquire_once"
                on_acquisition_timeout="cancel"
                minimum_consistent_observations="1"
                maximum_observation_age_ms="100"
                maximum_robot_angular_speed_deg_s="60"
                acquisition_timeout_ms="500"
                consistency_translation_m="0.05"
                consistency_heading_deg="3"/>
        </Target>)",
                "no visual");

    // duplicate wire ids
    expectError(R"(
        <Target id="a" type="robot_relative" wire_id="1"
                controlled_frame_id="robot_body">
            <Snapshot frame_id="robot_body" delta_axes="robot_at_activation"/>
            <Delta x_m="0.5" y_m="0" heading_deg="0"/>
            <VisionCorrection type="none"/>
        </Target>
        <Target id="b" type="robot_relative" wire_id="1"
                controlled_frame_id="robot_body">
            <Snapshot frame_id="robot_body" delta_axes="robot_at_activation"/>
            <Delta x_m="0.5" y_m="0" heading_deg="0"/>
            <VisionCorrection type="none"/>
        </Target>)",
                "duplicate Target wire_id");

    // an allowed mount that belongs to no landmark in the map
    expectError(R"(
        <Target id="a" type="landmark_relative" wire_id="1" landmark_id="goal"
                approach_frame_id="face" controlled_frame_id="rear_contact">
            <DesiredControlledFramePose calibration_status="verified"
                x_m="0.1" y_m="0" heading_deg="180"/>
            <VisionCorrection type="acquire_once"
                on_acquisition_timeout="cancel"
                minimum_consistent_observations="1"
                maximum_observation_age_ms="100"
                maximum_robot_angular_speed_deg_s="60"
                acquisition_timeout_ms="500"
                consistency_translation_m="0.05"
                consistency_heading_deg="3">
                <AllowedTagMount instance_id="phantom_tag"/>
            </VisionCorrection>
        </Target>)",
                "phantom_tag");

    // a correct declaration builds and resolves its transforms
    const std::string good = prelude + R"(
        <Target id="a" type="landmark_relative" wire_id="1" landmark_id="goal"
                approach_frame_id="face" controlled_frame_id="rear_contact">
            <DesiredControlledFramePose calibration_status="verified"
                x_m="0.1" y_m="0" heading_deg="180"/>
            <VisionCorrection type="none"/>
        </Target>)" + epilogue;
    ASSERT_TRUE(f.build(good.c_str(), map, err)) << err;
    auto set = map.require<const TargetSet>(ResourceId{"targets"}, err);
    ASSERT_NE(set, nullptr) << err;
    ASSERT_EQ(set->targets.size(), 1u);
    EXPECT_NEAR(set->targets[0].T_robot_controlled.x_m, -0.4, 1e-12);
}
