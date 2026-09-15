// inspection_document_gtest.cpp
// The inspection documents are well-formed JSON built only from published
// runtime snapshots: the writer escapes and never emits NaN, hello carries
// the configured field and cameras, and a snapshot taken after real frames
// were processed binds detections to the frame identity they came from.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>

#include "core/host_clock.h"
#include "inspection/inspection_document.h"
#include "inspection/json_writer.h"
#include "runtime/register_all.h"
#include "runtime/system.h"

using namespace navigatr;

namespace
{

// Minimal recursive-descent JSON syntax check: enough to prove a document
// parses, deliberately not a JSON library.
struct JsonCheck {
    const std::string& s;
    std::size_t        i = 0;
    bool               ok = true;

    void ws() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) {
            ++i;
        }
    }
    bool lit(const char* word) {
        const std::size_t n = std::string(word).size();
        if (s.compare(i, n, word) == 0) {
            i += n;
            return true;
        }
        return false;
    }
    void string() {
        if (i >= s.size() || s[i] != '"') {
            ok = false;
            return;
        }
        ++i;
        while (i < s.size() && s[i] != '"') {
            if (s[i] == '\\') {
                ++i;
                if (i < s.size() && s[i] == 'u') {
                    i += 4;
                }
            }
            ++i;
        }
        if (i >= s.size()) {
            ok = false;
            return;
        }
        ++i;
    }
    void number() {
        const std::size_t start = i;
        if (i < s.size() && s[i] == '-') {
            ++i;
        }
        while (i < s.size() && ((s[i] >= '0' && s[i] <= '9') || s[i] == '.' || s[i] == 'e' ||
                                s[i] == 'E' || s[i] == '+' || s[i] == '-')) {
            ++i;
        }
        if (i == start) {
            ok = false;
        }
    }
    void value() {
        ws();
        if (!ok || i >= s.size()) {
            ok = false;
            return;
        }
        if (s[i] == '{') {
            ++i;
            ws();
            if (i < s.size() && s[i] == '}') {
                ++i;
                return;
            }
            while (ok) {
                ws();
                string();
                ws();
                if (i >= s.size() || s[i] != ':') {
                    ok = false;
                    return;
                }
                ++i;
                value();
                ws();
                if (i < s.size() && s[i] == ',') {
                    ++i;
                    continue;
                }
                if (i < s.size() && s[i] == '}') {
                    ++i;
                    return;
                }
                ok = false;
                return;
            }
        } else if (s[i] == '[') {
            ++i;
            ws();
            if (i < s.size() && s[i] == ']') {
                ++i;
                return;
            }
            while (ok) {
                value();
                ws();
                if (i < s.size() && s[i] == ',') {
                    ++i;
                    continue;
                }
                if (i < s.size() && s[i] == ']') {
                    ++i;
                    return;
                }
                ok = false;
                return;
            }
        } else if (s[i] == '"') {
            string();
        } else if (lit("true") || lit("false") || lit("null")) {
            return;
        } else {
            number();
        }
    }
    bool run() {
        value();
        ws();
        return ok && i == s.size();
    }
};

bool validJson(const std::string& s) {
    JsonCheck check{s};
    return check.run();
}

// A small system with one camera, driven by the synthetic rig so real frames
// and detections exist without hardware.
const char* kRig = R"(
<System>
    <Loop rate_hz="100"/>
    <Inspection enabled="true" port="0"/>
    <Resources>
        <Resource id="robot_geometry" type="robot_frame_map">
            <Frame id="cam" parent_frame_id="robot_body" calibration_status="verified">
                <PoseOfChildInParent x_m="0.15" y_m="0" z_m="0.20"
                    roll_deg="0" pitch_deg="0" yaw_deg="0"/>
            </Frame>
        </Resource>
        <Resource id="field" type="field_map" name="doc test field">
            <Dimensions inside_x_m="3.5664" inside_y_m="3.5664" wall_height_m="0.293"
                        wall_thickness_m="0.0508" tile_m="0.5944"
                        source="test" revision="none"/>
            <Feature id="marker" kind="box" x_m="0.5" y_m="0.5" z_m="0.05"
                     size_x_m="0.1" size_y_m="0.1" size_z_m="0.1" color="#336699"/>
            <Landmark id="goal">
                <NominalPose calibration_status="verified" x_m="1.7832" y_m="1.7832" heading_deg="0"/>
                <Visual shape="octagonal_prism" height_m="0.2227" base_across_flats_m="0.1425"
                        top_across_flats_m="0.0888" color="#ffcc00"/>
                <TagMount instance_id="goal_west" calibration_status="verified"
                          family="tagCircle21h7" observed_id="0" detection_size_m="0.03">
                    <PoseOfTagSurfaceInLandmark x_m="-0.05" y_m="0" z_m="0.20"
                        roll_deg="0" pitch_deg="0" yaw_deg="180"/>
                </TagMount>
            </Landmark>
        </Resource>
        <Resource id="wheel_geometry" type="wheel_geometry">
            <Wheel id="left_wheel" sensor_id="enc_a" calibration_status="verified"
                   radius_m="0.0254" position_x_m="0" position_y_m="0.13"
                   measurement_angle_deg="0" direction="positive"/>
            <Wheel id="right_wheel" sensor_id="enc_b" calibration_status="verified"
                   radius_m="0.0254" position_x_m="0" position_y_m="-0.13"
                   measurement_angle_deg="0" direction="positive"/>
            <Wheel id="rear_wheel" sensor_id="enc_c" calibration_status="verified"
                   radius_m="0.0254" position_x_m="-0.12" position_y_m="0"
                   measurement_angle_deg="90" direction="positive"/>
        </Resource>
        <Resource id="rig" type="synthetic_rig">
            <Field resource_id="field"/>
            <Wheels resource_id="wheel_geometry" counts_per_revolution="4000"/>
            <Trajectory center_x_m="1.7832" center_y_m="1.7832" radius_m="0.6"
                        period_s="30" facing="center" start_deg="180" hold_s="0.5"/>
            <Telemetry tick_hz="50" device_offset_ms="5000" gyro_bias_mdps="0"/>
            <Attitude mode="measured" rock_deg="2" period_s="2.5"/>
            <Camera frame_id="cam" robot_frames_resource_id="robot_geometry"
                    width_px="640" height_px="480" fx_px="600" fy_px="600" cx_px="320" cy_px="240"
                    frame_rate_hz="10" latency_ms="30"/>
            <Output id="encoder_a" wheel_id="left_wheel"/>
            <Output id="encoder_b" wheel_id="right_wheel"/>
            <Output id="encoder_c" wheel_id="rear_wheel"/>
            <Output id="imu" channel="imu"/>
            <Output id="attitude" channel="attitude"/>
            <Output id="frame" channel="camera"/>
        </Resource>
        <Resource id="tag_detector" type="apriltag_detector">
            <Family name="tagCircle21h7" detection_size_m="0.03"/>
            <Detector quad_decimate="1.0" nthreads="2"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_b" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_c" type="pico_encoder_channel">
            <Source resource_id="rig" output_id="encoder_c"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="robot_imu" type="pico_imu_channel">
            <Source resource_id="rig" output_id="imu"/>
        </Sensor>
        <Sensor id="robot_attitude" type="attitude_channel">
            <Source resource_id="rig" output_id="attitude"/>
        </Sensor>
        <Sensor id="front_camera" type="camera_frame">
            <Source resource_id="rig" output_id="frame"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">
                <Wheels resource_id="wheel_geometry">
                    <Use wheel_id="left_wheel"/>
                    <Use wheel_id="right_wheel"/>
                    <Use wheel_id="rear_wheel"/>
                </Wheels>
                <HeadingConstraint sensor_id="robot_imu" bias_samples="10"
                                   max_calibration_travel_m="0.005"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Observation id="attitude" type="attitude_reference">
                <Input sensor_id="robot_attitude"/>
                <Freshness max_age_ms="100"/>
                <Output observation_id="attitude"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
                <Attitude observation_id="attitude" max_age_ms="200"/>
            </Estimator>
            <History retention_s="5" capacity="1024" max_interpolation_gap_ms="100"/>
            <InitialPlacement x_m="1.1832" y_m="1.7832" heading_deg="0"/>
        </Localization>
        <FieldEstimation type="landmark_field">
            <FieldMap resource_id="field"/>
            <Pipeline>
                <ObservationExtraction type="apriltag_tag_observation">
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="tag_detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <Association type="tag_mount_association">
                    <Observations observation_id="tag_observations"/>
                    <FieldMap resource_id="field"/>
                    <RobotFrames resource_id="robot_geometry"/>
                    <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
                           ambiguity_margin_m="0.15" max_range_m="3.0"
                           min_decision_margin="10" max_hamming="0"/>
                    <Output association_id="landmark_pose_observations"/>
                    <Trace association_id="tag_association_trace"/>
                </Association>
                <Estimator type="landmark_estimator" commit="always" blend="0.5"/>
            </Pipeline>
        </FieldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";

} // namespace

TEST(JsonWriter, EscapesAndNeverEmitsNonFiniteNumbers) {
    JsonWriter w;
    w.beginObject();
    w.field("text", "quote \" backslash \\ newline \n tab \t control \x01");
    w.field("nan", std::numeric_limits<double>::quiet_NaN());
    w.field("inf", std::numeric_limits<double>::infinity());
    w.field("num", 1.5);
    w.field("neg", static_cast<int64_t>(-7));
    w.field("big", static_cast<uint64_t>(18446744073709551615ULL));
    w.field("yes", true);
    w.fieldNull("none");
    w.key("list");
    w.beginArray();
    w.value(1);
    w.value("two");
    w.beginObject();
    w.endObject();
    w.beginArray();
    w.endArray();
    w.endArray();
    w.endObject();
    const std::string out = w.str();
    EXPECT_TRUE(validJson(out)) << out;
    EXPECT_NE(out.find("\"text\":\"quote \\\" backslash \\\\ newline \\n tab \\t control \\u0001\""),
              std::string::npos)
        << out;
    EXPECT_NE(out.find("\"nan\":null"), std::string::npos);
    EXPECT_NE(out.find("\"inf\":null"), std::string::npos);
    EXPECT_NE(out.find("\"big\":18446744073709551615"), std::string::npos);
    EXPECT_NE(out.find("\"list\":[1,\"two\",{},[]]"), std::string::npos) << out;
    EXPECT_EQ(out.find(",,"), std::string::npos);
}

TEST(InspectionDocuments, HelloCarriesConfigurationFieldAndCameras) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string             err;
    std::unique_ptr<System> system = System::buildFromString(kRig, functions, err);
    ASSERT_NE(system, nullptr) << err;

    const std::string hello = helloDocument(*system, hostTime(10));
    ASSERT_TRUE(validJson(hello)) << hello;
    EXPECT_NE(hello.find("\"contract\":\"navigatr.inspect/1\""), std::string::npos);
    EXPECT_NE(hello.find("\"id\":\"" + system->sessionId() + "\""), std::string::npos);
    EXPECT_NE(hello.find("\"name\":\"doc test field\""), std::string::npos);
    EXPECT_NE(hello.find("\"inside_x_m\":3.5664"), std::string::npos);
    EXPECT_NE(hello.find("\"id\":\"marker\""), std::string::npos);
    EXPECT_NE(hello.find("\"shape\":\"octagonal_prism\""), std::string::npos);
    EXPECT_NE(hello.find("\"instance_id\":\"goal_west\""), std::string::npos);
    EXPECT_NE(hello.find("\"tagCircle21h7\":{\"width_at_border\":5,\"total_width\":9}"),
              std::string::npos)
        << hello;
    EXPECT_NE(hello.find("\"camera_sensors\":[\"front_camera\"]"), std::string::npos);
    EXPECT_NE(hello.find("\"estimator_type\":\"planar_motion_integrator\""),
              std::string::npos);
    EXPECT_NE(hello.find("\"retention_ms\":5000"), std::string::npos);
    // the rig is not a CameraDevice resource: no device entry, but the
    // frame sensor is listed and every snapshot names the frame identity
    EXPECT_NE(hello.find("\"cameras\":[]"), std::string::npos);
}

TEST(InspectionDocuments, SnapshotBindsDetectionsToTheirFrameIdentity) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string             err;
    std::unique_ptr<System> system = System::buildFromString(kRig, functions, err);
    ASSERT_NE(system, nullptr) << err;

    InspectionServiceStats service;
    // before any cycle: still a valid document with empty collections
    std::string snap = snapshotDocument(*system, service, hostTime(1));
    ASSERT_TRUE(validJson(snap)) << snap;
    EXPECT_NE(snap.find("\"detection_frames\":[]"), std::string::npos);

    // drive the rig long enough for frames to be rendered and detected
    int64_t now = 1;
    for (int i = 0; i < 180; ++i) {
        now += 10;
        system->step(hostTime(now));
    }
    const auto frames = system->detectionFrames();
    ASSERT_EQ(frames.size(), 1u);
    const DetectionFrameSnapshot& f = *frames.begin()->second;
    ASSERT_TRUE(f.has_observations);
    ASSERT_FALSE(f.observations.tags.empty());
    EXPECT_EQ(f.frame_sequence, f.observations.frame_sequence);
    EXPECT_EQ(f.frame_epoch, f.observations.frame_epoch);
    ASSERT_NE(f.y8, nullptr);
    EXPECT_EQ(f.y8->size(), static_cast<std::size_t>(f.width_px) * f.height_px);
    EXPECT_TRUE(f.has_trace);
    EXPECT_EQ(f.trace.frame_sequence, f.frame_sequence);

    snap = snapshotDocument(*system, service, hostTime(now));
    ASSERT_TRUE(validJson(snap)) << snap;
    EXPECT_NE(snap.find("\"camera\":\"front_camera\""), std::string::npos);
    EXPECT_NE(snap.find("\"sequence\":" + std::to_string(f.frame_sequence)),
              std::string::npos);
    EXPECT_NE(snap.find("\"family\":\"tagCircle21h7\""), std::string::npos);
    EXPECT_NE(snap.find("\"has_image\":true"), std::string::npos);
    EXPECT_NE(snap.find("\"pose_at_exposure\":{\"status\":\"ok\""), std::string::npos)
        << snap;
    EXPECT_NE(snap.find("\"association\":{\"accepted\":"), std::string::npos);
    EXPECT_NE(snap.find("\"mounted\":true"), std::string::npos);
    EXPECT_NE(snap.find("\"id\":\"goal\""), std::string::npos);
    EXPECT_NE(snap.find("\"attitude\":{\"valid\":true"), std::string::npos) << snap;
    EXPECT_NE(snap.find("\"kind\":\"sensor\",\"id\":\"front_camera\",\"state\":\"valid\""),
              std::string::npos)
        << snap;
    EXPECT_EQ(snap.find("nan"), std::string::npos);

    const std::string header =
        frameHeaderDocument(f, f.width_px / 2, f.height_px / 2, 70, 1.5, hostTime(now));
    ASSERT_TRUE(validJson(header)) << header;
    EXPECT_NE(header.find("\"type\":\"frame\""), std::string::npos);
    EXPECT_NE(header.find("\"preview_width_px\":" + std::to_string(f.width_px / 2)),
              std::string::npos);
}
