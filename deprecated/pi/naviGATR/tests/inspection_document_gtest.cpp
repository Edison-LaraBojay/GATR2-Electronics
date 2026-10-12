// inspection_document_gtest.cpp
// The inspection documents are well-formed JSON built only from published
// runtime snapshots: the writer escapes and never emits NaN, hello carries
// the configured field and cameras, and a snapshot taken after real frames
// were processed binds detections to the frame identity they came from, and a
// camera with world estimation switched off still previews its raw frames.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include "core/host_clock.h"
#include "inspection/inspection_document.h"
#include "inspection/json_writer.h"
#include "runtime/register_all.h"
#include "runtime/system.h"
#include "translaGATR/frame_codec.h"

using namespace navigatr;

// Minimal recursive-descent JSON syntax check: enough to prove a document
// parses, deliberately not a JSON library. brain_profile_gtest.cpp uses it.
namespace inspection_test
{

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

} // namespace inspection_test

using inspection_test::validJson;

namespace
{

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
                <HeadingConstraint sensor_id="robot_imu" bias_samples="10" window_ms="150"
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
        <WorldEstimation>
            <Estimator id="goals" type="apriltag">
                <FieldMap resource_id="field"/>
                <ObservationExtraction>
                    <Camera sensor_id="front_camera"/>
                    <Detector resource_id="tag_detector"/>
                    <Output observation_id="tag_observations"/>
                </ObservationExtraction>
                <Association>
                    <Observations observation_id="tag_observations"/>
                    <FieldMap resource_id="field"/>
                    <RobotFrames resource_id="robot_geometry"/>
                    <Gates max_translation_error_m="0.5" max_heading_error_deg="30"
                           ambiguity_margin_m="0.15" max_range_m="3.0"
                           min_decision_margin="10" max_hamming="0"/>
                    <Output association_id="landmark_pose_observations"/>
                    <Trace association_id="tag_association_trace"/>
                </Association>
                <LandmarkEstimation commit="always" blend="0.5"/>
            </Estimator>
        </WorldEstimation>
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

TEST(JsonWriter, IntegralTypesAndAliasesPreserveExactValues) {
    JsonWriter w;
    w.beginArray();
    w.value(static_cast<signed char>(-8));
    w.value(static_cast<unsigned char>(250));
    w.value(static_cast<short>(-32000));
    w.value(static_cast<unsigned short>(65000));
    w.value(-123);
    w.value(4294967295U);
    w.value(-123456L);
    w.value(123456UL);
    w.value(-9223372036854775807LL - 1);
    w.value(18446744073709551615ULL);
    w.value(std::numeric_limits<int64_t>::min());
    w.value(std::numeric_limits<int64_t>::max());
    w.value(std::numeric_limits<uint64_t>::max());
    w.value(std::size_t{42});
    w.value(true);
    w.value(false);
    w.value(1.5f);
    w.value(2.5);
    w.value("literal");
    w.value(std::string("string"));
    w.endArray();
    EXPECT_EQ(w.str(),
              "[-8,250,-32000,65000,-123,4294967295,-123456,123456,"
              "-9223372036854775808,18446744073709551615,"
              "-9223372036854775808,9223372036854775807,18446744073709551615,"
              "42,true,false,1.5,2.5,\"literal\",\"string\"]");
}

TEST(InspectionDocuments, HelloCarriesConfigurationFieldAndCameras) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string             err;
    std::unique_ptr<System> system = System::buildFromString(kRig, functions, err);
    ASSERT_NE(system, nullptr) << err;

    const std::string hello = helloDocument(*system, hostTime(10));
    ASSERT_TRUE(validJson(hello)) << hello;
    EXPECT_NE(hello.find("\"contract\":\"navigatr.inspect/2\""), std::string::npos);
    EXPECT_NE(hello.find("\"features\":{"), std::string::npos);
    EXPECT_NE(hello.find("\"history_max\":300"), std::string::npos);
    // no Brain link in this rig: no TELEMETRY can arrive
    EXPECT_NE(hello.find("\"telemetry\":false"), std::string::npos) << hello;
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
    // a display-only field: no planning data, and no map for a Brain
    EXPECT_NE(hello.find("\"boundary\":null"), std::string::npos);
    EXPECT_NE(hello.find("\"obstacles\":[]"), std::string::npos);
    EXPECT_NE(hello.find("\"map_id\":null,\"map_error\":\""), std::string::npos) << hello;
    EXPECT_NE(hello.find("\"id\":\"goal\",\"wire_id\":null"), std::string::npos) << hello;
    EXPECT_NE(hello.find("\"collision_box\":null"), std::string::npos);
    EXPECT_NE(hello.find("\"commands_type\":\"noop\",\"brain_profile\":false"),
              std::string::npos)
        << hello;
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
    // no Brain link and no Pico link here; the calibration window still shows
    EXPECT_NE(snap.find("\"brain_link\":null"), std::string::npos);
    EXPECT_NE(snap.find("\"pico\":null"), std::string::npos);
    EXPECT_NE(snap.find("\"events\":["), std::string::npos);
    EXPECT_NE(snap.find("\"stillness\":{\"monitored\":true"), std::string::npos) << snap;
    EXPECT_NE(snap.find("\"continuity_breaks\":0"), std::string::npos);

    const std::string header =
        frameHeaderDocument(f, f.width_px / 2, f.height_px / 2, 70, 1.5, hostTime(now));
    ASSERT_TRUE(validJson(header)) << header;
    EXPECT_NE(header.find("\"type\":\"frame\""), std::string::npos);
    EXPECT_NE(header.find("\"preview_width_px\":" + std::to_string(f.width_px / 2)),
              std::string::npos);
}

TEST(InspectionDocuments, CameraPreviewWorksWithoutWorldEstimation) {
    // the same rig and camera with world estimation switched off in config
    std::string       xml   = kRig;
    const std::string close = "</WorldEstimation>";
    const std::size_t a     = xml.find("<WorldEstimation>");
    const std::size_t b     = xml.find(close);
    ASSERT_NE(a, std::string::npos);
    ASSERT_NE(b, std::string::npos);
    xml.replace(a, b + close.size() - a,
                R"(<WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>)");

    FunctionRegistry functions;
    registerAll(functions);
    std::string             err;
    std::unique_ptr<System> system = System::buildFromString(xml.c_str(), functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->worldEstimation().estimatorType(), "noop");

    int64_t now = 1;
    for (int i = 0; i < 180; ++i) {
        now += 10;
        system->step(hostTime(now));
    }
    auto frames = system->detectionFrames();
    ASSERT_EQ(frames.size(), 1u);
    const auto first = frames.begin()->second;
    EXPECT_EQ(first->camera, SensorId{"front_camera"});
    EXPECT_FALSE(first->has_observations);   // preview only, nothing decoded
    EXPECT_FALSE(first->has_trace);
    ASSERT_NE(first->y8, nullptr);
    EXPECT_EQ(first->y8->size(), static_cast<std::size_t>(first->width_px) * first->height_px);
    EXPECT_TRUE(system->field().objects.empty());
    EXPECT_TRUE(system->robot().valid);   // localization never waits on the camera

    std::string snap = snapshotDocument(*system, InspectionServiceStats{}, hostTime(now));
    ASSERT_TRUE(validJson(snap)) << snap;
    EXPECT_NE(snap.find("\"has_observations\":false"), std::string::npos);
    EXPECT_NE(snap.find("\"has_image\":true"), std::string::npos);
    EXPECT_NE(snap.find("\"tags\":[]"), std::string::npos);
    EXPECT_NE(snap.find("\"field_objects\":[]"), std::string::npos);
    EXPECT_NE(snap.find("\"pose_at_exposure\":{\"status\":\"ok\""), std::string::npos) << snap;

    // the preview follows new frames, each under its own identity
    for (int i = 0; i < 50; ++i) {
        now += 10;
        system->step(hostTime(now));
    }
    frames = system->detectionFrames();
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_GT(frames.begin()->second->frame_sequence, first->frame_sequence);
    EXPECT_FALSE(frames.begin()->second->has_observations);
}

// ---- inspect/2 documents ------------------------------------------------------

namespace
{

// The balanced {...} starting at start.
std::string balancedObject(const std::string& json, std::size_t start) {
    if (start >= json.size() || json[start] != '{') {
        return {};
    }
    int               depth  = 0;
    bool              in_str = false;
    for (std::size_t i = start; i < json.size(); ++i) {
        const char c = json[i];
        if (in_str) {
            if (c == '\\') {
                ++i;
            } else if (c == '"') {
                in_str = false;
            }
            continue;
        }
        if (c == '"') {
            in_str = true;
        } else if (c == '{') {
            ++depth;
        } else if (c == '}' && --depth == 0) {
            return json.substr(start, i - start + 1);
        }
    }
    return {};
}

// The balanced {...} value of the first "key": at or after from.
std::string objectValue(const std::string& json, const std::string& key, std::size_t from = 0) {
    const std::string pattern = "\"" + key + "\":{";
    const std::size_t k       = json.find(pattern, from);
    if (k == std::string::npos) {
        return {};
    }
    return balancedObject(json, k + pattern.size() - 1);
}

std::unique_ptr<System> buildRig(FunctionRegistry& functions) {
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromString(kRig, functions, err);
    EXPECT_NE(system, nullptr) << err;
    return system;
}

} // namespace

TEST(InspectionDocuments, StateHistoryAndDiagSplitTheSnapshot) {
    FunctionRegistry functions;
    auto             system = buildRig(functions);
    ASSERT_NE(system, nullptr);
    int64_t now = 1;
    for (int i = 0; i < 60; ++i) {
        now += 10;
        system->step(hostTime(now));
    }
    const MonotonicTime at = hostTime(now);

    uint64_t          publication = 0;
    const std::string state       = stateDocument(*system, 7, at, now * 1000, &publication);
    ASSERT_TRUE(validJson(state)) << state;
    EXPECT_NE(state.find("\"type\":\"state\",\"seq\":7,"), std::string::npos) << state;
    EXPECT_NE(state.find("\"host_us\":" + std::to_string(now * 1000)), std::string::npos);
    EXPECT_EQ(publication, system->robotFeed()->publication());
    EXPECT_GT(publication, 0u);
    EXPECT_NE(state.find("\"publication\":" + std::to_string(publication)), std::string::npos);
    EXPECT_NE(state.find("\"localization\":{\"all_ready\":"), std::string::npos) << state;
    EXPECT_EQ(state.find("\"trail\""), std::string::npos);
    EXPECT_EQ(state.find("\"detection_frames\""), std::string::npos);

    // the state robot is exactly the snapshot robot at the same instant
    const std::string snap = snapshotDocument(*system, InspectionServiceStats{}, at);
    ASSERT_TRUE(validJson(snap));
    EXPECT_NE(snap.find("\"contract\":\"navigatr.inspect/1\""), std::string::npos);
    const std::string robot_state = objectValue(state, "robot");
    ASSERT_FALSE(robot_state.empty());
    EXPECT_EQ(robot_state, objectValue(snap, "robot"));
    // the rig measures attitude and it was just folded
    EXPECT_NE(robot_state.find("\"status\":\"measured\""), std::string::npos) << robot_state;

    const std::string history = historyDocument(*system, 3, at, 50);
    ASSERT_TRUE(validJson(history)) << history;
    EXPECT_NE(history.find("\"type\":\"history\",\"seq\":3,"), std::string::npos);
    EXPECT_NE(history.find("\"trail\":[{\"host_ms\":"), std::string::npos) << history;
    EXPECT_NE(history.find("\"max_entries\":50"), std::string::npos);
    std::size_t entries = 0;
    for (std::size_t p = history.find("{\"host_ms\":"); p != std::string::npos;
         p = history.find("{\"host_ms\":", p + 1)) {
        ++entries;
    }
    EXPECT_GT(entries, 0u);
    EXPECT_LE(entries, 50u);

    InspectionFeedStats feed;
    feed.state_hz = 30.0;
    feed.diag_hz  = 4.0;
    uint64_t          hash = 0;
    const std::string diag = diagDocument(*system, InspectionServiceStats{}, feed, 9, at, &hash);
    ASSERT_TRUE(validJson(diag)) << diag;
    EXPECT_NE(diag.find("\"type\":\"diag\",\"contract\":\"navigatr.inspect/2\",\"seq\":9,"),
              std::string::npos)
        << diag.substr(0, 200);
    EXPECT_EQ(diag.find("\"trail\""), std::string::npos);
    EXPECT_NE(diag.find("\"detection_frames\":["), std::string::npos);
    EXPECT_NE(diag.find("\"events\":["), std::string::npos);
    EXPECT_NE(diag.find("\"inspection\":{\"contract\":\"navigatr.inspect/2\""),
              std::string::npos);
    EXPECT_NE(diag.find("\"hub\":{\"posted\":{\"robot_state\":"), std::string::npos) << diag;
    EXPECT_NE(hash, 0u);
}

TEST(InspectionDocuments, DiagHashIgnoresVaryingValuesButNotContent) {
    FunctionRegistry functions;
    auto             system = buildRig(functions);
    ASSERT_NE(system, nullptr);
    // never stepped: only seq, time and the transport counters differ
    InspectionFeedStats    feed;
    InspectionServiceStats a, b;
    b.states_queued = 99;
    b.bytes_sent    = 123456;
    FeedClientStats client;
    client.queue.id = 4;
    feed.clients.push_back(client);
    uint64_t h1 = 0, h2 = 0;
    diagDocument(*system, a, InspectionFeedStats{}, 1, hostTime(10), &h1);
    diagDocument(*system, b, feed, 2, hostTime(500), &h2);
    EXPECT_EQ(h1, h2);

    // a reset is content (the session); the steady-motion case, where only
    // varying values change, is in inspection_feed_gtest (a rig without
    // camera, whose detections are content)
    system->reset();
    uint64_t          after_reset = 0;
    const std::string reset_doc =
        diagDocument(*system, a, feed, 6, hostTime(500), &after_reset);
    EXPECT_NE(after_reset, h2);
    EXPECT_NE(reset_doc.find("\"reset_count\":1"), std::string::npos);
}

TEST(InspectionDocuments, AttitudeStatusSeparatesMeasuredStaleAssumedAndUnavailable) {
    Attitude a;
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "unavailable");
    a = assumedLevelAttitude(0.3);
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "assumed_level");
    a            = Attitude{};
    a.valid      = true;
    a.measuredAt = hostTime(1000);
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000 + kAttitudeFreshMs)), "measured");
    EXPECT_STREQ(attitudeStatus(a, hostTime(1001 + kAttitudeFreshMs)), "stale");
    // valid but with no host time: freshness is unknown, never "measured"
    a.measuredAt = MonotonicTime{};
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "stale");
}

// Each shape AttitudeFold publishes (motion_step.cpp; pinned by
// BrainImuBench.TelemetryAttitudeIsMeasuredWhileFreshThenStale,
// BrainProfile.VexProfileShowsTheTelemetryTiltAsAttitude and
// BrainProfile.PicoImuProfilesCarryNoAttitude) reads as what it is.
TEST(InspectionDocuments, AttitudeStatusFollowsTheEstimatorsInvalidShapes) {
    // aged out: level, source and the old measurement time kept
    Attitude a   = assumedLevelAttitude(0.2);
    a.source     = "brain_vex_imu";
    a.measuredAt = hostTime(700);
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "stale");
    // even a measurement time on another clock is a measurement that aged out
    a.measuredAt = deviceTime(700);
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "stale");

    // configured, nothing measured yet (or the sample cannot be timed): the
    // observation id names the source, no measurement time
    a        = assumedLevelAttitude(0.2);
    a.source = "vex_attitude";
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "unavailable");

    // no source configured: level by assumption
    a = assumedLevelAttitude(0.2);
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "assumed_level");

    // no pose yet: neither measured nor assumed
    a = Attitude{};
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "unavailable");
    a.source = "vex_attitude";
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "unavailable");
    a.measuredAt = hostTime(900);
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "stale");

    // the valid branch ignores assumed_level and source
    a            = Attitude{};
    a.valid      = true;
    a.source     = "brain_vex_imu";
    a.measuredAt = hostTime(900);
    EXPECT_STREQ(attitudeStatus(a, hostTime(1000)), "measured");
}

// The rig's estimator with an attitude source that never measures, and with
// no attitude source at all, in the state message.
TEST(InspectionDocuments, StateAttitudeStatusIsUnavailableOrAssumedByConfiguration) {
    const auto statusAfterRunning = [](const std::string& xml, std::string& robot) {
        FunctionRegistry functions;
        registerAll(functions);
        std::string err;
        auto        system = System::buildFromString(xml.c_str(), functions, err);
        EXPECT_NE(system, nullptr) << err;
        if (system == nullptr) {
            return;
        }
        int64_t now = 1;
        for (int i = 0; i < 60; ++i) {
            now += 10;
            system->step(hostTime(now));
        }
        uint64_t          publication = 0;
        const std::string state = stateDocument(*system, 1, hostTime(now), now * 1000, &publication);
        EXPECT_TRUE(validJson(state)) << state;
        const std::size_t at = state.find("\"attitude\":{");
        robot                = at == std::string::npos ? std::string() : state.substr(at, 200);
    };
    const std::string measured = "<Attitude mode=\"measured\"";
    std::string       xml      = kRig;
    const std::size_t m        = xml.find(measured);
    ASSERT_NE(m, std::string::npos);
    xml.replace(m, measured.size(), "<Attitude mode=\"unavailable\"");
    std::string robot;
    statusAfterRunning(xml, robot);
    EXPECT_EQ(robot.rfind("\"attitude\":{\"valid\":false,\"assumed_level\":true,"
                          "\"status\":\"unavailable\",\"reference\":\"odometry\","
                          "\"source\":\"attitude\",",
                          0),
              0u)
        << robot;

    const std::string fold = "<Attitude observation_id=\"attitude\" max_age_ms=\"200\"/>";
    xml                    = kRig;
    const std::size_t f    = xml.find(fold);
    ASSERT_NE(f, std::string::npos);
    xml.erase(f, fold.size());
    statusAfterRunning(xml, robot);
    EXPECT_EQ(robot.rfind("\"attitude\":{\"valid\":false,\"assumed_level\":true,"
                          "\"status\":\"assumed_level\",\"reference\":\"odometry\","
                          "\"source\":\"\",",
                          0),
              0u)
        << robot;
}

namespace
{

// A Pico link that reports a fixed status frame.
struct StatusOnlyPico : PicoControl {
    PicoLinkState state;
    PicoLinkState link() const override { return state; }
    uint32_t      submit(uint8_t, uint8_t, MonotonicTime, double) override { return 0; }
    PicoRequestStatus request(uint32_t) const override { return {}; }
};

} // namespace

TEST(InspectionDocuments, PicoLastCommandNamesEveryPicoOp) {
    auto pico                          = std::make_shared<StatusOnlyPico>();
    pico->state.frames_fresh           = true;
    pico->state.identity               = true;
    pico->state.status_known           = true;
    pico->state.last_status            = hostTime(5);
    pico->state.status.last_request_id = 9;
    pico->state.status.last_status     = translagatr::kPicoCommandCompleted;
    FunctionRegistry functions;
    registerAll(functions);
    ASSERT_TRUE(functions.add<ResourceMakeFunction>(
        FunctionKey{"status_only_pico"},
        [pico](const ConfigNode&, ResourceInitializationContext&, std::string&) {
            return ResourceInstance::asContract<PicoControl>(pico);
        }));
    std::string       xml = kRig;
    const std::string end = "</Resources>";
    const std::size_t at  = xml.find(end);
    ASSERT_NE(at, std::string::npos);
    xml.insert(at, "<Resource id=\"pico_link\" type=\"status_only_pico\"/>");
    std::string err;
    auto        system = System::buildFromString(xml.c_str(), functions, err);
    ASSERT_NE(system, nullptr) << err;

    const struct {
        uint8_t     op;
        const char* name;
    } ops[] = {{0, "none"},
               {translagatr::kPicoOpConfigure, "configure"},
               {translagatr::kPicoOpReinitImu, "reinit_imu"},
               {translagatr::kPicoOpRestartAcquisition, "restart_acquisition"},
               {translagatr::kPicoOpDiagnostics, "diagnostics"},
               {translagatr::kPicoOpDiagnostics + 1, "unknown"}};
    for (const auto& o : ops) {
        pico->state.status.last_op = o.op;
        const std::string diag =
            diagDocument(*system, InspectionServiceStats{}, InspectionFeedStats{}, 1, hostTime(10));
        ASSERT_TRUE(validJson(diag));
        EXPECT_NE(diag.find("\"pico\":{\"resource_id\":\"pico_link\","), std::string::npos)
            << diag;
        EXPECT_NE(diag.find(std::string("\"last_command\":{\"request_id\":9,\"op\":\"") + o.name +
                            "\",\"status\":\"completed\""),
                  std::string::npos)
            << static_cast<int>(o.op);
    }
}

TEST(InspectionDocuments, TelemetryRecordDecodesThroughTheCodecWithNames) {
    translagatr::BrainRequest req;
    req.op                         = translagatr::kOpTelemetry;
    req.session                    = 0x01020304u;
    req.request_id                 = 77;
    translagatr::BrainTelemetry& t = req.telemetry;
    t.flags = translagatr::kTelemetryAttitude | translagatr::kTelemetryMotion |
              translagatr::kTelemetryWheels | translagatr::kTelemetryTarget;
    t.stamp_ms            = 123456;
    t.roll_cdeg           = -250;
    t.pitch_cdeg          = 125;
    t.command_id          = 42;
    t.motion_state        = 2;    // running
    t.motion_reason       = 20;   // tracking_error
    t.plan_mode           = 1;    // avoiding
    t.segment             = 1;
    t.segment_count       = 3;
    t.target_x_mm         = 1500;
    t.target_y_mm         = -250;
    t.target_heading_cdeg = 9000;
    t.cmd_vx_mm_s         = 400;
    t.cmd_vy_mm_s         = -100;
    t.cmd_omega_cdeg_s    = 4500;
    t.cross_track_mm      = 12;
    t.distance_error_mm   = 800;
    t.heading_error_cdeg  = -300;
    t.drive_fault         = 4;   // stale
    t.wheel_count         = 2;
    t.wheel_rpm_x10[0]    = 1234;
    t.wheel_rpm_x10[1]    = -55;
    uint8_t        frame[translagatr::kMaxFrameLen];
    const uint16_t n = translagatr::encodeBrainRequest(req, frame, sizeof(frame));
    ASSERT_EQ(n, translagatr::kBrainRequestHeaderLen + translagatr::kTelemetryBodyLen +
                     translagatr::kLinkEnvelopeLen);

    DiagBrainTelemetry rec;
    rec.session = req.session;
    rec.len     = translagatr::kTelemetryBodyLen;
    std::memcpy(rec.body, frame + 4 + translagatr::kBrainRequestHeaderLen, rec.len);
    translagatr::BrainTelemetry out;
    ASSERT_TRUE(decodeTelemetryRecord(rec, out));
    EXPECT_EQ(out.stamp_ms, t.stamp_ms);
    EXPECT_EQ(out.roll_cdeg, t.roll_cdeg);
    EXPECT_EQ(out.target_y_mm, t.target_y_mm);
    EXPECT_EQ(out.wheel_rpm_x10[1], t.wheel_rpm_x10[1]);

    DiagRecord record;
    record.kind    = DiagKind::kBrainTelemetry;
    record.host_us = 2000000;
    record.payload = rec;
    const std::string doc = telemetryDocument(5, hostTime(2100), record, rec, out);
    ASSERT_TRUE(validJson(doc)) << doc;
    EXPECT_NE(doc.find("\"type\":\"telemetry\",\"seq\":5,"), std::string::npos);
    EXPECT_NE(doc.find("\"received_host_ms\":2000,\"age_ms\":100"), std::string::npos) << doc;
    EXPECT_NE(doc.find("\"session\":16909060"), std::string::npos);
    EXPECT_NE(doc.find("\"attitude\":{\"roll_deg\":-2.5,\"pitch_deg\":1.25}"), std::string::npos)
        << doc;
    EXPECT_NE(doc.find("\"state\":2,\"state_name\":\"running\""), std::string::npos);
    EXPECT_NE(doc.find("\"reason\":20,\"reason_name\":\"tracking_error\""), std::string::npos);
    EXPECT_NE(doc.find("\"mode\":1,\"mode_name\":\"avoiding\""), std::string::npos);
    EXPECT_NE(doc.find("\"target\":{\"x_m\":1.5,\"y_m\":-0.25,\"heading_deg\":90}"),
              std::string::npos)
        << doc;
    EXPECT_NE(doc.find("\"drive_fault\":4,\"drive_fault_name\":\"stale\""), std::string::npos);
    EXPECT_NE(doc.find("\"wheels\":{\"rpm\":[123.4,-5.5]}"), std::string::npos) << doc;
    EXPECT_NE(doc.find("\"flags\":15,"), std::string::npos) << doc;

    // without the target bit target_* is no destination: null, the rest of
    // motion and the raw flags unchanged; an unknown bit passes through raw
    const auto flagsAt = [&](uint8_t flags) {
        uint8_t                   f[translagatr::kMaxFrameLen];
        translagatr::BrainRequest r = req;
        r.telemetry.flags           = flags;
        EXPECT_GT(translagatr::encodeBrainRequest(r, f, sizeof(f)), 0);
        DiagBrainTelemetry body = rec;
        std::memcpy(body.body, f + 4 + translagatr::kBrainRequestHeaderLen, body.len);
        translagatr::BrainTelemetry decoded;
        EXPECT_TRUE(decodeTelemetryRecord(body, decoded));
        return telemetryDocument(7, hostTime(2100), record, body, decoded);
    };
    const uint8_t     no_target = translagatr::kTelemetryMotion | 0x80;
    const std::string idle      = flagsAt(no_target);
    ASSERT_TRUE(validJson(idle)) << idle;
    EXPECT_NE(idle.find("\"flags\":" + std::to_string(no_target) + ","), std::string::npos)
        << idle;
    EXPECT_NE(idle.find("\"segment_count\":3,\"target\":null,\"cmd\":{\"vx_m_s\":0.4,"),
              std::string::npos)
        << idle;
    EXPECT_NE(idle.find("\"state\":2,\"state_name\":\"running\""), std::string::npos);
    EXPECT_NE(idle.find("\"attitude\":null"), std::string::npos);
    // the target bit alone carries no motion group
    const std::string target_only = flagsAt(translagatr::kTelemetryTarget);
    EXPECT_NE(target_only.find("\"flags\":8,\"attitude\":null,\"motion\":null,\"wheels\":null"),
              std::string::npos)
        << target_only;

    // groups whose flag is clear are null; a wrong length is not decoded
    rec.body[0] = 0;
    ASSERT_TRUE(decodeTelemetryRecord(rec, out));
    const std::string empty = telemetryDocument(6, hostTime(2100), record, rec, out);
    EXPECT_NE(empty.find("\"attitude\":null,\"motion\":null,\"wheels\":null"), std::string::npos)
        << empty;
    rec.len = 53;
    EXPECT_FALSE(decodeTelemetryRecord(rec, out));
}

TEST(InspectionDocuments, EventPongAndCaptureMessagesAreFlatAndEcho) {
    RuntimeEvent e;
    e.sequence = 12;
    e.at       = hostTime(345);
    e.text     = "Pico \"rebooted\"";
    const std::string event = eventDocument(e);
    ASSERT_TRUE(validJson(event)) << event;
    EXPECT_EQ(event,
              "{\"type\":\"event\",\"seq\":12,\"host_ms\":345,\"text\":\"Pico \\\"rebooted\\\"\"}");

    const std::string pong = pongDocument("7", "1234.56789", hostTime(10), 10500);
    ASSERT_TRUE(validJson(pong)) << pong;
    // client_ms comes back exactly as sent, no rounding
    EXPECT_EQ(pong, "{\"type\":\"pong\",\"id\":7,\"client_ms\":1234.56789,\"host_ms\":10,"
                    "\"host_us\":10500}");
    EXPECT_EQ(pongDocument("", "", hostTime(1), 1000),
              "{\"type\":\"pong\",\"id\":null,\"client_ms\":null,\"host_ms\":1,\"host_us\":1000}");

    FunctionRegistry functions;
    auto             system = buildRig(functions);
    ASSERT_NE(system, nullptr);
    ASSERT_NE(system->capture(), nullptr);
    const std::string capture = captureDocument(*system->capture(), 3, hostTime(20));
    ASSERT_TRUE(validJson(capture)) << capture;
    EXPECT_EQ(capture.rfind("{\"type\":\"capture\",\"seq\":3,\"host_ms\":20,", 0), 0u) << capture;
    EXPECT_NE(capture.find("\"available\":"), std::string::npos);
}

TEST(InspectionDocuments, FrameHeaderCarriesTheDetectionEntryOfExactlyItsFrame) {
    FunctionRegistry functions;
    auto             system = buildRig(functions);
    ASSERT_NE(system, nullptr);
    int64_t now = 1;
    for (int i = 0; i < 180; ++i) {
        now += 10;
        system->step(hostTime(now));
    }
    ASSERT_EQ(system->detectionFrames().size(), 1u);
    const std::shared_ptr<const DetectionFrameSnapshot> first =
        system->detectionFrames().begin()->second;
    const DetectionFrameSnapshot& f = *first;

    // the header's detection is byte for byte the diag's entry for the frame
    const std::string entries = "\"detection_frames\":[";
    std::string       diag =
        diagDocument(*system, InspectionServiceStats{}, InspectionFeedStats{}, 1, hostTime(now));
    std::size_t at = diag.find(entries);
    ASSERT_NE(at, std::string::npos);
    const std::string in_diag = balancedObject(diag, at + entries.size());
    ASSERT_FALSE(in_diag.empty()) << diag.substr(at, 200);
    const std::string header =
        frameHeaderDocument(f, f.width_px / 2, f.height_px / 2, 70, 1.5, hostTime(now), system.get());
    ASSERT_TRUE(validJson(header)) << header;
    const std::string detection = objectValue(header, "detection");
    EXPECT_EQ(detection, in_diag);
    const std::string identity = "{\"camera\":\"front_camera\",\"frame_id\":\"cam\",\"epoch\":" +
                                 std::to_string(f.frame_epoch) + ",\"sequence\":" +
                                 std::to_string(f.frame_sequence) + ",";
    EXPECT_EQ(detection.rfind(identity, 0), 0u) << detection.substr(0, 200);
    EXPECT_NE(header.find("\"epoch\":" + std::to_string(f.frame_epoch) + ",\"sequence\":" +
                          std::to_string(f.frame_sequence) + ","),
              std::string::npos);
    EXPECT_NE(detection.find("\"has_observations\":true"), std::string::npos);

    // a newer frame arrives: an encode of the older one still carries the
    // older frame's entry, never the newest one
    for (int i = 0; i < 100 && system->detectionFrames().begin()->second->frame_sequence ==
                                   f.frame_sequence;
         ++i) {
        now += 10;
        system->step(hostTime(now));
    }
    const std::shared_ptr<const DetectionFrameSnapshot> newest =
        system->detectionFrames().begin()->second;
    ASSERT_NE(newest->frame_sequence, f.frame_sequence);
    diag = diagDocument(*system, InspectionServiceStats{}, InspectionFeedStats{}, 2, hostTime(now));
    at   = diag.find(entries);
    ASSERT_NE(at, std::string::npos);
    const std::string newest_entry = balancedObject(diag, at + entries.size());
    const std::string old_header =
        frameHeaderDocument(f, f.width_px / 2, f.height_px / 2, 70, 1.5, hostTime(now), system.get());
    const std::string old_detection = objectValue(old_header, "detection");
    EXPECT_EQ(old_detection.rfind(identity, 0), 0u) << old_detection.substr(0, 200);
    EXPECT_NE(old_detection, newest_entry);
    EXPECT_NE(newest_entry.find("\"sequence\":" + std::to_string(newest->frame_sequence)),
              std::string::npos);

    // without the system there is nothing to bind: no detection key
    EXPECT_EQ(frameHeaderDocument(f, 320, 240, 70, 1.5, hostTime(now)).find("\"detection\""),
              std::string::npos);
}
