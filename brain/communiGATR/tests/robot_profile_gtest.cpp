// robot_profile_gtest.cpp
// SI robot profile to wire document: units, rounding, flags, and the shared
// check reported with the Pi's reason codes before anything is sent.

#include "communigatr/robot_profile.h"

#include <cmath>
#include <cstring>
#include <gtest/gtest.h>
#include <limits>
#include <set>
#include <string>

using namespace communigatr;
using investigatr::kPi;

namespace
{

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

// Current bench layout: encoder port 0 forward, port 1 sideways, VEX IMU on
// Smart Port 1. Offsets and footprint are test values.
RobotProfile perpendicularVex() {
    RobotProfile p;
    p.topology       = LocalizationTopology::kTwoWheelImu;
    p.wheels         = {{0, 0.024, 2048, 0.0, 0.10, 0.0, false},
                        {1, 0.024, 2048, -0.08, 0.0, kPi / 2, false}};
    p.imu_source     = ImuSource::kBrainVex;
    p.vex_smart_port = 1;
    p.footprint      = {0.20, 0.25, 0.22, 0.22};
    return p;
}

RobotProfile threeWheelPico() {
    RobotProfile p;
    p.topology   = LocalizationTopology::kThreeWheel;
    p.wheels     = {{0, 0.024, 2048, 0.05, 0.15, 0.0, false},
                    {1, 0.024, 2048, 0.05, -0.15, 0.0, true},
                    {2, 0.024, 2048, -0.12, 0.0, kPi / 2, false}};
    p.imu_source = ImuSource::kPico;
    p.imu_port   = 0;
    p.imu_invert = true;
    p.footprint  = {0.2, 0.2, 0.2, 0.2};
    p.cameras    = {{0, 0.1, 0.02, 0.3, 0.0, -0.2, 0.0}};
    return p;
}

void expectReason(const RobotProfile& p, uint8_t reason, uint8_t detail = 0) {
    gatr2::RobotProfileDoc doc;
    uint8_t                r = 0;
    uint8_t                d = 0;
    EXPECT_FALSE(toProfileDoc(p, doc, r, d));
    EXPECT_EQ(r, reason) << profileReasonName(r);
    EXPECT_EQ(d, detail);
    const ProfileDocument encoded = makeProfileDocument(p);
    EXPECT_EQ(encoded.reason, reason);
    EXPECT_EQ(encoded.detail, detail);
    EXPECT_TRUE(encoded.configured());
}

} // namespace

TEST(RobotProfile, PerpendicularWheelsWithVexImuConvertToWireUnits) {
    gatr2::RobotProfileDoc doc;
    uint8_t                reason = 0xFF;
    uint8_t                detail = 0xFF;
    ASSERT_TRUE(toProfileDoc(perpendicularVex(), doc, reason, detail));
    EXPECT_EQ(reason, gatr2::kProfileReasonNone);
    EXPECT_EQ(doc.topology, gatr2::kTopologyTwoWheelImu);
    ASSERT_EQ(doc.wheel_count, 2);
    EXPECT_EQ(doc.wheels[0].encoder_port, 0);
    EXPECT_EQ(doc.wheels[0].radius_um, 24000u);
    EXPECT_EQ(doc.wheels[0].counts_per_rev, 2048u);
    EXPECT_EQ(doc.wheels[0].x_um, 0);
    EXPECT_EQ(doc.wheels[0].y_um, 100000);
    EXPECT_EQ(doc.wheels[0].angle_mdeg, 0);
    EXPECT_EQ(doc.wheels[1].encoder_port, 1);
    EXPECT_EQ(doc.wheels[1].x_um, -80000);
    EXPECT_EQ(doc.wheels[1].angle_mdeg, 90000);
    EXPECT_EQ(doc.wheels[1].flags, 0);
    EXPECT_EQ(doc.imu_source, gatr2::kImuSourceBrainVex);
    EXPECT_EQ(doc.vex_smart_port, 1);
    EXPECT_EQ(doc.imu_port, 0);
    EXPECT_EQ(doc.imu_flags, 0);
    EXPECT_EQ(doc.footprint_front_um, 200000);
    EXPECT_EQ(doc.footprint_back_um, 250000);
    EXPECT_EQ(doc.footprint_left_um, 220000);
    EXPECT_EQ(doc.camera_count, 0);
}

TEST(RobotProfile, RoundsAndWrapsAnglesAndSetsFlags) {
    RobotProfile p          = threeWheelPico();
    p.wheels[0].x           = 0.0123456;  // 12345.6 um
    p.wheels[0].y           = -0.0000004; // -0.4 um
    p.wheels[0].angle       = -kPi;       // wraps to +180 deg
    p.wheels[1].angle       = 2 * kPi;    // 0
    p.wheels[2].angle       = 1.5 * kPi;  // -90 deg
    p.cameras[0].yaw        = -kPi;
    gatr2::RobotProfileDoc doc;
    uint8_t                reason = 0;
    uint8_t                detail = 0;
    ASSERT_TRUE(toProfileDoc(p, doc, reason, detail)) << profileReasonName(reason);
    EXPECT_EQ(doc.wheels[0].x_um, 12346);
    EXPECT_EQ(doc.wheels[0].y_um, 0);
    EXPECT_EQ(doc.wheels[0].angle_mdeg, 180000);
    EXPECT_EQ(doc.wheels[1].angle_mdeg, 0);
    EXPECT_EQ(doc.wheels[1].flags, gatr2::kWheelReversed);
    EXPECT_EQ(doc.wheels[2].angle_mdeg, -90000);
    EXPECT_EQ(doc.imu_flags, gatr2::kImuInvert);
    EXPECT_EQ(doc.imu_source, gatr2::kImuSourcePico);
    ASSERT_EQ(doc.camera_count, 1);
    EXPECT_EQ(doc.cameras[0].z_um, 300000);
    EXPECT_EQ(doc.cameras[0].pitch_mdeg, static_cast<int32_t>(std::lround(-0.2 * 180000 / kPi)));
    EXPECT_EQ(doc.cameras[0].yaw_mdeg, 180000);
}

TEST(RobotProfile, EncoderGeometryAndEmpiricalValuesStaySeparate) {
    RobotProfile p              = perpendicularVex();
    p.wheels[0].gear_ratio      = 2.5;       // encoder turns 2.5 times per wheel turn
    p.wheels[1].travel_scale    = 1.0123456; // measured sideways correction
    p.calibration.window        = 3.0;
    p.calibration.still_rate    = 0.5 * kPi / 180.0; // 0.5 deg/s
    p.calibration.still_travel  = 0.0004;
    gatr2::RobotProfileDoc doc;
    uint8_t                reason = 0;
    uint8_t                detail = 0;
    ASSERT_TRUE(toProfileDoc(p, doc, reason, detail)) << profileReasonName(reason);
    EXPECT_EQ(doc.wheels[0].gear_micro, 2500000u);
    EXPECT_EQ(doc.wheels[0].travel_scale_ppm, gatr2::kUnitMicro);
    EXPECT_EQ(doc.wheels[1].gear_micro, gatr2::kUnitMicro);
    EXPECT_EQ(doc.wheels[1].travel_scale_ppm, 1012346u);
    // Radius and counts stay as given: the correction is not folded into them.
    EXPECT_EQ(doc.wheels[1].radius_um, 24000u);
    EXPECT_EQ(doc.wheels[0].counts_per_rev, 2048u);
    EXPECT_EQ(doc.calibration_window_ms, 3000);
    EXPECT_EQ(doc.still_rate_cdps, 50);
    EXPECT_EQ(doc.still_travel_um, 400);

    // Defaults: unit gearing and scale, Pi calibration defaults.
    ASSERT_TRUE(toProfileDoc(perpendicularVex(), doc, reason, detail));
    EXPECT_EQ(doc.wheels[1].gear_micro, gatr2::kUnitMicro);
    EXPECT_EQ(doc.wheels[1].travel_scale_ppm, gatr2::kUnitMicro);
    EXPECT_EQ(doc.calibration_window_ms, 0);
    EXPECT_EQ(doc.still_rate_cdps, 0);
    EXPECT_EQ(doc.still_travel_um, 0);

    // A new travel scale is a new profile.
    EXPECT_NE(profileId(makeProfileDocument(p)), profileId(makeProfileDocument(perpendicularVex())));
}

TEST(RobotProfile, GearingScaleAndCalibrationRanges) {
    RobotProfile p           = perpendicularVex();
    p.wheels[1].travel_scale = 1.2;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 1);
    p.wheels[1].travel_scale = kNan;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 1);
    p.wheels[1].travel_scale = 1.1;
    gatr2::RobotProfileDoc doc;
    uint8_t                reason = 0;
    uint8_t                detail = 0;
    EXPECT_TRUE(toProfileDoc(p, doc, reason, detail));

    p                      = perpendicularVex();
    p.wheels[0].gear_ratio = 0.05;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 0);
    p.wheels[0].gear_ratio = -1.0;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 0);

    p                    = perpendicularVex();
    p.calibration.window = 0.2;
    expectReason(p, gatr2::kProfileReasonCalibration);
    p.calibration.window = 100.0; // over u16 ms
    expectReason(p, gatr2::kProfileReasonCalibration);
    p.calibration.window     = 1.0;
    p.calibration.still_rate = -0.1;
    expectReason(p, gatr2::kProfileReasonCalibration);
    p.calibration.still_rate   = 0.0;
    p.calibration.still_travel = 0.01; // 10 mm per window
    expectReason(p, gatr2::kProfileReasonCalibration);
}

TEST(RobotProfile, SupportedTopologiesValidate) {
    gatr2::RobotProfileDoc doc;
    uint8_t                reason = 0;
    uint8_t                detail = 0;
    EXPECT_TRUE(toProfileDoc(perpendicularVex(), doc, reason, detail));
    EXPECT_TRUE(toProfileDoc(threeWheelPico(), doc, reason, detail));

    RobotProfile none = threeWheelPico();
    none.imu_source   = ImuSource::kNone;
    none.imu_invert   = false;
    EXPECT_TRUE(toProfileDoc(none, doc, reason, detail)) << profileReasonName(reason);

    RobotProfile pico = perpendicularVex();
    pico.imu_source   = ImuSource::kPico;
    pico.vex_smart_port = 0;
    EXPECT_TRUE(toProfileDoc(pico, doc, reason, detail)) << profileReasonName(reason);

    RobotProfile forward = perpendicularVex();
    forward.topology     = LocalizationTopology::kTwoForwardWheelImu;
    forward.wheels[1]    = {1, 0.024, 2048, 0.0, -0.10, kPi, false};
    EXPECT_TRUE(toProfileDoc(forward, doc, reason, detail)) << profileReasonName(reason);
    EXPECT_EQ(doc.topology, gatr2::kTopologyTwoForwardWheelImu);
    EXPECT_EQ(doc.wheels[1].angle_mdeg, 180000);
}

TEST(RobotProfile, LocalRejectionsUseThePiReasonCodes) {
    RobotProfile p = perpendicularVex();
    p.wheels.push_back(p.wheels[0]);
    p.wheels.push_back(p.wheels[0]);
    expectReason(p, gatr2::kProfileReasonWheelCount);

    p = perpendicularVex();
    p.wheels.pop_back();
    expectReason(p, gatr2::kProfileReasonWheelCount);

    p                        = perpendicularVex();
    p.wheels[1].encoder_port = 0;
    expectReason(p, gatr2::kProfileReasonEncoderPort, 1);

    p                  = perpendicularVex();
    p.wheels[1].radius = kNan;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 1);
    p.wheels[1].radius = 0.0;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 1);
    p.wheels[1].radius = -0.024;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 1);

    p                          = perpendicularVex();
    p.wheels[0].counts_per_rev = 0;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 0);

    p             = perpendicularVex();
    p.wheels[0].x = 1e300;
    expectReason(p, gatr2::kProfileReasonWheelGeometry, 0);

    p                 = perpendicularVex();
    p.wheels[1].angle = 0.0; // parallel to wheel 0
    expectReason(p, gatr2::kProfileReasonObservability);

    p                = perpendicularVex();
    p.vex_smart_port = 0;
    expectReason(p, gatr2::kProfileReasonImuPort);
    p.vex_smart_port = 22;
    expectReason(p, gatr2::kProfileReasonImuPort);

    p            = perpendicularVex();
    p.imu_source = ImuSource::kNone;
    p.vex_smart_port = 0;
    expectReason(p, gatr2::kProfileReasonImuCombination);

    p                = threeWheelPico();
    p.imu_source     = ImuSource::kBrainVex;
    p.imu_invert     = false;
    p.vex_smart_port = 1;
    expectReason(p, gatr2::kProfileReasonImuCombination);

    p            = perpendicularVex();
    p.imu_invert = true; // only the Pico IMU takes it
    expectReason(p, gatr2::kProfileReasonImuSource);

    p                = perpendicularVex();
    p.footprint.left = kNan;
    expectReason(p, gatr2::kProfileReasonFootprint);
    p.footprint.left  = 0.0;
    p.footprint.right = 0.0;
    expectReason(p, gatr2::kProfileReasonFootprint);

    p         = threeWheelPico();
    p.cameras = {{0, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, kNan}};
    expectReason(p, gatr2::kProfileReasonCamera, 1);
    p.cameras = {{0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}};
    expectReason(p, gatr2::kProfileReasonCamera, 1);
    p.cameras.assign(5, CameraMount{});
    expectReason(p, gatr2::kProfileReasonCamera, gatr2::kProfileMaxCameras);
}

TEST(RobotProfile, DocumentBytesIdAndRoundTrip) {
    const ProfileDocument doc = makeProfileDocument(threeWheelPico());
    ASSERT_EQ(doc.reason, gatr2::kProfileReasonNone);
    EXPECT_TRUE(doc.configured());
    EXPECT_EQ(doc.len, gatr2::kProfileHeaderLen + 3 * gatr2::kProfileWheelLen +
                           1 * gatr2::kProfileCameraLen);
    EXPECT_EQ(profileId(doc), gatr2::crc32(doc.bytes, doc.len));
    EXPECT_NE(profileId(doc), 0u);

    gatr2::RobotProfileDoc decoded;
    ASSERT_TRUE(gatr2::decodeRobotProfile(doc.bytes, doc.len, decoded));
    gatr2::RobotProfileDoc direct;
    uint8_t                reason = 0;
    uint8_t                detail = 0;
    ASSERT_TRUE(toProfileDoc(threeWheelPico(), direct, reason, detail));
    uint8_t again[gatr2::kProfileMaxLen];
    ASSERT_EQ(gatr2::encodeRobotProfile(decoded, again, sizeof(again)), doc.len);
    EXPECT_EQ(std::memcmp(again, doc.bytes, doc.len), 0);
    EXPECT_EQ(gatr2::encodeRobotProfile(direct, again, sizeof(again)), doc.len);
    EXPECT_EQ(std::memcmp(again, doc.bytes, doc.len), 0);

    // Any change of geometry is a different profile id.
    RobotProfile moved      = threeWheelPico();
    moved.wheels[2].x      += 0.000001;
    EXPECT_NE(profileId(makeProfileDocument(moved)), profileId(doc));
}

TEST(RobotProfile, FailedDocumentsCarryTheirReason) {
    // Converts but fails the shared check: bytes kept for inspection.
    RobotProfile parallel     = perpendicularVex();
    parallel.wheels[1].angle  = 0.0;
    const ProfileDocument bad = makeProfileDocument(parallel);
    EXPECT_GT(bad.len, 0u);
    EXPECT_EQ(bad.reason, gatr2::kProfileReasonObservability);

    // Does not convert: no bytes.
    RobotProfile nan         = perpendicularVex();
    nan.footprint.front      = kNan;
    const ProfileDocument no = makeProfileDocument(nan);
    EXPECT_EQ(no.len, 0u);
    EXPECT_EQ(profileId(no), 0u);
    EXPECT_EQ(no.reason, gatr2::kProfileReasonFootprint);
    EXPECT_TRUE(no.configured());

    const ProfileDocument none;
    EXPECT_FALSE(none.configured());
}

TEST(RobotProfile, ReasonNamesAreDistinct) {
    std::set<std::string> names;
    for (uint8_t r = gatr2::kProfileReasonNone; r <= gatr2::kProfileReasonCalibration; ++r) {
        names.insert(profileReasonName(r));
    }
    EXPECT_EQ(names.size(), static_cast<std::size_t>(gatr2::kProfileReasonCalibration + 1));
    EXPECT_STREQ(profileReasonName(200), "?");
}
