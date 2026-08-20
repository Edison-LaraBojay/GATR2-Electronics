// localization_gtest.cpp
// wheel_imu_prediction: chord integration, edge-triggered init, orientation
// override, reference validation.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "core/clock_sync.h"
#include "impl/localization/wheel_imu_prediction.h"
#include "math/angles.h"
#include "payloads/preprocessing_products.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

struct Fixture {
    tinyxml2::XMLDocument doc;
    SensorResultsMap      results;
    ArtifactMap           artifacts;
    CommandState          command;
    RobotState            previous;

    std::unique_ptr<Localization> prediction;

    explicit Fixture(const char* xml = R"(
        <Localization type="wheel_imu_prediction">
            <Motion artifact_id="motion"/>
        </Localization>)") {
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        SlotInitializationContext context;
        context.artifacts = {
            ArtifactOutputDecl{ArtifactId{"motion"},
                               PayloadDescriptor::of<PlanarMotionDelta>(
                                   payload_names::kPlanarMotionDelta)},
            ArtifactOutputDecl{
                ArtifactId{"orientation"},
                PayloadDescriptor::of<ImuDelta>(payload_names::kImuDelta)}};
        std::string err;
        prediction =
            WheelImuPrediction::create(ConfigNode{doc.RootElement()}, context, err);
        EXPECT_NE(prediction, nullptr) << err;
    }

    void putMotion(double dx, double dy, double dtheta, double dt = 0.005) {
        ArtifactRecord record;
        record.measuredAt = deviceTime(100);
        record.payload    = TypedPayload::store(PlanarMotionDelta{dx, dy, dtheta, dt},
                                             payload_names::kPlanarMotionDelta);
        artifacts[ArtifactId{"motion"}] = std::move(record);
    }

    void putOrientation(double delta_rad, double dt = 0.005) {
        ArtifactRecord record;
        record.measuredAt = deviceTime(100);
        ImuDelta       delta;
        delta.delta_rad = delta_rad;
        delta.dt_s      = dt;
        record.payload  = TypedPayload::store(delta, payload_names::kImuDelta);
        artifacts[ArtifactId{"orientation"}] = std::move(record);
    }

    LocalizationOutput run() {
        const LocalizationOutput out =
            prediction->run({results, artifacts, command, previous, hostTime(1)});
        previous = out.robot;
        return out;
    }
};

} // namespace

TEST(WheelImuPrediction, StraightLine) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.0);
    const LocalizationOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kOk);
    EXPECT_NEAR(out.robot.fieldPose().x_m, 0.1, 1e-12);
    EXPECT_TRUE(out.robot.valid);
    EXPECT_NEAR(out.robot.vx_m_s, 20.0, 1e-9);
}

TEST(WheelImuPrediction, QuarterCircleChord) {
    Fixture      f;
    const double R = 0.5;
    f.putMotion(R * kPi / 2.0, 0.0, kPi / 2.0);
    f.run();
    EXPECT_NEAR(f.previous.fieldPose().x_m, R, 1e-9);
    EXPECT_NEAR(f.previous.fieldPose().y_m, R, 1e-9);
    EXPECT_NEAR(f.previous.fieldPose().heading_rad, kPi / 2.0, 1e-12);
}

TEST(WheelImuPrediction, TranslationFollowsHeading) {
    Fixture f;
    f.previous.odom_pose.heading_rad = kPi / 2.0;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    EXPECT_NEAR(f.previous.fieldPose().x_m, 0.0, 1e-12);
    EXPECT_NEAR(f.previous.fieldPose().y_m, 0.1, 1e-12);
}

TEST(WheelImuPrediction, InitEdgeTriggered) {
    Fixture f;
    f.command.init_sequence          = 1;
    f.command.init_pose.x_m          = 0.61;
    f.command.init_pose.y_m          = 0.457;
    f.command.init_pose.heading_rad  = kPi / 2.0;

    LocalizationOutput out = f.run();   // no motion data, init still applies
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_TRUE(out.robot.initialized);
    EXPECT_NEAR(out.robot.fieldPose().x_m, 0.61, 1e-12);

    f.putMotion(0.1, 0.0, 0.0);
    out = f.run();   // same init_sequence: no re-init
    EXPECT_NEAR(out.robot.fieldPose().y_m, 0.457 + 0.1, 1e-9);

    f.command.init_sequence = 2;   // a new command resets again
    f.artifacts.clear();
    out = f.run();
    EXPECT_NEAR(out.robot.fieldPose().y_m, 0.457, 1e-12);
}

TEST(WheelImuPrediction, OrientationOverridesMotionHeading) {
    Fixture f(R"(
        <Localization type="wheel_imu_prediction">
            <Motion artifact_id="motion"/>
            <Orientation artifact_id="orientation"/>
        </Localization>)");
    f.putMotion(0.0, 0.0, 0.5);        // wheels say half a radian
    f.putOrientation(0.25);            // gyro says a quarter
    f.run();
    EXPECT_NEAR(f.previous.fieldPose().heading_rad, 0.25, 1e-12);
}

TEST(WheelImuPrediction, MissingMotionHoldsPose) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    f.artifacts.clear();
    const LocalizationOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_NEAR(out.robot.fieldPose().x_m, 0.1, 1e-12);
    EXPECT_TRUE(out.robot.valid);
}

TEST(DeviceToHostClock, WarmsUpThenTracksMinimumLatencyOffset) {
    DeviceToHostClock clock;
    EXPECT_FALSE(clock.valid());

    clock.observe(deviceTime(1000), hostTime(2030));   // 30 ms of latency
    clock.observe(deviceTime(1010), hostTime(2015));   // 5 ms, the best pairing
    clock.observe(deviceTime(1020), hostTime(2060));   // batched, 40 ms
    EXPECT_FALSE(clock.valid());   // one pairing is not a clock model

    for (int i = 0; i < 5; ++i) {
        clock.observe(deviceTime(1030 + 10 * i), hostTime(2055 + 10 * i));   // 25 ms
    }
    ASSERT_TRUE(clock.valid());

    // the minimum-latency offset wins: 2015 - 1010 = 1005
    const MonotonicTime mapped = clock.toHost(deviceTime(1020));
    EXPECT_EQ(mapped.ms, 2025);
    EXPECT_EQ(mapped.domain, ClockDomain::kHost);

    // wrong-domain observations are ignored, never poison the estimate
    clock.observe(hostTime(5), hostTime(6));
    EXPECT_EQ(clock.toHost(deviceTime(1020)).ms, 2025);
}

TEST(WheelImuPrediction, PoseTimeUsesDeviceToHostMappingAfterWarmup) {
    Fixture f;
    LocalizationOutput out;
    for (int i = 0; i < 8; ++i) {
        // device stamp stays 100; host receipt advances, so the first
        // pairing (offset 50) is the window minimum
        f.putMotion(0.01, 0.0, 0.0);
        out = f.prediction->run(
            {f.results, f.artifacts, f.command, f.previous, hostTime(150 + 10 * i)});
        f.previous = out.robot;
        if (i < 7) {
            EXPECT_FALSE(out.robot.measuredAtHost.isSet());   // still warming up
        }
    }
    ASSERT_TRUE(out.robot.measuredAtHost.isSet());
    EXPECT_EQ(out.robot.measuredAtHost.domain, ClockDomain::kHost);
    EXPECT_EQ(out.robot.measuredAtHost.ms, 150);   // 100 + minimum offset 50
    EXPECT_EQ(out.robot.measuredAt.domain, ClockDomain::kDevice);
}

TEST(WheelImuPrediction, ReferencesValidateAtBuild) {
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(R"(
        <Localization type="wheel_imu_prediction">
            <Motion artifact_id="ghost"/>
        </Localization>)"),
              tinyxml2::XML_SUCCESS);
    SlotInitializationContext context;   // no artifacts declared
    std::string               err;
    EXPECT_EQ(WheelImuPrediction::create(ConfigNode{doc.RootElement()}, context, err),
              nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);
}
