// localization_gtest.cpp
// wheel_imu_prediction: chord integration, edge-triggered init, orientation
// override, reference validation.

#include <gtest/gtest.h>

#include <memory>
#include <string>

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
        <LocalizationPrediction type="localization/wheel_imu_prediction">
            <Motion artifact_id="motion"/>
        </LocalizationPrediction>)") {
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
    EXPECT_NEAR(out.robot.pose.x_m, 0.1, 1e-12);
    EXPECT_TRUE(out.robot.valid);
    EXPECT_NEAR(out.robot.vx_m_s, 20.0, 1e-9);
}

TEST(WheelImuPrediction, QuarterCircleChord) {
    Fixture      f;
    const double R = 0.5;
    f.putMotion(R * kPi / 2.0, 0.0, kPi / 2.0);
    f.run();
    EXPECT_NEAR(f.previous.pose.x_m, R, 1e-9);
    EXPECT_NEAR(f.previous.pose.y_m, R, 1e-9);
    EXPECT_NEAR(f.previous.pose.heading_rad, kPi / 2.0, 1e-12);
}

TEST(WheelImuPrediction, TranslationFollowsHeading) {
    Fixture f;
    f.previous.pose.heading_rad = kPi / 2.0;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    EXPECT_NEAR(f.previous.pose.x_m, 0.0, 1e-12);
    EXPECT_NEAR(f.previous.pose.y_m, 0.1, 1e-12);
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
    EXPECT_NEAR(out.robot.pose.x_m, 0.61, 1e-12);

    f.putMotion(0.1, 0.0, 0.0);
    out = f.run();   // same init_sequence: no re-init
    EXPECT_NEAR(out.robot.pose.y_m, 0.457 + 0.1, 1e-9);

    f.command.init_sequence = 2;   // a new command resets again
    f.artifacts.clear();
    out = f.run();
    EXPECT_NEAR(out.robot.pose.y_m, 0.457, 1e-12);
}

TEST(WheelImuPrediction, OrientationOverridesMotionHeading) {
    Fixture f(R"(
        <LocalizationPrediction type="localization/wheel_imu_prediction">
            <Motion artifact_id="motion"/>
            <Orientation artifact_id="orientation"/>
        </LocalizationPrediction>)");
    f.putMotion(0.0, 0.0, 0.5);        // wheels say half a radian
    f.putOrientation(0.25);            // gyro says a quarter
    f.run();
    EXPECT_NEAR(f.previous.pose.heading_rad, 0.25, 1e-12);
}

TEST(WheelImuPrediction, MissingMotionHoldsPose) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    f.artifacts.clear();
    const LocalizationOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_NEAR(out.robot.pose.x_m, 0.1, 1e-12);
    EXPECT_TRUE(out.robot.valid);
}

TEST(WheelImuPrediction, ReferencesValidateAtBuild) {
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(R"(
        <LocalizationPrediction type="localization/wheel_imu_prediction">
            <Motion artifact_id="ghost"/>
        </LocalizationPrediction>)"),
              tinyxml2::XML_SUCCESS);
    SlotInitializationContext context;   // no artifacts declared
    std::string               err;
    EXPECT_EQ(WheelImuPrediction::create(ConfigNode{doc.RootElement()}, context, err),
              nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);
}
