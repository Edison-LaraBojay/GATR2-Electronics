// preprocessing_gtest.cpp
// Geometry-owned tracking wheel odometry and IMU normalization, driven
// directly through their contracts with hand-built sensor results. Ids are
// arbitrary; labels are cosmetic; geometry is what changes behavior.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "impl/preprocessing/configured_collection.h"
#include "impl/preprocessing/imu_normalization.h"
#include "impl/preprocessing/tracking_wheel_odometry.h"
#include "math/angles.h"
#include "payloads/preprocessing_products.h"
#include "runtime/register_all.h"
#include "runtime/sensor_catalog.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

struct Fixture {
    tinyxml2::XMLDocument doc;
    SensorCatalog         catalog;
    SensorResultsMap      results;
    FunctionRegistry      functions;

    Fixture() {
        register_preprocessing(functions);
        catalog.add(SensorId{"enc_a"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"enc_b"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"enc_c"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"imu"},
                    PayloadDescriptor::of<ImuSample>(payload_names::kImuSample));
    }

    ConfigNode parse(const std::string& xml) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml.c_str()), tinyxml2::XML_SUCCESS);
        return ConfigNode{doc.RootElement()};
    }

    std::unique_ptr<PreprocessorExecutable> makeOdometry(const std::string& xml,
                                                         std::string&       err) {
        PreprocessorInitializationContext context;
        context.sensors   = &catalog;
        context.functions = &functions;
        return TrackingWheelOdometry::create(parse(xml), context, err);
    }

    void putEncoder(const char* id, double angle_rad, int64_t stamp_ms,
                    uint64_t sequence) {
        SensorRecord record;
        record.state = SensorState::kValid;
        StoredSensorSample stored;
        stored.measuredAt = deviceTime(stamp_ms);
        stored.receivedAt = hostTime(stamp_ms);
        stored.sequence   = sequence;
        stored.payload =
            TypedPayload::store(EncoderSample{angle_rad}, payload_names::kEncoderSample);
        record.latest        = std::move(stored);
        results[SensorId{id}] = std::move(record);
    }

    void putImu(double rate_rad_s, int64_t stamp_ms, uint64_t sequence) {
        SensorRecord record;
        record.state = SensorState::kValid;
        StoredSensorSample stored;
        stored.measuredAt = deviceTime(stamp_ms);
        stored.receivedAt = hostTime(stamp_ms);
        stored.sequence   = sequence;
        stored.payload =
            TypedPayload::store(ImuSample{rate_rad_s}, payload_names::kImuSample);
        record.latest       = std::move(stored);
        results[SensorId{"imu"}] = std::move(record);
    }

    PreprocessingInput input() {
        return PreprocessingInput{results, hostTime(1), 1, nullptr};
    }
};

// three wheels per the reference layout: left, right, rear
std::string threeWheelXml(const char* a, const char* b, const char* c,
                          const char* label_a = "left") {
    std::string xml = R"(<Preprocessor id="tracking_motion"
        type="tracking_wheel_odometry">)";
    xml += std::string(R"(<TrackingWheel sensor_id=")") + a + R"(" label=")" + label_a +
           R"(" radius_m="0.0254" position_x_m="0" position_y_m="0.13"
              measurement_angle_deg="0"/>)";
    xml += std::string(R"(<TrackingWheel sensor_id=")") + b +
           R"(" label="right" radius_m="0.0254" position_x_m="0" position_y_m="-0.13"
              measurement_angle_deg="0"/>)";
    xml += std::string(R"(<TrackingWheel sensor_id=")") + c +
           R"(" label="rear" radius_m="0.0254" position_x_m="-0.12" position_y_m="0"
              measurement_angle_deg="90"/>)";
    xml += R"(<Output artifact_id="motion"/></Preprocessor>)";
    return xml;
}

// feed wheel angles for a known body motion and return the solved delta
PlanarMotionDelta solveKnownMotion(Fixture& f, PreprocessorExecutable& odom, double dx,
                                   double dy, double dtheta) {
    const double k_a = -0.13, k_b = 0.13, k_c = -0.12;   // x*uy - y*ux
    const double r   = 0.0254;

    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putEncoder("enc_c", 0.0, 1000, 1);
    ArtifactMap artifacts;
    EXPECT_EQ(odom.run(f.input(), artifacts), FunctionStatus::kOk);   // seeds
    EXPECT_TRUE(artifacts.empty());

    f.putEncoder("enc_a", (dx + k_a * dtheta) / r, 1005, 2);
    f.putEncoder("enc_b", (dx + k_b * dtheta) / r, 1005, 2);
    f.putEncoder("enc_c", (dy + k_c * dtheta) / r, 1005, 2);
    EXPECT_EQ(odom.run(f.input(), artifacts), FunctionStatus::kOk);

    const auto it = artifacts.find(ArtifactId{"motion"});
    EXPECT_NE(it, artifacts.end());
    const PlanarMotionDelta* delta = it->second.payload.get<PlanarMotionDelta>();
    EXPECT_NE(delta, nullptr);
    return delta != nullptr ? *delta : PlanarMotionDelta{};
}

} // namespace

TEST(TrackingWheelOdometry, ThreeWheelsSolvePlanarMotion) {
    Fixture     f;
    std::string err;
    auto        odom = f.makeOdometry(threeWheelXml("enc_a", "enc_b", "enc_c"), err);
    ASSERT_NE(odom, nullptr) << err;

    const PlanarMotionDelta delta = solveKnownMotion(f, *odom, 0.05, 0.01, 0.1);
    EXPECT_NEAR(delta.dx_m, 0.05, 1e-9);
    EXPECT_NEAR(delta.dy_m, 0.01, 1e-9);
    EXPECT_NEAR(delta.dtheta_rad, 0.1, 1e-9);
    EXPECT_NEAR(delta.dt_s, 0.005, 1e-12);
}

TEST(TrackingWheelOdometry, SensorIdsAreOpaqueRenamingChangesNothing) {
    // same geometry, sensors called banana/kiwi/mango
    Fixture f;
    f.catalog.add(SensorId{"banana"},
                  PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
    f.catalog.add(SensorId{"kiwi"},
                  PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
    f.catalog.add(SensorId{"mango"},
                  PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));

    std::string err;
    auto        odom = f.makeOdometry(threeWheelXml("banana", "kiwi", "mango"), err);
    ASSERT_NE(odom, nullptr) << err;

    const double r = 0.0254;
    f.putEncoder("banana", 0.0, 1000, 1);
    f.putEncoder("kiwi", 0.0, 1000, 1);
    f.putEncoder("mango", 0.0, 1000, 1);
    ArtifactMap artifacts;
    odom->run(f.input(), artifacts);
    f.putEncoder("banana", 0.05 / r, 1005, 2);
    f.putEncoder("kiwi", 0.05 / r, 1005, 2);
    f.putEncoder("mango", 0.0, 1005, 2);
    odom->run(f.input(), artifacts);

    const PlanarMotionDelta* delta =
        artifacts.at(ArtifactId{"motion"}).payload.get<PlanarMotionDelta>();
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->dx_m, 0.05, 1e-9);
    EXPECT_NEAR(delta->dtheta_rad, 0.0, 1e-9);
}

TEST(TrackingWheelOdometry, LabelsDoNotAffectBehaviorGeometryDoes) {
    Fixture     f;
    std::string err;

    // identical geometry, different label
    auto odom_label_a =
        f.makeOdometry(threeWheelXml("enc_a", "enc_b", "enc_c", "left"), err);
    ASSERT_NE(odom_label_a, nullptr) << err;
    Fixture f2;
    auto    odom_label_b =
        f2.makeOdometry(threeWheelXml("enc_a", "enc_b", "enc_c", "port_side"), err);
    ASSERT_NE(odom_label_b, nullptr) << err;

    const PlanarMotionDelta from_a = solveKnownMotion(f, *odom_label_a, 0.02, 0.0, 0.05);
    const PlanarMotionDelta from_b = solveKnownMotion(f2, *odom_label_b, 0.02, 0.0, 0.05);
    EXPECT_NEAR(from_a.dtheta_rad, from_b.dtheta_rad, 1e-12);

    // changed geometry, same sensor data: different answer
    Fixture     f3;
    std::string wide = threeWheelXml("enc_a", "enc_b", "enc_c");
    const auto  pos  = wide.find("position_y_m=\"0.13\"");
    ASSERT_NE(pos, std::string::npos);
    wide.replace(pos, 20, "position_y_m=\"0.26\"");
    auto odom_wide = f3.makeOdometry(wide, err);
    ASSERT_NE(odom_wide, nullptr) << err;

    // feed the same encoder readings that meant dtheta 0.05 for the narrow rig
    const double k_a = -0.13, k_b = 0.13, k_c = -0.12, r = 0.0254;
    f3.putEncoder("enc_a", 0.0, 1000, 1);
    f3.putEncoder("enc_b", 0.0, 1000, 1);
    f3.putEncoder("enc_c", 0.0, 1000, 1);
    ArtifactMap artifacts;
    odom_wide->run(f3.input(), artifacts);
    f3.putEncoder("enc_a", (0.02 + k_a * 0.05) / r, 1005, 2);
    f3.putEncoder("enc_b", (0.02 + k_b * 0.05) / r, 1005, 2);
    f3.putEncoder("enc_c", (k_c * 0.05) / r, 1005, 2);
    odom_wide->run(f3.input(), artifacts);
    const PlanarMotionDelta* delta =
        artifacts.at(ArtifactId{"motion"}).payload.get<PlanarMotionDelta>();
    ASSERT_NE(delta, nullptr);
    // the wider base reads the same counts as meaningfully less turn
    EXPECT_GT(std::fabs(delta->dtheta_rad - 0.05), 0.01);
}

TEST(TrackingWheelOdometry, TwoWheelsNeedHeadingConstraint) {
    Fixture     f;
    std::string err;

    // without a constraint: cannot observe rotation
    EXPECT_EQ(f.makeOdometry(R"(
<Preprocessor id="m" type="tracking_wheel_odometry">
    <TrackingWheel sensor_id="enc_a" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="0"/>
    <TrackingWheel sensor_id="enc_b" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="90"/>
    <Output artifact_id="motion"/>
</Preprocessor>)",
                             err),
              nullptr);
    EXPECT_NE(err.find("HeadingConstraint"), std::string::npos);

    // with the constraint it works, including bias calibration
    auto odom = f.makeOdometry(R"(
<Preprocessor id="m" type="tracking_wheel_odometry">
    <TrackingWheel sensor_id="enc_a" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="0"/>
    <TrackingWheel sensor_id="enc_b" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="90"/>
    <HeadingConstraint sensor_id="imu" bias_samples="2"/>
    <Output artifact_id="motion"/>
</Preprocessor>)",
                               err);
    ASSERT_NE(odom, nullptr) << err;

    ArtifactMap artifacts;
    // calibration: two stationary imu readings with bias 0.02 rad/s
    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putImu(0.02, 1000, 1);
    odom->run(f.input(), artifacts);
    f.putImu(0.02, 1005, 2);
    odom->run(f.input(), artifacts);
    EXPECT_TRUE(artifacts.empty());

    // motion: forward 0.01 m while the gyro reads bias only (no rotation)
    f.putEncoder("enc_a", 0.01 / 0.0254, 1010, 2);
    f.putEncoder("enc_b", 0.0, 1010, 2);
    f.putImu(0.02, 1010, 3);   // first post-cal sample seeds the integrator
    odom->run(f.input(), artifacts);
    f.putEncoder("enc_a", 0.02 / 0.0254, 1015, 3);
    f.putEncoder("enc_b", 0.0, 1015, 3);
    f.putImu(0.02, 1015, 4);
    EXPECT_EQ(odom->run(f.input(), artifacts), FunctionStatus::kOk);

    const PlanarMotionDelta* delta =
        artifacts.at(ArtifactId{"motion"}).payload.get<PlanarMotionDelta>();
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->dtheta_rad, 0.0, 1e-9);   // bias removed
    EXPECT_GT(delta->dx_m, 0.015);               // accumulated travel, none lost
}

TEST(TrackingWheelOdometry, ConfigurationErrors) {
    Fixture     f;
    std::string err;

    // wrong payload type: an imu sensor on a tracking wheel
    EXPECT_EQ(f.makeOdometry(threeWheelXml("imu", "enc_b", "enc_c"), err), nullptr);
    EXPECT_NE(err.find("different payload"), std::string::npos);
    EXPECT_NE(err.find(payload_names::kImuSample), std::string::npos);

    // unknown sensor id
    EXPECT_EQ(f.makeOdometry(threeWheelXml("ghost", "enc_b", "enc_c"), err), nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);

    // duplicate sensor reference
    EXPECT_EQ(f.makeOdometry(threeWheelXml("enc_a", "enc_a", "enc_c"), err), nullptr);
    EXPECT_NE(err.find("more than one TrackingWheel"), std::string::npos);

    // zero radius
    std::string bad = threeWheelXml("enc_a", "enc_b", "enc_c");
    bad.replace(bad.find("radius_m=\"0.0254\""), 17, "radius_m=\"0\"");
    EXPECT_EQ(f.makeOdometry(bad, err), nullptr);
    EXPECT_NE(err.find("radius_m"), std::string::npos);

    // NaN geometry
    bad = threeWheelXml("enc_a", "enc_b", "enc_c");
    bad.replace(bad.find("position_y_m=\"0.13\""), 19, "position_y_m=\"nan\"");
    EXPECT_EQ(f.makeOdometry(bad, err), nullptr);
    EXPECT_NE(err.find("invalid value"), std::string::npos);

    // bad direction enum
    bad = threeWheelXml("enc_a", "enc_b", "enc_c");
    bad.insert(bad.find("/>"), " direction=\"sideways\"");
    EXPECT_EQ(f.makeOdometry(bad, err), nullptr);
    EXPECT_NE(err.find("direction"), std::string::npos);
}

TEST(ConfiguredCollection, DuplicateArtifactOutputsFail) {
    Fixture     f;
    std::string err;
    PreprocessorInitializationContext context;
    context.sensors   = &f.catalog;
    context.functions = &f.functions;

    const ConfigNode node = f.parse(R"(
<Preprocessing type="configured_collection">
    <Preprocessor id="a" type="imu_normalization">
        <Input sensor_id="imu"/><Output artifact_id="same"/>
    </Preprocessor>
    <Preprocessor id="b" type="imu_normalization">
        <Input sensor_id="imu"/><Output artifact_id="same"/>
    </Preprocessor>
</Preprocessing>)");
    EXPECT_EQ(ConfiguredCollection::create(node, context, err), nullptr);
    EXPECT_NE(err.find("duplicate artifact output id"), std::string::npos);
}

TEST(ConfiguredCollection, EmptyCollectionIsAnError) {
    Fixture     f;
    std::string err;
    PreprocessorInitializationContext context;
    context.sensors   = &f.catalog;
    context.functions = &f.functions;

    const ConfigNode node =
        f.parse(R"(<Preprocessing type="configured_collection"/>)");
    EXPECT_EQ(ConfiguredCollection::create(node, context, err), nullptr);
    EXPECT_NE(err.find("noop"), std::string::npos);
}

TEST(ImuNormalization, OutageGapReseedsInsteadOfIntegrating) {
    Fixture     f;
    std::string err;
    PreprocessorInitializationContext context;
    context.sensors   = &f.catalog;
    context.functions = &f.functions;

    auto fn = ImuNormalization::create(f.parse(R"(
<Preprocessor id="imu_normalization" type="imu_normalization">
    <Input sensor_id="imu"/>
    <Calibration bias_samples="0" max_gap_ms="250"/>
    <Output artifact_id="orientation"/>
</Preprocessor>)"),
                                       context, err);
    ASSERT_NE(fn, nullptr) << err;

    ArtifactMap artifacts;
    f.putImu(1.0, 0, 1);   // seed
    fn->run(f.input(), artifacts);
    f.putImu(1.0, 5, 2);
    fn->run(f.input(), artifacts);
    EXPECT_EQ(artifacts.count(ArtifactId{"orientation"}), 1u);
    artifacts.clear();

    // a five second outage: integrating across it would be garbage
    f.putImu(1.0, 5005, 3);
    EXPECT_EQ(fn->run(f.input(), artifacts), FunctionStatus::kOk);
    EXPECT_TRUE(artifacts.empty());   // reseeded, interval dropped

    f.putImu(1.0, 5010, 4);   // normal cadence resumes
    fn->run(f.input(), artifacts);
    const ImuDelta* delta =
        artifacts.at(ArtifactId{"orientation"}).payload.get<ImuDelta>();
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->delta_rad, 1.0 * 0.005, 1e-12);
}

TEST(TrackingWheelOdometry, UnhealthySourcesAreNotConsumed) {
    Fixture     f;
    std::string err;
    auto        odom = f.makeOdometry(threeWheelXml("enc_a", "enc_b", "enc_c"), err);
    ASSERT_NE(odom, nullptr) << err;

    ArtifactMap artifacts;
    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putEncoder("enc_c", 0.0, 1000, 1);
    odom->run(f.input(), artifacts);

    // one wheel goes into fault while its record still advances; the
    // preprocessor must not act on an unhealthy source
    f.putEncoder("enc_a", 1.0, 1005, 2);
    f.putEncoder("enc_b", 1.0, 1005, 2);
    f.putEncoder("enc_c", 1.0, 1005, 2);
    f.results[SensorId{"enc_c"}].state = SensorState::kFault;

    odom->run(f.input(), artifacts);
    EXPECT_TRUE(artifacts.empty());   // no solve without every healthy input
}

TEST(ImuNormalization, CalibratesThenIntegrates) {
    Fixture     f;
    std::string err;
    PreprocessorInitializationContext context;
    context.sensors   = &f.catalog;
    context.functions = &f.functions;

    auto fn = ImuNormalization::create(f.parse(R"(
<Preprocessor id="imu_normalization" type="imu_normalization">
    <Input sensor_id="imu"/>
    <Calibration bias_samples="2"/>
    <Output artifact_id="orientation"/>
</Preprocessor>)"),
                                       context, err);
    ASSERT_NE(fn, nullptr) << err;

    ArtifactMap artifacts;
    f.putImu(0.5, 0, 1);
    fn->run(f.input(), artifacts);
    f.putImu(0.3, 5, 2);   // bias average 0.4
    fn->run(f.input(), artifacts);
    EXPECT_TRUE(artifacts.empty());

    f.putImu(0.4 + 1.0, 10, 3);   // seeds the integrator at 1 rad/s
    fn->run(f.input(), artifacts);
    EXPECT_TRUE(artifacts.empty());

    f.putImu(0.4 + 1.0, 15, 4);   // steady 1 rad/s for 5 ms
    EXPECT_EQ(fn->run(f.input(), artifacts), FunctionStatus::kOk);

    const ImuDelta* delta =
        artifacts.at(ArtifactId{"orientation"}).payload.get<ImuDelta>();
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->rate_rad_s, 1.0, 1e-12);
    EXPECT_NEAR(delta->delta_rad, 1.0 * 0.005, 1e-12);
}
