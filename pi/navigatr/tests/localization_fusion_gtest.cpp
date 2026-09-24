// localization_fusion_gtest.cpp
// weighted_planar_fusion through its contract with hand-built
// observations, then through make_localization with the real wheel and
// gyro models, then the checked-in synthetic fusion demo end to end.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "common/frame_codec.h"
#include "core/diagnostics.h"
#include "impl/localization/weighted_planar_fusion.h"
#include "impl/resources/serial_links.h"
#include "impl/resources/synthetic_rig.h"
#include "math/angles.h"
#include "payloads/field_object_evidence.h"
#include "payloads/robot_observations.h"
#include "payloads/sensor_samples.h"
#include "runtime/localization_stage.h"
#include "runtime/register_all.h"
#include "runtime/sensor_catalog.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

// one second steps and flat noise floors make the expected numbers exact:
// var_d = 1e-6, var_w = 4e-4, var_g = 1e-4
const char* kDefaultXml = R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
    </Motion>
    <Heading observation_id="heading" max_wait_ms="50">
        <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
    </Heading>
    <Attitude observation_id="attitude" max_age_ms="50"/>
</Estimator>)";

const char* kMotionOnlyXml = R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
    </Motion>
</Estimator>)";

constexpr double kVarD = 1e-6;
constexpr double kVarW = 4e-4;
constexpr double kVarG = 1e-4;

StateEstimatorInitializationContext declaredContext() {
    StateEstimatorInitializationContext context;
    context.observations = {
        RobotObservationOutputDecl{ObservationId{"motion"},
                                   PayloadDescriptor::of<BodyMotionIncrement>(
                                       payload_names::kBodyMotionIncrement)},
        RobotObservationOutputDecl{ObservationId{"heading"},
                                   PayloadDescriptor::of<HeadingIncrement>(
                                       payload_names::kHeadingIncrement)},
        RobotObservationOutputDecl{ObservationId{"attitude"},
                                   PayloadDescriptor::of<AttitudeObservation>(
                                       payload_names::kAttitudeObservation)}};
    return context;
}

std::unique_ptr<StateEstimator> makeEstimator(const char* xml, std::string& err) {
    tinyxml2::XMLDocument doc;
    EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
    StateEstimatorInitializationContext context = declaredContext();
    return WeightedPlanarFusion::create(ConfigNode{doc.RootElement()}, context, err);
}

struct Fixture {
    RobotObservationMap  observations;
    LocalizationRequests requests;
    RobotState           previous;
    Diagnostics          diagnostics;
    int64_t              now_ms = 2000;

    std::unique_ptr<StateEstimator> estimator;

    explicit Fixture(const char* xml = kDefaultXml) {
        std::string err;
        estimator = makeEstimator(xml, err);
        EXPECT_NE(estimator, nullptr) << err;
    }

    void putMotion(double dx, double dy, double dtheta, int64_t start_ms = 0,
                   int64_t end_ms = 1000, const char* source = "enc", bool rotation = true,
                   bool coupling = false, double jx = 0.0, double jy = 0.0,
                   const char* clock = "pico") {
        BodyMotionIncrement m;
        m.dx_m                  = dx;
        m.dy_m                  = dy;
        m.dtheta_rad            = dtheta;
        m.has_rotation          = rotation;
        m.startAt               = deviceTime(start_ms);
        m.endAt                 = deviceTime(end_ms);
        m.dt_s                  = (end_ms - start_ms) / 1000.0;
        m.has_rotation_coupling = coupling;
        m.dx_per_dtheta_m_rad   = jx;
        m.dy_per_dtheta_m_rad   = jy;
        Provenance p;
        p.source = source;
        p.clock  = clock;
        m.sources.push_back(p);
        RobotObservationRecord record;
        record.measuredAt = m.endAt;
        record.receivedAt = hostTime(now_ms);
        record.payload    = TypedPayload::store(m, payload_names::kBodyMotionIncrement);
        observations[ObservationId{"motion"}] = std::move(record);
    }

    void putHeading(double dtheta, int64_t start_ms = 0, int64_t end_ms = 1000,
                    const char* source = "imu", const char* clock = "pico",
                    uint64_t epoch = 0) {
        HeadingIncrement h;
        h.dtheta_rad = dtheta;
        h.startAt    = deviceTime(start_ms);
        h.endAt      = deviceTime(end_ms);
        h.dt_s       = (end_ms - start_ms) / 1000.0;
        Provenance p;
        p.source = source;
        p.clock  = clock;
        p.epoch  = epoch;
        h.sources.push_back(p);
        RobotObservationRecord record;
        record.measuredAt = h.endAt;
        record.payload    = TypedPayload::store(h, payload_names::kHeadingIncrement);
        observations[ObservationId{"heading"}] = std::move(record);
    }

    void putAttitude(double roll_rad, int64_t at_ms) {
        AttitudeObservation a;
        a.q_reference_body = quaternionFromEuler(roll_rad, 0.0, 1.5);   // source yaw is noise
        a.reference        = "gravity";
        a.has_yaw          = false;
        a.measuredAt       = hostTime(at_ms);
        a.quality          = 1.0;
        a.source.source    = "attitude";
        RobotObservationRecord record;
        record.measuredAt = a.measuredAt;
        record.payload    = TypedPayload::store(a, payload_names::kAttitudeObservation);
        observations[ObservationId{"attitude"}] = std::move(record);
    }

    StateEstimatorOutput run() {
        ExecutionContext           context{hostTime(now_ms), 1, &diagnostics};
        const StateEstimatorOutput out =
            estimator->run({observations, previous, requests, context});
        previous = out.robot;
        observations.clear();
        ++now_ms;
        return out;
    }
};

bool contains(const std::vector<ObservationId>& ids, const char* id) {
    for (const ObservationId& i : ids) {
        if (i.value == id) return true;
    }
    return false;
}

// chord of ((dx, dy), dtheta), as the estimator header states it
void chord(double dx, double dy, double dtheta, double& lx, double& ly) {
    const double s = std::sin(dtheta) / dtheta;
    const double c = (1.0 - std::cos(dtheta)) / dtheta;
    lx             = dx * s - dy * c;
    ly             = dx * c + dy * s;
}

void expectSymmetricPositive(const PoseCovariance& c) {
    EXPECT_GE(c.xx, 0.0);
    EXPECT_GE(c.yy, 0.0);
    EXPECT_GE(c.hh, 0.0);
    EXPECT_GE(c.xx * c.yy - c.xy * c.xy, -1e-18);
    EXPECT_GE(c.yy * c.hh - c.yh * c.yh, -1e-18);
    EXPECT_GE(c.xx * c.hh - c.xh * c.xh, -1e-18);
}

} // namespace

TEST(WeightedPlanarFusion, BuildRequiresExplicitNoiseAndKnownReferences) {
    std::string err;
    EXPECT_NE(makeEstimator(kDefaultXml, err), nullptr) << err;
    EXPECT_NE(makeEstimator(kMotionOnlyXml, err), nullptr) << err;

    // no Noise element
    EXPECT_EQ(makeEstimator(R"(<Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion"/></Estimator>)",
                            err),
              nullptr);
    EXPECT_NE(err.find("Noise"), std::string::npos);

    // a missing attribute is an error, never a default
    EXPECT_EQ(makeEstimator(R"(<Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion">
            <Noise translation_floor_m="0.001" rotation_floor_rad="0.02"/>
        </Motion></Estimator>)",
                            err),
              nullptr);
    EXPECT_NE(err.find("translation_per_m"), std::string::npos);

    // zero floors are refused
    EXPECT_EQ(makeEstimator(R"(<Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion">
            <Noise translation_floor_m="0" translation_per_m="0"
                   rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
        </Motion></Estimator>)",
                            err),
              nullptr);
    EXPECT_NE(err.find("positive"), std::string::npos);

    // negative noise is refused
    EXPECT_EQ(makeEstimator(R"(<Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion">
            <Noise translation_floor_m="0.001" translation_per_m="-1"
                   rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
        </Motion></Estimator>)",
                            err),
              nullptr);
    EXPECT_NE(err.find("negative"), std::string::npos);

    // the gyro needs its random walk
    EXPECT_EQ(makeEstimator(R"(<Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion">
            <Noise translation_floor_m="0.001" translation_per_m="0"
                   rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
        </Motion>
        <Heading observation_id="heading">
            <Noise angle_random_walk_rad_per_sqrt_s="0" bias_rad_per_s="0.1"/>
        </Heading></Estimator>)",
                            err),
              nullptr);
    EXPECT_NE(err.find("angle_random_walk"), std::string::npos);

    // unknown references and elements
    EXPECT_EQ(makeEstimator(R"(<Estimator type="weighted_planar_fusion">
        <Motion observation_id="ghost">
            <Noise translation_floor_m="0.001" translation_per_m="0"
                   rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
        </Motion></Estimator>)",
                            err),
              nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);
    EXPECT_EQ(makeEstimator(R"(<Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion">
            <Noise translation_floor_m="0.001" translation_per_m="0"
                   rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
        </Motion><Landmarks observation_id="motion"/></Estimator>)",
                            err),
              nullptr);
    EXPECT_NE(err.find("Landmarks"), std::string::npos);
}

TEST(WeightedPlanarFusion, LandmarkEvidenceIsNotARobotObservation) {
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(kMotionOnlyXml), tinyxml2::XML_SUCCESS);
    StateEstimatorInitializationContext context;
    context.observations = {RobotObservationOutputDecl{
        ObservationId{"motion"}, PayloadDescriptor::of<FieldObjectPoseEvidenceSet>(
                                     payload_names::kFieldObjectPoseEvidenceSet)}};
    std::string err;
    EXPECT_EQ(WeightedPlanarFusion::create(ConfigNode{doc.RootElement()}, context, err),
              nullptr);
    EXPECT_NE(err.find("different payload"), std::string::npos);
}

TEST(WeightedPlanarFusion, RotationIsThePrecisionWeightedMean) {
    Fixture f;
    f.putMotion(0.0, 0.0, 0.5);
    f.putHeading(0.25);
    const StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kOk);
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.accepted, "motion"));
    EXPECT_TRUE(contains(out.accepted, "heading"));
    EXPECT_TRUE(out.rejected.empty());
    // w = 4e-4 / (4e-4 + 1e-4) = 0.8: dtheta = 0.5 + 0.8 * (0.25 - 0.5)
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.3, 1e-12);
    ASSERT_TRUE(out.robot.has_covariance);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW * kVarG / (kVarW + kVarG), 1e-15);
    EXPECT_NE(out.diagnostic.find("gyro weight 0.800"), std::string::npos) << out.diagnostic;
    EXPECT_NEAR(out.robot.yaw_rate_rad_s, 0.3, 1e-12);
}

TEST(WeightedPlanarFusion, InfluenceFollowsUncertainty) {
    // a nearly exact gyro owns the rotation
    Fixture precise(R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
    </Motion>
    <Heading observation_id="heading">
        <Noise angle_random_walk_rad_per_sqrt_s="0.0001" bias_rad_per_s="0"/>
    </Heading>
</Estimator>)");
    precise.putMotion(0.0, 0.0, 0.5);
    precise.putHeading(0.25);
    EXPECT_NEAR(precise.run().robot.odom_pose.heading_rad, 0.25, 1e-4);

    // a noisy gyro barely moves the wheel answer
    Fixture noisy(R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
    </Motion>
    <Heading observation_id="heading">
        <Noise angle_random_walk_rad_per_sqrt_s="1" bias_rad_per_s="0"/>
    </Heading>
</Estimator>)");
    noisy.putMotion(0.0, 0.0, 0.5);
    noisy.putHeading(0.25);
    EXPECT_NEAR(noisy.run().robot.odom_pose.heading_rad, 0.5, 1e-3);

    // distance dependent wheel noise: a long step trusts the gyro more
    Fixture per_m(R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.001" rotation_per_rad="0" rotation_per_m="0.1"/>
    </Motion>
    <Heading observation_id="heading">
        <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
    </Heading>
</Estimator>)");
    per_m.putMotion(0.0, 0.0, 0.5);
    per_m.putHeading(0.25);
    const double short_step = per_m.run().robot.odom_pose.heading_rad;   // near 0.5
    Fixture per_m_long(R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.001" rotation_per_rad="0" rotation_per_m="0.1"/>
    </Motion>
    <Heading observation_id="heading">
        <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
    </Heading>
</Estimator>)");
    per_m_long.putMotion(2.0, 0.0, 0.5);   // 2 m of travel: wheel sigma 0.2 rad
    per_m_long.putHeading(0.25);
    const double long_step = per_m_long.run().robot.odom_pose.heading_rad;
    EXPECT_GT(short_step, 0.45);
    EXPECT_LT(long_step, 0.26);
}

TEST(WeightedPlanarFusion, CouplingReattributesTranslationWhenRotationIsRevised) {
    // wheels read a pure spin; the gyro says it was smaller, so the
    // rear wheel travel that was blamed on rotation becomes translation
    Fixture f;
    f.putMotion(0.0, 0.0, 0.5, 0, 1000, "enc", true, true, 0.0, 0.12);
    f.putHeading(0.25);
    const StateEstimatorOutput out = f.run();
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.3, 1e-12);
    double lx = 0.0, ly = 0.0;
    chord(0.0, 0.12 * (0.3 - 0.5), 0.3, lx, ly);
    EXPECT_NEAR(out.robot.odom_pose.x_m, lx, 1e-12);
    EXPECT_NEAR(out.robot.odom_pose.y_m, ly, 1e-12);
    // the revision carries its own uncertainty into the lateral axis
    const PoseCovariance& c = out.robot.odom_covariance;
    EXPECT_GT(c.yy, c.xx);
    EXPECT_GT(c.yy, kVarD);
    expectSymmetricPositive(c);

    // without a coupling the published translation stands
    Fixture g;
    g.putMotion(0.0, 0.0, 0.5);
    g.putHeading(0.25);
    const StateEstimatorOutput plain = g.run();
    EXPECT_NEAR(plain.robot.odom_pose.x_m, 0.0, 1e-12);
    EXPECT_NEAR(plain.robot.odom_pose.y_m, 0.0, 1e-12);
}

TEST(WeightedPlanarFusion, PartialMotionTakesRotationFromTheHeading) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.0, 0, 1000, "enc", false, true, 0.0, 0.12);
    StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kNoData);   // nothing fabricated
    EXPECT_FALSE(out.advanced);
    EXPECT_FALSE(out.robot.has_covariance);
    EXPECT_FALSE(contains(out.rejected, "motion"));   // stays pending

    f.putMotion(0.1, 0.0, 0.0, 0, 1000, "enc", false, true, 0.0, 0.12);
    f.putHeading(0.5);
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.5, 1e-12);
    double lx = 0.0, ly = 0.0;
    chord(0.1, 0.12 * 0.5, 0.5, lx, ly);
    EXPECT_NEAR(out.robot.odom_pose.x_m, lx, 1e-12);
    EXPECT_NEAR(out.robot.odom_pose.y_m, ly, 1e-12);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarG, 1e-15);
    EXPECT_NE(out.diagnostic.find("heading only"), std::string::npos);
    expectSymmetricPositive(out.robot.odom_covariance);
}

TEST(WeightedPlanarFusion, CovarianceGrowsWithHeadingCoupledIntoPosition) {
    Fixture      f(kMotionOnlyXml);
    const double g = 0.1;   // straight steps along +x
    // reference recurrence: per step Qu = diag(a, a + (g/2)^2 b, b) with
    // cross (g/2) b from the chord, then P' = F P F' + Qu with F carrying
    // the heading to lateral coupling g
    double xx = 0.0, yy = 0.0, yh = 0.0, hh = 0.0;
    for (int i = 0; i < 6; ++i) {
        f.putMotion(g, 0.0, 0.0, 1000 * i, 1000 * (i + 1));
        const StateEstimatorOutput out = f.run();
        ASSERT_TRUE(out.advanced);
        const double half = g / 2.0;
        yy = yy + 2.0 * g * yh + g * g * hh + kVarD + half * half * kVarW;
        yh = yh + g * hh + half * kVarW;
        hh = hh + kVarW;
        xx = xx + kVarD;
        const PoseCovariance& c = out.robot.odom_covariance;
        EXPECT_NEAR(c.xx, xx, 1e-15) << i;
        EXPECT_NEAR(c.yy, yy, 1e-15) << i;
        EXPECT_NEAR(c.yh, yh, 1e-15) << i;
        EXPECT_NEAR(c.hh, hh, 1e-15) << i;
        EXPECT_NEAR(c.xy, 0.0, 1e-15) << i;
        EXPECT_NEAR(c.xh, 0.0, 1e-15) << i;
        expectSymmetricPositive(c);
    }
    // lateral uncertainty outgrows the pure translation noise
    EXPECT_GT(f.previous.odom_covariance.yy, 6.0 * kVarD * 10.0);
    EXPECT_NEAR(f.previous.odom_pose.x_m, 0.6, 1e-12);
}

TEST(WeightedPlanarFusion, RotatedFrameKeepsTheCovarianceConsistent) {
    // the same step taken at heading pi/2 puts the lateral growth on x
    Fixture f(kMotionOnlyXml);
    f.previous.odom_pose.heading_rad = kPi / 2.0;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    f.putMotion(0.1, 0.0, 0.0, 1000, 2000);
    const StateEstimatorOutput out = f.run();
    EXPECT_NEAR(out.robot.odom_pose.y_m, 0.2, 1e-12);
    EXPECT_GT(out.robot.odom_covariance.xx, out.robot.odom_covariance.yy);
    expectSymmetricPositive(out.robot.odom_covariance);
}

TEST(WeightedPlanarFusion, SharedSourceIsNeverCountedTwice) {
    Fixture f;
    f.putMotion(0.0, 0.0, 0.5, 0, 1000, "imu");   // the wheel model folded this gyro
    f.putHeading(0.25, 0, 1000, "imu");
    const StateEstimatorOutput out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.accepted, "motion"));
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NE(out.diagnostic.find("not counted twice"), std::string::npos);
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.5, 1e-12);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW, 1e-15);
}

TEST(WeightedPlanarFusion, MismatchedIntervalsAreNotFused) {
    Fixture f;
    f.putMotion(0.0, 0.0, 0.5);
    f.putHeading(0.25, 60, 65);   // does not start the motion window: discarded
    StateEstimatorOutput out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NE(out.diagnostic.find("does not continue"), std::string::npos);
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.5, 1e-12);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW, 1e-15);

    f.putMotion(0.0, 0.0, 0.5, 1000, 2000);
    f.putHeading(0.25, 1000, 2500);   // overruns the window: can never align
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NE(out.diagnostic.find("overruns"), std::string::npos);
    EXPECT_FALSE(contains(out.accepted, "heading"));
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 1.0, 1e-12);
}

TEST(WeightedPlanarFusion, RepeatedSampleIsConsumedExactlyOnce) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.2);
    f.putHeading(0.2);
    f.run();
    const RobotState after_first = f.previous;
    f.putMotion(0.1, 0.0, 0.2);   // identical stamps offered again
    f.putHeading(0.2);
    const StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_FALSE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "motion"));
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NEAR(out.robot.odom_pose.x_m, after_first.odom_pose.x_m, 1e-15);
    EXPECT_NEAR(out.robot.odom_covariance.hh, after_first.odom_covariance.hh, 1e-18);
    EXPECT_NEAR(out.robot.odom_covariance.yy, after_first.odom_covariance.yy, 1e-18);
}

TEST(WeightedPlanarFusion, GyroDropoutGrowsHeadingUncertaintyFaster) {
    Fixture with;
    Fixture without;
    for (int i = 0; i < 3; ++i) {
        with.putMotion(0.1, 0.0, 0.1, 1000 * i, 1000 * (i + 1));
        with.putHeading(0.1, 1000 * i, 1000 * (i + 1));
        with.run();
        without.putMotion(0.1, 0.0, 0.1, 1000 * i, 1000 * (i + 1));
        if (i != 1) {
            without.putHeading(0.1, 1000 * i, 1000 * (i + 1));
        }
        const StateEstimatorOutput out = without.run();
        EXPECT_TRUE(out.advanced) << i;   // the wheels still carry the step
    }
    const double fused = kVarW * kVarG / (kVarW + kVarG);
    EXPECT_NEAR(with.previous.odom_covariance.hh, 3.0 * fused, 1e-15);
    EXPECT_NEAR(without.previous.odom_covariance.hh, 2.0 * fused + kVarW, 1e-15);
    EXPECT_GT(without.previous.odom_covariance.hh, with.previous.odom_covariance.hh);
    EXPECT_NEAR(with.previous.odom_pose.heading_rad, without.previous.odom_pose.heading_rad,
                1e-12);
}

TEST(WeightedPlanarFusion, HoldsWithoutMotionAndRejectsBadIntervals) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_FALSE(out.advanced);
    EXPECT_EQ(out.robot.measuredAt.ms, 1000);
    EXPECT_NEAR(out.robot.odom_covariance.xx, kVarD, 1e-15);

    f.putMotion(0.1, 0.0, 0.0, 1000, 1000);
    out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kFault);
    EXPECT_TRUE(contains(out.rejected, "motion"));
    EXPECT_NEAR(out.robot.odom_covariance.xx, kVarD, 1e-15);
}

TEST(WeightedPlanarFusion, SourceTimeRegressionAdvancesTheEpoch) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    const uint64_t epoch = f.previous.odometry_epoch;
    f.putMotion(0.1, 0.0, 0.0, 10, 15);
    f.putHeading(0.0, 10, 15);
    const StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kFault);
    EXPECT_EQ(out.robot.odometry_epoch, epoch + 1);
    EXPECT_TRUE(contains(out.rejected, "motion"));
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NEAR(out.robot.odom_pose.x_m, 0.1, 1e-12);
}

TEST(WeightedPlanarFusion, ResetRestartsAtAnExactOrigin) {
    Fixture f;
    f.requests.placement.requested = true;
    f.requests.placement.origin    = "configuration";
    f.requests.placement.sequence  = 1;
    f.requests.placement.pose      = Pose2D{1.0, 2.0, 0.0};
    f.putMotion(0.1, 0.0, 0.1);
    f.putHeading(0.1);
    f.putAttitude(degToRad(5.0), f.now_ms);
    StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.robot.anchor_revision, 1u);
    EXPECT_TRUE(out.robot.attitude.valid);
    EXPECT_TRUE(out.robot.has_covariance);

    // the stage resets the estimator and hands it a fresh state in the
    // next odometry epoch
    f.estimator->reset();
    const uint64_t next_epoch = f.previous.odometry_epoch + 1;
    f.previous                = RobotState{};
    f.previous.odometry_epoch = next_epoch;
    out                       = f.run();   // same placement request: a new edge after reset
    EXPECT_EQ(out.robot.anchor_revision, 1u);
    EXPECT_TRUE(out.robot.initialized);
    EXPECT_FALSE(out.robot.has_covariance);
    EXPECT_FALSE(out.robot.attitude.valid);   // retained attitude is gone
    EXPECT_TRUE(out.robot.attitude.assumed_level);

    f.putMotion(0.1, 0.0, 0.1);
    f.putHeading(0.1);
    out = f.run();
    EXPECT_TRUE(out.robot.has_covariance);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW * kVarG / (kVarW + kVarG), 1e-15);
    EXPECT_NEAR(out.robot.fieldPose().x_m, 1.0 + out.robot.odom_pose.x_m, 1e-12);
}

TEST(WeightedPlanarFusion, PlacementReanchorsWithoutTouchingTheCovariance) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    const PoseCovariance before = f.previous.odom_covariance;
    f.requests.placement.requested = true;
    f.requests.placement.origin    = "command";
    f.requests.placement.sequence  = 7;
    f.requests.placement.pose      = Pose2D{0.5, 0.5, kPi / 2.0};
    const StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.robot.anchor_revision, 1u);
    EXPECT_NEAR(out.robot.fieldPose().x_m, 0.5, 1e-12);
    EXPECT_NEAR(out.robot.odom_pose.x_m, 0.1, 1e-12);
    EXPECT_NEAR(out.robot.odom_covariance.xx, before.xx, 1e-18);
    EXPECT_NEAR(out.robot.odom_covariance.hh, before.hh, 1e-18);
}

TEST(WeightedPlanarFusion, TiltOnlyAttitudeNeverBecomesYaw) {
    Fixture f;
    f.putMotion(0.0, 0.0, 0.3);
    f.putHeading(0.3);
    f.putAttitude(degToRad(10.0), f.now_ms);
    const StateEstimatorOutput out = f.run();
    ASSERT_TRUE(out.robot.attitude.valid);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    attitudeEuler(out.robot.attitude, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), 10.0, 1e-9);
    EXPECT_NEAR(yaw, 0.3, 1e-9);   // the source yaw of 1.5 was never applied
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.3, 1e-12);
    EXPECT_TRUE(contains(out.accepted, "attitude"));
}

TEST(WeightedPlanarFusion, UnlistedObservationsAreRejectedNotSilentlyDropped) {
    Fixture f(kMotionOnlyXml);
    f.putMotion(0.1, 0.0, 0.0);
    f.putHeading(0.0);   // not configured on this estimator
    const StateEstimatorOutput out = f.run();
    EXPECT_TRUE(contains(out.accepted, "motion"));
    EXPECT_TRUE(contains(out.rejected, "heading"));
}

// ---- through make_localization with the real models -----------------------

namespace
{

struct StageFixture {
    tinyxml2::XMLDocument doc;
    FunctionRegistry      functions;
    SensorCatalog         catalog;
    ResourceStore         store;
    SensorMap             sensors;
    Diagnostics           diagnostics;
    int64_t               now_ms = 1;

    StageFixture() {
        register_localization(functions);
        catalog.add(SensorId{"enc_a"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"enc_b"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"enc_c"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"imu"},
                    PayloadDescriptor::of<ImuSample>(payload_names::kImuSample));
    }

    std::optional<LocalizationExecutor> make(const char* xml, std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        return make_localization(ConfigNode{doc.RootElement()}, functions, catalog, store,
                                 nullptr, err);
    }

    void put(const char* id, TypedPayload payload, int64_t stamp_ms, uint64_t sequence,
             const char* measurement = "") {
        MeasurementRecord record;
        record.state = SourceState::kValid;
        StoredSample stored;
        stored.measuredAt      = deviceTime(stamp_ms);
        stored.receivedAt      = hostTime(stamp_ms + 3);
        stored.sequence        = sequence;
        stored.upstream.clock  = "pico";
        stored.upstream.source = id;
        stored.upstream.measurement = measurement;
        stored.payload         = std::move(payload);
        record.latest          = std::move(stored);
        sensors[SensorId{id}]  = std::move(record);
    }

    // encoder angles for a known body motion on the reference three wheel layout
    void putWheels(double dx, double dy, double dtheta, int64_t stamp_ms,
                   uint64_t sequence) {
        const double k_a = -0.13, k_b = 0.13, k_c = -0.12, r = 0.0254;
        EncoderSample a, b, c;
        a.angle_rad = (dx + k_a * dtheta) / r;
        b.angle_rad = (dx + k_b * dtheta) / r;
        c.angle_rad = (dy + k_c * dtheta) / r;
        put("enc_a", TypedPayload::store(a, payload_names::kEncoderSample), stamp_ms, sequence);
        put("enc_b", TypedPayload::store(b, payload_names::kEncoderSample), stamp_ms, sequence);
        put("enc_c", TypedPayload::store(c, payload_names::kEncoderSample), stamp_ms, sequence);
    }

    void putImu(double rate_rad_s, int64_t stamp_ms, uint64_t sequence) {
        ImuSample s;
        s.yaw_rate_rad_s = rate_rad_s;
        put("imu", TypedPayload::store(s, payload_names::kImuSample), stamp_ms, sequence);
    }

    RobotState step(LocalizationExecutor& exec) {
        ExecutionContext context{hostTime(now_ms++), 1, &diagnostics};
        return exec(sensors, {}, context);
    }
};

// three wheels solve rotation on their own; the gyro is a separate,
// independent contributor
const char* kThreeWheelPlusGyro = R"(
<Localization>
    <Observation id="wheels" type="tracking_wheel_motion">
        <TrackingWheel sensor_id="enc_a" label="left" radius_m="0.0254" position_x_m="0"
                       position_y_m="0.13" measurement_angle_deg="0" direction="positive"/>
        <TrackingWheel sensor_id="enc_b" label="right" radius_m="0.0254" position_x_m="0"
                       position_y_m="-0.13" measurement_angle_deg="0" direction="positive"/>
        <TrackingWheel sensor_id="enc_c" label="rear" radius_m="0.0254" position_x_m="-0.12"
                       position_y_m="0" measurement_angle_deg="90" direction="positive"/>
        <Output observation_id="motion"/>
    </Observation>
    <Observation id="gyro" type="imu_heading_increment">
        <Input sensor_id="imu"/>
        <Calibration bias_samples="0"/>
        <Output observation_id="heading"/>
    </Observation>
    <Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion">
            <Noise translation_floor_m="0.001" translation_per_m="0"
                   rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
        </Motion>
        <Heading observation_id="heading" max_wait_ms="50">
            <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
        </Heading>
    </Estimator>
    <History retention_s="1" capacity="8" max_interpolation_gap_ms="100"/>
</Localization>)";

// two wheels cannot see rotation alone: the constraint folds the gyro
// into the wheel solve, and the separate heading from the same gyro is
// then not independent
const char* kTwoWheelConstrainedPlusSameGyro = R"(
<Localization>
    <Observation id="wheels" type="tracking_wheel_motion">
        <TrackingWheel sensor_id="enc_a" radius_m="0.0254" position_x_m="0"
                       position_y_m="0" measurement_angle_deg="0" direction="positive"/>
        <TrackingWheel sensor_id="enc_c" radius_m="0.0254" position_x_m="0"
                       position_y_m="0" measurement_angle_deg="90" direction="positive"/>
        <HeadingConstraint sensor_id="imu" bias_samples="0"/>
        <Output observation_id="motion"/>
    </Observation>
    <Observation id="gyro" type="imu_heading_increment">
        <Input sensor_id="imu"/>
        <Calibration bias_samples="0"/>
        <Output observation_id="heading"/>
    </Observation>
    <Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion">
            <Noise translation_floor_m="0.001" translation_per_m="0"
                   rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
        </Motion>
        <Heading observation_id="heading" max_wait_ms="50">
            <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
        </Heading>
    </Estimator>
</Localization>)";

} // namespace

TEST(LocalizationFusion, ThreeWheelsAndAnIndependentGyroBlend) {
    StageFixture f;
    std::string  err;
    auto         exec = f.make(kThreeWheelPlusGyro, err);
    ASSERT_TRUE(exec.has_value()) << err;

    // seeds
    f.putWheels(0.0, 0.0, 0.0, 1000, 1);
    f.putImu(20.0, 1000, 1);
    RobotState r = f.step(*exec);
    EXPECT_FALSE(r.valid);

    // both agree on 0.1 rad over 5 ms (trapezoid of 20 rad/s): the fused
    // rotation is exactly that, and the wheel model published its coupling
    f.putWheels(0.05, 0.0, 0.1, 1005, 2);
    f.putImu(20.0, 1005, 2);
    r = f.step(*exec);
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(r.odom_pose.heading_rad, 0.1, 1e-9);
    ASSERT_TRUE(r.has_covariance);
    const double var_g = 0.01 * 0.01 * 0.005;
    EXPECT_NEAR(r.odom_covariance.hh, 4e-4 * var_g / (4e-4 + var_g), 1e-15);
    EXPECT_EQ(exec->feed()->status().updates, 1u);
    {
        const auto& offered = exec->lastObservations();
        const auto  it      = offered.find(ObservationId{"motion"});
        ASSERT_NE(it, offered.end());
        const BodyMotionIncrement* m = it->second.payload.get<BodyMotionIncrement>();
        ASSERT_NE(m, nullptr);
        EXPECT_TRUE(m->has_rotation_coupling);
        EXPECT_NEAR(m->dx_per_dtheta_m_rad, 0.0, 1e-12);
        EXPECT_NEAR(m->dy_per_dtheta_m_rad, 0.12, 1e-12);
    }

    // the wheels now claim 0.1 rad the gyro did not see (a slipping rear
    // wheel); the gyro dominates and the rear travel becomes translation
    f.putWheels(0.10, 0.0, 0.2, 1010, 3);   // cumulative: another 0.05 m, another 0.1 rad
    f.putImu(0.0, 1010, 3);                  // trapezoid 0.5 * (20 + 0) * 0.005 = 0.05
    r = f.step(*exec);
    const double w        = 4e-4 / (4e-4 + var_g);
    EXPECT_NEAR(r.odom_pose.heading_rad, 0.1 + 0.1 + w * (0.05 - 0.1), 1e-9);
    EXPECT_EQ(exec->feed()->status().updates, 2u);
    // every offered observation was settled: the next quiet cycle offers nothing
    r = f.step(*exec);
    EXPECT_EQ(exec->feed()->status().updates, 2u);
    EXPECT_TRUE(exec->lastObservations().empty());
    EXPECT_EQ(f.diagnostics.functions.count("Localization/weighted_planar_fusion"), 1u);
}

TEST(LocalizationFusion, TwoWheelsWithTheConstraintStillWorkAndTheGyroCountsOnce) {
    StageFixture f;
    std::string  err;
    auto         exec = f.make(kTwoWheelConstrainedPlusSameGyro, err);
    ASSERT_TRUE(exec.has_value()) << err;
    const double r_m = 0.0254;

    EncoderSample a, c;
    f.put("enc_a", TypedPayload::store(a, payload_names::kEncoderSample), 1000, 1);
    f.put("enc_c", TypedPayload::store(c, payload_names::kEncoderSample), 1000, 1);
    f.putImu(20.0, 1000, 1);
    f.step(*exec);

    a.angle_rad = 0.05 / r_m;   // 5 cm forward while the gyro reads 20 rad/s
    f.put("enc_a", TypedPayload::store(a, payload_names::kEncoderSample), 1005, 2);
    f.put("enc_c", TypedPayload::store(c, payload_names::kEncoderSample), 1005, 2);
    f.putImu(20.0, 1005, 2);
    const RobotState r = f.step(*exec);
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(r.odom_pose.heading_rad, 0.1, 1e-9);   // from the constraint
    ASSERT_TRUE(r.has_covariance);
    // the rotation variance is the motion's own: the same gyro offered again
    // as a heading did not shrink it
    EXPECT_NEAR(r.odom_covariance.hh, 4e-4, 1e-15);
    EXPECT_EQ(exec->feed()->status().updates, 1u);
}

TEST(LocalizationFusion, CheckedInSyntheticFusionDemoBuildsAndTracksTheRig) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromFile(
        std::string(NAVIGATR_CONFIG_DIR) + "/demo/synthetic_fusion_demo.xml", functions, err);
    ASSERT_NE(system, nullptr) << err;
    EXPECT_EQ(system->localization().estimatorType(), "weighted_planar_fusion");
    auto rig = system->resources().require<const SyntheticRig>(ResourceId{"rig"}, err);
    ASSERT_NE(rig, nullptr) << err;

    // two seconds stationary (gyro bias), then the circle
    int64_t now_ms = 0;
    for (int i = 0; i < 400; ++i) {
        now_ms += 10;
        system->step(hostTime(now_ms));
    }
    const RobotState robot = system->robot();
    ASSERT_TRUE(robot.valid);
    EXPECT_TRUE(robot.initialized);
    ASSERT_TRUE(robot.has_covariance);
    EXPECT_GT(robot.odom_covariance.hh, 0.0);
    expectSymmetricPositive(robot.odom_covariance);
    EXPECT_GT(std::hypot(robot.odom_pose.x_m, robot.odom_pose.y_m), 0.05);   // it moved

    const LocalizationStatus status = system->robotFeed()->status();
    EXPECT_TRUE(status.allReady());
    EXPECT_NE(status.find("imu_heading"), nullptr);
    EXPECT_GT(status.updates, 100u);

    // synthetic truth at the effective time of the estimate
    ASSERT_TRUE(robot.measuredAtHost.isSet());
    const RigTruth truth = rig->truthAt(robot.measuredAtHost);
    const Pose2D   pose  = robot.fieldPose();
    EXPECT_NEAR(pose.x_m, truth.pose.x_m, 0.05);
    EXPECT_NEAR(pose.y_m, truth.pose.y_m, 0.05);
    EXPECT_NEAR(wrapAngle(pose.heading_rad - truth.pose.heading_rad), 0.0, degToRad(3.0));
    EXPECT_EQ(system->diagnostics().functions.count("Localization/weighted_planar_fusion"),
              1u);
}

// ---- interval support, clock identity, lineage ---------------------------

TEST(WeightedPlanarFusion, DifferentDurationsInsideTheOldToleranceAccumulateInstead) {
    // a 20 ms wheel window and a 10 ms gyro window were within the old
    // tolerance; they are different motion windows and never blend as is
    Fixture f;
    f.putMotion(0.0, 0.0, 0.2, 0, 20);
    f.putHeading(0.05, 0, 10);   // fast start
    StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_FALSE(out.advanced);
    EXPECT_TRUE(contains(out.accepted, "heading"));    // stashed, consumed once
    EXPECT_FALSE(contains(out.accepted, "motion"));    // held
    EXPECT_FALSE(contains(out.rejected, "motion"));
    EXPECT_NE(out.diagnostic.find("covers part"), std::string::npos) << out.diagnostic;

    f.putMotion(0.0, 0.0, 0.2, 0, 20);   // the same window, offered again
    f.putHeading(0.15, 10, 20);          // the rate changed inside the window
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.accepted, "motion"));
    EXPECT_TRUE(contains(out.accepted, "heading"));
    // summed support 0.05 + 0.15 = 0.20 over exactly 0..20, no duration scaling
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.2, 1e-12);
    EXPECT_NEAR(out.robot.yaw_rate_rad_s, 10.0, 1e-9);
    const double var_g = 0.01 * 0.01 * 0.02;
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW * var_g / (kVarW + var_g), 1e-18);
}

TEST(WeightedPlanarFusion, NonOverlappingWindowsInsideTheOldToleranceAreRejected) {
    Fixture f;
    f.putMotion(0.0, 0.0, 0.2, 0, 10);
    f.putHeading(0.2, 10, 20);   // a later window, 10 ms off: not this motion
    const StateEstimatorOutput out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NE(out.diagnostic.find("does not continue"), std::string::npos);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW, 1e-18);   // wheel rotation alone
}

TEST(WeightedPlanarFusion, BatchedWindowsFuseAndALateHeadingNeverBlocks) {
    Fixture f;
    // both models saw one batched packet run: identical 0..30 support
    f.putMotion(0.1, 0.0, 0.3, 0, 30);
    f.putHeading(0.3, 0, 30);
    StateEstimatorOutput out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.accepted, "heading"));
    EXPECT_NE(out.diagnostic.find("fused heading"), std::string::npos);

    // a heading arriving a cycle after its motion was already integrated
    f.putHeading(0.1, 30, 40);
    out = f.run();   // no motion: nothing to align to, the heading stays pending
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_FALSE(contains(out.accepted, "heading"));
    EXPECT_FALSE(contains(out.rejected, "heading"));

    f.putMotion(0.1, 0.0, 0.1, 30, 40);   // its motion arrives: they align
    f.putHeading(0.1, 30, 40);
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.accepted, "heading"));

    f.putHeading(0.1, 40, 50);
    f.run();
    f.putMotion(0.1, 0.0, 0.1, 50, 60);   // the motion moved on: the stale heading goes
    f.putHeading(0.1, 40, 50);
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NEAR(out.robot.odom_covariance.hh > 0.0, true, 0);
}

TEST(WeightedPlanarFusion, PartialSupportTimesOutAndReleasesTheMotion) {
    Fixture f(R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
    </Motion>
    <Heading observation_id="heading" max_wait_ms="30">
        <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
    </Heading>
</Estimator>)");
    f.putMotion(0.0, 0.0, 0.2, 0, 20);
    f.putHeading(0.05, 0, 10);
    StateEstimatorOutput out = f.run();
    EXPECT_FALSE(out.advanced);
    f.putMotion(0.0, 0.0, 0.2, 0, 20);
    out = f.run();   // still waiting, inside max_wait_ms
    EXPECT_FALSE(out.advanced);
    EXPECT_NE(out.diagnostic.find("holding"), std::string::npos);
    f.now_ms += 50;
    f.putMotion(0.0, 0.0, 0.2, 0, 20);
    out = f.run();   // gave up: the wheels carry the step alone
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.accepted, "motion"));
    EXPECT_NE(out.diagnostic.find("never completed"), std::string::npos) << out.diagnostic;
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.2, 1e-12);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW, 1e-18);

    // without observed rotation the window can never be completed: it is
    // rejected rather than held forever
    f.putMotion(0.1, 0.0, 0.0, 20, 40, "enc", false);
    f.putHeading(0.05, 20, 30);
    out = f.run();
    EXPECT_FALSE(out.advanced);
    f.now_ms += 50;
    f.putMotion(0.1, 0.0, 0.0, 20, 40, "enc", false);
    out = f.run();
    EXPECT_FALSE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "motion"));

    // recovery: the next aligned pair fuses normally
    f.putMotion(0.0, 0.0, 0.1, 40, 60);
    f.putHeading(0.1, 40, 60);
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_NE(out.diagnostic.find("fused heading"), std::string::npos);
}

TEST(WeightedPlanarFusion, IdenticalStampsOnDifferentNamedClocksNeverCombine) {
    Fixture f;
    f.putMotion(0.0, 0.0, 0.5, 0, 1000, "enc", true, false, 0.0, 0.0, "pico_a");
    f.putHeading(0.25, 0, 1000, "imu", "pico_b");   // same numbers, unrelated boot
    const StateEstimatorOutput out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NE(out.diagnostic.find("different clocks pico_a and pico_b"), std::string::npos)
        << out.diagnostic;
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.5, 1e-12);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW, 1e-18);
}

TEST(WeightedPlanarFusion, UnnamedDeviceClockIsInsufficientToCombine) {
    Fixture f;
    f.putMotion(0.0, 0.0, 0.5);
    f.putHeading(0.25, 0, 1000, "imu", "");   // device stamps with no clock identity
    StateEstimatorOutput out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NE(out.diagnostic.find("insufficient clock identity"), std::string::npos)
        << out.diagnostic;

    // a motion whose own sources disagree about their clock is a fault
    f.putMotion(0.0, 0.0, 0.5, 1000, 2000);
    auto& record = f.observations[ObservationId{"motion"}];
    auto  motion = *record.payload.get<BodyMotionIncrement>();
    Provenance other;
    other.source = "enc_b";
    other.clock  = "pico_other";
    motion.sources.push_back(other);
    record.payload = TypedPayload::store(motion, payload_names::kBodyMotionIncrement);
    out            = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kFault);
    EXPECT_TRUE(contains(out.rejected, "motion"));
    EXPECT_NE(out.diagnostic.find("different clocks"), std::string::npos);
}

TEST(WeightedPlanarFusion, HeadingSourceRestartDiscardsPartialSupport) {
    Fixture f;
    f.putMotion(0.0, 0.0, 0.2, 0, 20);
    f.putHeading(0.05, 0, 10, "imu", "pico", 0);
    StateEstimatorOutput out = f.run();
    EXPECT_FALSE(out.advanced);
    f.putMotion(0.0, 0.0, 0.2, 0, 20);
    f.putHeading(0.15, 10, 20, "imu", "pico", 1);   // the gyro restarted in between
    out = f.run();
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_NE(out.diagnostic.find("restarted"), std::string::npos) << out.diagnostic;
    EXPECT_TRUE(out.advanced);   // nothing left to wait for: wheels alone
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW, 1e-18);
}

TEST(WeightedPlanarFusion, MotionClockChangeIsADiscontinuity) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.0, 0, 1000);
    f.run();
    const uint64_t epoch = f.previous.odometry_epoch;
    f.putMotion(0.1, 0.0, 0.0, 1000, 2000, "enc", true, false, 0.0, 0.0, "pico_other");
    StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kFault);
    EXPECT_TRUE(contains(out.rejected, "motion"));
    EXPECT_EQ(out.robot.odometry_epoch, epoch + 1);
    EXPECT_NE(out.diagnostic.find("clock changed"), std::string::npos);
    EXPECT_NEAR(out.robot.odom_pose.x_m, 0.1, 1e-12);   // the step was not integrated

    // on the new clock the stream continues; a heading on the old clock cannot join
    f.putMotion(0.1, 0.0, 0.0, 2000, 3000, "enc", true, false, 0.0, 0.0, "pico_other");
    f.putHeading(0.0, 2000, 3000, "imu", "pico");
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "heading"));
}

TEST(WeightedPlanarFusion, RepeatedWindowRejectsItsHeadingToo) {
    Fixture f;
    f.putMotion(0.1, 0.0, 0.2);
    f.putHeading(0.2);
    f.run();
    f.putMotion(0.1, 0.0, 0.2);
    f.putHeading(0.2);
    StateEstimatorOutput out = f.run();
    EXPECT_FALSE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "motion"));
    EXPECT_TRUE(contains(out.rejected, "heading"));
    // a heading for another window offered beside a repeat stays pending
    f.putMotion(0.1, 0.0, 0.2);
    f.putHeading(0.2, 1000, 2000);
    out = f.run();
    EXPECT_TRUE(contains(out.rejected, "motion"));
    EXPECT_FALSE(contains(out.rejected, "heading"));
    EXPECT_FALSE(contains(out.accepted, "heading"));
}

TEST(WeightedPlanarFusion, ContinuousMismatchedHeadingsStillHitTheDeadline) {
    Fixture f(R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
    </Motion>
    <Heading observation_id="heading" max_wait_ms="30">
        <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
    </Heading>
</Estimator>)");
    f.putMotion(0.0, 0.0, 0.2, 0, 20);
    f.putHeading(0.05, 0, 10);   // starts the stash
    StateEstimatorOutput out = f.run();
    EXPECT_FALSE(out.advanced);
    // headings keep arriving but never continue at 10: each is rejected,
    // and the deadline still runs
    for (int i = 0; i < 6; ++i) {
        f.now_ms += 10;
        f.putMotion(0.0, 0.0, 0.2, 0, 20);
        f.putHeading(0.02, 15 + i, 25 + i);
        out = f.run();
        EXPECT_TRUE(contains(out.rejected, "heading")) << i;
        if (out.advanced) {
            break;
        }
    }
    EXPECT_TRUE(out.advanced);
    EXPECT_TRUE(contains(out.accepted, "motion"));
    EXPECT_NE(out.diagnostic.find("never completed"), std::string::npos) << out.diagnostic;
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.2, 1e-12);
    EXPECT_NEAR(out.robot.odom_covariance.hh, kVarW, 1e-18);
}

TEST(WeightedPlanarFusion, MotionWithoutRotationHasADeadlineEvenWithoutAnyStash) {
    Fixture f(R"(
<Estimator type="weighted_planar_fusion">
    <Motion observation_id="motion">
        <Noise translation_floor_m="0.001" translation_per_m="0"
               rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
    </Motion>
    <Heading observation_id="heading" max_wait_ms="30">
        <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
    </Heading>
</Estimator>)");
    // no heading at all
    f.putMotion(0.1, 0.0, 0.0, 0, 20, "enc", false);
    StateEstimatorOutput out = f.run();
    EXPECT_FALSE(out.advanced);
    EXPECT_TRUE(out.rejected.empty());   // still inside the deadline: pending
    f.now_ms += 50;
    f.putMotion(0.1, 0.0, 0.0, 0, 20, "enc", false);
    out = f.run();
    EXPECT_FALSE(out.advanced);
    EXPECT_TRUE(contains(out.rejected, "motion"));
    EXPECT_NE(out.diagnostic.find("max_wait_ms"), std::string::npos) << out.diagnostic;

    // an oversized heading that can never fit, offered every cycle
    f.putMotion(0.1, 0.0, 0.0, 20, 40, "enc", false);
    f.putHeading(0.3, 20, 60);
    out = f.run();
    EXPECT_TRUE(contains(out.rejected, "heading"));
    EXPECT_FALSE(contains(out.rejected, "motion"));   // first sighting: pending
    f.now_ms += 50;
    f.putMotion(0.1, 0.0, 0.0, 20, 40, "enc", false);
    f.putHeading(0.3, 20, 60);
    out = f.run();
    EXPECT_TRUE(contains(out.rejected, "motion"));

    // the next window starts a fresh deadline and fuses normally
    f.putMotion(0.1, 0.0, 0.0, 40, 60, "enc", false);
    f.putHeading(0.1, 40, 60);
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.1, 1e-12);
}

// ---- lineage --------------------------------------------------------------

namespace
{

// two wheels constrained by imu_a, a separate heading from imu_b
const char* kTwoWheelsTwoImuIds = R"(
<Localization>
    <Observation id="wheels" type="tracking_wheel_motion">
        <TrackingWheel sensor_id="enc_a" radius_m="0.0254" position_x_m="0"
                       position_y_m="0" measurement_angle_deg="0" direction="positive"/>
        <TrackingWheel sensor_id="enc_c" radius_m="0.0254" position_x_m="0"
                       position_y_m="0" measurement_angle_deg="90" direction="positive"/>
        <HeadingConstraint sensor_id="imu_a" bias_samples="0"/>
        <Output observation_id="motion"/>
    </Observation>
    <Observation id="gyro" type="imu_heading_increment">
        <Input sensor_id="imu_b"/>
        <Calibration bias_samples="0"/>
        <Output observation_id="heading"/>
    </Observation>
    <Estimator type="weighted_planar_fusion">
        <Motion observation_id="motion">
            <Noise translation_floor_m="0.001" translation_per_m="0"
                   rotation_floor_rad="0.02" rotation_per_rad="0" rotation_per_m="0"/>
        </Motion>
        <Heading observation_id="heading" max_wait_ms="50">
            <Noise angle_random_walk_rad_per_sqrt_s="0.01" bias_rad_per_s="0"/>
        </Heading>
    </Estimator>
</Localization>)";

std::vector<uint8_t> sensorPacket(uint8_t seq, uint32_t stamp_ms, int32_t enc0, int32_t enc1,
                                  int32_t gyro_mdps) {
    gatr2::SensorSample s{};
    s.seq      = seq;
    s.stamp_ms = stamp_ms;
    s.mask     = gatr2::kSensorEnc0 | gatr2::kSensorEnc1 | gatr2::kSensorGyroZ;
    s.enc[0]   = enc0;
    s.enc[1]   = enc1;
    s.gyro_z   = gyro_mdps;
    std::vector<uint8_t> buf(gatr2::kMaxFrameLen);
    buf.resize(gatr2::encodeSensorFrame(s, buf.data(), gatr2::kMaxFrameLen));
    return buf;
}

} // namespace

TEST(LocalizationFusion, TwoSensorIdsOnOneImuOutputAreOneMeasurement) {
    StageFixture f;
    f.catalog.add(SensorId{"imu_a"},
                  PayloadDescriptor::of<ImuSample>(payload_names::kImuSample));
    f.catalog.add(SensorId{"imu_b"},
                  PayloadDescriptor::of<ImuSample>(payload_names::kImuSample));
    std::string err;
    auto        exec = f.make(kTwoWheelsTwoImuIds, err);
    ASSERT_TRUE(exec.has_value()) << err;
    const auto putBoth = [&](double rate, int64_t stamp, uint64_t sequence) {
        ImuSample s;
        s.yaw_rate_rad_s = rate;
        f.put("imu_a", TypedPayload::store(s, payload_names::kImuSample), stamp, sequence,
              "pico_telemetry.imu");
        f.put("imu_b", TypedPayload::store(s, payload_names::kImuSample), stamp, sequence,
              "pico_telemetry.imu");
    };
    EncoderSample a, c;
    f.put("enc_a", TypedPayload::store(a, payload_names::kEncoderSample), 1000, 1,
          "pico_telemetry.encoder_a");
    f.put("enc_c", TypedPayload::store(c, payload_names::kEncoderSample), 1000, 1,
          "pico_telemetry.encoder_c");
    putBoth(20.0, 1000, 1);
    f.step(*exec);
    a.angle_rad = 0.05 / 0.0254;
    f.put("enc_a", TypedPayload::store(a, payload_names::kEncoderSample), 1005, 2,
          "pico_telemetry.encoder_a");
    f.put("enc_c", TypedPayload::store(c, payload_names::kEncoderSample), 1005, 2,
          "pico_telemetry.encoder_c");
    putBoth(20.0, 1005, 2);
    const RobotState r = f.step(*exec);
    ASSERT_TRUE(r.valid);
    EXPECT_NEAR(r.odom_pose.heading_rad, 0.1, 1e-9);
    // the alias did not shrink the rotation variance: one gyro, counted once
    EXPECT_NEAR(r.odom_covariance.hh, 4e-4, 1e-15);
    EXPECT_EQ(exec->lastObservations().count(ObservationId{"heading"}), 1u);
    f.step(*exec);
    EXPECT_TRUE(exec->lastObservations().empty());   // rejected once, not stuck
}

TEST(LocalizationFusion, DistinctOutputsOfOnePicoStayIndependent) {
    // three wheels and the gyro all arrive on one Pico and one clock; they
    // are different measurements and do fuse
    StageFixture f;
    std::string  err;
    auto         exec = f.make(kThreeWheelPlusGyro, err);
    ASSERT_TRUE(exec.has_value()) << err;
    const auto putAll = [&](double dx, double dtheta, double rate, int64_t stamp,
                            uint64_t sequence) {
        const double k_a = -0.13, k_b = 0.13, k_c = -0.12, r = 0.0254;
        EncoderSample a, b, c;
        a.angle_rad = (dx + k_a * dtheta) / r;
        b.angle_rad = (dx + k_b * dtheta) / r;
        c.angle_rad = (k_c * dtheta) / r;
        f.put("enc_a", TypedPayload::store(a, payload_names::kEncoderSample), stamp, sequence,
              "pico_telemetry.encoder_a");
        f.put("enc_b", TypedPayload::store(b, payload_names::kEncoderSample), stamp, sequence,
              "pico_telemetry.encoder_b");
        f.put("enc_c", TypedPayload::store(c, payload_names::kEncoderSample), stamp, sequence,
              "pico_telemetry.encoder_c");
        ImuSample s;
        s.yaw_rate_rad_s = rate;
        f.put("imu", TypedPayload::store(s, payload_names::kImuSample), stamp, sequence,
              "pico_telemetry.imu");
    };
    putAll(0.0, 0.0, 20.0, 1000, 1);
    f.step(*exec);
    putAll(0.05, 0.1, 20.0, 1005, 2);
    const RobotState r = f.step(*exec);
    ASSERT_TRUE(r.valid);
    const double var_g = 0.01 * 0.01 * 0.005;
    EXPECT_NEAR(r.odom_covariance.hh, 4e-4 * var_g / (4e-4 + var_g), 1e-15);
    EXPECT_LT(r.odom_covariance.hh, 4e-4);
}

TEST(LocalizationFusion, AliasedImuSensorsThroughTheRealPicoPathCountOnce) {
    FunctionRegistry functions;
    registerAll(functions);
    const std::string xml = std::string(R"(
<System>
    <Resources>
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="imu" channel="imu"/>
        </Resource>
    </Resources>
    <Sensors>
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_c" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="imu_a" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/>
        </Sensor>
        <Sensor id="imu_b" type="pico_imu_channel">
            <Source resource_id="pico_telemetry" output_id="imu"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="noop"/>)") +
                            kTwoWheelsTwoImuIds + R"(
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>)";
    std::string err;
    auto        system = System::buildFromString(xml.c_str(), functions, err);
    ASSERT_NE(system, nullptr) << err;
    auto  link = system->resources().require<SerialLink>(ResourceId{"pico_uart"}, err);
    auto* uart = dynamic_cast<MemoryLink*>(link.get());
    ASSERT_NE(uart, nullptr);

    uart->input().feed(sensorPacket(1, 1000, 0, 0, 0));
    system->step(hostTime(1));
    uart->input().feed(sensorPacket(2, 1005, 400, 0, 0));   // 0.1 turn forward, no rotation
    system->step(hostTime(2));
    const RobotState r = system->robot();
    ASSERT_TRUE(r.valid);
    ASSERT_TRUE(r.has_covariance);
    EXPECT_NEAR(r.odom_pose.heading_rad, 0.0, 1e-9);
    EXPECT_GT(r.odom_pose.x_m, 0.0);
    EXPECT_NEAR(r.odom_covariance.hh, 4e-4, 1e-15);   // imu_b is imu_a: not counted twice
    EXPECT_EQ(system->diagnostics().functions.at("Localization/weighted_planar_fusion").runs,
              2u);
    // every offered observation was settled; nothing is stuck pending
    system->step(hostTime(3));
    EXPECT_TRUE(system->localization().lastObservations().empty());
}
