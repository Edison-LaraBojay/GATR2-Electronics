// localization_models_gtest.cpp
// The robot observation models driven directly through their contracts with
// hand-built sensor records: geometry-owned tracking wheel motion, the gyro
// heading increment, and attitude forwarding. Ids are arbitrary; labels are
// cosmetic; geometry, timing and provenance are what change behavior.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "core/diagnostics.h"
#include "impl/localization/attitude_reference.h"
#include "impl/localization/imu_heading_increment.h"
#include "impl/localization/tracking_wheel_motion.h"
#include "math/angles.h"
#include "payloads/attitude_samples.h"
#include "payloads/robot_observations.h"
#include "runtime/register_all.h"
#include "runtime/sensor_catalog.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

struct Fixture {
    tinyxml2::XMLDocument doc;
    SensorCatalog         catalog;
    SensorMap             sensors;
    FunctionRegistry      functions;
    Diagnostics           diagnostics;
    int64_t               now_ms = 1;

    Fixture() {
        register_localization(functions);
        catalog.add(SensorId{"enc_a"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"enc_b"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"enc_c"},
                    PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
        catalog.add(SensorId{"imu"},
                    PayloadDescriptor::of<ImuSample>(payload_names::kImuSample));
        catalog.add(SensorId{"attitude"},
                    PayloadDescriptor::of<AttitudeSample>(payload_names::kAttitudeSample));
    }

    ConfigNode parse(const std::string& xml) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml.c_str()), tinyxml2::XML_SUCCESS);
        return ConfigNode{doc.RootElement()};
    }

    RobotObservationInitializationContext context() {
        RobotObservationInitializationContext c;
        c.sensors   = &catalog;
        c.functions = &functions;
        return c;
    }

    std::unique_ptr<RobotObservationFunction> makeMotion(const std::string& xml,
                                                         std::string&       err) {
        auto c = context();
        return TrackingWheelMotion::create(parse(xml), c, err);
    }

    std::unique_ptr<RobotObservationFunction> makeHeading(const std::string& xml,
                                                          std::string&       err) {
        auto c = context();
        return ImuHeadingIncrement::create(parse(xml), c, err);
    }

    void put(const char* id, TypedPayload payload, int64_t stamp_ms, uint64_t sequence,
             uint64_t epoch = 0, SourceState state = SourceState::kValid) {
        MeasurementRecord record;
        record.state = state;
        record.epoch = epoch;
        StoredSample stored;
        stored.measuredAt      = deviceTime(stamp_ms);
        stored.receivedAt      = hostTime(stamp_ms + 3);
        stored.sequence        = sequence;
        stored.epoch           = epoch;
        stored.upstream.clock  = "pico";
        stored.upstream.source = id;
        stored.payload         = std::move(payload);
        record.latest          = std::move(stored);
        sensors[SensorId{id}]  = std::move(record);
    }

    void putEncoder(const char* id, double angle_rad, int64_t stamp_ms, uint64_t sequence,
                    uint64_t discontinuity = 0, uint64_t epoch = 0) {
        EncoderSample s;
        s.angle_rad           = angle_rad;
        s.discontinuity_epoch = discontinuity;
        put(id, TypedPayload::store(s, payload_names::kEncoderSample), stamp_ms, sequence,
            epoch);
    }

    void putImu(double rate_rad_s, int64_t stamp_ms, uint64_t sequence,
                bool has_accum = false, double accum = 0.0, uint64_t accum_epoch = 0) {
        ImuSample s;
        s.yaw_rate_rad_s        = rate_rad_s;
        s.has_accumulated       = has_accum;
        s.accumulated_angle_rad = accum;
        s.accumulated_epoch     = accum_epoch;
        put("imu", TypedPayload::store(s, payload_names::kImuSample), stamp_ms, sequence);
    }

    RobotObservationMap run(RobotObservationFunction& fn, FunctionStatus* status = nullptr,
                            bool accept = true) {
        RobotObservationMap out;
        ExecutionContext    context{hostTime(now_ms++), 1, &diagnostics};
        const FunctionStatus s = fn.run({sensors, context}, out);
        if (status != nullptr) {
            *status = s;
        }
        if (accept) {
            for (const auto& record : out) fn.settle(record.first, true);
        }
        return out;
    }

    static const BodyMotionIncrement* motion(const RobotObservationMap& out) {
        const auto it = out.find(ObservationId{"motion"});
        return it == out.end() ? nullptr : it->second.payload.get<BodyMotionIncrement>();
    }
};

// three wheels per the reference layout: left, right, rear
std::string threeWheelXml(const char* a, const char* b, const char* c,
                          const char* label_a = "left") {
    std::string xml = R"(<Observation id="tracking_motion" type="tracking_wheel_motion">)";
    xml += std::string(R"(<TrackingWheel sensor_id=")") + a + R"(" label=")" + label_a +
           R"(" radius_m="0.0254" position_x_m="0" position_y_m="0.13"
              measurement_angle_deg="0" direction="positive"/>)";
    xml += std::string(R"(<TrackingWheel sensor_id=")") + b +
           R"(" label="right" radius_m="0.0254" position_x_m="0" position_y_m="-0.13"
              measurement_angle_deg="0" direction="positive"/>)";
    xml += std::string(R"(<TrackingWheel sensor_id=")") + c +
           R"(" label="rear" radius_m="0.0254" position_x_m="-0.12" position_y_m="0"
              measurement_angle_deg="90" direction="positive"/>)";
    xml += R"(<Output observation_id="motion"/></Observation>)";
    return xml;
}

const char* kTwoWheelXml = R"(
<Observation id="m" type="tracking_wheel_motion">
    <TrackingWheel sensor_id="enc_a" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="0" direction="positive"/>
    <TrackingWheel sensor_id="enc_b" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="90" direction="positive"/>
    <HeadingConstraint sensor_id="imu" bias_samples="2"/>
    <Output observation_id="motion"/>
</Observation>)";

// feed wheel angles for a known body motion and return the solved delta
BodyMotionIncrement solveKnownMotion(Fixture& f, RobotObservationFunction& fn, double dx,
                                     double dy, double dtheta) {
    const double k_a = -0.13, k_b = 0.13, k_c = -0.12;   // x*uy - y*ux
    const double r   = 0.0254;

    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putEncoder("enc_c", 0.0, 1000, 1);
    RobotObservationMap out = f.run(fn);
    EXPECT_TRUE(out.empty());   // seeds

    f.putEncoder("enc_a", (dx + k_a * dtheta) / r, 1005, 2);
    f.putEncoder("enc_b", (dx + k_b * dtheta) / r, 1005, 2);
    f.putEncoder("enc_c", (dy + k_c * dtheta) / r, 1005, 2);
    FunctionStatus status;
    out = f.run(fn, &status);
    EXPECT_EQ(status, FunctionStatus::kOk);
    const BodyMotionIncrement* delta = Fixture::motion(out);
    EXPECT_NE(delta, nullptr);
    return delta != nullptr ? *delta : BodyMotionIncrement{};
}

} // namespace

TEST(TrackingWheelMotion, ThreeWheelsSolvePlanarMotionWithProvenance) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeMotion(threeWheelXml("enc_a", "enc_b", "enc_c"), err);
    ASSERT_NE(fn, nullptr) << err;
    EXPECT_TRUE(fn->readiness().ready);

    const BodyMotionIncrement delta = solveKnownMotion(f, *fn, 0.05, 0.01, 0.1);
    EXPECT_NEAR(delta.dx_m, 0.05, 1e-9);
    EXPECT_NEAR(delta.dy_m, 0.01, 1e-9);
    EXPECT_NEAR(delta.dtheta_rad, 0.1, 1e-9);
    EXPECT_TRUE(delta.has_rotation);
    EXPECT_NEAR(delta.dt_s, 0.005, 1e-12);
    EXPECT_EQ(delta.startAt.ms, 1000);
    EXPECT_EQ(delta.endAt.ms, 1005);
    EXPECT_EQ(delta.startAt.domain, ClockDomain::kDevice);
    ASSERT_EQ(delta.sources.size(), 3u);
    EXPECT_EQ(delta.sources[0].source, "enc_a");
    EXPECT_EQ(delta.sources[0].sequence, 2u);
    EXPECT_EQ(delta.sources[0].clock, "pico");
}

TEST(TrackingWheelMotion, OutstandingOfferRetainsLaterMotionUntilDisposition) {
    Fixture f;
    std::string err;
    auto fn = f.makeMotion(threeWheelXml("enc_a", "enc_b", "enc_c"), err);
    ASSERT_NE(fn, nullptr) << err;
    const auto put = [&](double x, int64_t at, uint64_t sequence) {
        f.putEncoder("enc_a", x / 0.0254, at, sequence);
        f.putEncoder("enc_b", x / 0.0254, at, sequence);
        f.putEncoder("enc_c", 0, at, sequence);
    };
    put(0, 100, 1);
    f.run(*fn);
    put(0.1, 110, 2);
    const auto first = f.run(*fn, nullptr, false);
    ASSERT_NE(Fixture::motion(first), nullptr);
    EXPECT_NEAR(Fixture::motion(first)->dx_m, 0.1, 1e-9);
    put(0.2, 120, 3);
    EXPECT_TRUE(f.run(*fn, nullptr, false).empty());
    put(0.3, 130, 4);
    EXPECT_TRUE(f.run(*fn, nullptr, false).empty());
    fn->settle(ObservationId{"motion"}, true);
    const auto later = f.run(*fn); // no newer sensor sample required
    ASSERT_NE(Fixture::motion(later), nullptr);
    EXPECT_NEAR(Fixture::motion(later)->dx_m, 0.2, 1e-9);
    EXPECT_EQ(Fixture::motion(later)->startAt.ms, 110);
    EXPECT_EQ(Fixture::motion(later)->endAt.ms, 130);
    EXPECT_TRUE(f.run(*fn).empty());
}

TEST(ImuHeadingIncrement, PendingOfferStillIntegratesEveryRateSample) {
    Fixture f;
    std::string err;
    auto fn = f.makeHeading(R"(<Observation id="heading" type="imu_heading_increment">
        <Input sensor_id="imu"/><Calibration bias_samples="0"/>
        <Output observation_id="heading"/></Observation>)", err);
    ASSERT_NE(fn, nullptr) << err;
    f.putImu(0, 100, 1);
    f.run(*fn);
    f.putImu(2, 110, 2);
    const auto first = f.run(*fn, nullptr, false);
    ASSERT_EQ(first.size(), 1u);
    EXPECT_NEAR(first.begin()->second.payload.get<HeadingIncrement>()->dtheta_rad,
                0.01, 1e-12);
    f.putImu(0, 120, 3);
    EXPECT_TRUE(f.run(*fn, nullptr, false).empty());
    f.putImu(4, 130, 4);
    EXPECT_TRUE(f.run(*fn, nullptr, false).empty());
    fn->settle(ObservationId{"heading"}, false); // rejects only the offered 100..110
    const auto later = f.run(*fn);
    ASSERT_EQ(later.size(), 1u);
    const auto* increment = later.begin()->second.payload.get<HeadingIncrement>();
    ASSERT_NE(increment, nullptr);
    EXPECT_NEAR(increment->dtheta_rad, 0.03, 1e-12);
    EXPECT_EQ(increment->startAt.ms, 110);
    EXPECT_EQ(increment->endAt.ms, 130);
    EXPECT_TRUE(f.run(*fn).empty());
}

TEST(TrackingWheelMotion, SensorIdsAreOpaqueRenamingChangesNothing) {
    Fixture f;
    f.catalog.add(SensorId{"banana"},
                  PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
    f.catalog.add(SensorId{"kiwi"},
                  PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
    f.catalog.add(SensorId{"mango"},
                  PayloadDescriptor::of<EncoderSample>(payload_names::kEncoderSample));
    std::string err;
    auto        fn = f.makeMotion(threeWheelXml("banana", "kiwi", "mango"), err);
    ASSERT_NE(fn, nullptr) << err;

    const double r = 0.0254;
    f.putEncoder("banana", 0.0, 1000, 1);
    f.putEncoder("kiwi", 0.0, 1000, 1);
    f.putEncoder("mango", 0.0, 1000, 1);
    f.run(*fn);
    f.putEncoder("banana", 0.05 / r, 1005, 2);
    f.putEncoder("kiwi", 0.05 / r, 1005, 2);
    f.putEncoder("mango", 0.0, 1005, 2);
    const auto out   = f.run(*fn);
    const auto delta = Fixture::motion(out);
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->dx_m, 0.05, 1e-9);
    EXPECT_NEAR(delta->dtheta_rad, 0.0, 1e-9);
}

TEST(TrackingWheelMotion, LabelsDoNotAffectBehaviorGeometryDoes) {
    Fixture     f;
    std::string err;
    auto fn_a = f.makeMotion(threeWheelXml("enc_a", "enc_b", "enc_c", "left"), err);
    ASSERT_NE(fn_a, nullptr) << err;
    Fixture f2;
    auto    fn_b = f2.makeMotion(threeWheelXml("enc_a", "enc_b", "enc_c", "port_side"), err);
    ASSERT_NE(fn_b, nullptr) << err;
    const BodyMotionIncrement from_a = solveKnownMotion(f, *fn_a, 0.02, 0.0, 0.05);
    const BodyMotionIncrement from_b = solveKnownMotion(f2, *fn_b, 0.02, 0.0, 0.05);
    EXPECT_NEAR(from_a.dtheta_rad, from_b.dtheta_rad, 1e-12);

    // changed geometry, same sensor data: different answer
    Fixture     f3;
    std::string wide = threeWheelXml("enc_a", "enc_b", "enc_c");
    const auto  pos  = wide.find("position_y_m=\"0.13\"");
    ASSERT_NE(pos, std::string::npos);
    wide.replace(pos, 20, "position_y_m=\"0.26\"");
    auto fn_wide = f3.makeMotion(wide, err);
    ASSERT_NE(fn_wide, nullptr) << err;
    const double k_a = -0.13, k_b = 0.13, k_c = -0.12, r = 0.0254;
    f3.putEncoder("enc_a", 0.0, 1000, 1);
    f3.putEncoder("enc_b", 0.0, 1000, 1);
    f3.putEncoder("enc_c", 0.0, 1000, 1);
    f3.run(*fn_wide);
    f3.putEncoder("enc_a", (0.02 + k_a * 0.05) / r, 1005, 2);
    f3.putEncoder("enc_b", (0.02 + k_b * 0.05) / r, 1005, 2);
    f3.putEncoder("enc_c", (k_c * 0.05) / r, 1005, 2);
    const auto delta = Fixture::motion(f3.run(*fn_wide));
    ASSERT_NE(delta, nullptr);
    EXPECT_GT(std::fabs(delta->dtheta_rad - 0.05), 0.01);
}

TEST(TrackingWheelMotion, TwoWheelsNeedHeadingConstraintAndCalibrateFirst) {
    Fixture     f;
    std::string err;
    EXPECT_EQ(f.makeMotion(R"(
<Observation id="m" type="tracking_wheel_motion">
    <TrackingWheel sensor_id="enc_a" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="0" direction="positive"/>
    <TrackingWheel sensor_id="enc_b" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="90" direction="positive"/>
    <Output observation_id="motion"/>
</Observation>)",
                           err),
              nullptr);
    EXPECT_NE(err.find("HeadingConstraint"), std::string::npos);

    auto fn = f.makeMotion(kTwoWheelXml, err);
    ASSERT_NE(fn, nullptr) << err;
    EXPECT_FALSE(fn->readiness().ready);   // still calibrating

    // calibration: two stationary imu readings with bias 0.02 rad/s
    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putImu(0.02, 1000, 1);
    f.run(*fn);
    f.putImu(0.02, 1005, 2);
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_TRUE(fn->readiness().ready);

    // motion: forward 0.01 m while the gyro reads bias only (no rotation)
    f.putEncoder("enc_a", 0.01 / 0.0254, 1010, 2);
    f.putEncoder("enc_b", 0.0, 1010, 2);
    f.putImu(0.02, 1010, 3);   // first post-cal sample seeds the integrator
    f.run(*fn);
    f.putEncoder("enc_a", 0.02 / 0.0254, 1015, 3);
    f.putEncoder("enc_b", 0.0, 1015, 3);
    f.putImu(0.02, 1015, 4);
    FunctionStatus status;
    const auto     out   = f.run(*fn, &status);
    const auto     delta = Fixture::motion(out);
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->dtheta_rad, 0.0, 1e-9);   // bias removed
    EXPECT_GT(delta->dx_m, 0.015);               // accumulated travel, none lost
    EXPECT_EQ(delta->sources.size(), 3u);        // the imu is part of the lineage
    EXPECT_EQ(delta->sources[2].source, "imu");
}

TEST(TrackingWheelMotion, CalibrationRebasesWheelsAndRestartsOnMotion) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeMotion(kTwoWheelXml, err);
    ASSERT_NE(fn, nullptr) << err;
    const double r = 0.0254;

    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putImu(0.02, 1000, 1);
    f.run(*fn);   // seeds; bias sample 1

    // the robot moves 20 mm while bias collection runs: collection restarts
    f.putEncoder("enc_a", 0.02 / r, 1005, 2);
    f.putEncoder("enc_b", 0.0, 1005, 2);
    f.putImu(0.02, 1005, 2);
    EXPECT_TRUE(f.run(*fn).empty());

    f.putImu(0.02, 1010, 3);   // stationary again: completes the restarted run
    EXPECT_TRUE(f.run(*fn).empty());

    // post-calibration motion contains only travel after calibration
    f.putEncoder("enc_a", 0.03 / r, 1015, 3);
    f.putEncoder("enc_b", 0.0, 1015, 3);
    f.putImu(0.02, 1015, 4);   // seeds the integrator
    EXPECT_TRUE(f.run(*fn).empty());
    f.putEncoder("enc_a", 0.04 / r, 1020, 4);
    f.putEncoder("enc_b", 0.0, 1020, 4);
    f.putImu(0.02, 1020, 5);
    const auto delta = Fixture::motion(f.run(*fn));
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->dx_m, 0.02, 1e-9);   // 0.04 total would leak cal travel
    EXPECT_NEAR(delta->dtheta_rad, 0.0, 1e-9);
}

TEST(TrackingWheelMotion, ConfigurationErrors) {
    Fixture     f;
    std::string err;

    EXPECT_EQ(f.makeMotion(threeWheelXml("imu", "enc_b", "enc_c"), err), nullptr);
    EXPECT_NE(err.find("different payload"), std::string::npos);
    EXPECT_NE(err.find(payload_names::kImuSample), std::string::npos);

    EXPECT_EQ(f.makeMotion(threeWheelXml("ghost", "enc_b", "enc_c"), err), nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);

    EXPECT_EQ(f.makeMotion(threeWheelXml("enc_a", "enc_a", "enc_c"), err), nullptr);
    EXPECT_NE(err.find("more than one wheel"), std::string::npos);

    std::string bad = threeWheelXml("enc_a", "enc_b", "enc_c");
    bad.replace(bad.find("radius_m=\"0.0254\""), 17, "radius_m=\"0\"");
    EXPECT_EQ(f.makeMotion(bad, err), nullptr);
    EXPECT_NE(err.find("radius_m"), std::string::npos);

    bad = threeWheelXml("enc_a", "enc_b", "enc_c");
    bad.replace(bad.find("position_y_m=\"0.13\""), 19, "position_y_m=\"nan\"");
    EXPECT_EQ(f.makeMotion(bad, err), nullptr);
    EXPECT_NE(err.find("invalid value"), std::string::npos);

    bad = threeWheelXml("enc_a", "enc_b", "enc_c");
    bad.replace(bad.find("direction=\"positive\""), 20, "direction=\"sideways\"");
    EXPECT_EQ(f.makeMotion(bad, err), nullptr);
    EXPECT_NE(err.find("direction"), std::string::npos);

    bad            = threeWheelXml("enc_a", "enc_b", "enc_c");
    const auto pos = bad.find(" position_y_m=\"0.13\"");
    ASSERT_NE(pos, std::string::npos);
    bad.erase(pos, 20);
    EXPECT_EQ(f.makeMotion(bad, err), nullptr);
    EXPECT_NE(err.find("missing required attribute position_y_m"), std::string::npos);

    // the output declaration is required
    bad = threeWheelXml("enc_a", "enc_b", "enc_c");
    const std::string output_decl = "<Output observation_id=\"motion\"/>";
    bad.replace(bad.find(output_decl), output_decl.size(), "");
    EXPECT_EQ(f.makeMotion(bad, err), nullptr);
    EXPECT_NE(err.find("Output"), std::string::npos);
}

TEST(TrackingWheelMotion, ImuOutageDropsTheWholeFusionWindow) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeMotion(R"(
<Observation id="m" type="tracking_wheel_motion">
    <TrackingWheel sensor_id="enc_a" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="0" direction="positive"/>
    <TrackingWheel sensor_id="enc_b" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="90" direction="positive"/>
    <HeadingConstraint sensor_id="imu" bias_samples="0" max_gap_ms="250"/>
    <Timing interval_tolerance_ms="20" max_pending_ms="5000"/>
    <Output observation_id="motion"/>
</Observation>)",
                                  err);
    ASSERT_NE(fn, nullptr) << err;
    const double r = 0.0254;

    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putImu(0.0, 1000, 1);
    f.run(*fn);
    f.putEncoder("enc_a", 0.10 / r, 1005, 2);
    f.putEncoder("enc_b", 0.0, 1005, 2);
    f.putImu(0.0, 1005, 2);
    {
        const auto d = Fixture::motion(f.run(*fn));
        ASSERT_NE(d, nullptr);
        EXPECT_NEAR(d->dx_m, 0.10, 1e-9);
    }

    // IMU outage: the wheels travel 0.60 m that no gyro interval covers
    f.putEncoder("enc_a", 0.40 / r, 1010, 3);
    f.putEncoder("enc_b", 0.0, 1010, 3);
    EXPECT_TRUE(f.run(*fn).empty());
    f.putEncoder("enc_a", 0.70 / r, 1310, 4);
    f.putEncoder("enc_b", 0.0, 1310, 4);
    EXPECT_TRUE(f.run(*fn).empty());

    // the IMU returns past max_gap_ms: the whole window is dropped
    f.putImu(0.0, 1320, 3);
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_NE(fn->readiness().note.find("gyro gap"), std::string::npos);

    // the wheels re-baseline first: their next samples measure nothing,
    // because their previous interval spans the dropped gyro stretch
    f.putEncoder("enc_a", 0.72 / r, 1325, 5);
    f.putEncoder("enc_b", 0.0, 1325, 5);
    f.putImu(0.0, 1325, 4);
    EXPECT_TRUE(f.run(*fn).empty());

    // the next matched interval fuses only its own motion
    f.putEncoder("enc_a", 0.74 / r, 1330, 6);
    f.putEncoder("enc_b", 0.0, 1330, 6);
    f.putImu(0.0, 1330, 5);
    const auto d = Fixture::motion(f.run(*fn));
    ASSERT_NE(d, nullptr);
    EXPECT_NEAR(d->dx_m, 0.02, 1e-9);   // not 0.64: the outage travel is gone
    EXPECT_NEAR(d->dt_s, 0.005, 1e-12);
    EXPECT_EQ(d->startAt.ms, 1325);
}

TEST(TrackingWheelMotion, UnhealthySourcesAreNotConsumed) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeMotion(threeWheelXml("enc_a", "enc_b", "enc_c"), err);
    ASSERT_NE(fn, nullptr) << err;

    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putEncoder("enc_c", 0.0, 1000, 1);
    f.run(*fn);

    f.putEncoder("enc_a", 1.0, 1005, 2);
    f.putEncoder("enc_b", 1.0, 1005, 2);
    f.putEncoder("enc_c", 1.0, 1005, 2);
    f.sensors[SensorId{"enc_c"}].state = SourceState::kFault;
    EXPECT_TRUE(f.run(*fn).empty());   // no solve without every healthy input
}

TEST(TrackingWheelMotion, RetainedRecordsAreNeverConsumedTwice) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeMotion(threeWheelXml("enc_a", "enc_b", "enc_c"), err);
    ASSERT_NE(fn, nullptr) << err;
    solveKnownMotion(f, *fn, 0.05, 0.0, 0.0);
    FunctionStatus status;
    const auto     again = f.run(*fn, &status);   // same records, nothing new
    EXPECT_TRUE(again.empty());
    EXPECT_EQ(status, FunctionStatus::kNoData);
}

TEST(TrackingWheelMotion, NonpositiveIntervalsAndDiscontinuitiesDropTheWindow) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeMotion(threeWheelXml("enc_a", "enc_b", "enc_c"), err);
    ASSERT_NE(fn, nullptr) << err;
    const double r = 0.0254;

    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putEncoder("enc_c", 0.0, 1000, 1);
    f.run(*fn);

    // a sample stamped before its predecessor measures nothing
    f.putEncoder("enc_a", 0.1 / r, 990, 2);
    f.putEncoder("enc_b", 0.1 / r, 1005, 2);
    f.putEncoder("enc_c", 0.0, 1005, 2);
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_NE(fn->readiness().note.find("nonpositive"), std::string::npos);

    // recovery: the next consistent interval solves on its own
    f.putEncoder("enc_a", 0.2 / r, 1010, 3);
    f.putEncoder("enc_b", 0.2 / r, 1010, 3);
    f.putEncoder("enc_c", 0.0, 1010, 3);
    {
        const auto d = Fixture::motion(f.run(*fn));
        ASSERT_NE(d, nullptr);
        EXPECT_NEAR(d->dx_m, 0.1, 1e-9);   // only 1005..1010 travel on b, a rebased at 990
    }

    // an encoder baseline rebase (source restart) invalidates the span
    f.putEncoder("enc_a", 5.0, 1015, 4, 1);   // discontinuity epoch 1
    f.putEncoder("enc_b", 0.2 / r, 1015, 4);
    f.putEncoder("enc_c", 0.0, 1015, 4);
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_NE(fn->readiness().note.find("discontinuity"), std::string::npos);

    f.putEncoder("enc_a", 5.0 + 0.05 / r, 1020, 5, 1);
    f.putEncoder("enc_b", 0.25 / r, 1020, 5);
    f.putEncoder("enc_c", 0.0, 1020, 5);
    const auto d = Fixture::motion(f.run(*fn));
    ASSERT_NE(d, nullptr);
    EXPECT_NEAR(d->dx_m, 0.05, 1e-9);
}

TEST(TrackingWheelMotion, MisalignedIntervalsWaitForAlignmentThenSolveOnce) {
    Fixture     f;
    std::string err;
    std::string xml = threeWheelXml("enc_a", "enc_b", "enc_c");
    xml.insert(xml.find("<Output"), R"(<Timing interval_tolerance_ms="10" max_pending_ms="200"/>)");
    auto fn = f.makeMotion(xml, err);
    ASSERT_NE(fn, nullptr) << err;
    const double r = 0.0254;

    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putEncoder("enc_c", 0.0, 1000, 1);
    f.run(*fn);

    // wheel a reports early; b and c report a later, longer interval
    f.putEncoder("enc_a", 0.02 / r, 1005, 2);
    EXPECT_TRUE(f.run(*fn).empty());
    f.putEncoder("enc_b", 0.05 / r, 1050, 2);
    f.putEncoder("enc_c", 0.0, 1050, 2);
    EXPECT_TRUE(f.run(*fn).empty());   // a covers 1000..1005, the others 1000..1050

    // once a catches up the window covers one common span, nothing lost
    f.putEncoder("enc_a", 0.05 / r, 1050, 3);
    const auto d = Fixture::motion(f.run(*fn));
    ASSERT_NE(d, nullptr);
    EXPECT_NEAR(d->dx_m, 0.05, 1e-9);
    EXPECT_NEAR(d->dt_s, 0.05, 1e-12);
    EXPECT_EQ(d->startAt.ms, 1000);
    EXPECT_EQ(d->endAt.ms, 1050);

    // sources that never realign are dropped after max_pending_ms
    f.putEncoder("enc_a", 0.06 / r, 1055, 4);
    f.run(*fn);
    f.putEncoder("enc_b", 0.06 / r, 1300, 3);
    f.putEncoder("enc_c", 0.0, 1300, 3);
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_NE(fn->readiness().note.find("misaligned"), std::string::npos);
}

TEST(TrackingWheelMotion, GyroAccumulatorDiscontinuityIsNotBridged) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeMotion(R"(
<Observation id="m" type="tracking_wheel_motion">
    <TrackingWheel sensor_id="enc_a" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="0" direction="positive"/>
    <TrackingWheel sensor_id="enc_b" radius_m="0.0254" position_x_m="0"
                   position_y_m="0" measurement_angle_deg="90" direction="positive"/>
    <HeadingConstraint sensor_id="imu" bias_samples="0"/>
    <Output observation_id="motion"/>
</Observation>)",
                                  err);
    ASSERT_NE(fn, nullptr) << err;

    f.putEncoder("enc_a", 0.0, 1000, 1);
    f.putEncoder("enc_b", 0.0, 1000, 1);
    f.putImu(0.0, 1000, 1, true, 0.0, 0);
    f.run(*fn);
    f.putEncoder("enc_a", 0.0, 1005, 2);
    f.putEncoder("enc_b", 0.0, 1005, 2);
    f.putImu(0.0, 1005, 2, true, 0.5, 1);   // epoch bump: 0.5 rad is not rotation
    EXPECT_TRUE(f.run(*fn).empty());
    f.putEncoder("enc_a", 0.0, 1010, 3);
    f.putEncoder("enc_b", 0.0, 1010, 3);
    f.putImu(0.0, 1010, 3, true, 0.5, 1);
    EXPECT_TRUE(f.run(*fn).empty());   // wheels re-baseline after the drop
    f.putEncoder("enc_a", 0.0, 1015, 4);
    f.putEncoder("enc_b", 0.0, 1015, 4);
    f.putImu(0.0, 1015, 4, true, 0.5, 1);
    const auto d = Fixture::motion(f.run(*fn));
    ASSERT_NE(d, nullptr);
    EXPECT_NEAR(d->dtheta_rad, 0.0, 1e-12);
    EXPECT_EQ(d->startAt.ms, 1010);
}

TEST(ImuHeadingIncrement, CalibratesThenIntegratesWithProvenance) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeHeading(R"(
<Observation id="imu_heading" type="imu_heading_increment">
    <Input sensor_id="imu"/>
    <Calibration bias_samples="2"/>
    <Output observation_id="heading"/>
</Observation>)",
                                   err);
    ASSERT_NE(fn, nullptr) << err;
    EXPECT_FALSE(fn->readiness().ready);

    f.putImu(0.5, 0, 1);
    f.run(*fn);
    f.putImu(0.3, 5, 2);   // bias average 0.4
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_TRUE(fn->readiness().ready);

    f.putImu(0.4 + 1.0, 10, 3);   // seeds the integrator at 1 rad/s
    EXPECT_TRUE(f.run(*fn).empty());
    f.putImu(0.4 + 1.0, 15, 4);
    const auto out = f.run(*fn);
    const auto it  = out.find(ObservationId{"heading"});
    ASSERT_NE(it, out.end());
    const HeadingIncrement* delta = it->second.payload.get<HeadingIncrement>();
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->rate_rad_s, 1.0, 1e-12);
    EXPECT_NEAR(delta->dtheta_rad, 1.0 * 0.005, 1e-12);
    EXPECT_EQ(delta->startAt.ms, 10);
    EXPECT_EQ(delta->endAt.ms, 15);
    ASSERT_EQ(delta->sources.size(), 1u);
    EXPECT_EQ(delta->sources[0].source, "imu");
    EXPECT_EQ(delta->sources[0].sequence, 4u);
    EXPECT_EQ(it->second.receivedAt.ms, 18);   // upstream receipt preserved
}

TEST(ImuHeadingIncrement, OutagesDiscontinuitiesAndBadIntervalsReseed) {
    Fixture     f;
    std::string err;
    auto        fn = f.makeHeading(R"(
<Observation id="imu_heading" type="imu_heading_increment">
    <Input sensor_id="imu"/>
    <Calibration bias_samples="0" max_gap_ms="250"/>
    <Output observation_id="heading"/>
</Observation>)",
                                   err);
    ASSERT_NE(fn, nullptr) << err;

    f.putImu(1.0, 0, 1, true, 0.0, 0);
    f.run(*fn);
    f.putImu(1.0, 5, 2, true, 0.005, 0);
    EXPECT_EQ(f.run(*fn).count(ObservationId{"heading"}), 1u);

    // a five second outage: reseeded, interval dropped
    f.putImu(1.0, 5005, 3, true, 5.005, 0);
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_NE(fn->readiness().note.find("gap"), std::string::npos);

    // accumulator epoch change: reseeded, not bridged by the trapezoid
    f.putImu(1.0, 5010, 4, true, 9.0, 1);
    EXPECT_TRUE(f.run(*fn).empty());

    // time going backwards: reseeded
    f.putImu(1.0, 5000, 5, true, 9.0, 1);
    EXPECT_TRUE(f.run(*fn).empty());
    EXPECT_NE(fn->readiness().note.find("nonpositive"), std::string::npos);

    f.putImu(1.0, 5005, 6, true, 9.005, 1);   // normal cadence resumes
    const auto out   = f.run(*fn);
    const auto delta = out.at(ObservationId{"heading"}).payload.get<HeadingIncrement>();
    ASSERT_NE(delta, nullptr);
    EXPECT_NEAR(delta->dtheta_rad, 0.005, 1e-12);
}

TEST(AttitudeReference, ForwardsFreshSamplesAndRejectsStaleOrBroken) {
    Fixture     f;
    std::string err;
    auto        context = f.context();
    auto        fn      = AttitudeReference::create(f.parse(R"(
<Observation id="attitude" type="attitude_reference">
    <Input sensor_id="attitude"/>
    <Freshness max_age_ms="50"/>
    <Output observation_id="attitude"/>
</Observation>)"),
                                                    context, err);
    ASSERT_NE(fn, nullptr) << err;

    AttitudeSample sample;
    sample.q_reference_body = quaternionFromEuler(degToRad(5.0), 0.0, 0.0);
    sample.reference        = "gravity";
    sample.quality          = 0.9;
    f.now_ms                = 10;
    f.put("attitude", TypedPayload::store(sample, payload_names::kAttitudeSample), 2, 1);
    auto out = f.run(*fn);
    ASSERT_EQ(out.count(ObservationId{"attitude"}), 1u);
    const auto* obs = out.at(ObservationId{"attitude"}).payload.get<AttitudeObservation>();
    ASSERT_NE(obs, nullptr);
    EXPECT_EQ(obs->reference, "gravity");
    EXPECT_FALSE(obs->has_yaw);
    EXPECT_EQ(obs->source.source, "attitude");
    EXPECT_NEAR(obs->quality, 0.9, 1e-12);

    // the retained sample is not republished
    EXPECT_TRUE(f.run(*fn).empty());

    // a sample that sat around longer than max_age_ms is not current
    f.now_ms = 500;
    f.put("attitude", TypedPayload::store(sample, payload_names::kAttitudeSample), 100, 2);
    EXPECT_TRUE(f.run(*fn).empty());

    // a non-rotation is a fault, never silently normalized into evidence
    sample.q_reference_body = Quaternion{0.0, 0.0, 0.0, 0.0};
    f.now_ms                = 600;
    f.put("attitude", TypedPayload::store(sample, payload_names::kAttitudeSample), 598, 3);
    FunctionStatus status;
    f.run(*fn, &status);
    EXPECT_EQ(status, FunctionStatus::kFault);
}
