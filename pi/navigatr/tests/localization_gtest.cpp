// localization_gtest.cpp
// The planar motion integrator through its contract, then make_localization
// and the executor: build validation, the observation loop, finalization,
// history publication, placement requests, resets, and the clock mapping.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "core/clock_sync.h"
#include "core/diagnostics.h"
#include "impl/localization/planar_motion_integrator.h"
#include "math/angles.h"
#include "payloads/robot_observations.h"
#include "runtime/localization_stage.h"
#include "runtime/register_all.h"
#include "runtime/sensor_catalog.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

struct EstimatorFixture {
    tinyxml2::XMLDocument doc;
    RobotObservationMap   observations;
    LocalizationRequests  requests;
    RobotState            previous;
    Diagnostics           diagnostics;
    int64_t               now_ms = 1;

    std::unique_ptr<StateEstimator> estimator;

    explicit EstimatorFixture(const char* xml = R"(
        <Estimator type="planar_motion_integrator">
            <Motion observation_id="motion"/>
        </Estimator>)") {
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
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
        std::string err;
        estimator = PlanarMotionIntegrator::create(ConfigNode{doc.RootElement()}, context, err);
        EXPECT_NE(estimator, nullptr) << err;
    }

    void putMotion(double dx, double dy, double dtheta, int64_t start_ms = 95,
                   int64_t end_ms = 100, const char* source = "enc", bool rotation = true) {
        BodyMotionIncrement m;
        m.dx_m         = dx;
        m.dy_m         = dy;
        m.dtheta_rad   = dtheta;
        m.has_rotation = rotation;
        m.startAt      = deviceTime(start_ms);
        m.endAt        = deviceTime(end_ms);
        m.dt_s         = (end_ms - start_ms) / 1000.0;
        Provenance p;
        p.source = source;
        m.sources.push_back(p);
        RobotObservationRecord record;
        record.measuredAt = m.endAt;
        record.receivedAt = hostTime(now_ms);
        record.payload = TypedPayload::store(m, payload_names::kBodyMotionIncrement);
        observations[ObservationId{"motion"}] = std::move(record);
    }

    void putHeading(double dtheta, int64_t start_ms = 95, int64_t end_ms = 100,
                    const char* source = "imu") {
        HeadingIncrement h;
        h.dtheta_rad = dtheta;
        h.startAt    = deviceTime(start_ms);
        h.endAt      = deviceTime(end_ms);
        h.dt_s       = (end_ms - start_ms) / 1000.0;
        Provenance p;
        p.source = source;
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
        const StateEstimatorOutput out = estimator->run({observations, previous, requests, context});
        previous                       = out.robot;
        observations.clear();
        ++now_ms;
        return out;
    }
};

// Observation function registered only in this test binary: publishes
// scripted host-stamped motion increments.
struct MotionScript {
    bool    emit     = false;
    double  dx       = 0.0;
    double  dtheta   = 0.0;
    int64_t start_ms = 0, end_ms = 0;
};

class ScriptedMotionFunction : public RobotObservationFunction
{
public:
    ScriptedMotionFunction(std::shared_ptr<MotionScript> script, ObservationFunctionId id)
        : script_(std::move(script)), id_(std::move(id)) {}

    FunctionStatus run(const RobotObservationInput& in, RobotObservationMap& out) override {
        if (!script_->emit) {
            return FunctionStatus::kNoData;
        }
        BodyMotionIncrement m;
        m.dx_m       = script_->dx;
        m.dtheta_rad = script_->dtheta;
        m.startAt    = hostTime(script_->start_ms);
        m.endAt      = hostTime(script_->end_ms);
        m.dt_s       = (script_->end_ms - script_->start_ms) / 1000.0;
        RobotObservationRecord record;
        record.measuredAt = m.endAt;
        record.receivedAt = in.context.now;
        record.payload    = TypedPayload::store(m, payload_names::kBodyMotionIncrement);
        out[ObservationId{"scripted"}] = std::move(record);
        script_->emit                  = false;
        return FunctionStatus::kOk;
    }
    const ObservationFunctionId& id() const override { return id_; }
    const std::string&           type() const override { return type_; }
    std::vector<RobotObservationOutputDecl> outputs() const override {
        return {RobotObservationOutputDecl{
            ObservationId{"scripted"},
            PayloadDescriptor::of<BodyMotionIncrement>(payload_names::kBodyMotionIncrement)}};
    }
    ObservationReadiness readiness() const override { return ObservationReadiness{true, ""}; }

private:
    std::shared_ptr<MotionScript> script_;
    ObservationFunctionId         id_;
    std::string                   type_ = "scripted_motion";
};

struct StageFixture {
    tinyxml2::XMLDocument         doc;
    FunctionRegistry              functions;
    SensorCatalog                 catalog;
    ResourceStore                 store;
    SensorMap                     sensors;
    Diagnostics                   diagnostics;
    std::shared_ptr<MotionScript> script = std::make_shared<MotionScript>();
    int64_t                       now_ms = 1;

    StageFixture() {
        register_localization(functions);
        auto s = script;
        functions.add<RobotObservationMakeFunction>(
            FunctionKey{"scripted_motion"},
            [s](const ConfigNode& node, RobotObservationInitializationContext&,
                std::string&) -> std::unique_ptr<RobotObservationFunction> {
                return std::make_unique<ScriptedMotionFunction>(
                    s, ObservationFunctionId{node.attr("id")});
            });
    }

    std::optional<LocalizationExecutor> make(const char* xml, std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        return make_localization(ConfigNode{doc.RootElement()}, functions, catalog, store,
                                 nullptr, err);
    }

    RobotState step(LocalizationExecutor& exec, const LocalizationRequests& requests = {}) {
        ExecutionContext context{hostTime(now_ms++), 1, &diagnostics};
        return exec(sensors, requests, context);
    }

    void emit(double dx, int64_t start_ms, int64_t end_ms) {
        script->emit     = true;
        script->dx       = dx;
        script->start_ms = start_ms;
        script->end_ms   = end_ms;
    }
};

const char* kScriptedLocalization = R"(
<Localization>
    <Observation id="scripted" type="scripted_motion"/>
    <Estimator type="planar_motion_integrator">
        <Motion observation_id="scripted"/>
    </Estimator>
    <History retention_s="1" capacity="8" max_interpolation_gap_ms="100"/>
</Localization>)";

} // namespace

TEST(PlanarMotionIntegrator, StraightLineAndVelocity) {
    EstimatorFixture f;
    f.putMotion(0.1, 0.0, 0.0);
    const StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kOk);
    EXPECT_TRUE(out.advanced);
    EXPECT_NEAR(out.robot.fieldPose().x_m, 0.1, 1e-12);
    EXPECT_TRUE(out.robot.valid);
    EXPECT_NEAR(out.robot.vx_m_s, 20.0, 1e-9);
    EXPECT_EQ(out.robot.measuredAt.ms, 100);
    EXPECT_FALSE(out.robot.measuredAtHost.isSet());   // no clock mapping yet
}

TEST(PlanarMotionIntegrator, QuarterCircleChordAndHeadingFrame) {
    EstimatorFixture f;
    const double     R = 0.5;
    f.putMotion(R * kPi / 2.0, 0.0, kPi / 2.0);
    f.run();
    EXPECT_NEAR(f.previous.fieldPose().x_m, R, 1e-9);
    EXPECT_NEAR(f.previous.fieldPose().y_m, R, 1e-9);
    EXPECT_NEAR(f.previous.fieldPose().heading_rad, kPi / 2.0, 1e-12);

    f.putMotion(0.1, 0.0, 0.0, 100, 105);
    f.run();
    EXPECT_NEAR(f.previous.fieldPose().x_m, R, 1e-9);   // translation follows heading
    EXPECT_NEAR(f.previous.fieldPose().y_m, R + 0.1, 1e-9);
}

TEST(PlanarMotionIntegrator, PlacementRequestsAreEdgeTriggeredPerOrigin) {
    EstimatorFixture f;
    f.requests.placement.requested = true;
    f.requests.placement.origin    = "command";
    f.requests.placement.sequence  = 1;
    f.requests.placement.pose      = Pose2D{0.61, 0.457, kPi / 2.0};

    StateEstimatorOutput out = f.run();   // no motion, the placement still applies
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_FALSE(out.advanced);
    EXPECT_TRUE(out.robot.initialized);
    EXPECT_EQ(out.robot.anchor_revision, 1u);
    EXPECT_NEAR(out.robot.fieldPose().x_m, 0.61, 1e-12);

    f.putMotion(0.1, 0.0, 0.0);
    out = f.run();   // same request: no re-anchor
    EXPECT_NEAR(out.robot.fieldPose().y_m, 0.457 + 0.1, 1e-9);
    EXPECT_EQ(out.robot.anchor_revision, 1u);

    f.requests.placement.sequence = 2;   // a new command re-anchors
    out                           = f.run();
    EXPECT_NEAR(out.robot.fieldPose().y_m, 0.457, 1e-12);
    EXPECT_EQ(out.robot.anchor_revision, 2u);

    f.requests.placement.origin   = "configuration";   // a different origin is its own edge
    f.requests.placement.sequence = 1;
    out                           = f.run();
    EXPECT_EQ(out.robot.anchor_revision, 3u);
}

TEST(PlanarMotionIntegrator, HeadingReplacesRotationOnlyWhenAlignedAndIndependent) {
    EstimatorFixture f(R"(
        <Estimator type="planar_motion_integrator">
            <Motion observation_id="motion"/>
            <Heading observation_id="heading" interval_tolerance_ms="5"/>
        </Estimator>)");

    f.putMotion(0.0, 0.0, 0.5);   // wheels say half a radian
    f.putHeading(0.25);           // gyro says a quarter, same interval
    f.run();
    EXPECT_NEAR(f.previous.odom_pose.heading_rad, 0.25, 1e-12);

    f.putMotion(0.0, 0.0, 0.5, 100, 105);
    f.putHeading(0.25, 60, 65);   // an unrelated interval is not synchronization
    StateEstimatorOutput out = f.run();
    EXPECT_NEAR(f.previous.odom_pose.heading_rad, 0.75, 1e-12);
    EXPECT_NE(out.diagnostic.find("interval"), std::string::npos);

    f.putMotion(0.0, 0.0, 0.5, 105, 110, "imu");   // the motion already folded this imu
    f.putHeading(0.25, 105, 110, "imu");
    out = f.run();
    EXPECT_NEAR(f.previous.odom_pose.heading_rad, 1.25, 1e-12);
    EXPECT_NE(out.diagnostic.find("shares a source"), std::string::npos);
}

TEST(PlanarMotionIntegrator, UnobservedRotationIsNeverFabricated) {
    EstimatorFixture f(R"(
        <Estimator type="planar_motion_integrator">
            <Motion observation_id="motion"/>
            <Heading observation_id="heading"/>
        </Estimator>)");
    f.putMotion(0.1, 0.0, 0.0, 95, 100, "enc", false);   // rotation not observed
    StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_FALSE(out.advanced);
    EXPECT_NEAR(out.robot.odom_pose.x_m, 0.0, 1e-12);

    f.putMotion(0.1, 0.0, 0.0, 95, 100, "enc", false);
    f.putHeading(0.2, 95, 100);   // an aligned heading completes the step
    out = f.run();
    EXPECT_TRUE(out.advanced);
    EXPECT_NEAR(out.robot.odom_pose.heading_rad, 0.2, 1e-12);
}

TEST(PlanarMotionIntegrator, MissingMotionHoldsPoseAndEffectiveTime) {
    EstimatorFixture f;
    f.putMotion(0.1, 0.0, 0.0);
    f.run();
    const StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kNoData);
    EXPECT_FALSE(out.advanced);
    EXPECT_NEAR(out.robot.fieldPose().x_m, 0.1, 1e-12);
    EXPECT_EQ(out.robot.measuredAt.ms, 100);   // the loop did not refresh the estimate
    EXPECT_TRUE(out.robot.valid);
}

TEST(PlanarMotionIntegrator, InvalidIntervalsAndTimeRegression) {
    EstimatorFixture f;
    f.putMotion(0.1, 0.0, 0.0, 100, 100);   // zero interval
    StateEstimatorOutput out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kFault);
    EXPECT_FALSE(out.advanced);
    EXPECT_NEAR(out.robot.odom_pose.x_m, 0.0, 1e-12);

    f.putMotion(0.1, 0.0, 0.0, 95, 100);
    f.run();
    const uint64_t epoch = f.previous.odometry_epoch;
    f.putMotion(0.1, 0.0, 0.0, 10, 15);   // device time ran backwards: source reboot
    out = f.run();
    EXPECT_EQ(out.status, FunctionStatus::kFault);
    EXPECT_EQ(out.robot.odometry_epoch, epoch + 1);
    EXPECT_NEAR(out.robot.odom_pose.x_m, 0.1, 1e-12);   // the garbage step was not integrated
    EXPECT_FALSE(out.robot.measuredAtHost.isSet());
}

TEST(PlanarMotionIntegrator, DeviceToHostMappingAfterWarmupUsesReceipt) {
    EstimatorFixture     f;
    StateEstimatorOutput out;
    for (int i = 0; i < 8; ++i) {
        // Distinct measurement times, with increasing receipt latency: the
        // first pairing remains the minimum clock offset.
        f.putMotion(0.01, 0.0, 0.0, 95 + 10 * i, 100 + 10 * i);
        f.observations[ObservationId{"motion"}].receivedAt = hostTime(150 + 20 * i);
        out = f.run();
        if (i < 7) {
            EXPECT_FALSE(out.robot.measuredAtHost.isSet()) << i;
            EXPECT_FALSE(out.clock_mapped);
        }
    }
    ASSERT_TRUE(out.robot.measuredAtHost.isSet());
    EXPECT_TRUE(out.clock_mapped);
    EXPECT_EQ(out.robot.measuredAtHost.domain, ClockDomain::kHost);
    EXPECT_EQ(out.robot.measuredAtHost.ms, 220);   // 170 + minimum offset 50
    EXPECT_EQ(out.robot.measuredAt.domain, ClockDomain::kDevice);
}

TEST(PlanarMotionIntegrator, AttitudeFusesUnderThePlanarHeadingAndAges) {
    EstimatorFixture f(R"(
        <Estimator type="planar_motion_integrator">
            <Motion observation_id="motion"/>
            <Attitude observation_id="attitude" max_age_ms="50"/>
        </Estimator>)");
    f.now_ms = 100;
    f.putMotion(0.0, 0.0, 0.3, 95, 100);
    f.putAttitude(degToRad(10.0), 100);
    StateEstimatorOutput out = f.run();
    ASSERT_TRUE(out.robot.attitude.valid);
    EXPECT_FALSE(out.robot.attitude.assumed_level);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    attitudeEuler(out.robot.attitude, roll, pitch, yaw);
    EXPECT_NEAR(radToDeg(roll), 10.0, 1e-9);
    EXPECT_NEAR(yaw, 0.3, 1e-9);   // the source yaw of 1.5 was never applied
    EXPECT_EQ(out.robot.attitude.source, "attitude");
    EXPECT_EQ(out.robot.attitude.measuredAt.ms, 100);

    // the same attitude is still fresh a little later, aged by its own time
    f.now_ms = 130;
    out      = f.run();
    EXPECT_TRUE(out.robot.attitude.valid);
    EXPECT_EQ(out.robot.attitude.measuredAt.ms, 100);   // not refreshed by the loop

    // past max_age_ms the state says assumed level, explicitly
    f.now_ms = 200;
    out      = f.run();
    EXPECT_FALSE(out.robot.attitude.valid);
    EXPECT_TRUE(out.robot.attitude.assumed_level);
    attitudeEuler(out.robot.attitude, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0, 1e-12);
    EXPECT_NEAR(yaw, 0.3, 1e-9);
}

TEST(PlanarMotionIntegrator, ReferencesValidateAtBuild) {
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(R"(
        <Estimator type="planar_motion_integrator">
            <Motion observation_id="ghost"/>
        </Estimator>)"),
              tinyxml2::XML_SUCCESS);
    StateEstimatorInitializationContext context;   // nothing declared
    std::string                         err;
    EXPECT_EQ(PlanarMotionIntegrator::create(ConfigNode{doc.RootElement()}, context, err),
              nullptr);
    EXPECT_NE(err.find("ghost"), std::string::npos);

    // the wrong payload behind a valid id fails too
    context.observations = {RobotObservationOutputDecl{
        ObservationId{"ghost"},
        PayloadDescriptor::of<HeadingIncrement>(payload_names::kHeadingIncrement)}};
    EXPECT_EQ(PlanarMotionIntegrator::create(ConfigNode{doc.RootElement()}, context, err),
              nullptr);
    EXPECT_NE(err.find("different payload"), std::string::npos);
}

TEST(DeviceToHostClock, WarmsUpThenTracksMinimumLatencyOffset) {
    DeviceToHostClock clock;
    EXPECT_FALSE(clock.valid());
    clock.observe(deviceTime(1000), hostTime(2030));   // 30 ms of latency
    clock.observe(deviceTime(1010), hostTime(2015));   // 5 ms, the best pairing
    clock.observe(deviceTime(1020), hostTime(2060));   // batched, 40 ms
    EXPECT_FALSE(clock.valid());
    for (int i = 0; i < 5; ++i) {
        clock.observe(deviceTime(1030 + 10 * i), hostTime(2055 + 10 * i));
    }
    ASSERT_TRUE(clock.valid());
    const MonotonicTime mapped = clock.toHost(deviceTime(1020));
    EXPECT_EQ(mapped.ms, 2025);
    EXPECT_EQ(mapped.domain, ClockDomain::kHost);
    clock.observe(hostTime(5), hostTime(6));   // wrong domain, ignored
    EXPECT_EQ(clock.toHost(deviceTime(1020)).ms, 2025);
}

TEST(MakeLocalization, ValidatesTheAggregateSchema) {
    StageFixture f;
    std::string  err;

    EXPECT_TRUE(f.make(R"(<Localization><Estimator type="noop"/></Localization>)", err)
                    .has_value())
        << err;   // the minimal noop aggregate builds

    EXPECT_FALSE(f.make(R"(<Localization/>)", err).has_value());
    EXPECT_NE(err.find("Estimator"), std::string::npos);

    EXPECT_FALSE(f.make(R"(<Localization><Estimator type="quantum"/></Localization>)", err)
                     .has_value());
    EXPECT_NE(err.find("quantum"), std::string::npos);

    EXPECT_FALSE(f.make(R"(<Localization>
        <Observation id="a" type="scripted_motion"/>
        <Observation id="a" type="scripted_motion"/>
        <Estimator type="noop"/></Localization>)",
                        err)
                     .has_value());
    EXPECT_NE(err.find("duplicate Observation id"), std::string::npos);

    EXPECT_FALSE(f.make(R"(<Localization>
        <Observation id="a" type="scripted_motion"/>
        <Observation id="b" type="scripted_motion"/>
        <Estimator type="noop"/></Localization>)",
                        err)
                     .has_value());
    EXPECT_NE(err.find("duplicate observation output id"), std::string::npos);

    EXPECT_FALSE(f.make(R"(<Localization>
        <Estimator type="planar_motion_integrator"><Motion observation_id="nothing"/></Estimator>
        </Localization>)",
                        err)
                     .has_value());
    EXPECT_NE(err.find("nothing"), std::string::npos);

    EXPECT_FALSE(f.make(R"(<Localization><Estimator type="noop"/><Estimator type="noop"/></Localization>)",
                        err)
                     .has_value());
    EXPECT_NE(err.find("more than one Estimator"), std::string::npos);

    EXPECT_FALSE(f.make(R"(<Localization><Estimator type="noop"/><Bogus/></Localization>)", err)
                     .has_value());
    EXPECT_NE(err.find("Bogus"), std::string::npos);

    EXPECT_FALSE(f.make(R"(<Localization><Estimator type="noop"/>
        <History retention_s="0"/></Localization>)",
                        err)
                     .has_value());
    EXPECT_NE(err.find("retention_s"), std::string::npos);
}

TEST(LocalizationExecutor, PublishesHistoryOnlyForAdvancesAndAnswersLookups) {
    StageFixture f;
    std::string  err;
    auto         exec = f.make(kScriptedLocalization, err);
    ASSERT_TRUE(exec.has_value()) << err;

    f.emit(0.1, 90, 100);
    RobotState r = f.step(*exec);
    EXPECT_TRUE(r.valid);
    EXPECT_NEAR(r.odom_pose.x_m, 0.1, 1e-12);
    EXPECT_EQ(exec->feed()->historySize(), 1u);
    EXPECT_EQ(exec->feed()->status().updates, 1u);
    EXPECT_TRUE(exec->feed()->status().clock_mapped);   // host stamps map trivially

    r = f.step(*exec);   // a quiet cycle: no new history entry
    EXPECT_EQ(exec->feed()->historySize(), 1u);
    EXPECT_EQ(r.measuredAtHost.ms, 100);

    f.emit(0.1, 100, 110);
    f.step(*exec);
    EXPECT_EQ(exec->feed()->historySize(), 2u);
    const PoseLookupResult mid = exec->feed()->poseAt(hostTime(105));
    ASSERT_EQ(mid.status, LookupStatus::kOk);
    EXPECT_NEAR(mid.odom_pose.x_m, 0.15, 1e-12);
    EXPECT_EQ(exec->feed()->poseAt(hostTime(200)).status, LookupStatus::kPending);

    // reset: new odometry epoch, empty history, models reset
    const uint64_t epoch = exec->state().odometry_epoch;
    exec->reset();
    EXPECT_EQ(exec->state().odometry_epoch, epoch + 1);
    EXPECT_FALSE(exec->state().valid);
    EXPECT_EQ(exec->feed()->historySize(), 0u);
    EXPECT_EQ(exec->feed()->latest().odometry_epoch, epoch + 1);
}

TEST(LocalizationExecutor, ConfiguredPlacementAppliesOnceThenCommandsWin) {
    StageFixture f;
    std::string  err;
    auto         exec = f.make(R"(
<Localization>
    <Observation id="scripted" type="scripted_motion"/>
    <Estimator type="planar_motion_integrator">
        <Motion observation_id="scripted"/>
    </Estimator>
    <InitialPlacement x_m="1" y_m="2" heading_deg="90"/>
</Localization>)",
                               err);
    ASSERT_TRUE(exec.has_value()) << err;

    RobotState r = f.step(*exec);
    EXPECT_TRUE(r.initialized);
    EXPECT_EQ(r.anchor_revision, 1u);
    EXPECT_NEAR(r.fieldPose().x_m, 1.0, 1e-12);
    EXPECT_NEAR(r.fieldPose().heading_rad, kPi / 2.0, 1e-12);

    f.emit(0.1, 90, 100);
    r = f.step(*exec);
    EXPECT_NEAR(r.fieldPose().y_m, 2.1, 1e-9);   // forward along field +y
    EXPECT_EQ(r.anchor_revision, 1u);             // the configured placement is one edge

    LocalizationRequests requests;
    requests.placement.requested = true;
    requests.placement.origin    = "command";
    requests.placement.sequence  = 1;
    requests.placement.pose      = Pose2D{0.0, 0.0, 0.0};
    r                            = f.step(*exec, requests);
    EXPECT_EQ(r.anchor_revision, 2u);
    EXPECT_NEAR(r.fieldPose().x_m, 0.0, 1e-12);
}

TEST(LocalizationExecutor, UndeclaredObservationOutputsNeverReachTheEstimator) {
    StageFixture f;
    // a function whose declared output differs from what it publishes
    class Liar : public RobotObservationFunction
    {
    public:
        FunctionStatus run(const RobotObservationInput&, RobotObservationMap& out) override {
            RobotObservationRecord record;
            record.payload =
                TypedPayload::store(HeadingIncrement{}, payload_names::kHeadingIncrement);
            out[ObservationId{"lie"}] = record;
            return FunctionStatus::kOk;
        }
        const ObservationFunctionId& id() const override { return id_; }
        const std::string&           type() const override { return type_; }
        std::vector<RobotObservationOutputDecl> outputs() const override {
            return {RobotObservationOutputDecl{
                ObservationId{"lie"},
                PayloadDescriptor::of<BodyMotionIncrement>(payload_names::kBodyMotionIncrement)}};
        }
        ObservationReadiness readiness() const override { return ObservationReadiness{true, ""}; }
        ObservationFunctionId id_{"liar"};
        std::string           type_ = "liar";
    };
    f.functions.add<RobotObservationMakeFunction>(
        FunctionKey{"liar"},
        [](const ConfigNode&, RobotObservationInitializationContext&,
           std::string&) -> std::unique_ptr<RobotObservationFunction> {
            return std::make_unique<Liar>();
        });
    std::string err;
    auto        exec = f.make(R"(<Localization>
        <Observation id="liar" type="liar"/>
        <Estimator type="planar_motion_integrator"><Motion observation_id="lie"/></Estimator>
        </Localization>)",
                              err);
    ASSERT_TRUE(exec.has_value()) << err;
    f.step(*exec);
    EXPECT_TRUE(exec->lastObservations().empty());
    EXPECT_EQ(f.diagnostics.functions.count("Observation/liar/undeclared_output:lie"), 1u);
}

TEST(LocalizationExecutor, DeferredObservationIsRetriedThenConsumedExactlyOnce) {
    StageFixture f;
    class DeferredEstimator : public StateEstimator {
    public:
        StateEstimatorOutput run(const StateEstimatorInput& in) override {
            StateEstimatorOutput out;
            out.robot = in.previous;
            if (in.observations.empty()) return out;
            if (attempts_++ == 0) {
                out.status = FunctionStatus::kNoData;
                return out; // pending: input must survive the next stage call
            }
            const auto& item = *in.observations.begin();
            const auto* increment = item.second.payload.get<BodyMotionIncrement>();
            out.robot.odom_pose.x_m += increment->dx_m;
            out.robot.valid = true;
            out.robot.measuredAtHost = increment->endAt;
            out.advanced = true;
            out.accepted.push_back(item.first);
            return out;
        }
        const std::string& type() const override { return type_; }
        int attempts_ = 0;
        std::string type_ = "deferred";
    };
    f.functions.add<StateEstimatorMakeFunction>(FunctionKey{"deferred"},
        [](const ConfigNode&, StateEstimatorInitializationContext&, std::string&) {
            return std::make_unique<DeferredEstimator>();
        });
    std::string err;
    auto exec = f.make(R"(<Localization>
        <Observation id="scripted" type="scripted_motion"/>
        <Estimator type="deferred"/></Localization>)", err);
    ASSERT_TRUE(exec.has_value()) << err;
    f.emit(0.5, 90, 100);
    EXPECT_DOUBLE_EQ(f.step(*exec).odom_pose.x_m, 0.0);
    EXPECT_EQ(exec->feed()->historySize(), 0u);
    EXPECT_DOUBLE_EQ(f.step(*exec).odom_pose.x_m, 0.5);
    EXPECT_EQ(exec->feed()->historySize(), 1u);
    EXPECT_DOUBLE_EQ(f.step(*exec).odom_pose.x_m, 0.5);
    EXPECT_TRUE(exec->lastObservations().empty());
}

TEST(PlanarMotionIntegrator, DispositionsDistinguishPendingAcceptedAndRejectedMotion) {
    EstimatorFixture f(R"(<Estimator type="planar_motion_integrator">
        <Motion observation_id="motion"/><Heading observation_id="heading"/>
        </Estimator>)");
    f.putMotion(0.1, 0, 0, 95, 100, "enc", false);
    auto out = f.run();
    EXPECT_TRUE(out.accepted.empty());
    EXPECT_TRUE(out.rejected.empty());
    f.putMotion(0.1, 0, 0, 95, 100, "enc", false);
    f.putHeading(0, 95, 100);
    out = f.run();
    EXPECT_EQ(out.accepted.size(), 2u);
    EXPECT_TRUE(out.advanced);
    f.putMotion(0.1, 0, 0, 95, 100);
    out = f.run();
    EXPECT_FALSE(out.advanced);
    ASSERT_EQ(out.rejected.size(), 1u);
    EXPECT_EQ(out.rejected.front(), ObservationId{"motion"});
    EXPECT_DOUBLE_EQ(out.robot.odom_pose.x_m, 0.1);
}

TEST(PlanarMotionIntegrator, MissingReceiptNeverUsesProcessingTimeForClockMapping) {
    EstimatorFixture f;
    for (int i = 0; i < 12; ++i) {
        f.putMotion(0.01, 0, 0, 95 + i * 5, 100 + i * 5);
        f.observations[ObservationId{"motion"}].receivedAt = {};
        const auto out = f.run();
        EXPECT_TRUE(out.advanced);
        EXPECT_FALSE(out.clock_mapped);
        EXPECT_FALSE(out.robot.measuredAtHost.isSet());
    }
}

TEST(PlanarMotionIntegrator, InvalidQuaternionAndFutureAttitudeDoNotBecomeMeasuredLevel) {
    EstimatorFixture f(R"(<Estimator type="planar_motion_integrator">
        <Motion observation_id="motion"/><Attitude observation_id="attitude"/>
        </Estimator>)");
    f.now_ms = 100;
    f.putAttitude(0.1, 100);
    auto& record = f.observations[ObservationId{"attitude"}];
    AttitudeObservation invalid = *record.payload.get<AttitudeObservation>();
    invalid.q_reference_body = Quaternion{0, 0, 0, 0};
    record.payload = TypedPayload::store(invalid, payload_names::kAttitudeObservation);
    auto out = f.run();
    EXPECT_FALSE(out.robot.attitude.valid);
    ASSERT_EQ(out.rejected.size(), 1u);
    EXPECT_EQ(out.rejected.front(), ObservationId{"attitude"});
    f.putAttitude(0.1, 200);
    out = f.run();
    EXPECT_FALSE(out.robot.attitude.valid);
    EXPECT_TRUE(out.robot.attitude.assumed_level);
}

TEST(PlanarMotionIntegrator, AttitudeCannotBorrowAnUnrelatedDeviceClockMapping) {
    EstimatorFixture f(R"(<Estimator type="planar_motion_integrator">
        <Motion observation_id="motion"/><Attitude observation_id="attitude"/>
        </Estimator>)");
    StateEstimatorOutput out;
    for (int i = 0; i < 10; ++i) {
        const int64_t source_ms = 100 + i * 5;
        f.now_ms = source_ms + 50;
        f.putMotion(0.01, 0, 0, source_ms - 5, source_ms);
        auto& motion_record = f.observations[ObservationId{"motion"}];
        auto motion = *motion_record.payload.get<BodyMotionIncrement>();
        motion.sources.front().clock = "wheel_device";
        motion_record.payload = TypedPayload::store(motion, payload_names::kBodyMotionIncrement);
        AttitudeObservation attitude;
        attitude.q_reference_body = quaternionFromEuler(0.1, 0, 0);
        attitude.measuredAt = deviceTime(source_ms);
        attitude.source.clock = i == 9 ? "wheel_device" : "another_device";
        RobotObservationRecord record;
        record.measuredAt = attitude.measuredAt;
        record.payload = TypedPayload::store(attitude, payload_names::kAttitudeObservation);
        f.observations[ObservationId{"attitude"}] = record;
        out = f.run();
        EXPECT_EQ(out.robot.attitude.valid, i == 9);
    }
    EXPECT_TRUE(out.clock_mapped);
    EXPECT_EQ(out.robot.attitude.measuredAt.ms, 195);
}
