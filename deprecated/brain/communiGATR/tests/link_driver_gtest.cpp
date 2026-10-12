// link_driver_gtest.cpp
// LinkDriver over the client, fake bus and fake Pi: robot status order,
// unit conversion, ages, frame generations shared by robot and field, the
// Field built from the published documents, placement conversion and path
// reports.

#include "communigatr/link_driver.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "investigatr/reference.h"
#include "sim/link_rig.h"

using namespace communigatr;
using investigatr::EstimateSource;
using investigatr::Field;
using investigatr::kPi;
using investigatr::ObjectKind;
using investigatr::Pose;
using investigatr::RobotState;
using investigatr::RobotStatus;

namespace
{

constexpr Seconds kLimit = 5.0;
constexpr double  kEps   = 1e-9;
constexpr uint8_t kLocalized =
    translagatr::kRobotPoseValid | translagatr::kRobotLocalized | translagatr::kRobotAgeKnown;

void openSession(LinkRig& rig) {
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().ready(); }, kLimit));
}

void place(LinkRig& rig, const Pose& pose) {
    const PlacementTicket ticket = rig.driver().place(pose);
    ASSERT_NE(ticket, 0u);
    ASSERT_TRUE(rig.runUntil(
        [&] { return rig.client().placementResult(ticket) == PlacementResult::kApplied; },
        kLimit));
}

translagatr::BrainRequest lastSetPose(const FakePi& pi) {
    translagatr::BrainRequest last;
    for (const translagatr::BrainRequest& r : pi.requests()) {
        if (r.op == translagatr::kOpSetPose) {
            last = r;
        }
    }
    return last;
}

void expectPose(const Pose& actual, const Pose& expected) {
    EXPECT_NEAR(actual.x, expected.x, kEps);
    EXPECT_NEAR(actual.y, expected.y, kEps);
    EXPECT_NEAR(investigatr::wrapAngle(actual.heading - expected.heading), 0.0, kEps);
}

RobotStatus statusAfter(LinkRig& rig, Seconds dt = 0.1) {
    rig.run(dt);
    return rig.driver().robot(rig.now()).status;
}

ClientConfig withProfile() {
    RobotProfile p;
    p.topology       = LocalizationTopology::kTwoWheelImu;
    p.wheels         = {{0, 0.024, 2048, 0.0, 0.10, 0.0, false},
                        {1, 0.024, 2048, -0.08, 0.0, kPi / 2, false}};
    p.imu_source     = ImuSource::kBrainVex;
    p.vex_smart_port = 1;
    p.footprint      = {0.2, 0.2, 0.2, 0.2};
    ClientConfig config;
    config.profile = makeProfileDocument(p);
    return config;
}

} // namespace

// ---------------------------------------------------------------------------
// Robot state
// ---------------------------------------------------------------------------

TEST(LinkDriver, StatusFollowsTheDocumentedOrder) {
    LinkRig rig(withProfile());
    rig.pi.setProfileMode(true);
    rig.pi.setProfileApplyDelay(30);
    LinkDriver& driver = rig.driver();

    RobotState s = driver.robot(0.0);
    EXPECT_EQ(s.status, RobotStatus::kNoLink);
    EXPECT_FALSE(s.connected);
    EXPECT_TRUE(std::isinf(s.link_age));
    EXPECT_EQ(s.frame, 0u);

    openSession(rig);
    EXPECT_EQ(driver.robot(rig.now()).status, RobotStatus::kNoProfile);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().profileApplied(); }, kLimit));
    EXPECT_EQ(statusAfter(rig), RobotStatus::kUnplaced);

    for (uint8_t cal : {translagatr::kCalibrationRunning, translagatr::kCalibrationWaitingStill,
                        translagatr::kCalibrationWaitingData}) {
        rig.pi.robot().calibration = cal;
        EXPECT_EQ(statusAfter(rig), RobotStatus::kCalibrating) << int(cal);
    }
    for (uint8_t cal : {translagatr::kCalibrationNone, translagatr::kCalibrationDone,
                        translagatr::kCalibrationFailed}) {
        rig.pi.robot().calibration = cal;
        EXPECT_EQ(statusAfter(rig), RobotStatus::kUnplaced) << int(cal);
    }

    place(rig, Pose{0.5, 0.5, 0.0});
    s = driver.robot(rig.now());
    EXPECT_EQ(s.status, RobotStatus::kValid);
    EXPECT_TRUE(s.valid());
    EXPECT_TRUE(s.connected);
    EXPECT_NE(s.frame, 0u);

    rig.pi.robot().robot_flags = kLocalized; // pose without an anchor origin
    EXPECT_EQ(statusAfter(rig), RobotStatus::kUnplaced);
    rig.pi.robot().robot_flags = translagatr::kRobotLocalized | translagatr::kRobotAgeKnown |
                                 translagatr::kRobotAnchorCommand;
    EXPECT_EQ(statusAfter(rig), RobotStatus::kNoPose);
    rig.pi.robot().robot_flags = translagatr::kRobotPoseValid | translagatr::kRobotLocalized |
                                 translagatr::kRobotAnchorCommand;
    EXPECT_EQ(statusAfter(rig), RobotStatus::kNoPose);
    rig.pi.robot().robot_flags = kLocalized | translagatr::kRobotAnchorCommand;
    EXPECT_EQ(statusAfter(rig), RobotStatus::kValid);

    // Link down.
    rig.bus.setPiPresent(false);
    ASSERT_TRUE(rig.runUntil([&] { return !rig.client().connected(rig.now()); }, kLimit));
    s = driver.robot(rig.now());
    EXPECT_EQ(s.status, RobotStatus::kNoLink);
    EXPECT_FALSE(s.connected);
    EXPECT_GT(s.link_age, rig.client().config().link_timeout);
}

TEST(LinkDriver, ConfiguredAnchorOnlyWhenAccepted) {
    LinkRig rig;
    openSession(rig);
    LinkDriverConfig accept;
    accept.accept_configured_anchor = true;
    LinkDriver configured(rig.client(), accept);
    rig.pi.robot().robot_flags = kLocalized | translagatr::kRobotAnchorConfigured;
    rig.run(0.1);
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kUnplaced);
    EXPECT_EQ(configured.robot(rig.now()).status, RobotStatus::kValid);
}

TEST(LinkDriver, PendingPlacementHidesThePose) {
    LinkRig rig;
    openSession(rig);
    place(rig, Pose{0.5, 0.5, 0.0});
    rig.pi.setApplyDelay(10);
    const PlacementTicket ticket = rig.driver().place(Pose{1.0, -1.0, kPi / 2});
    ASSERT_NE(ticket, 0u);
    bool valid_seen = false;
    while (rig.client().placementPending()) {
        rig.step();
        if (rig.client().placementPending()) {
            valid_seen = valid_seen || rig.driver().robot(rig.now()).valid();
        }
    }
    EXPECT_FALSE(valid_seen);
    ASSERT_EQ(rig.client().placementResult(ticket), PlacementResult::kApplied);
    expectPose(rig.driver().robot(rig.now()).pose, Pose{1.0, -1.0, kPi / 2});
}

TEST(LinkDriver, ConvertsWireUnitsAndAges) {
    FakeBusConfig bus;
    bus.reply_delay = 0.010;
    LinkRig rig({}, bus);
    openSession(rig);
    place(rig, Pose{1.2344, -0.5676, kPi / 2.0});
    const translagatr::BrainRequest sent = lastSetPose(rig.pi);
    EXPECT_EQ(sent.x_mm, 1234);
    EXPECT_EQ(sent.y_mm, -568);
    EXPECT_EQ(sent.heading_cdeg, 9000);
    expectPose(rig.driver().robot(rig.now()).pose, Pose{1.234, -0.568, kPi / 2.0});

    rig.pi.robot().x_mm         = 1500;
    rig.pi.robot().y_mm         = -250;
    rig.pi.robot().heading_cdeg = 18000;
    rig.pi.robot().robot_age_ms = 40;
    rig.run(0.1);
    const StateSample sample = rig.client().state();
    const Seconds     later  = rig.now() + 0.05;
    const RobotState  s      = rig.driver().robot(later);
    expectPose(s.pose, Pose{1.5, -0.25, kPi});
    EXPECT_NEAR(s.pose.heading, kPi, kEps);
    EXPECT_GT(sample.round_trip, bus.reply_delay);
    EXPECT_NEAR(s.age, 0.040 + sample.round_trip + (later - sample.received_at), kEps);
}

TEST(LinkDriver, PlacementIsRoundedWrappedAndRangeChecked) {
    LinkRig      rig;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    LinkDriver&  driver = rig.driver();
    openSession(rig);
    EXPECT_EQ(driver.place(Pose{nan, 0.0, 0.0}), 0u);
    EXPECT_EQ(driver.place(Pose{0.0, 0.0, inf}), 0u);
    EXPECT_EQ(driver.place(Pose{3.0e6, 0.0, 0.0}), 0u); // beyond int32 mm
    EXPECT_FALSE(rig.client().placementPending());
    place(rig, Pose{0.0, 0.0, -kPi + 1e-9});
    EXPECT_EQ(lastSetPose(rig.pi).heading_cdeg, 18000);
    place(rig, Pose{0.0, 0.0, 1.5 * kPi});
    EXPECT_EQ(lastSetPose(rig.pi).heading_cdeg, -9000);
}

TEST(LinkDriver, FrameGenerationFollowsInstanceSessionEpochAndAnchor) {
    LinkRig rig;
    auto    frame = [&] { return rig.driver().robot(rig.now()).frame; };
    EXPECT_EQ(frame(), 0u);
    openSession(rig);
    EXPECT_EQ(frame(), 0u); // unplaced
    place(rig, Pose{0.1, 0.2, kPi / 6.0});
    const auto g1 = frame();
    EXPECT_NE(g1, 0u);
    rig.run(0.2);
    EXPECT_EQ(frame(), g1);

    place(rig, Pose{0.3, 0.2, 0.0}); // new anchor
    const auto g2 = frame();
    EXPECT_GT(g2, g1);

    rig.pi.robot().odometry_epoch += 1; // Pico restart
    rig.run(0.1);
    const auto g3 = frame();
    EXPECT_GT(g3, g2);
    EXPECT_TRUE(rig.driver().robot(rig.now()).valid());

    // Another HELLO takes the session. The pose carries over, the frame does not.
    translagatr::BrainRequest hello;
    hello.op         = translagatr::kOpHello;
    hello.request_id = 1;
    hello.nonce      = 0xABCDEF;
    ASSERT_EQ(rig.pi.answer(hello).result, translagatr::kResultOk);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().session_losses == 1; }, kLimit));
    EXPECT_EQ(frame(), 0u);
    openSession(rig);
    const auto g4 = frame();
    EXPECT_GT(g4, g3);
    expectPose(rig.driver().robot(rig.now()).pose, Pose{0.3, 0.2, 0.0});

    rig.pi.restart(0x1234); // anchor gone
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().stats().pi_restarts == 1; }, kLimit));
    openSession(rig);
    EXPECT_EQ(frame(), 0u);
    EXPECT_EQ(rig.driver().robot(rig.now()).status, RobotStatus::kUnplaced);
}

// ---------------------------------------------------------------------------
// Field
// ---------------------------------------------------------------------------

TEST(LinkDriver, FieldCarriesMapObjectsBoxesAndEstimatesInSiUnits) {
    LinkRig   rig;
    FakeField fake = makeFakeField(6, 9);
    fake.min_x_mm  = -100;
    fake.max_y_mm  = 3600;
    rig.pi.setField(fake);
    openSession(rig);
    place(rig, Pose{1.0, 1.0, 0.0});

    // Landmark 10 observed, landmark 50 lost, the rest nominal.
    std::vector<translagatr::FieldEstimateRecord> records = rig.pi.nominalRecords();
    records[0].source       = translagatr::kEstimateSourceObserved;
    records[0].x_mm         = 350;
    records[0].y_mm         = 280;
    records[0].heading_cdeg = -9000;
    records[0].age_ms       = 120;
    records[4].source       = translagatr::kEstimateSourceNone;
    records[4].flags        = 0;
    const Seconds  t_pub    = rig.now();
    const uint32_t id       = rig.pi.publishEstimate(records);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().field().estimate_id == id; }, kLimit));

    Field field;
    ASSERT_TRUE(rig.driver().field(field));
    const FieldPublication& p = rig.client().field();
    EXPECT_EQ(field.generation, p.generation);
    EXPECT_EQ(field.map.id, rig.pi.mapId());
    EXPECT_EQ(field.map.revision, 9);
    EXPECT_NEAR(field.bounds.min_x, -0.1, kEps);
    EXPECT_NEAR(field.bounds.max_x, 3.658, kEps);
    EXPECT_NEAR(field.bounds.max_y, 3.6, kEps);
    EXPECT_EQ(field.received_at, p.completed_at);
    ASSERT_EQ(field.objects.size(), 6u);

    const investigatr::FieldObject& lm = field.objects[0];
    EXPECT_EQ(lm.id, 10);
    EXPECT_EQ(lm.kind, ObjectKind::kLandmark);
    EXPECT_TRUE(lm.obstacle);
    EXPECT_TRUE(lm.estimated);
    EXPECT_TRUE(lm.reference);
    expectPose(lm.nominal, Pose{0.3, 0.3, -3 * kPi / 4});
    expectPose(lm.box.center, Pose{0.02, -0.01, kPi / 4});
    EXPECT_NEAR(lm.box.length, 0.15, kEps);
    EXPECT_NEAR(lm.box.width, 0.10, kEps);
    EXPECT_EQ(lm.source, EstimateSource::kObserved);
    EXPECT_TRUE(lm.valid);
    expectPose(lm.pose, Pose{0.35, 0.28, -kPi / 2});
    EXPECT_TRUE(lm.age_known);
    // Observation age at completion: the Pi's age plus the time since the
    // snapshot, bounded by the poll before the id appeared (one poll period
    // and one exchange before the publication at most).
    EXPECT_GE(lm.age, 0.120 + (field.received_at - t_pub));
    EXPECT_LE(lm.age, 0.120 + (field.received_at - t_pub) + 0.035);

    const investigatr::FieldObject& fixed = field.objects[1];
    EXPECT_EQ(fixed.kind, ObjectKind::kFixed);
    EXPECT_TRUE(fixed.obstacle);
    EXPECT_FALSE(fixed.estimated);
    EXPECT_EQ(fixed.source, EstimateSource::kNominal);
    EXPECT_FALSE(fixed.age_known);
    EXPECT_EQ(fixed.age, 0.0);
    expectPose(fixed.pose, fixed.nominal);

    const investigatr::FieldObject& lost = field.objects[4];
    EXPECT_EQ(lost.source, EstimateSource::kNone);
    EXPECT_FALSE(lost.valid);

    const investigatr::FieldObject& reference_only = field.objects[5];
    EXPECT_FALSE(reference_only.obstacle);
    EXPECT_TRUE(reference_only.reference);
    EXPECT_EQ(reference_only.box.length, 0.0);

    EXPECT_EQ(field.find(30), &field.objects[2]);
    EXPECT_EQ(field.find(31), nullptr);

    // Same generation: not copied again; the caller's copy stays as it is.
    field.objects.clear();
    EXPECT_TRUE(rig.driver().field(field));
    EXPECT_TRUE(field.objects.empty());
}

TEST(LinkDriver, FieldFrameMatchesTheRobotFrameUnderTheSameAnchor) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(4));
    openSession(rig);
    place(rig, Pose{1.0, 1.0, 0.0});
    const uint32_t id = rig.pi.publishNominalEstimate(); // under the new anchor
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().field().estimate_id == id; }, kLimit));
    Field field;
    ASSERT_TRUE(rig.driver().field(field));
    const RobotState robot = rig.driver().robot(rig.now());
    ASSERT_TRUE(robot.valid());
    EXPECT_EQ(field.frame, robot.frame);

    // A new anchor: the published estimate belongs to the old frame until the
    // Pi takes one under the new anchor.
    place(rig, Pose{2.0, 1.0, 0.0});
    Field stale = field;
    rig.driver().field(stale);
    EXPECT_NE(stale.frame, rig.driver().robot(rig.now()).frame);
    const uint32_t next = rig.pi.publishNominalEstimate();
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().field().estimate_id == next; }, kLimit));
    ASSERT_TRUE(rig.driver().field(field));
    EXPECT_EQ(field.frame, rig.driver().robot(rig.now()).frame);
    EXPECT_GT(field.generation, stale.generation);
}

TEST(LinkDriver, EstimateOlderThanTheSessionHasUnknownAge) {
    LinkRig rig;
    rig.pi.setField(makeFakeField(2));
    std::vector<translagatr::FieldEstimateRecord> records = rig.pi.nominalRecords();
    records[0].source = translagatr::kEstimateSourceObserved;
    records[0].age_ms = 10;
    rig.pi.publishEstimate(records); // before the Brain connects
    openSession(rig);
    ASSERT_TRUE(rig.runUntil([&] { return rig.client().field().generation != 0; }, kLimit));
    Field field;
    ASSERT_TRUE(rig.driver().field(field));
    EXPECT_TRUE(field.objects[0].age_known);
    EXPECT_TRUE(std::isinf(field.objects[0].age));

    // Resolution treats it as stale for any age limit.
    investigatr::ReferencePolicy policy;
    policy.max_age = 5.0;
    const investigatr::Resolved r = investigatr::resolve(
        investigatr::Reference::object(10, field.map.id), Pose{}, &field, Pose{}, 0, policy,
        rig.now());
    EXPECT_EQ(r.status, investigatr::ResolveStatus::kStale);
}

// ---------------------------------------------------------------------------
// Path reports
// ---------------------------------------------------------------------------

TEST(LinkDriver, PathSinkReportsTranslationEndPoints) {
    LinkRig rig;
    openSession(rig);
    investigatr::Path path;
    path.mode = investigatr::PlanMode::kAvoiding;
    investigatr::PathSegment turn;
    turn.kind  = investigatr::SegmentKind::kTurn;
    turn.start = Pose{0.1, 0.2, 0.0};
    turn.end   = Pose{0.1, 0.2, kPi / 2};
    investigatr::PathSegment up;
    up.start = Pose{0.1, 0.2, kPi / 2};
    up.end   = Pose{0.1, 1.2, kPi / 2};
    investigatr::PathSegment right;
    right.start = up.end;
    right.end   = Pose{0.9, 1.2, 0.0};
    path.segments = {turn, up, turn, right};
    path.segments[2].start = up.end;
    path.segments[2].end   = Pose{0.1, 1.2, 0.0};

    investigatr::PathSink& sink = rig.driver();
    sink.reportPath(42, path);
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.path().have; }, kLimit));
    const FakePath& got = rig.pi.path();
    EXPECT_EQ(got.command_id, 42u);
    EXPECT_EQ(got.mode, translagatr::kPathAvoiding);
    ASSERT_EQ(got.points.size(), 3u);
    EXPECT_EQ(got.points[0].x_mm, 100);
    EXPECT_EQ(got.points[0].y_mm, 200);
    EXPECT_EQ(got.points[1].y_mm, 1200);
    EXPECT_EQ(got.points[2].x_mm, 900);

    // Direct mode, then an empty path clears.
    path.mode = investigatr::PlanMode::kDirect;
    sink.reportPath(43, path);
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.path().command_id == 43; }, kLimit));
    EXPECT_EQ(rig.pi.path().mode, translagatr::kPathDirect);
    sink.reportPath(43, investigatr::Path{});
    ASSERT_TRUE(rig.runUntil([&] { return !rig.pi.path().have; }, kLimit));
}

TEST(LinkDriver, LongPathsAreThinnedAndNonFinitePathsDropped) {
    LinkRig rig;
    openSession(rig);
    investigatr::Path path;
    for (int i = 0; i < 30; ++i) {
        investigatr::PathSegment s;
        s.start = Pose{0.1 * i, 0.0, 0.0};
        s.end   = Pose{0.1 * (i + 1), 0.0, 0.0};
        path.segments.push_back(s);
    }
    rig.driver().reportPath(7, path);
    ASSERT_TRUE(rig.runUntil([&] { return rig.pi.path().have; }, kLimit));
    ASSERT_EQ(rig.pi.path().points.size(), translagatr::kPathReportMaxPoints);
    EXPECT_EQ(rig.pi.path().points.front().x_mm, 0);
    EXPECT_EQ(rig.pi.path().points.back().x_mm, 3000);

    path.segments[3].end.x = std::numeric_limits<double>::quiet_NaN();
    const uint32_t before  = rig.client().stats().path_reports;
    rig.driver().reportPath(8, path);
    rig.run(0.3);
    EXPECT_EQ(rig.client().stats().path_reports, before);
    EXPECT_EQ(rig.pi.path().command_id, 7u);
}
