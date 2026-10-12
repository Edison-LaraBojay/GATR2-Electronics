// reference_gtest.cpp

#include "investigatr/reference.h"

#include <cmath>
#include <gtest/gtest.h>
#include <set>
#include <string>

using namespace investigatr;

namespace
{

constexpr MapId kMap = 0x5A17C0DE;

// Shaped like a generated field reference table.
struct TestField : FieldReferences {
    static constexpr Reference Goal    = Reference::object(4, kMap);
    static constexpr Reference Loader  = Reference::object(20, kMap);
    static constexpr Reference Plain   = Reference::object(30, kMap);
    static constexpr Reference Missing = Reference::object(99, kMap);
};

FieldObject landmark(ObjectId id, const Pose& nominal) {
    FieldObject o;
    o.id        = id;
    o.kind      = ObjectKind::kLandmark;
    o.obstacle  = true;
    o.estimated = true;
    o.reference = true;
    o.nominal   = nominal;
    o.box       = Box{Pose{}, 0.15, 0.15};
    o.source    = EstimateSource::kNominal;
    o.valid     = true;
    o.pose      = nominal;
    return o;
}

Field field() {
    Field f;
    f.generation   = 3;
    f.map.id       = kMap;
    f.map.revision = 2;
    f.bounds       = Bounds{0, 0, 3.6, 3.6};
    f.frame        = 11;
    f.received_at  = 100.0;

    FieldObject goal = landmark(4, Pose{2.0, 1.0, kPi / 2.0});
    goal.source      = EstimateSource::kObserved;
    goal.pose        = Pose{2.02, 0.99, kPi / 2.0 + 0.01};
    goal.age_known   = true;
    goal.age         = 0.2;

    FieldObject loader = landmark(20, Pose{0.05, 0.3, 0.0});
    loader.kind        = ObjectKind::kFixed;
    loader.estimated   = false;

    FieldObject plain = landmark(30, Pose{1.0, 1.0, 0.0});
    plain.reference   = false;

    f.objects = {goal, loader, plain};
    return f;
}

void expectPose(const Pose& got, const Pose& want) {
    EXPECT_NEAR(got.x, want.x, 1e-12);
    EXPECT_NEAR(got.y, want.y, 1e-12);
    EXPECT_NEAR(wrapAngle(got.heading - want.heading), 0.0, 1e-12);
}

const ReferencePolicy kAnyEstimate{false, 0};

} // namespace

TEST(Reference, TableConstants) {
    EXPECT_EQ(TestField::Origin.kind, Reference::Kind::kOrigin);
    EXPECT_EQ(TestField::RobotAtStart.kind, Reference::Kind::kRobotAtStart);
    EXPECT_EQ(TestField::Goal.kind, Reference::Kind::kObject);
    EXPECT_EQ(TestField::Goal.object_id, 4);
    EXPECT_EQ(TestField::Goal.map, kMap);
    EXPECT_EQ(Reference{}.kind, Reference::Kind::kOrigin);
}

TEST(Reference, OriginNeedsNoField) {
    const Pose     rel{1.0, 0.5, 0.3};
    const Resolved r = resolve(TestField::Origin, rel, nullptr, Pose{}, 0, {}, 0.0);
    EXPECT_EQ(r.status, ResolveStatus::kOk);
    expectPose(r.reference, Pose{});
    expectPose(r.destination, rel);
    EXPECT_EQ(r.source, EstimateSource::kNone);
}

TEST(Reference, RobotAtStartComposesOnce) {
    const Pose     start{1.0, 2.0, kPi / 2.0};
    const Resolved r =
        resolve(TestField::RobotAtStart, Pose{0.5, 0.0, 0.0}, nullptr, start, 0, {}, 0.0);
    EXPECT_EQ(r.status, ResolveStatus::kOk);
    expectPose(r.reference, start);
    expectPose(r.destination, Pose{1.0, 2.5, kPi / 2.0});
}

TEST(Reference, ObjectAppliesTranslationAndRotationOnce) {
    const Field    f = field();
    const Resolved r = resolve(TestField::Goal, Pose{-0.4, 0.1, kPi}, &f, Pose{}, 11, {}, 100.1);
    ASSERT_EQ(r.status, ResolveStatus::kOk);
    EXPECT_EQ(r.source, EstimateSource::kObserved);
    const Pose& lm = f.find(4)->pose;
    expectPose(r.reference, lm);
    // 0.4 m behind the goal along its facing, 0.1 m to its left, facing it.
    const double c = std::cos(lm.heading);
    const double s = std::sin(lm.heading);
    expectPose(r.destination,
               Pose{lm.x - 0.4 * c - 0.1 * s, lm.y - 0.4 * s + 0.1 * c, lm.heading + kPi});
}

TEST(Reference, ObjectNeedsCompleteField) {
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, nullptr, Pose{}, 0, {}, 0).status,
              ResolveStatus::kNoField);
    Field empty      = field();
    empty.generation = 0;
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &empty, Pose{}, 0, {}, 0).status,
              ResolveStatus::kNoField);
}

TEST(Reference, MapIdentityChecked) {
    const Field f = field();
    EXPECT_EQ(resolve(Reference::object(4, kMap + 1), Pose{}, &f, Pose{}, 0, {}, 100).status,
              ResolveStatus::kMapMismatch);
    EXPECT_EQ(resolve(TestField::Missing, Pose{}, &f, Pose{}, 0, {}, 100).status,
              ResolveStatus::kUnknownObject);
    EXPECT_EQ(resolve(TestField::Plain, Pose{}, &f, Pose{}, 0, kAnyEstimate, 100).status,
              ResolveStatus::kNotReference);
}

TEST(Reference, EstimatePolicy) {
    Field f = field();

    // Observed goal passes the default policy.
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &f, Pose{}, 11, {}, 100.1).status,
              ResolveStatus::kOk);

    // Nominal only: refused unless the policy accepts it.
    FieldObject& goal = f.objects[0];
    goal.source       = EstimateSource::kNominal;
    goal.pose         = goal.nominal;
    goal.age_known    = false;
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &f, Pose{}, 11, {}, 100).status,
              ResolveStatus::kNotObserved);
    const Resolved nominal = resolve(TestField::Goal, Pose{}, &f, Pose{}, 11, kAnyEstimate, 100);
    EXPECT_EQ(nominal.status, ResolveStatus::kOk);
    EXPECT_EQ(nominal.source, EstimateSource::kNominal);
    expectPose(nominal.destination, goal.nominal);

    // No estimate at all.
    goal.source = EstimateSource::kNone;
    goal.valid  = false;
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &f, Pose{}, 11, kAnyEstimate, 100).status,
              ResolveStatus::kNoEstimate);
}

TEST(Reference, FixedObjectNeedsNoObservation) {
    const Field    f = field();
    const Resolved r = resolve(TestField::Loader, Pose{0.3, 0.0, 0.0}, &f, Pose{}, 11, {}, 100);
    EXPECT_EQ(r.status, ResolveStatus::kOk);
    EXPECT_EQ(r.source, EstimateSource::kNominal);
    expectPose(r.destination, Pose{0.35, 0.3, 0.0});
}

TEST(Reference, ObservedAgeLimit) {
    const Field     f = field();
    ReferencePolicy policy;
    policy.max_age = 0.5;
    // age 0.2 at receipt plus 0.25 since: fresh.
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &f, Pose{}, 11, policy, 100.25).status,
              ResolveStatus::kOk);
    // plus 0.35 since: stale.
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &f, Pose{}, 11, policy, 100.35).status,
              ResolveStatus::kStale);
    // No limit.
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &f, Pose{}, 11, {}, 1000.0).status,
              ResolveStatus::kOk);
}

TEST(Reference, ObservedFrameChecked) {
    const Field f = field();
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &f, Pose{}, 12, {}, 100).status,
              ResolveStatus::kFrameMismatch);
    // 0 skips the check.
    EXPECT_EQ(resolve(TestField::Goal, Pose{}, &f, Pose{}, 0, {}, 100).status, ResolveStatus::kOk);
    // Nominal poses do not depend on the robot frame.
    EXPECT_EQ(resolve(TestField::Loader, Pose{}, &f, Pose{}, 12, {}, 100).status,
              ResolveStatus::kOk);
}

TEST(Reference, StatusStrings) {
    const ResolveStatus all[] = {
        ResolveStatus::kOk,
        ResolveStatus::kNoField,
        ResolveStatus::kMapMismatch,
        ResolveStatus::kUnknownObject,
        ResolveStatus::kNotReference,
        ResolveStatus::kNotObserved,
        ResolveStatus::kNoEstimate,
        ResolveStatus::kStale,
        ResolveStatus::kFrameMismatch,
    };
    std::set<std::string> seen;
    for (ResolveStatus s : all) {
        const std::string text = toString(s);
        EXPECT_NE(text, "?");
        EXPECT_TRUE(seen.insert(text).second) << text;
    }
}
