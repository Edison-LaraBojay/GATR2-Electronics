// wheel_calibration_gtest.cpp
// Measured-travel wheel calibration: accepted trials, every rejection, the
// repeatability gate, and independent calibration of perpendicular wheels.

#include "communigatr/wheel_calibration.h"

#include <cmath>
#include <gtest/gtest.h>

using namespace communigatr;

namespace
{

// Forward wheel on port 0, sideways wheel on port 1.
CalibrationSnapshot snapshot(Meters forward, Meters sideways, Radians heading = 0,
                             uint16_t forward_disc = 0, uint32_t profile = 7) {
    CalibrationSnapshot s;
    s.profile_id    = profile;
    s.heading_valid = true;
    s.heading       = heading;
    WheelSample f;
    f.port          = 0;
    f.fresh         = true;
    f.valid         = true;
    f.discontinuity = forward_disc;
    f.travel        = forward;
    f.age           = 0.02;
    WheelSample l   = f;
    l.port          = 1;
    l.discontinuity = 0;
    l.travel        = sideways;
    s.wheels        = {f, l};
    return s;
}

// Forward wheel 0.15 m left of the origin, sideways wheel 0.15 m ahead.
TrackingWheel forwardWheel() {
    TrackingWheel w;
    w.encoder_port = 0;
    w.y            = 0.15;
    return w;
}

TrackingWheel sidewaysWheel() {
    TrackingWheel w;
    w.encoder_port = 1;
    w.x            = 0.15;
    w.angle        = investigatr::kPi / 2;
    return w;
}

} // namespace

TEST(WheelCalibration, ReadingConversion) {
    translagatr::WheelReading r;
    r.port          = 2;
    r.flags         = translagatr::kWheelFresh | translagatr::kWheelValid;
    r.discontinuity = 9;
    r.counts        = -123;
    r.travel_um     = 1234567;
    r.age_ms        = 40;
    const WheelSample s = fromReading(r);
    EXPECT_EQ(s.port, 2);
    EXPECT_TRUE(s.fresh);
    EXPECT_TRUE(s.valid);
    EXPECT_EQ(s.discontinuity, 9);
    EXPECT_EQ(s.counts, -123);
    EXPECT_NEAR(s.travel, 1.234567, 1e-12);
    EXPECT_NEAR(s.age, 0.04, 1e-12);
}

TEST(WheelCalibration, RepeatableTrialsProposeTheMeanScale) {
    WheelCalibration cal(forwardWheel(), {1});
    const double     measured[] = {0.980, 0.981, 0.979};
    Meters           at         = 0.3;
    for (double m : measured) {
        ASSERT_EQ(cal.start(snapshot(at, 0.01)), TrialStatus::kAccepted);
        ASSERT_EQ(cal.finish(snapshot(at + m, 0.012), 1.0), TrialStatus::kAccepted);
        at += m;
    }
    const CalibrationProposal p = cal.proposal();
    EXPECT_TRUE(p.ready);
    EXPECT_EQ(p.trials, 3u);
    const double expected = (1 / 0.980 + 1 / 0.981 + 1 / 0.979) / 3;
    EXPECT_NEAR(p.scale, expected, 1e-12);
    EXPECT_LT(p.spread, 0.01);
}

TEST(WheelCalibration, OneTrialIsNeverEnough) {
    WheelCalibration cal(forwardWheel(), {1});
    ASSERT_EQ(cal.start(snapshot(0, 0)), TrialStatus::kAccepted);
    ASSERT_EQ(cal.finish(snapshot(0.98, 0), 1.0), TrialStatus::kAccepted);
    EXPECT_FALSE(cal.proposal().ready);
}

TEST(WheelCalibration, SpreadTooLargeIsNotReady) {
    WheelCalibration cal(forwardWheel(), {1});
    const double     measured[] = {0.97, 0.99, 1.00};
    for (double m : measured) {
        ASSERT_EQ(cal.start(snapshot(0, 0)), TrialStatus::kAccepted);
        ASSERT_EQ(cal.finish(snapshot(m, 0), 1.0), TrialStatus::kAccepted);
    }
    const CalibrationProposal p = cal.proposal();
    EXPECT_FALSE(p.ready);
    EXPECT_GT(p.spread, 0.01);
    cal.clear();
    EXPECT_EQ(cal.proposal().trials, 0u);
}

TEST(WheelCalibration, RejectsInvalidRuns) {
    WheelCalibration cal(forwardWheel(), {1});
    EXPECT_EQ(cal.finish(snapshot(1, 0), 1.0), TrialStatus::kNotStarted);

    auto run = [&cal](const CalibrationSnapshot& a, const CalibrationSnapshot& b, Meters ref) {
        EXPECT_EQ(cal.start(a), TrialStatus::kAccepted);
        return cal.finish(b, ref);
    };
    EXPECT_EQ(run(snapshot(0, 0), snapshot(1.0, 0), 0.2), TrialStatus::kReferenceInvalid);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(1.0, 0), std::nan("")), TrialStatus::kReferenceInvalid);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(1.0, 0), 9.0), TrialStatus::kReferenceInvalid);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(1.0, 0, 0.01), 1.0), TrialStatus::kRotated);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(1.0, 0.2), 1.0), TrialStatus::kNotStraight);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(1.0, 0, 0, 1), 1.0), TrialStatus::kDiscontinuity);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(1.0, 0, 0, 0, 8), 1.0), TrialStatus::kProfileChanged);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(-1.0, 0), 1.0), TrialStatus::kWrongDirection);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(0.01, 0), 1.0), TrialStatus::kTooShort);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(0.8, 0), 1.0), TrialStatus::kOutOfRange);
    EXPECT_EQ(run(snapshot(0, 0), snapshot(1.2, 0), 1.0), TrialStatus::kOutOfRange);
    EXPECT_TRUE(cal.trials().empty());
}

TEST(WheelCalibration, StaleMissingAndInvalidReadings) {
    WheelCalibration    cal(forwardWheel(), {1});
    CalibrationSnapshot stale = snapshot(0, 0);
    stale.wheels[0].fresh     = false;
    EXPECT_EQ(cal.start(stale), TrialStatus::kReadingStale);
    CalibrationSnapshot old = snapshot(0, 0);
    old.wheels[0].age       = 1.0;
    EXPECT_EQ(cal.start(old), TrialStatus::kReadingStale);
    CalibrationSnapshot empty = snapshot(0, 0);
    empty.wheels[0].valid     = false;
    EXPECT_EQ(cal.start(empty), TrialStatus::kReadingInvalid);
    CalibrationSnapshot missing = snapshot(0, 0);
    missing.wheels.erase(missing.wheels.begin());
    EXPECT_EQ(cal.start(missing), TrialStatus::kReadingMissing);
    CalibrationSnapshot cross_stale = snapshot(0, 0);
    cross_stale.wheels[1].fresh     = false;
    EXPECT_EQ(cal.start(cross_stale), TrialStatus::kReadingStale);
    CalibrationSnapshot no_heading = snapshot(0, 0);
    no_heading.heading_valid       = false;
    EXPECT_EQ(cal.start(no_heading), TrialStatus::kRotated);
    EXPECT_FALSE(cal.started());

    ASSERT_EQ(cal.start(snapshot(0, 0)), TrialStatus::kAccepted);
    CalibrationSnapshot end = snapshot(1.0, 0);
    end.wheels[0].age       = 0.5;
    EXPECT_EQ(cal.finish(end, 1.0), TrialStatus::kReadingStale);
    EXPECT_FALSE(cal.started());
}

TEST(WheelCalibration, PerpendicularWheelsCalibrateIndependently) {
    // Forward pushes move port 0 only; sideways pushes move port 1 only.
    WheelCalibration forward(forwardWheel(), {1});
    WheelCalibration sideways(sidewaysWheel(), {0});
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(forward.start(snapshot(0, 0.5)), TrialStatus::kAccepted);
        ASSERT_EQ(forward.finish(snapshot(1.02, 0.505), 1.0), TrialStatus::kAccepted);
        ASSERT_EQ(sideways.start(snapshot(1.02, 0.5)), TrialStatus::kAccepted);
        ASSERT_EQ(sideways.finish(snapshot(1.025, 1.47), 1.0), TrialStatus::kAccepted);
    }
    EXPECT_NEAR(forward.proposal().scale, 1 / 1.02, 1e-12);
    EXPECT_NEAR(sideways.proposal().scale, 1 / 0.97, 1e-12);
    EXPECT_TRUE(forward.proposal().ready);
    EXPECT_TRUE(sideways.proposal().ready);

    // A sideways push is refused as a forward trial.
    ASSERT_EQ(forward.start(snapshot(0, 0)), TrialStatus::kAccepted);
    EXPECT_EQ(forward.finish(snapshot(0.0, 1.0), 1.0), TrialStatus::kNotStraight);
}

TEST(WheelCalibration, AllowedRotationIsRemovedFromTheWheelTravel) {
    // True scale 1.0, a 0.5 m push with 0.4 deg of rotation. The forward
    // wheel at y = 0.15 moves back by 0.15 * rotation; the sideways wheel at
    // x = 0.15 forward by 0.15 * rotation.
    const Radians    rot = 0.007;
    WheelCalibration forward(forwardWheel(), {1});
    ASSERT_EQ(forward.start(snapshot(0, 0)), TrialStatus::kAccepted);
    ASSERT_EQ(forward.finish(snapshot(0.5 - 0.15 * rot, 0.15 * rot, rot), 0.5),
              TrialStatus::kAccepted);
    EXPECT_NEAR(forward.trials().back().scale, 1.0, 1e-9);

    WheelCalibration sideways(sidewaysWheel(), {0});
    ASSERT_EQ(sideways.start(snapshot(0, 0)), TrialStatus::kAccepted);
    ASSERT_EQ(sideways.finish(snapshot(-0.15 * rot, 0.5 + 0.15 * rot, rot), 0.5),
              TrialStatus::kAccepted);
    EXPECT_NEAR(sideways.trials().back().scale, 1.0, 1e-9);
}
