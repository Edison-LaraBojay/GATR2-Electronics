// navigator_gtest.cpp
// Navigator closed loop on the drivetrain sim, plus scripted input cases.

#include "investigatr/navigator.h"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "sim/differential_drive_sim.h"
#include "sim/simulated_source.h"

using namespace investigatr;

namespace
{

constexpr Seconds    kPeriod   = 0.01;
constexpr LandmarkId kLandmark = 7;

struct Rig {
    explicit Rig(const Pose& start = {}, const NavigatorConfig& config = {},
                 const DifferentialDriveConfig& drive_config = {})
        : drive(drive_config, start), navigator(source, config) {}

    // One control period: truth into the source, update, drive.
    DriveCommand step() {
        source.setRobot(drive.pose(), now);
        updated_at = now;
        last       = navigator.update(now);
        drive.step(mixTank(last), kPeriod);
        now += kPeriod;
        return last;
    }

    MotionState run(Seconds limit) {
        const Seconds end = now + limit;
        while (now < end && !isTerminal(navigator.status().state)) {
            step();
        }
        return navigator.status().state;
    }

    bool runUntil(MotionState state, Seconds limit) {
        const Seconds end = now + limit;
        while (now < end && !isTerminal(navigator.status().state)) {
            step();
            if (navigator.status().state == state) {
                return true;
            }
        }
        return false;
    }

    const MotionStatus& status() const { return navigator.status(); }

    SimulatedSource      source;
    DifferentialDriveSim drive;
    Navigator            navigator;
    Seconds              now        = 0;
    Seconds              updated_at = 0;
    DriveCommand         last;
};

SimulatedLandmark landmarkAt(const Pose& pose, LandmarkSource source = LandmarkSource::kObserved) {
    SimulatedLandmark landmark;
    landmark.pose   = pose;
    landmark.source = source;
    return landmark;
}

Meters distance(const Pose& a, const Pose& b) { return std::hypot(a.x - b.x, a.y - b.y); }

void expectAt(const Pose& actual, const Pose& expected, const NavigatorConfig& config = {}) {
    EXPECT_LE(distance(actual, expected), config.position_tolerance)
        << "at " << actual.x << ", " << actual.y << " want " << expected.x << ", " << expected.y;
    EXPECT_LE(std::fabs(wrapAngle(actual.heading - expected.heading)), config.heading_tolerance)
        << "heading " << actual.heading << " want " << expected.heading;
}

void expectZero(const DriveCommand& command) {
    EXPECT_EQ(command.forward, 0.0);
    EXPECT_EQ(command.turn, 0.0);
}

bool inside(const MotionStatus& status, const NavigatorConfig& config) {
    return status.distance_error <= config.position_tolerance &&
           std::fabs(status.heading_error) <= config.heading_tolerance;
}

} // namespace

// Convergence and final orientation.

TEST(Navigator, ConvergesFromSeveralStartHeadings) {
    const Pose goal{1.0, 0.5, 1.0};
    for (const double heading : {0.0, kPi / 2.0, kPi, -kPi / 2.0, 2.5, -2.5}) {
        Rig             rig(Pose{0.0, 0.0, heading});
        const CommandId id = rig.navigator.goTo(goal);
        ASSERT_EQ(rig.run(10.0), MotionState::kCompleted) << "start heading " << heading;
        EXPECT_EQ(rig.status().command_id, id);
        EXPECT_EQ(rig.status().reason, MotionReason::kNone);
        expectAt(rig.drive.pose(), goal);
    }
}

TEST(Navigator, HeadingWrapTakesTheShortWay) {
    Rig        rig(Pose{0.0, 0.0, 3.1});
    const Pose goal{-1.0, -0.05, -3.1};
    rig.navigator.goTo(goal);
    double  rotation = 0;
    Radians previous = rig.drive.pose().heading;
    while (rig.now < 10.0 && !isTerminal(rig.status().state)) {
        rig.step();
        rotation += std::fabs(wrapAngle(rig.drive.pose().heading - previous));
        previous = rig.drive.pose().heading;
    }
    ASSERT_EQ(rig.status().state, MotionState::kCompleted);
    expectAt(rig.drive.pose(), goal);
    EXPECT_LT(rotation, 0.5);
}

TEST(Navigator, PureHeadingChangeTurnsInPlace) {
    const Pose start{0.5, 0.5, 0.2};
    const Pose goal{0.5, 0.5, 2.5};
    Rig        rig(start);
    rig.navigator.goTo(goal);
    double max_forward = 0;
    while (rig.now < 5.0 && !isTerminal(rig.status().state)) {
        rig.step();
        max_forward = std::max(max_forward, std::fabs(rig.last.forward));
        if (!isTerminal(rig.status().state)) {
            EXPECT_EQ(rig.status().state, MotionState::kAligning);
        }
    }
    ASSERT_EQ(rig.status().state, MotionState::kCompleted);
    EXPECT_EQ(max_forward, 0.0);
    expectAt(rig.drive.pose(), goal);
}

TEST(Navigator, OriginOffsetFromTurningCenterStillCompletes) {
    // Turning in place moves the origin up to twice the offset, which has to
    // fit between arrive_distance and position_tolerance.
    DifferentialDriveConfig drive;
    drive.origin_offset = Pose{0.01, 0.0, 0.0};
    NavigatorConfig config;
    config.position_tolerance = 0.045;
    Rig        rig(Pose{0.0, 0.0, 0.0}, config, drive);
    const Pose goal{0.6, -0.4, kPi / 2.0};
    rig.navigator.goTo(goal);
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    expectAt(rig.drive.pose(), goal, config);
}

TEST(Navigator, OvershootStillCompletes) {
    DifferentialDriveConfig drive;
    drive.wheel_time_constant = 0.15;
    Rig rig(Pose{}, NavigatorConfig{}, drive);
    rig.navigator.goTo(Pose{5.0, 0.0, 0.0});
    rig.run(1.5);
    // Replaced at speed by a goal the robot cannot stop at.
    const Pose goal{rig.drive.pose().x + 0.01, 0.0, 0.0};
    rig.navigator.goTo(goal);
    double furthest = 0;
    while (rig.now < 15.0 && !isTerminal(rig.status().state)) {
        rig.step();
        furthest = std::max(furthest, rig.drive.pose().x - goal.x);
    }
    EXPECT_GT(furthest, NavigatorConfig{}.position_tolerance);
    ASSERT_EQ(rig.status().state, MotionState::kCompleted);
    expectAt(rig.drive.pose(), goal, NavigatorConfig{});
}

TEST(Navigator, LateralOffsetInsideNearDistanceCompletes) {
    Rig rig;
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.0, 0.0}));
    const Pose offset{-0.5, 0.0, 0.0};
    rig.navigator.goToRelative(kLandmark, offset);
    while (rig.now < 5.0 &&
           !(rig.status().state == MotionState::kDriving && rig.status().distance_error < 0.08)) {
        rig.step();
    }
    ASSERT_EQ(rig.status().state, MotionState::kDriving);
    // Destination moves sideways while heading correction is off.
    const Pose moved{1.5, 0.06, 0.0};
    rig.source.setLandmark(kLandmark, landmarkAt(moved));
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    expectAt(rig.drive.pose(), compose(moved, offset));
}

// Landmark relative destinations.

TEST(Navigator, RelativeOffsetUnderRotatedLandmark) {
    Rig rig(Pose{1.9, -0.5, 0.0});
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{2.0, 1.0, kPi / 2.0}));
    rig.navigator.goToRelative(kLandmark, Pose{-0.5, 0.1, 0.0});
    rig.step();
    // Landmark faces +y: -0.5 along it is -y, 0.1 to its left is -x.
    const Pose want{1.9, 0.5, kPi / 2.0};
    ASSERT_TRUE(rig.status().has_destination);
    EXPECT_NEAR(rig.status().destination.x, want.x, 1e-9);
    EXPECT_NEAR(rig.status().destination.y, want.y, 1e-9);
    EXPECT_NEAR(rig.status().destination.heading, want.heading, 1e-9);
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    expectAt(rig.drive.pose(), want);
}

TEST(Navigator, SlowLandmarkDriftFollowedWithinOneCommand) {
    Rig  rig;
    Pose landmark{1.5, 0.0, 0.0};
    rig.source.setLandmark(kLandmark, landmarkAt(landmark));
    const Pose    offset{-0.4, 0.0, 0.0};
    MotionOptions options;
    options.timeout          = 8.0;
    const CommandId       id = rig.navigator.goToRelative(kLandmark, offset, options);
    const NavigatorConfig config;

    Pose    previous;
    bool    have_previous = false;
    double  max_step      = 0;
    Seconds last_elapsed  = -1;
    for (int i = 0; i < 800 && !isTerminal(rig.status().state); ++i) {
        // 0.5 m/s sideways for 0.5 s: under the jump limit per update, above
        // the follow speed.
        if (i >= 50 && i < 100) {
            landmark.y += 0.5 * kPeriod;
            rig.source.setLandmark(kLandmark, landmarkAt(landmark));
        }
        rig.step();
        EXPECT_EQ(rig.status().command_id, id);
        EXPECT_GE(rig.status().elapsed, last_elapsed);
        last_elapsed = rig.status().elapsed;
        if (rig.status().has_destination) {
            if (have_previous) {
                max_step = std::max(max_step, distance(rig.status().destination, previous));
            }
            previous      = rig.status().destination;
            have_previous = true;
        }
    }
    ASSERT_EQ(rig.status().state, MotionState::kCompleted);
    EXPECT_NEAR(landmark.y, 0.25, 1e-6);
    EXPECT_LE(max_step, config.landmark_follow_speed * kPeriod + 1e-9);
    EXPECT_NEAR(rig.status().elapsed, rig.updated_at, 1e-9);
    expectAt(rig.drive.pose(), compose(landmark, offset));
}

TEST(Navigator, LandmarkShiftWhileAligningCompletes) {
    Rig rig(Pose{0.0, 0.0, 0.0});
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.0, 0.0, kPi / 2.0}));
    const Pose offset{0.0, 0.0, 0.0};
    rig.navigator.goToRelative(kLandmark, offset);
    ASSERT_TRUE(rig.runUntil(MotionState::kAligning, 10.0));
    const Pose moved{1.0, 0.03, kPi / 2.0};
    rig.source.setLandmark(kLandmark, landmarkAt(moved));
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    expectAt(rig.drive.pose(), compose(moved, offset));
}

TEST(Navigator, LargeLandmarkJumpFails) {
    Rig rig;
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.0, 0.0}));
    rig.navigator.goToRelative(kLandmark, Pose{-0.5, 0.0, 0.0});
    rig.run(0.5);
    ASSERT_EQ(rig.status().state, MotionState::kDriving);
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.3, 0.0}));
    expectZero(rig.step());
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kLandmarkJump);
}

TEST(Navigator, LandmarkHeadingJumpFails) {
    Rig rig;
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.0, 0.0}));
    rig.navigator.goToRelative(kLandmark, Pose{-0.5, 0.0, 0.0});
    rig.run(0.5);
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.0, 0.4}));
    rig.step();
    EXPECT_EQ(rig.status().reason, MotionReason::kLandmarkJump);
}

TEST(Navigator, NominalLandmarkNotAcceptedByDefault) {
    Rig rig;
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.0, 0.0}, LandmarkSource::kNominal));
    rig.navigator.goToRelative(kLandmark, Pose{-0.5, 0.0, 0.0});
    const NavigatorConfig config;
    while (rig.now < config.input_wait_timeout - kPeriod) {
        expectZero(rig.step());
        ASSERT_EQ(rig.status().state, MotionState::kWaiting);
    }
    rig.run(0.5);
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kLandmarkUnavailable);
    EXPECT_EQ(rig.drive.pose().x, 0.0);
}

TEST(Navigator, NominalToObservedCorrectionCompletes) {
    Rig rig;
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.0, 0.0}, LandmarkSource::kNominal));
    MotionOptions options;
    options.require_observed_landmark = false;
    const Pose offset{-0.5, 0.0, 0.0};
    rig.navigator.goToRelative(kLandmark, offset, options);
    rig.run(0.6);
    ASSERT_EQ(rig.status().state, MotionState::kDriving);
    const Pose observed{1.5, 0.2, 0.1};
    rig.source.setLandmark(kLandmark, landmarkAt(observed));
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    expectAt(rig.drive.pose(), compose(observed, offset));
}

TEST(Navigator, NominalToObservedBeyondCorrectionFails) {
    Rig rig;
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.0, 0.0}, LandmarkSource::kNominal));
    MotionOptions options;
    options.require_observed_landmark = false;
    rig.navigator.goToRelative(kLandmark, Pose{-0.5, 0.0, 0.0}, options);
    rig.run(0.3);
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.7, 0.0}));
    rig.step();
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kLandmarkJump);
}

TEST(Navigator, PendingLandmarkWaitsThenResolves) {
    Rig               rig;
    SimulatedLandmark pending;
    pending.status = LandmarkStatus::kPending;
    rig.source.setLandmark(kLandmark, pending);
    const Pose landmark{1.0, 0.5, 0.0};
    const Pose offset{-0.3, 0.0, 0.0};
    rig.navigator.goToRelative(kLandmark, offset);
    for (int i = 0; i < 50; ++i) {
        expectZero(rig.step());
        ASSERT_EQ(rig.status().state, MotionState::kWaiting);
        EXPECT_FALSE(rig.status().has_destination);
        EXPECT_TRUE(rig.source.lastRequest().landmark);
        EXPECT_EQ(rig.source.lastRequest().landmark_id, kLandmark);
    }
    rig.source.setLandmark(kLandmark, landmarkAt(landmark));
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    expectAt(rig.drive.pose(), compose(landmark, offset));
    // Released once the command ends.
    rig.step();
    EXPECT_FALSE(rig.source.lastRequest().landmark);
}

TEST(Navigator, StaleLandmarkObservationNotUsed) {
    Rig               rig;
    SimulatedLandmark old = landmarkAt(Pose{1.0, 0.0, 0.0});
    old.age               = 2.0;
    rig.source.setLandmark(kLandmark, old);
    rig.navigator.goToRelative(kLandmark, Pose{-0.3, 0.0, 0.0});
    rig.run(1.0);
    EXPECT_EQ(rig.status().state, MotionState::kWaiting);
    rig.run(2.0);
    EXPECT_EQ(rig.status().reason, MotionReason::kLandmarkUnavailable);
}

TEST(Navigator, UnknownLandmarkFailsAtOnce) {
    Rig rig;
    rig.navigator.goToRelative(99, Pose{});
    expectZero(rig.step());
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kLandmarkUnknown);
}

TEST(Navigator, UnsupportedLandmarkFailsAtOnce) {
    Rig               rig;
    SimulatedLandmark unsupported;
    unsupported.status = LandmarkStatus::kUnsupported;
    rig.source.setLandmark(kLandmark, unsupported);
    rig.navigator.goToRelative(kLandmark, Pose{});
    expectZero(rig.step());
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kLandmarkUnsupported);
}

TEST(Navigator, RobotRelativeResolvesOnce) {
    Rig rig(Pose{1.0, 1.0, kPi / 2.0});
    rig.navigator.goToRobotRelative(Pose{0.5, 0.2, -kPi / 2.0});
    rig.step();
    const Pose want{0.8, 1.5, 0.0};
    ASSERT_TRUE(rig.status().has_destination);
    EXPECT_NEAR(rig.status().destination.x, want.x, 1e-9);
    EXPECT_NEAR(rig.status().destination.y, want.y, 1e-9);
    EXPECT_NEAR(rig.status().destination.heading, want.heading, 1e-9);
    while (rig.now < 10.0 && !isTerminal(rig.status().state)) {
        rig.step();
        EXPECT_NEAR(rig.status().destination.x, want.x, 1e-9);
        EXPECT_NEAR(rig.status().destination.y, want.y, 1e-9);
    }
    ASSERT_EQ(rig.status().state, MotionState::kCompleted);
    expectAt(rig.drive.pose(), want);
}

// Paths.

TEST(Navigator, PathPassesThroughAndSettlesOnlyAtLast) {
    Rig        rig;
    const Pose last{0.0, 1.0, kPi};
    const Path path{
        Waypoint{Destination::field(Pose{1.0, 0.0, 0.0}), false},
        Waypoint{Destination::field(Pose{1.0, 1.0, 0.0}), false},
        Waypoint{Destination::field(last), false},
    };
    rig.navigator.follow(path);
    EXPECT_EQ(rig.status().waypoint_count, 3u);
    std::size_t highest = 0;
    while (rig.now < 15.0 && !isTerminal(rig.status().state)) {
        rig.step();
        const MotionStatus& status = rig.status();
        EXPECT_GE(status.waypoint_index, highest);
        highest = std::max(highest, status.waypoint_index);
        if (status.waypoint_index < 2) {
            EXPECT_NE(status.state, MotionState::kAligning);
        }
    }
    ASSERT_EQ(rig.status().state, MotionState::kCompleted);
    EXPECT_EQ(rig.status().waypoint_index, 2u);
    expectAt(rig.drive.pose(), last);
}

TEST(Navigator, StopWaypointSettlesMidPath) {
    Rig        rig;
    const Pose first{0.6, 0.0, kPi / 2.0};
    const Pose last{0.6, 0.6, kPi / 2.0};
    rig.navigator.follow(
        Path{Waypoint{Destination::field(first), true}, Waypoint{Destination::field(last), true}});
    bool aligned_at_first = false;
    while (rig.now < 15.0 && !isTerminal(rig.status().state)) {
        rig.step();
        if (rig.status().waypoint_index == 0 && rig.status().state == MotionState::kAligning) {
            aligned_at_first = true;
        }
    }
    ASSERT_EQ(rig.status().state, MotionState::kCompleted);
    EXPECT_TRUE(aligned_at_first);
    expectAt(rig.drive.pose(), last);
}

TEST(Navigator, PassThroughAdvancesWhenRadiusMissed) {
    // Scripted pose sliding past the first waypoint 0.3 m to its side.
    NavigatorConfig config;
    config.turn_in_place_threshold = 2.0;
    SimulatedSource source;
    Navigator       navigator(source, config);
    navigator.follow(Path{Waypoint{Destination::field(Pose{1.0, 0.3, 0.0}), false},
                          Waypoint{Destination::field(Pose{20.0, 0.3, 0.0}), true}});
    Seconds now = 0;
    for (double x = -5.0; x < 1.2; x += 0.01) {
        source.setRobot(Pose{x, 0.0, 0.0}, now);
        navigator.update(now);
        now += kPeriod;
        ASSERT_EQ(navigator.status().state, MotionState::kDriving) << "x " << x;
        EXPECT_GT(navigator.status().distance_error, config.waypoint_pass_radius);
        if (x < 0.995) {
            EXPECT_EQ(navigator.status().waypoint_index, 0u) << "x " << x;
        } else if (x > 1.005) {
            EXPECT_EQ(navigator.status().waypoint_index, 1u) << "x " << x;
        }
    }
}

TEST(Navigator, SettleInterruptedRestartsTimer) {
    const NavigatorConfig config;
    Rig                   rig;
    const Pose            goal{0.5, 0.3, 1.0};
    rig.navigator.goTo(goal);
    while (rig.now < 10.0 &&
           !(rig.status().state == MotionState::kAligning && inside(rig.status(), config))) {
        rig.step();
    }
    ASSERT_EQ(rig.status().state, MotionState::kAligning);
    const Seconds first_inside = rig.updated_at;
    while (rig.now < first_inside + 0.15) {
        rig.step();
    }
    ASSERT_EQ(rig.status().state, MotionState::kAligning);
    // One unusable cycle.
    rig.source.setRobotValid(false);
    expectZero(rig.step());
    EXPECT_EQ(rig.status().state, MotionState::kWaiting);
    rig.source.setRobotValid(true);
    const Seconds back = rig.now;
    ASSERT_EQ(rig.run(5.0), MotionState::kCompleted);
    EXPECT_GE(rig.updated_at, back + config.settle_time - 1e-9);
    expectAt(rig.drive.pose(), goal);
}

// Replacement, cancel, timeout.

TEST(Navigator, ReplacementGetsNewIdAndRestartsTimeout) {
    Rig           rig;
    MotionOptions options;
    options.timeout     = 1.0;
    const CommandId one = rig.navigator.goTo(Pose{5.0, 0.0, 0.0}, options);
    rig.run(0.8);
    ASSERT_EQ(rig.status().state, MotionState::kDriving);
    const CommandId two = rig.navigator.goTo(Pose{5.0, 0.05, 0.0}, options);
    EXPECT_NE(one, two);
    EXPECT_EQ(rig.status().command_id, two);
    rig.step();
    EXPECT_EQ(rig.status().elapsed, 0.0);
    // Slew memory carries over: no ramp from zero.
    EXPECT_GT(rig.last.forward, 0.5);
    rig.run(0.7);
    EXPECT_FALSE(isTerminal(rig.status().state));
    rig.run(0.5);
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kTimedOut);
    EXPECT_EQ(rig.status().command_id, two);
}

TEST(Navigator, CancelStopsAndNeverResumes) {
    Rig rig;
    rig.navigator.goTo(Pose{3.0, 0.0, 0.0});
    rig.run(0.5);
    ASSERT_GT(rig.last.forward, 0.0);
    rig.navigator.cancel();
    EXPECT_EQ(rig.status().state, MotionState::kCancelled);
    EXPECT_EQ(rig.status().reason, MotionReason::kCancelledByCaller);
    expectZero(rig.step());
    rig.source.setConnected(false);
    for (int i = 0; i < 20; ++i) {
        expectZero(rig.step());
    }
    rig.source.setConnected(true);
    for (int i = 0; i < 100; ++i) {
        expectZero(rig.step());
    }
    EXPECT_EQ(rig.status().state, MotionState::kCancelled);
    EXPECT_EQ(rig.status().reason, MotionReason::kCancelledByCaller);
}

TEST(Navigator, CancelHasNoEffectOnTerminalCommand) {
    Rig rig;
    rig.navigator.goTo(Pose{0.3, 0.0, 0.0});
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    rig.navigator.cancel();
    EXPECT_EQ(rig.status().state, MotionState::kCompleted);
    EXPECT_EQ(rig.status().reason, MotionReason::kNone);
}

TEST(Navigator, CompletedStaysCompleted) {
    Rig        rig;
    const Pose goal{0.4, 0.2, 0.5};
    rig.navigator.goTo(goal);
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    rig.drive.setPose(Pose{2.0, 2.0, 0.0});
    for (int i = 0; i < 100; ++i) {
        expectZero(rig.step());
    }
    EXPECT_EQ(rig.status().state, MotionState::kCompleted);
}

TEST(Navigator, TimesOut) {
    Rig           rig;
    MotionOptions options;
    options.timeout = 0.5;
    rig.navigator.goTo(Pose{5.0, 0.0, 0.0}, options);
    EXPECT_EQ(rig.run(2.0), MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kTimedOut);
    EXPECT_GT(rig.status().elapsed, 0.5);
    EXPECT_LT(rig.status().elapsed, 0.5 + 2.0 * kPeriod);
    expectZero(rig.last);
}

// Input rules.

TEST(Navigator, MissingInputWaitsThenFails) {
    const NavigatorConfig config;
    SimulatedSource       source;
    Navigator             navigator(source, config);
    navigator.goTo(Pose{1.0, 0.0, 0.0});
    Seconds now = 0;
    while (now < config.input_wait_timeout - kPeriod) {
        expectZero(navigator.update(now));
        ASSERT_EQ(navigator.status().state, MotionState::kWaiting);
        now += kPeriod;
    }
    for (int i = 0; i < 5; ++i) {
        expectZero(navigator.update(now));
        now += kPeriod;
    }
    EXPECT_EQ(navigator.status().state, MotionState::kFailed);
    EXPECT_EQ(navigator.status().reason, MotionReason::kInputUnavailable);
}

TEST(Navigator, WaitsForFirstPoseThenDrives) {
    Rig rig;
    rig.source.setFrame(0);
    rig.navigator.goTo(Pose{0.5, 0.0, 0.0});
    for (int i = 0; i < 50; ++i) {
        expectZero(rig.step());
        EXPECT_EQ(rig.status().state, MotionState::kWaiting);
    }
    rig.source.setFrame(3);
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
}

TEST(Navigator, StalePoseMidCommandZeroThenLost) {
    const NavigatorConfig config;
    Rig                   rig;
    rig.navigator.goTo(Pose{3.0, 0.0, 0.0});
    rig.run(0.5);
    ASSERT_GT(rig.last.forward, 0.0);
    rig.source.setExtraAge(config.max_pose_age + 0.1);
    const Seconds stale_at = rig.now;
    while (rig.now < stale_at + config.input_loss_timeout - kPeriod) {
        expectZero(rig.step());
        ASSERT_EQ(rig.status().state, MotionState::kWaiting);
    }
    rig.run(0.1);
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kInputLost);
    expectZero(rig.last);
}

TEST(Navigator, StalePoseRecoversWithinLossTimeout) {
    Rig        rig;
    const Pose goal{1.5, 0.0, 0.0};
    rig.navigator.goTo(goal);
    rig.run(0.5);
    rig.source.setLinkAge(1.0);
    for (int i = 0; i < 10; ++i) {
        expectZero(rig.step());
    }
    EXPECT_EQ(rig.status().state, MotionState::kWaiting);
    rig.source.setLinkAge(0.0);
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    expectAt(rig.drive.pose(), goal);
}

TEST(Navigator, DisconnectFailsAtOnce) {
    Rig rig;
    rig.navigator.goTo(Pose{3.0, 0.0, 0.0});
    rig.run(0.5);
    rig.source.setConnected(false);
    expectZero(rig.step());
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kInputLost);
}

TEST(Navigator, FrameChangeFails) {
    Rig rig;
    rig.navigator.goTo(Pose{3.0, 0.0, 0.0});
    rig.run(0.5);
    rig.source.setFrame(2);
    expectZero(rig.step());
    EXPECT_EQ(rig.status().state, MotionState::kFailed);
    EXPECT_EQ(rig.status().reason, MotionReason::kFrameChanged);
}

TEST(Navigator, FrameChangeWhileUnusableStillFails) {
    Rig rig;
    rig.navigator.goTo(Pose{3.0, 0.0, 0.0});
    rig.run(0.5);
    rig.source.setRobotValid(false);
    rig.step();
    EXPECT_EQ(rig.status().state, MotionState::kWaiting);
    rig.source.setFrame(2);
    rig.step();
    EXPECT_EQ(rig.status().reason, MotionReason::kFrameChanged);
}

// Sources.

TEST(Navigator, RequestsCurrentNeedEveryUpdate) {
    Rig rig;
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.0, 0.0, 0.0}));
    rig.step();
    EXPECT_EQ(rig.source.requestCount(), 1);
    EXPECT_FALSE(rig.source.lastRequest().landmark);
    rig.navigator.follow(
        Path{Waypoint{Destination::field(Pose{0.5, 0.0, 0.0}), false},
             Waypoint{Destination::relative(kLandmark, Pose{-0.2, 0.0, 0.0}), true}});
    int updates = 1;
    while (rig.now < 5.0 && rig.status().waypoint_index == 0) {
        rig.step();
        ++updates;
        EXPECT_EQ(rig.source.requestCount(), updates);
        EXPECT_FALSE(rig.source.lastRequest().landmark);
    }
    ASSERT_EQ(rig.status().waypoint_index, 1u);
    rig.step();
    EXPECT_TRUE(rig.source.lastRequest().landmark);
    EXPECT_EQ(rig.source.lastRequest().landmark_id, kLandmark);
    ASSERT_EQ(rig.run(10.0), MotionState::kCompleted);
    expectAt(rig.drive.pose(), Pose{0.8, 0.0, 0.0});
}

TEST(Navigator, SetSourceCancelsAndReleasesOldSource) {
    Rig rig;
    rig.source.setLandmark(kLandmark, landmarkAt(Pose{1.5, 0.0, 0.0}));
    rig.navigator.goToRelative(kLandmark, Pose{-0.5, 0.0, 0.0});
    rig.run(0.3);
    ASSERT_TRUE(rig.source.lastRequest().landmark);

    SimulatedSource other;
    other.setRobot(rig.drive.pose(), rig.now);
    rig.navigator.setSource(other);
    EXPECT_EQ(rig.status().state, MotionState::kCancelled);
    EXPECT_EQ(rig.status().reason, MotionReason::kSourceChanged);
    EXPECT_FALSE(rig.source.lastRequest().landmark);
    const int before = rig.source.requestCount();
    expectZero(rig.navigator.update(rig.now));
    EXPECT_EQ(rig.source.requestCount(), before);
    EXPECT_EQ(other.requestCount(), 1);
}

TEST(Navigator, OutputSlewLimitedAndZeroWhileWaiting) {
    const NavigatorConfig config;
    Rig                   rig;
    rig.navigator.goTo(Pose{3.0, 0.0, 0.0});
    DriveCommand previous;
    for (int i = 0; i < 60; ++i) {
        const DriveCommand out = rig.step();
        EXPECT_LE(out.forward - previous.forward, config.forward_slew * kPeriod + 1e-12);
        EXPECT_LE(out.forward, config.max_forward);
        previous = out;
    }
    EXPECT_GT(previous.forward, 0.5);
    rig.source.setRobotValid(false);
    expectZero(rig.step());
    rig.source.setRobotValid(true);
    // Slew memory reset: ramps from zero again.
    EXPECT_LE(rig.step().forward, config.forward_slew * kPeriod + 1e-12);
}

TEST(Navigator, MinimumOutputsOnlyOutsideTolerance) {
    NavigatorConfig config;
    config.min_forward  = 0.3;
    config.min_turn     = 0.2;
    config.forward_slew = 0;
    config.turn_slew    = 0;
    SimulatedSource source;
    Navigator       navigator(source, config);

    // Driving 5 cm out: 2.0 * 0.05 raised to min_forward.
    source.setRobot(Pose{}, 0.0);
    navigator.goTo(Pose{0.05, 0.0, 0.0});
    const DriveCommand driving = navigator.update(0.0);
    ASSERT_EQ(navigator.status().state, MotionState::kDriving);
    EXPECT_DOUBLE_EQ(driving.forward, 0.3);

    // Aligning, heading 0.05 rad out: raised. 0.02 rad out: inside, not raised.
    navigator.goTo(Pose{0.0, 0.0, 0.05});
    EXPECT_DOUBLE_EQ(navigator.update(0.0).turn, 0.2);
    ASSERT_EQ(navigator.status().state, MotionState::kAligning);
    navigator.goTo(Pose{0.0, 0.0, 0.02});
    EXPECT_NEAR(navigator.update(0.0).turn, config.turn_pid.kP * 0.02, 1e-12);
}

// Validation.

TEST(Navigator, DefaultConfigValid) {
    const char* why = "unset";
    EXPECT_TRUE(Navigator::valid(NavigatorConfig{}, &why));
    EXPECT_EQ(why, nullptr);
}

TEST(Navigator, InvalidConfigRejected) {
    const char*     why = nullptr;
    NavigatorConfig config;
    config.arrive_distance = config.position_tolerance;
    EXPECT_FALSE(Navigator::valid(config, &why));
    ASSERT_NE(why, nullptr);

    config                         = NavigatorConfig{};
    config.turn_exit               = 0.6;
    config.turn_in_place_threshold = 0.5;
    EXPECT_FALSE(Navigator::valid(config));

    config                      = NavigatorConfig{};
    config.waypoint_pass_radius = config.near_distance - 0.01;
    EXPECT_FALSE(Navigator::valid(config));

    config        = NavigatorConfig{};
    config.max_dt = 0;
    EXPECT_FALSE(Navigator::valid(config));

    config             = NavigatorConfig{};
    config.turn_pid.kP = -1;
    EXPECT_FALSE(Navigator::valid(config));

    config              = NavigatorConfig{};
    config.max_pose_age = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(Navigator::valid(config));
}

TEST(Navigator, InvalidConfigFailsCommands) {
    NavigatorConfig config;
    config.settle_time = -1;
    SimulatedSource source;
    Navigator       navigator(source, config);
    navigator.goTo(Pose{1.0, 0.0, 0.0});
    EXPECT_EQ(navigator.status().state, MotionState::kFailed);
    EXPECT_EQ(navigator.status().reason, MotionReason::kInvalidCommand);
    source.setRobot(Pose{}, 0.0);
    expectZero(navigator.update(0.0));
}

TEST(Navigator, InvalidCommandsFailAtOnce) {
    SimulatedSource source;
    Navigator       navigator(source);
    const double    nan = std::numeric_limits<double>::quiet_NaN();

    const CommandId empty = navigator.follow(Path{});
    EXPECT_EQ(navigator.status().command_id, empty);
    EXPECT_EQ(navigator.status().reason, MotionReason::kInvalidCommand);

    navigator.goTo(Pose{nan, 0.0, 0.0});
    EXPECT_EQ(navigator.status().reason, MotionReason::kInvalidCommand);

    navigator.goToRelative(0, Pose{});
    EXPECT_EQ(navigator.status().reason, MotionReason::kInvalidCommand);

    MotionOptions options;
    options.timeout = -1;
    navigator.goTo(Pose{}, options);
    EXPECT_EQ(navigator.status().reason, MotionReason::kInvalidCommand);

    EXPECT_EQ(navigator.status().state, MotionState::kFailed);
}
