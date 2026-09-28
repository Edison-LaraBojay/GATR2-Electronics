// telemetry_gtest.cpp
// TELEMETRY motion and wheels groups from drive snapshots: units, rounding,
// wrapping, saturation, missing destination, and the enum values the wire
// carries.

#include "actugatr/telemetry.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

#include "actugatr/drive.h"
#include "sim/sim_motor_output.h"

using namespace actugatr;
using investigatr::kPi;

namespace
{

constexpr double kDeg = kPi / 180.0;

WheelDrive wheels() {
    WheelDrive w;
    w.wheel_diameter  = 0.1;
    w.gear_ratio      = 1.0;
    w.cartridge       = Cartridge::kBlue;
    w.usable_fraction = 1.0;
    return w;
}

} // namespace

TEST(Telemetry, IdleSnapshotHasMotionAndNoWheels) {
    const translagatr::BrainTelemetry t = telemetryOf(DriveSnapshot{});
    EXPECT_EQ(t.flags, translagatr::kTelemetryMotion);
    EXPECT_EQ(t.command_id, 0u);
    EXPECT_EQ(t.motion_state, static_cast<uint8_t>(MotionState::kIdle));
    EXPECT_EQ(t.stamp_ms, 0u);
    EXPECT_EQ(t.roll_cdeg, 0);
    EXPECT_EQ(t.wheel_count, 0);
}

TEST(Telemetry, UnitsRoundingAndWrapping) {
    DriveSnapshot s;
    MotionStatus& m    = s.motion;
    m.command_id       = 42;
    m.state            = MotionState::kRunning;
    m.reason           = MotionReason::kNone;
    m.mode             = investigatr::PlanMode::kAvoiding;
    m.segment          = 3;
    m.segment_count    = 300;
    m.has_destination  = true;
    m.destination      = {1.2344, -0.5006, 270.0 * kDeg};
    m.distance_error   = 0.0126;
    m.heading_error    = -0.5;
    m.cross_track      = -0.0074;
    s.drive.command.vx    = 0.8;
    s.drive.command.vy    = -0.1234;
    s.drive.command.omega = 3.0;
    s.drive.fault         = DriveFault::kStale;
    s.drive.wheels.count  = 2;
    s.drive.motor_rpm[0]  = 523.44;
    s.drive.motor_rpm[1]  = -700.0;

    const translagatr::BrainTelemetry t = telemetryOf(s);
    EXPECT_EQ(t.flags, translagatr::kTelemetryMotion | translagatr::kTelemetryWheels);
    EXPECT_EQ(t.command_id, 42u);
    EXPECT_EQ(t.motion_state, 2);
    EXPECT_EQ(t.plan_mode, 1);
    EXPECT_EQ(t.segment, 3);
    EXPECT_EQ(t.segment_count, 255); // saturated
    EXPECT_EQ(t.target_x_mm, 1234);
    EXPECT_EQ(t.target_y_mm, -501);
    EXPECT_EQ(t.target_heading_cdeg, -9000); // 270 wraps to -90
    EXPECT_EQ(t.distance_error_mm, 13);
    EXPECT_EQ(t.heading_error_cdeg, -2865);
    EXPECT_EQ(t.cross_track_mm, -7);
    EXPECT_EQ(t.cmd_vx_mm_s, 800);
    EXPECT_EQ(t.cmd_vy_mm_s, -123);
    EXPECT_EQ(t.cmd_omega_cdeg_s, 17189); // 3 rad/s
    EXPECT_EQ(t.drive_fault, 4);
    EXPECT_EQ(t.wheel_count, 2);
    EXPECT_EQ(t.wheel_rpm_x10[0], 5234);
    EXPECT_EQ(t.wheel_rpm_x10[1], -7000);
    EXPECT_EQ(t.wheel_rpm_x10[2], 0);
}

TEST(Telemetry, SaturatesAndZeroesNonFinite) {
    DriveSnapshot s;
    s.motion.has_destination = true;
    s.motion.destination     = {5000.0, -5000.0, std::numeric_limits<double>::quiet_NaN()};
    s.motion.distance_error  = 50.0;
    s.motion.cross_track     = -40.0;
    s.drive.command.vx       = 40.0;
    s.drive.command.vy       = std::numeric_limits<double>::infinity();
    s.drive.command.omega    = -10.0;
    s.drive.wheels.count     = 4;
    s.drive.motor_rpm[0]     = 4000.0;
    s.drive.motor_rpm[3]     = std::numeric_limits<double>::quiet_NaN();

    const translagatr::BrainTelemetry t = telemetryOf(s);
    EXPECT_EQ(t.target_x_mm, 5000000);
    EXPECT_EQ(t.target_y_mm, -5000000);
    EXPECT_EQ(t.target_heading_cdeg, 0);
    EXPECT_EQ(t.distance_error_mm, 32767);
    EXPECT_EQ(t.cross_track_mm, -32768);
    EXPECT_EQ(t.cmd_vx_mm_s, 32767);
    EXPECT_EQ(t.cmd_vy_mm_s, 0);
    EXPECT_EQ(t.cmd_omega_cdeg_s, -32768);
    EXPECT_EQ(t.wheel_count, 4);
    EXPECT_EQ(t.wheel_rpm_x10[0], 32767);
    EXPECT_EQ(t.wheel_rpm_x10[3], 0);
}

// Waiting for a reference, idle, or ended before resolving: no destination.
TEST(Telemetry, NoDestinationLeavesTargetZero) {
    DriveSnapshot s;
    s.motion.command_id      = 9;
    s.motion.state           = MotionState::kWaiting;
    s.motion.reason          = MotionReason::kReferenceUnavailable;
    s.motion.destination     = {1.0, 2.0, 1.0};
    s.motion.has_destination = false;
    const translagatr::BrainTelemetry t = telemetryOf(s);
    EXPECT_EQ(t.target_x_mm, 0);
    EXPECT_EQ(t.target_y_mm, 0);
    EXPECT_EQ(t.target_heading_cdeg, 0);
    EXPECT_EQ(t.motion_reason, static_cast<uint8_t>(MotionReason::kReferenceUnavailable));
}

// The wheels group carries what the drive sent to the motors.
TEST(Telemetry, WheelTargetsAreTheMotorTargetsSent) {
    const TankKinematics tank(0.3);
    SimMotorOutput       out(2);
    Drive                drive(tank, wheels(), out);
    ASSERT_EQ(drive.apply(ChassisCommand{ChassisFrame::kBody, 0.5, 0, 1.0}, 1.0, 1.0),
              DriveFault::kNone);
    DriveSnapshot s;
    s.drive = drive.status();
    s.mode  = DriveMode::kManual;
    translagatr::BrainTelemetry t = telemetryOf(s);
    ASSERT_EQ(t.wheel_count, 2);
    EXPECT_EQ(t.wheel_rpm_x10[0], std::lround(out.rpm(0) * 10.0));
    EXPECT_EQ(t.wheel_rpm_x10[1], std::lround(out.rpm(1) * 10.0));
    EXPECT_NE(t.wheel_rpm_x10[0], t.wheel_rpm_x10[1]);
    EXPECT_EQ(t.cmd_vx_mm_s, 500);
    EXPECT_EQ(t.cmd_omega_cdeg_s, 5730);

    // A stop is a commanded 0 on every group, not missing data.
    drive.stop(DriveFault::kStale);
    s.drive = drive.status();
    t       = telemetryOf(s);
    EXPECT_EQ(t.flags & translagatr::kTelemetryWheels, translagatr::kTelemetryWheels);
    ASSERT_EQ(t.wheel_count, 2);
    EXPECT_EQ(t.wheel_rpm_x10[0], 0);
    EXPECT_EQ(t.wheel_rpm_x10[1], 0);
    EXPECT_EQ(t.cmd_vx_mm_s, 0);
    EXPECT_EQ(t.drive_fault, static_cast<uint8_t>(DriveFault::kStale));

    // Below stop_below the drive stops the motors: targets 0.
    ASSERT_EQ(drive.apply(ChassisCommand{ChassisFrame::kBody, 1e-5, 0, 0}, 2.0, 2.0),
              DriveFault::kNone);
    s.drive = drive.status();
    t       = telemetryOf(s);
    EXPECT_TRUE(out.stopped());
    EXPECT_EQ(t.wheel_rpm_x10[0], 0);
}

// The tables in docs/actugatr.md; the viewer names these numbers.
TEST(Telemetry, EnumValuesMatchTheDocumentedTables) {
    EXPECT_EQ(static_cast<int>(MotionState::kIdle), 0);
    EXPECT_EQ(static_cast<int>(MotionState::kWaiting), 1);
    EXPECT_EQ(static_cast<int>(MotionState::kRunning), 2);
    EXPECT_EQ(static_cast<int>(MotionState::kSettling), 3);
    EXPECT_EQ(static_cast<int>(MotionState::kCompleted), 4);
    EXPECT_EQ(static_cast<int>(MotionState::kCancelled), 5);
    EXPECT_EQ(static_cast<int>(MotionState::kFailed), 6);

    const MotionReason reasons[] = {
        MotionReason::kNone,
        MotionReason::kInvalidCommand,
        MotionReason::kInvalidConfig,
        MotionReason::kInputUnavailable,
        MotionReason::kNoProfile,
        MotionReason::kCalibrating,
        MotionReason::kPlacementRequired,
        MotionReason::kInputLost,
        MotionReason::kFrameChanged,
        MotionReason::kFieldUnavailable,
        MotionReason::kMapMismatch,
        MotionReason::kUnknownReference,
        MotionReason::kNotReference,
        MotionReason::kReferenceUnavailable,
        MotionReason::kUnsupportedModel,
        MotionReason::kStartOutOfBounds,
        MotionReason::kStartBlocked,
        MotionReason::kGoalOutOfBounds,
        MotionReason::kGoalBlocked,
        MotionReason::kNoPath,
        MotionReason::kTrackingError,
        MotionReason::kPlanLimit,
        MotionReason::kTimedOut,
        MotionReason::kSourceChanged,
        MotionReason::kCancelledByCaller,
    };
    for (int i = 0; i < 25; ++i) {
        EXPECT_EQ(static_cast<int>(reasons[i]), i);
    }

    EXPECT_EQ(static_cast<int>(investigatr::PlanMode::kDirect), 0);
    EXPECT_EQ(static_cast<int>(investigatr::PlanMode::kAvoiding), 1);

    EXPECT_EQ(static_cast<int>(DriveFault::kNone), 0);
    EXPECT_EQ(static_cast<int>(DriveFault::kWrongFrame), 1);
    EXPECT_EQ(static_cast<int>(DriveFault::kNonFinite), 2);
    EXPECT_EQ(static_cast<int>(DriveFault::kUnsupportedMotion), 3);
    EXPECT_EQ(static_cast<int>(DriveFault::kStale), 4);

    EXPECT_EQ(static_cast<int>(DriveMode::kDisabled), 0);
}
