// main.cpp
// GATR2 testing program: localization from the Pi, planning from
// investiGATR, following and drive control from actuGATR. The robot profile
// comes from brain/robot/gatr2_robot.h, the drivetrain, gains and tests from
// include/robot_config.h.
//
// Controller: left stick Y forward, right stick X turn, left stick X strafe
// (mecanum). A direct test, X avoiding test, Y landmark test, B cancel, UP
// place at the start pose, DOWN recalibrate the IMU (drive idle, robot
// still), LEFT/RIGHT speed scale. autonomous() runs the direct then the
// avoiding test. The drive task is the only motor writer.

#include "main.h"
#include "robot_config.h"

#include <cmath>
#include <cstdio>
#include <memory>

#include "actugatr/drive_owner.h"
#include "actugatr/ports.h"
#include "actugatr/pros_drive.h"
#include "actugatr/pros_motor_output.h"
#include "communigatr/link_events.h"
#include "communigatr/pros_link.h"
#include "communigatr/pros_vex_imu.h"
#include "communigatr/startup_placement.h"
#include "communigatr/vex_imu_recalibration.h"
#include "investigatr/planner.h"

namespace
{

using communigatr::Seconds;
using investigatr::Pose;

std::unique_ptr<communigatr::ProsVexImu>       g_vex;
std::unique_ptr<communigatr::ProsLink>         g_link;
std::unique_ptr<investigatr::GeometricPlanner> g_planner;
std::unique_ptr<actugatr::Follower>            g_follower;
std::unique_ptr<actugatr::Motion>              g_motion;
std::unique_ptr<actugatr::Kinematics>          g_kinematics;
std::unique_ptr<actugatr::ProsMotorOutput>     g_motors;
std::unique_ptr<actugatr::Drive>               g_drive;
std::unique_ptr<actugatr::DriveOwner>          g_owner;
std::unique_ptr<actugatr::ProsDrive>           g_task;
std::unique_ptr<communigatr::StartupPlacement> g_startup;
communigatr::LinkEvents                        g_events;

char        g_config_error[80] = {};
const char* g_message        = "Starting";
double      g_speed_scale    = robot_config::kSpeedScale;

Seconds now() {
    return communigatr::ProsLink::now();
}

bool motionReady() {
    return g_task != nullptr;
}

bool tank() {
    return robot_config::kDrivetrain == robot_config::Drivetrain::kTank;
}

investigatr::MotionModel model() {
    return tank() ? actugatr::motionModel(robot_config::tank(), gatr2_robot::kFootprint,
                                          robot_config::kClearance, robot_config::limits())
                  : actugatr::motionModel(robot_config::mecanum(), gatr2_robot::kFootprint,
                                          robot_config::kClearance, robot_config::limits());
}

bool configError(const char* what, const char* why) {
    std::snprintf(g_config_error, sizeof(g_config_error), "%s: %s", what, why ? why : "invalid");
    return false;
}

// Every active Smart Port device on its own port and every drive setting
// valid, or no motors at all.
bool checkConfig() {
    actugatr::PortMap map;
    if (tank()) {
        map.add(robot_config::tank());
    } else {
        map.add(robot_config::mecanum());
    }
    if (gatr2_robot::usesVexImu()) {
        map.add(gatr2_robot::kVexImuPort, "VEX IMU");
    }
    if (!gatr2_robot::kUseUsb) {
        map.add(gatr2_robot::kLinkPort, "Pi link");
    }
    if (!map.ok()) {
        return configError("ports", map.error());
    }
    const char* why = nullptr;
    if (tank() ? !actugatr::valid(robot_config::tank(), &why)
               : !actugatr::valid(robot_config::mecanum(), &why)) {
        return configError("drivetrain", why);
    }
    const actugatr::FollowerConfig follower = robot_config::follower();
    if (!actugatr::valid(follower, &why)) {
        return configError("follower", why);
    }
    if (!actugatr::valid(robot_config::motion(model()), &why)) {
        return configError("motion", why);
    }
    // The planner's clearance must cover the most drift the follower allows.
    if (!(std::hypot(follower.position_tolerance, follower.tracking_tolerance) <
          robot_config::kClearance)) {
        return configError("follower", "tolerances not below kClearance");
    }
    return true;
}

void createLink() {
    communigatr::LinkConfig config;
    if (gatr2_robot::kUseUsb) {
        config.transport = communigatr::Transport::kUsb;
    } else {
        config.transport  = communigatr::Transport::kSmartPort;
        config.smart_port = gatr2_robot::kLinkPort;
        config.baud       = gatr2_robot::kLinkBaud;
    }
    if (gatr2_robot::kSendProfile) {
        config.profile = gatr2_robot::profile();
    }
    if (gatr2_robot::usesVexImu()) {
        g_vex.reset(new communigatr::ProsVexImu(gatr2_robot::kVexImuPort));
        communigatr::ProsVexImu* vex = g_vex.get();
        config.client.bench_imu      = [vex] { return vex->sample(); };
    }
    g_link.reset(new communigatr::ProsLink(config));
}

void createDrive() {
    actugatr::StopMode   stop = actugatr::StopMode::kBrake;
    actugatr::WheelDrive wheels;
    if (tank()) {
        const actugatr::TankConfig c = robot_config::tank();
        wheels                       = c.wheels;
        stop                         = c.stop_mode;
        g_kinematics.reset(new actugatr::TankKinematics(c.track_width));
        g_motors.reset(new actugatr::ProsMotorOutput(c));
        g_follower.reset(new actugatr::DifferentialFollower(robot_config::follower()));
    } else {
        const actugatr::MecanumConfig c = robot_config::mecanum();
        wheels = c.wheels;
        stop   = c.stop_mode;
        g_kinematics.reset(new actugatr::MecanumKinematics(c.track_width, c.wheelbase));
        g_motors.reset(new actugatr::ProsMotorOutput(c));
        g_follower.reset(new actugatr::HolonomicFollower(robot_config::follower()));
    }
    g_planner.reset(new investigatr::GeometricPlanner());
    g_motion.reset(new actugatr::Motion(*g_link, *g_planner, *g_follower,
                                        robot_config::motion(model())));
    g_motion->setPathSink(g_link.get());
    actugatr::DriveConfig drive;
    drive.stop_mode = stop;
    g_drive.reset(new actugatr::Drive(*g_kinematics, wheels, *g_motors, drive));
    g_owner.reset(new actugatr::DriveOwner(*g_motion, *g_drive, robot_config::manual()));
    g_task.reset(new actugatr::ProsDrive(*g_owner, *g_drive, &communigatr::ProsLink::now));
}

actugatr::MoveOptions testOptions() {
    actugatr::MoveOptions o;
    o.timeout     = 20.0;
    o.speed_scale = g_speed_scale;
    return o;
}

actugatr::CommandId runDirect() {
    return g_task->goToDirect(robot_config::kDirectGoal, investigatr::Reference::origin(),
                              testOptions());
}

actugatr::CommandId runAvoiding() {
    return g_task->goToAvoiding(robot_config::kAvoidGoal, investigatr::Reference::origin(),
                                testOptions());
}

actugatr::CommandId runLandmark() {
    actugatr::MoveOptions o        = testOptions();
    o.require_observed_reference = robot_config::kRequireObservedLandmark;
    return g_task->goToAvoiding(robot_config::kLandmarkOffset, robot_config::landmark(), o);
}

bool moving(const actugatr::DriveSnapshot& s) {
    return s.motion.command_id != 0 && s.motion.state != actugatr::MotionState::kIdle &&
           !actugatr::isTerminal(s.motion.state);
}

// Waits for command id to end, at most limit seconds. False unless completed.
bool waitFor(actugatr::CommandId id, Seconds limit) {
    if (id == 0) {
        return false;
    }
    const Seconds end = now() + limit;
    while (now() < end) {
        actugatr::DriveSnapshot s;
        if (g_task->status(s)) {
            if (s.motion.command_id != id) {
                return false; // replaced
            }
            if (actugatr::isTerminal(s.motion.state)) {
                return s.motion.state == actugatr::MotionState::kCompleted;
            }
        }
        pros::delay(20);
    }
    g_task->cancel();
    return false;
}

// ---------------------------------------------------------------------------
// Screen and background work.
// ---------------------------------------------------------------------------

template <typename... Args> void row(int index, const char* format, Args... args) {
    const int y = 4 + index * 20;
    pros::screen::erase_rect(0, y, 479, y + 19);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, y, format, args...);
}

void display() {
    const communigatr::ProsLinkStatus link  = g_link->status();
    const investigatr::RobotState     robot = g_link->robot(now());
    row(0, "GATR2 testing | %s | %s", gatr2_robot::kUseUsb ? "USB" : "RS-485",
        link.busy ? "busy" : communigatr::toString(link.readiness));
    if (!motionReady()) {
        row(1, "DRIVE OFF: %s", g_config_error);
    } else {
        static actugatr::DriveSnapshot drive; // kept when a read is busy
        g_task->status(drive);
        const actugatr::MotionStatus& m = drive.motion;
        row(1, "Drive %s | cmd %lu %s %s", actugatr::toString(drive.mode),
            static_cast<unsigned long>(m.command_id), actugatr::toString(m.state),
            actugatr::toString(m.reason));
        row(2, "err %.3f m %.1f deg xt %.3f  seg %u/%u  plans %u", m.distance_error,
            m.heading_error * 180.0 / investigatr::kPi, m.cross_track, unsigned(m.segment),
            unsigned(m.segment_count), unsigned(m.plans));
    }
    if (robot.valid()) {
        row(3, "x %+.3f y %+.3f h %+.1f  age %.0f ms", robot.pose.x, robot.pose.y,
            robot.pose.heading * 180.0 / investigatr::kPi, robot.age * 1000.0);
    } else {
        row(3, "Pose: %s", investigatr::toString(robot.status));
    }
    row(4, "Speed scale %.2f  Start: %s", g_speed_scale, communigatr::toString(g_startup->state()));
    row(5, "Last: %s", g_events.size() > 0 ? communigatr::toString(g_events.at(0).event) : "-");
    row(6, "A direct  X avoid  Y landmark  B cancel");
    row(7, "UP place  DOWN recal IMU  LEFT/RIGHT speed");
    row(8, "%s", g_message);
}

// VEX IMU recalibration, after the Pi's stillness check. opcontrol only.
communigatr::VexImuRecalibration g_recal;

// Background task only.
uint32_t g_events_seen = 0;

// True when a placement loss is among the events since the last call.
bool placementLostSinceLastCheck() {
    bool lost = false;
    for (uint32_t i = g_events_seen; i < g_events.total(); ++i) {
        const std::size_t newest = g_events.total() - 1 - i;
        lost = lost || (newest < g_events.size() &&
                        g_events.at(newest).event == communigatr::LinkEvent::kPlacementLost);
    }
    g_events_seen = g_events.total();
    return lost;
}

// Link, profile and sensors are up: the readiness got past them.
bool sensorsReady(communigatr::Readiness r) {
    using communigatr::Readiness;
    return r != Readiness::kConnecting && r != Readiness::kReconnecting &&
           r != Readiness::kProfilePending && r != Readiness::kProfileRejected &&
           r != Readiness::kSensorsUnavailable && r != Readiness::kSensorsInitializing;
}

// Pi IMU calibration not settled; a failed one waits for a recalibration.
bool calibrating(communigatr::Readiness r) {
    using communigatr::Readiness;
    return r == Readiness::kWaitingStill || r == Readiness::kCalibrating ||
           r == Readiness::kCalibrationFailed;
}

void background() {
    double   next_start   = 0;
    uint32_t last_display = pros::millis();
    while (true) {
        const Seconds t = now();
        if (!g_link->status().started && t >= next_start) {
            g_link->start();
            next_start = t + 1.0;
        }
        // A busy link skips the history and start-up decisions this cycle.
        const communigatr::ProsLinkStatus link = g_link->status();
        if (!link.busy) {
            communigatr::LinkSnapshot snap;
            snap.connected       = link.connected;
            snap.session         = link.session;
            snap.pi_instance     = link.pi_instance;
            snap.profile         = link.profile.state;
            snap.state_valid     = link.state.valid;
            snap.localized       = link.summary.localized;
            snap.health          = link.state.valid ? link.state.state.health : 0;
            snap.calibration     = link.summary.calibration;
            snap.odometry_epoch  = link.state.state.odometry_epoch;
            snap.anchor_revision = link.state.state.anchor_revision;
            g_events.update(snap, t);
            // Pi restart, reinitialize, or travel lost in a sensor gap: the
            // pose is invalid until the robot is placed again.
            if (placementLostSinceLastCheck()) {
                g_message = "POSE INVALID: put the robot at the start pose, press UP";
            }

            communigatr::StartupInputs in;
            in.connected     = link.connected;
            in.profile_ready = link.profile.state == communigatr::ProfileSync::kApplied;
            in.sensors_ready = sensorsReady(link.readiness);
            in.calibrating   = calibrating(link.readiness);
            in.localized     = link.summary.localized;
            if (g_startup->update(in, t) == communigatr::StartupState::kSubmit &&
                g_link->place(gatr2_robot::kStartPose) != 0) {
                g_startup->submitted();
            }
        }

        if (pros::millis() - last_display >= 100) {
            display();
            last_display = pros::millis();
        }
        pros::delay(20);
    }
}

double stick(int32_t value) {
    return std::abs(value) < robot_config::kStickDeadband ? 0.0 : value / 127.0;
}

} // namespace

void initialize() {
    pros::screen::set_eraser(0x00000000);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::erase();
    createLink();
    g_startup.reset(new communigatr::StartupPlacement(robot_config::kStartupPolicy,
                                                      robot_config::kStartupWaitSeconds));
    if (checkConfig()) {
        createDrive();
        g_message = "Ready: drive disabled until opcontrol or autonomous";
    } else {
        g_message = "Config error: fix robot_config.h, the drive stays off";
    }
    static pros::Task task(background, "testing ui");
}

void disabled() {
    if (motionReady()) {
        g_task->disable();
    }
}

void competition_initialize() {}

void autonomous() {
    if (!motionReady()) {
        return;
    }
    if (waitFor(runDirect(), 25.0)) {
        waitFor(runAvoiding(), 25.0);
    }
}

void opcontrol() {
    pros::Controller        master(pros::E_CONTROLLER_MASTER);
    actugatr::DriveSnapshot drive; // kept when a read is busy
    while (true) {
        if (motionReady()) {
            g_task->status(drive);
        }
        // The robot stays still while the IMU recalibrates: no tests, no sticks.
        const bool recalibrating = g_recal.active();
        if (motionReady()) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                g_message = recalibrating      ? "IMU calibrating: hold still"
                            : runDirect() != 0 ? "Direct test running"
                                               : "Drive busy";
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                g_message = recalibrating        ? "IMU calibrating: hold still"
                            : runAvoiding() != 0 ? "Avoiding test running"
                                                 : "Drive busy";
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                g_message = recalibrating        ? "IMU calibrating: hold still"
                            : runLandmark() != 0 ? "Landmark test running"
                                                 : "Drive busy";
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                g_task->cancel();
                g_message = "Cancelled";
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
                g_speed_scale = std::max(0.1, g_speed_scale - 0.1);
            }
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                g_speed_scale = std::min(1.0, g_speed_scale + 0.1);
            }
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            g_message = g_link->place(gatr2_robot::kStartPose) != 0 ? "Placing at the start pose"
                                                                    : "Place refused";
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            if (moving(drive)) {
                g_message = "Recalibrate refused: a movement is running";
            } else if (g_vex) {
                g_message = g_recal.begin(g_link->recalibrate())
                                ? "Checking the robot is still..."
                                : "Recalibrate refused: no link, or a request pending";
            } else {
                g_message = g_link->recalibrate() != 0 ? "IMU recalibration requested"
                                                       : "Recalibrate refused";
            }
        }

        // VEX IMU recalibration: starts only after the Pi's stillness check.
        if (g_vex && g_recal.active()) {
            using communigatr::VexRecalibrationState;
            const VexRecalibrationState r =
                g_recal.update(g_link->control(g_recal.ticket()), g_vex->sample(), now());
            if (r == VexRecalibrationState::kStart) {
                g_recal.started(g_vex->recalibrate(), now()); // blocks about 1 s at most
                g_message = "VEX IMU calibrating: hold still";
            } else if (r == VexRecalibrationState::kDone) {
                g_message = "VEX IMU calibrated: put the robot at the start pose, press UP";
            } else if (r == VexRecalibrationState::kMoving) {
                g_message = "Recalibrate refused: robot moving";
            } else if (r == VexRecalibrationState::kRefused) {
                g_message = "Recalibrate refused: check the link";
            } else if (r == VexRecalibrationState::kImuFailed) {
                g_message = "VEX IMU did not calibrate: check the IMU";
            }
        }

        if (motionReady()) {
            actugatr::ManualDemand d;
            d.forward = stick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
            d.strafe  = -stick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
            d.turn    = -stick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
            if (g_recal.active()) {
                d = actugatr::ManualDemand{};
            }
            const bool sticks = d.forward != 0 || d.strafe != 0 || d.turn != 0;
            // Sticks take over from a running test; otherwise the test keeps the drive.
            if (sticks || !moving(drive)) {
                g_task->manual(d);
            }
        }
        pros::delay(10);
    }
}
