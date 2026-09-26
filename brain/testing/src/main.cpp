// main.cpp
// GATR2 testing application. initialize() connects to the Pi and places the
// robot, autonomous() runs the navigation demo, opcontrol() drives manually
// and runs the demo on a button. DriveControl is the only motor writer.
// Every wait is bounded.

#include "main.h"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <functional>

#include "communigatr/pros_driver.h"
#include "drive_control.h"
#include "investigatr/navigator.h"
#include "robot_config.h"

namespace
{

using communigatr::ProsDriver;
using investigatr::Seconds;

constexpr Seconds  kConnectWait   = 3.0; // initialize(): link up
constexpr Seconds  kPlacementWait = 2.0; // initialize(): starting pose applied
constexpr Seconds  kWaitMargin    = 1.0; // demo step wait beyond its motion timeout
constexpr uint32_t kLoopMs        = 10;

// Screen lines.
constexpr int16_t kLineTitle  = 0;
constexpr int16_t kLineInit   = 1;
constexpr int16_t kLineLink   = 2;
constexpr int16_t kLinePose   = 3;
constexpr int16_t kLineMotion = 4;
constexpr int16_t kLineDemo   = 5;

// Created once in initialize() and kept for the program's life.
ProsDriver*   navigatr = nullptr;
DriveControl* drive    = nullptr;

__attribute__((format(printf, 2, 3))) void show(int16_t line, const char* format, ...) {
    char    text[64];
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    pros::screen::print(pros::E_TEXT_MEDIUM, line, "%-48s", text);
}

const char* placementName(communigatr::PlacementResult result) {
    switch (result) {
    case communigatr::PlacementResult::kNone:
        return "not sent";
    case communigatr::PlacementResult::kPending:
        return "pending";
    case communigatr::PlacementResult::kApplied:
        return "applied";
    case communigatr::PlacementResult::kRejected:
        return "rejected";
    case communigatr::PlacementResult::kTimedOut:
        return "timed out, outcome unknown";
    case communigatr::PlacementResult::kSessionLost:
        return "session lost";
    }
    return "?";
}

// Link, pose and motion lines, at most every 100 ms.
void showStatus() {
    static uint32_t next = 0;
    if (pros::millis() < next) {
        return;
    }
    next = pros::millis() + 100;

    const communigatr::ProsDriverStatus link = navigatr->status();
    if (link.connected) {
        show(kLineLink, "link up s %08x pi %08x %d ms", static_cast<unsigned>(link.session),
             static_cast<unsigned>(link.pi_instance), static_cast<int>(link.link_age * 1000));
    } else {
        show(kLineLink, "link down ready %d err %d tx %u rx %u", link.ready,
             static_cast<int>(link.error), static_cast<unsigned>(link.stats.requests),
             static_cast<unsigned>(link.stats.replies));
    }

    const investigatr::InputSnapshot input = navigatr->latest(ProsDriver::now());
    if (input.robot.valid) {
        show(kLinePose, "pose %.3f %.3f m %.1f deg %d ms f%u", input.robot.pose.x,
             input.robot.pose.y, input.robot.pose.heading * 180.0 / investigatr::kPi,
             static_cast<int>(input.robot.age * 1000), static_cast<unsigned>(input.frame));
    } else {
        show(kLinePose, "pose invalid f%u", static_cast<unsigned>(input.frame));
    }

    const investigatr::MotionStatus motion = drive->status();
    show(kLineMotion, "cmd %u %s %s %.3f m", static_cast<unsigned>(motion.command_id),
         investigatr::toString(motion.state), investigatr::toString(motion.reason),
         motion.distance_error);
}

// Polls done every loop for at most limit seconds.
bool waitUntil(Seconds limit, const std::function<bool()>& done) {
    const Seconds end = ProsDriver::now() + limit;
    while (!done()) {
        if (ProsDriver::now() >= end) {
            return false;
        }
        showStatus();
        pros::delay(kLoopMs);
    }
    return true;
}

// Waits for command id to end. False, with the drive stopped, when it fails,
// is aborted or outlasts its motion timeout; false when another command
// replaced it.
bool finished(const char* name, investigatr::CommandId id, const std::function<bool()>& abort) {
    show(kLineDemo, "%s: running", name);
    const Seconds end = ProsDriver::now() + robot_config::kDemoTimeout + kWaitMargin;
    while (true) {
        const investigatr::MotionStatus status = drive->status();
        if (status.command_id != id) {
            show(kLineDemo, "%s: replaced", name);
            return false;
        }
        if (status.state == investigatr::MotionState::kCompleted) {
            show(kLineDemo, "%s: completed", name);
            return true;
        }
        if (investigatr::isTerminal(status.state)) {
            drive->stop();
            show(kLineDemo, "%s: %s %s", name, investigatr::toString(status.state),
                 investigatr::toString(status.reason));
            return false;
        }
        if (abort() || ProsDriver::now() >= end) {
            drive->stop();
            show(kLineDemo, "%s: stopped", name);
            return false;
        }
        showStatus();
        pros::delay(kLoopMs);
    }
}

// Absolute goal, waypoint path, landmark relative alignment. Ends at the
// first step that does not complete.
void runDemo(const std::function<bool()>& abort) {
    investigatr::MotionOptions options;
    options.timeout                   = robot_config::kDemoTimeout;
    options.require_observed_landmark = robot_config::kDemoRequireObserved;

    if (!finished("goal", drive->goTo(robot_config::kDemoGoal, options), abort)) {
        return;
    }

    investigatr::Path path;
    for (const investigatr::Pose& pose : robot_config::kDemoPath) {
        path.push_back({investigatr::Destination::field(pose), false});
    }
    if (!finished("path", drive->follow(path, options), abort)) {
        return;
    }

    finished("landmark",
             drive->goToRelative(robot_config::kDemoLandmarkId, robot_config::kDemoLandmarkOffset,
                                 options),
             abort);
}

double stick(int32_t value) {
    return std::abs(value) < robot_config::kStickDeadband ? 0.0 : value / 127.0;
}

} // namespace

// Link up, then the starting placement. Commands issued without an applied
// placement wait for input and fail with kInputUnavailable.
void initialize() {
    show(kLineTitle, "GATR2 testing");

    communigatr::ProsDriverConfig config;
    config.port = robot_config::kNavigatrPort;
    config.baud = robot_config::kNavigatrBaud;
    navigatr    = new ProsDriver(config);
    drive       = new DriveControl(*navigatr, &ProsDriver::now);

    const char* why = nullptr;
    if (!investigatr::Navigator::valid(robot_config::navigatorConfig(), &why)) {
        show(kLineInit, "init: navigator config invalid: %s", why);
        return;
    }
    if (!navigatr->start()) {
        show(kLineInit, "init: port %d not opened, errno %d", robot_config::kNavigatrPort, errno);
        return;
    }
    if (!waitUntil(kConnectWait, [] { return navigatr->status().connected; })) {
        show(kLineInit, "init: no link, robot not placed");
        return;
    }

    const communigatr::PlacementTicket ticket = navigatr->submitPlacement(robot_config::kStartPose);
    waitUntil(kPlacementWait, [ticket] {
        return navigatr->placementResult(ticket) != communigatr::PlacementResult::kPending;
    });
    const communigatr::PlacementStatus placement = navigatr->placementStatus(ticket);
    show(kLineInit, "init: placement %s, result %d", placementName(placement.state),
         placement.result);
}

void disabled() { drive->stop(); }

void competition_initialize() {}

void autonomous() {
    runDemo([] { return false; });
}

// Left stick Y drives forward, right stick X turns (right = clockwise).
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        if (master.get_digital_new_press(robot_config::kDemoButton)) {
            runDemo([&master] { return master.get_digital(robot_config::kCancelButton) != 0; });
        }
        investigatr::DriveCommand demand;
        demand.forward = stick(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        demand.turn    = -stick(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        drive->manual(demand);
        showStatus();
        pros::delay(kLoopMs);
    }
}
