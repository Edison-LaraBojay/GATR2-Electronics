// main.cpp
// spectaGATR demo: publishes a robot pose to the laptop viewer through the
// spectaGATR publisher. It never commands a motor.
//
// Pose sources, LEFT/RIGHT on the controller (a change also resets the trail):
//   SYNTHETIC        figure eight made up on the Brain
//   VEX IMU          attitude from the VEX IMU, x/y fixed: no position source
//   CALLER ODOMETRY  currentPose() in src/pose_source.cpp, a placeholder
// A resets the trail, B pauses or resumes pose updates (the viewer goes
// stale), X recalibrates the VEX IMU (IMU source only, robot still).

#include "main.h"

#include <cmath>
#include <cstdint>

#include "demo_config.h"
#include "demo_sources.h"
#include "pose_source.h"
#include "spectagatr/pros_publisher.h"

spectagatr::ProsPublisher viz(demo_config::kRateHz);

namespace
{

using demo_config::Source;
using spectagatr::radiansToDegrees;

constexpr int      kSourceCount = 3;
constexpr uint32_t kDrawMs      = 100;

enum class ImuState : uint8_t { kMissing, kCalibrating, kReady };

struct Screen {
    Source   source  = demo_config::kDefaultSource;
    bool     paused  = false;
    bool     started = false;
    ImuState imu     = ImuState::kMissing;
    bool     sent    = false; // a pose was supplied since the last source change
    DemoPose last;            // the last pose supplied to viz
    double   lines_per_second = 0;
};

int sourceIndex(Source source) { return static_cast<int>(source); }

Source step(Source source, int delta) {
    return static_cast<Source>((sourceIndex(source) + delta + kSourceCount) % kSourceCount);
}

const char* sourceName(Source source) {
    switch (source) {
    case Source::kSynthetic:
        return "SYNTHETIC x/y";
    case Source::kVexImu:
        return "VEX IMU ATTITUDE";
    case Source::kCallerOdometry:
        return "CALLER ODOMETRY";
    }
    return "?";
}

const char* sourceDetail(const Screen& s) {
    switch (s.source) {
    case Source::kSynthetic:
        return s.last.full_attitude ? "Figure eight, full attitude wobble"
                                    : "Figure eight, heading only";
    case Source::kVexImu:
        return "x/y FIXED: no position source";
    case Source::kCallerOdometry:
        return currentPoseIsPlaceholder() ? "currentPose() PLACEHOLDER: replace me"
                                          : "currentPose() in src/pose_source.cpp";
    }
    return "";
}

const char* imuText(ImuState state) {
    switch (state) {
    case ImuState::kMissing:
        return "IMU not found: nothing sent";
    case ImuState::kCalibrating:
        return "IMU calibrating, keep still";
    case ImuState::kReady:
        return "IMU ok";
    }
    return "";
}

ImuState readImu(pros::Imu& imu, VexImuReading& out) {
    if (!imu.is_installed()) {
        return ImuState::kMissing;
    }
    if (imu.is_calibrating()) {
        return ImuState::kCalibrating;
    }
    // PROS_ERR_F (infinity) on a failed read.
    out.rotation = imu.get_rotation();
    out.roll     = imu.get_roll();
    out.pitch    = imu.get_pitch();
    if (!std::isfinite(out.rotation) || !std::isfinite(out.roll) || !std::isfinite(out.pitch)) {
        return ImuState::kMissing;
    }
    return ImuState::kReady;
}

unsigned long ul(uint32_t value) { return static_cast<unsigned long>(value); }

template <typename... Args> void row(int line, const char* format, Args... args) {
    const int16_t y = static_cast<int16_t>(4 + line * 20);
    pros::screen::erase_rect(0, y, 479, y + 19);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, y, format, args...);
}

void draw(const Screen& s) {
    const spectagatr::PublisherStats stats = viz.stats();
    row(0, "%s", "spectaGATR demo: NOTHING MOVES");
    row(1, "Source %d/%d: %s", sourceIndex(s.source) + 1, kSourceCount, sourceName(s.source));
    row(2, "%s", sourceDetail(s));
    if (!s.started) {
        row(3, "%s", "PUBLISHER NOT STARTED: no task");
    } else if (s.paused) {
        row(3, "%s", "PAUSED (B): no updates, goes stale");
    } else if (s.source == Source::kVexImu) {
        row(3, "%s (port %d)", imuText(s.imu), static_cast<int>(demo_config::kImuPort));
    } else {
        row(3, "Supplying a pose every %lu ms", ul(demo_config::kLoopMs));
    }
    row(4, "Session %08lX  cap %.0f Hz  %.1f/s", ul(viz.session()), viz.rate(), s.lines_per_second);
    row(5, "Lines %lu  rejected %lu  write err %lu", ul(stats.lines), ul(stats.rejected),
        ul(stats.write_errors));
    row(6, "Overwritten %lu (newest pose wins)", ul(stats.overwritten));
    if (s.sent) {
        row(7, "x %.3f m  y %.3f m  yaw %.1f deg", s.last.x, s.last.y,
            radiansToDegrees(s.last.attitude.yaw));
        if (s.last.full_attitude) {
            row(8, "roll %.1f  pitch %.1f deg (full)", radiansToDegrees(s.last.attitude.roll),
                radiansToDegrees(s.last.attitude.pitch));
        } else {
            row(8, "%s", "heading only: roll 0, pitch 0");
        }
    } else {
        row(7, "%s", "No pose sent from this source");
        row(8, "%s", "");
    }
    row(9, "%s", "A reset trail  B pause  LEFT/RIGHT src");
    row(10, "%s", "X recalibrate IMU (robot still)");
}

void supply(Screen& s, const DemoPose& pose) {
    if (s.paused) {
        return;
    }
    if (pose.full_attitude) {
        viz.updatePose(pose.x, pose.y, pose.attitude);
    } else {
        viz.updatePose(pose.x, pose.y, pose.attitude.yaw);
    }
    s.last = pose;
    s.sent = true;
}

void demoLoop() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Imu        imu(demo_config::kImuPort);
    Screen           s;
    s.started = viz.running();

    spectagatr::Milliseconds synthetic_ms = 0; // frozen while paused
    uint32_t                 wake         = pros::millis();
    uint32_t                 last_ms      = wake;
    uint32_t                 next_draw    = wake;
    uint32_t                 rate_start   = wake;
    uint32_t                 rate_lines   = viz.stats().lines;
    while (true) {
        const uint32_t now = pros::millis();
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            viz.resetTrail();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            s.paused = !s.paused;
        }
        const bool left  = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
        const bool right = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
        if (left || right) {
            s.source = step(s.source, right ? 1 : -1);
            s.sent   = false;
            viz.resetTrail(); // another source is another frame
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) &&
            s.source == Source::kVexImu && imu.is_installed()) {
            imu.reset(false); // waits up to 1 s for the IMU to start calibrating
            viz.resetTrail();
        }

        if (!s.paused) {
            synthetic_ms += now - last_ms;
        }
        last_ms = now;

        switch (s.source) {
        case Source::kSynthetic:
            supply(s, syntheticPose(synthetic_ms));
            break;
        case Source::kVexImu: {
            VexImuReading reading;
            s.imu = readImu(imu, reading);
            if (s.imu == ImuState::kReady) {
                supply(s, imuPose(reading));
            }
            break;
        }
        case Source::kCallerOdometry: {
            const OdometryPose odometry = currentPose();
            DemoPose           pose;
            pose.x            = odometry.x;
            pose.y            = odometry.y;
            pose.attitude.yaw = odometry.heading;
            supply(s, pose);
            break;
        }
        }

        if (now - rate_start >= 1000) {
            const uint32_t lines = viz.stats().lines;
            s.lines_per_second   = (lines - rate_lines) * 1000.0 / (now - rate_start);
            rate_lines           = lines;
            rate_start           = now;
        }
        if (static_cast<int32_t>(now - next_draw) >= 0) {
            draw(s);
            next_draw = now + kDrawMs;
        }
        pros::Task::delay_until(&wake, demo_config::kLoopMs);
    }
}

} // namespace

void initialize() {
    pros::screen::set_eraser(0x00000000);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::erase();
    viz.start();
    pros::Imu imu(demo_config::kImuPort);
    if (imu.is_installed()) {
        imu.reset(false); // calibrates for about 2 s; the screen shows it
    }
    static pros::Task task(demoLoop, "spectagatr demo");
}

// Nothing to do in any competition mode: the demo task runs throughout and
// no motor is ever commanded.
void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {}
