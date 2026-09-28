// main.cpp
// Localization test: motor-free bench program. Sends the robot profile from
// brain/robot/gatr2_robot.h, keeps the Pi link up, places the robot once at
// program start, calibrates wheels and the IMU, and shows pose, readiness and
// recovery on the Brain screen. Move the robot by hand.
//
// Pages (B or the right screen button cycles):
//   Status       A place at the start pose, X recalibrate the IMU,
//                Y reinitialize the Pico IMU, hold L1+R1 and press A to
//                reinitialize localization (placement needed after)
//   Wheels       LEFT/RIGHT wheel, UP start a push, DOWN end it,
//                L1/R1 -/+1 cm, L2/R2 -/+10 cm reference, X clear trials,
//                A apply the proposed travel scale, then A again to place
//                at the last pose
//   Recovery     link, session, Pi, Pico and calibration history

#include "main.h"
#include "robot_config.h"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "communigatr/link_events.h"
#include "communigatr/pros_link.h"
#include "communigatr/pros_vex_imu.h"
#include "communigatr/startup_placement.h"
#include "communigatr/vex_imu_recalibration.h"
#include "communigatr/wheel_calibration.h"

namespace
{

using communigatr::Seconds;
using investigatr::Pose;

// ---------------------------------------------------------------------------
// Link access. Every communiGATR call of this program is in this section.
// ---------------------------------------------------------------------------

using Link = communigatr::ProsLink;

std::unique_ptr<communigatr::ProsVexImu> g_vex;
std::unique_ptr<Link>                    g_link;

Seconds now() {
    return Link::now();
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
    g_link.reset(new Link(config));
}

// What the screens and decisions need from the link, read once per loop.
struct View {
    bool                       started   = false;
    bool                       connected = false;
    communigatr::Readiness     readiness = communigatr::Readiness::kConnecting;
    uint32_t                   session     = 0;
    uint32_t                   pi_instance = 0;
    communigatr::ProfileStatus profile;
    bool                       state_valid = false;
    bool                       localized   = false;
    communigatr::HealthBits    health;
    uint8_t                    calibration = gatr2::kCalibrationNone;
    bool                       heading_valid = false;
    investigatr::Radians       heading       = 0; // raw Pi heading, placed or not
    investigatr::RobotState    robot;
    communigatr::LinkSnapshot  snapshot;
};

// False when the link was busy; v is then left as it was.
bool readView(Seconds t, View& v) {
    const communigatr::ProsLinkStatus s = g_link->status();
    if (s.busy) {
        return false;
    }
    v.started       = s.started;
    v.connected     = s.connected;
    v.readiness     = s.readiness;
    v.session       = s.session;
    v.pi_instance   = s.pi_instance;
    v.profile       = s.profile;
    v.state_valid   = s.state.valid;
    v.localized     = s.summary.localized;
    v.health        = s.summary.health;
    v.calibration   = s.summary.calibration;
    v.heading_valid = s.heading_valid;
    v.heading       = s.heading;
    v.robot                    = g_link->robot(t);
    v.snapshot.connected       = s.connected;
    v.snapshot.session         = s.session;
    v.snapshot.pi_instance     = s.pi_instance;
    v.snapshot.profile         = s.profile.state;
    v.snapshot.state_valid     = s.state.valid;
    v.snapshot.localized       = v.localized;
    v.snapshot.health          = s.state.valid ? s.state.state.health : 0;
    v.snapshot.calibration     = v.calibration;
    v.snapshot.odometry_epoch  = s.state.state.odometry_epoch;
    v.snapshot.anchor_revision = s.state.state.anchor_revision;
    return true;
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

// ---------------------------------------------------------------------------
// Program state.
// ---------------------------------------------------------------------------

enum class Page : uint8_t { kStatus, kWheels, kRecovery };

enum class Capture : uint8_t { kIdle, kStart, kEnd };

struct WheelTool {
    communigatr::WheelCalibration calibration;
    const char*                   name;
};

Page                                      g_page = Page::kStatus;
communigatr::LinkEvents                   g_events;
std::unique_ptr<communigatr::StartupPlacement> g_startup;
std::vector<WheelTool>                    g_wheels;
std::size_t                               g_wheel      = 0;
double                                    g_reference  = robot_config::kCalibrationDistance;
Capture                                   g_capture    = Capture::kIdle;
communigatr::WheelTicket                  g_capture_ticket = 0;
const char*                               g_message    = "Hold still while sensors start";
char                                      g_note[64]   = {};
communigatr::PlacementTicket              g_placement  = 0;
communigatr::ControlTicket                g_control    = 0;
bool                                      g_have_last_pose = false;
Pose                                      g_last_pose;
bool                                      g_offer_place = false; // after applying a calibration
communigatr::VexImuRecalibration          g_recal; // VEX IMU, after the Pi's stillness check
uint32_t                                  g_events_seen = 0;

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

void buildWheelTools() {
    const communigatr::RobotProfile p      = gatr2_robot::profile();
    const auto                      config = robot_config::wheelCalibration();
    for (std::size_t i = 0; i < p.wheels.size(); ++i) {
        // Cross wheels: those measuring roughly perpendicular to this one.
        std::vector<uint8_t> cross;
        for (std::size_t j = 0; j < p.wheels.size(); ++j) {
            const double between = std::fabs(investigatr::wrapAngle(p.wheels[j].angle - p.wheels[i].angle));
            if (j != i && std::fabs(between - investigatr::kPi / 2) < investigatr::kPi / 8) {
                cross.push_back(p.wheels[j].encoder_port);
            }
        }
        const bool forward = std::fabs(std::sin(p.wheels[i].angle)) < 0.5;
        g_wheels.push_back(WheelTool{
            communigatr::WheelCalibration(p.wheels[i], cross, config),
            forward ? "forward" : "sideways"});
    }
}

communigatr::CalibrationSnapshot snapshotFrom(const communigatr::WheelReadings& r, const View& v) {
    communigatr::CalibrationSnapshot s;
    s.profile_id    = v.profile.id;
    s.heading_valid = v.heading_valid;
    s.heading       = v.heading;
    for (uint8_t i = 0; i < r.count && i < gatr2::kWheelReadingsMax; ++i) {
        communigatr::WheelSample w = communigatr::fromReading(r.wheels[i]);
        // Age at the time of use: the Pi's age plus the time since the reply.
        w.age += now() - r.received_at + r.round_trip;
        s.wheels.push_back(w);
    }
    return s;
}

bool pressed(pros::Controller& c, pros::controller_digital_e_t button) {
    return c.is_connected() == 1 && c.get_digital_new_press(button) == 1;
}

bool held(pros::Controller& c, pros::controller_digital_e_t button) {
    return c.is_connected() == 1 && c.get_digital(button) == 1;
}

bool poseLive(const View& v) {
    return v.robot.valid() && v.robot.age <= robot_config::kMaxPoseAgeSeconds;
}

void placeAt(const Pose& pose, const char* done) {
    const communigatr::PlacementTicket t = g_link->place(pose);
    if (t == 0) {
        g_message = "Place refused: link, profile or a pending placement";
        return;
    }
    g_placement = t;
    g_message   = done;
}

void startControl(communigatr::ControlTicket t, const char* done) {
    if (t == 0) {
        g_message = "Refused: no link, or another request pending";
        return;
    }
    g_control = t;
    g_message = done;
}

// ---------------------------------------------------------------------------
// Pages.
// ---------------------------------------------------------------------------

void statusInput(pros::Controller& c, const View& v) {
    if (pressed(c, pros::E_CONTROLLER_DIGITAL_A)) {
        if (held(c, pros::E_CONTROLLER_DIGITAL_L1) && held(c, pros::E_CONTROLLER_DIGITAL_R1)) {
            startControl(g_link->reinitialize(), "Reinitializing: place the robot after");
        } else if (!v.connected) {
            g_message = "No link; connect, then A again";
        } else {
            placeAt(gatr2_robot::kStartPose, "Placing at the start pose");
        }
    }
    if (pressed(c, pros::E_CONTROLLER_DIGITAL_X)) {
        if (g_vex) {
            // The Pi checks the robot is still, then the VEX firmware
            // calibrates; the Pi holds the pose meanwhile.
            g_message = g_recal.begin(g_link->recalibrate())
                            ? "Checking the robot is still..."
                            : "Refused: no link, or another request pending";
        } else {
            startControl(g_link->recalibrate(), "IMU recalibration requested: hold still");
        }
    }
    if (pressed(c, pros::E_CONTROLLER_DIGITAL_Y)) {
        if (g_vex) {
            g_message = "Y reinitializes the Pico IMU; this profile uses the VEX IMU";
        } else {
            startControl(g_link->reinitImu(), "Pico IMU reinitializing: hold still");
        }
    }
}

void wheelsInput(pros::Controller& c, const View& v) {
    if (g_wheels.empty()) {
        return;
    }
    if (pressed(c, pros::E_CONTROLLER_DIGITAL_LEFT)) {
        g_wheel   = (g_wheel + g_wheels.size() - 1) % g_wheels.size();
        g_capture = Capture::kIdle;
    }
    if (pressed(c, pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        g_wheel   = (g_wheel + 1) % g_wheels.size();
        g_capture = Capture::kIdle;
    }
    const double steps[4] = {-0.01, 0.01, -0.1, 0.1};
    const pros::controller_digital_e_t keys[4] = {
        pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_R1,
        pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_R2};
    for (int i = 0; i < 4; ++i) {
        if (pressed(c, keys[i])) {
            g_reference = std::round((g_reference + steps[i]) * 100.0) / 100.0;
        }
    }
    if (pressed(c, pros::E_CONTROLLER_DIGITAL_UP) || pressed(c, pros::E_CONTROLLER_DIGITAL_DOWN)) {
        const bool                     start  = c.get_digital(pros::E_CONTROLLER_DIGITAL_UP) == 1;
        const communigatr::WheelTicket ticket = g_link->requestWheels();
        if (ticket == 0) {
            g_message = "Cannot read the wheels: no link, or a read pending";
        } else {
            g_capture        = start ? Capture::kStart : Capture::kEnd;
            g_capture_ticket = ticket;
            g_message        = start ? "Reading start..." : "Reading end...";
        }
    }
    if (pressed(c, pros::E_CONTROLLER_DIGITAL_X)) {
        g_wheels[g_wheel].calibration.clear();
        g_message = "Trials cleared";
    }
    if (pressed(c, pros::E_CONTROLLER_DIGITAL_A)) {
        if (g_offer_place) {
            g_offer_place = false;
            placeAt(g_last_pose, "Placing at the pose before the change");
            return;
        }
        if (!gatr2_robot::kSendProfile) {
            g_message = "Apply needs a Brain profile config (kSendProfile); copy the scale by hand";
            return;
        }
        const communigatr::CalibrationProposal p = g_wheels[g_wheel].calibration.proposal();
        if (!p.ready) {
            g_message = "Need 3 agreeing trials before applying";
            return;
        }
        communigatr::RobotProfile profile = g_link->profile();
        for (communigatr::TrackingWheel& w : profile.wheels) {
            if (w.encoder_port == g_wheels[g_wheel].calibration.port()) {
                w.travel_scale = p.scale;
            }
        }
        g_have_last_pose = poseLive(v);
        if (g_have_last_pose) {
            g_last_pose = v.robot.pose;
        }
        if (!g_link->setProfile(profile)) {
            g_message = "Profile refused by the Brain check";
            return;
        }
        std::snprintf(g_note, sizeof(g_note), "Port %u travel_scale = %.4f (copy to gatr2_robot.h)",
                      unsigned(g_wheels[g_wheel].calibration.port()), p.scale);
        g_offer_place = g_have_last_pose;
        g_message     = g_offer_place ? "Applied; A places at the last pose" : "Applied; place again";
    }
}

// Finishes a wheel capture once its read settles.
void serviceCapture(const View& v) {
    if (g_capture == Capture::kIdle) {
        return;
    }
    const communigatr::WheelStatus w = g_link->wheels(g_capture_ticket);
    if (w.state == communigatr::WheelResult::kPending) {
        return;
    }
    const Capture step = g_capture;
    g_capture          = Capture::kIdle;
    if (w.state == communigatr::WheelResult::kRejected) {
        std::snprintf(g_note, sizeof(g_note), "Wheel read refused (result %u): profile applied?",
                      unsigned(w.result));
        g_message = g_note;
        return;
    }
    if (w.state != communigatr::WheelResult::kOk) {
        g_message = "No answer to the wheel read: press again";
        return;
    }
    const communigatr::CalibrationSnapshot s = snapshotFrom(w.readings, v);
    communigatr::WheelCalibration&         cal = g_wheels[g_wheel].calibration;
    const communigatr::TrialStatus status =
        step == Capture::kStart ? cal.start(s) : cal.finish(s, g_reference);
    if (status == communigatr::TrialStatus::kAccepted) {
        g_message = step == Capture::kStart ? "Push along the wheel, then DOWN" : "Trial accepted";
    } else {
        std::snprintf(g_note, sizeof(g_note), "Rejected: %s", communigatr::toString(status));
        g_message = g_note;
    }
}

// ---------------------------------------------------------------------------
// Screen.
// ---------------------------------------------------------------------------

constexpr int kButtonTop = 208;

template <typename... Args> void row(int index, const char* format, Args... args) {
    const int y = 4 + index * 20;
    pros::screen::erase_rect(0, y, 479, y + 19);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, y, format, args...);
}

void showStatus(const View& v) {
    row(1, "%s | %s", gatr2_robot::kUseUsb ? "USB" : "RS-485", communigatr::toString(v.readiness));
    row(2, "Profile %08lx %s", static_cast<unsigned long>(v.profile.id),
        v.profile.state == communigatr::ProfileSync::kRejected
            ? communigatr::profileReasonName(v.profile.reason)
            : "");
    if (v.robot.valid()) {
        row(3, "x %+.3f  y %+.3f m  h %+.2f deg", v.robot.pose.x, v.robot.pose.y,
            v.robot.pose.heading * 180.0 / investigatr::kPi);
        row(4, "Pose %s, age %.0f ms", poseLive(v) ? "LIVE" : "STALE", v.robot.age * 1000.0);
    } else {
        row(3, "x --  y --  h --");
        row(4, "Pose: %s", investigatr::toString(v.robot.status));
    }
    row(5, "Enc %s  IMU %s  Pico %s  Still %s", v.health.encoders_fresh ? "ok" : "--",
        v.health.imu_fresh ? "ok" : "--", v.health.pico_link ? "ok" : "--",
        v.health.stationary ? "yes" : "no");
    if (g_vex) {
        const communigatr::BenchImuSample imu = g_vex->sample();
        row(6, "VEX IMU P%u: %s %+.2f deg  recal %s", unsigned(gatr2_robot::kVexImuPort),
            g_vex->calibrating() ? "calibrating" : imu.valid ? "ok" : "unavailable",
            imu.rotation_mdeg / 1000.0, communigatr::toString(g_recal.state()));
    } else {
        row(6, "Pi IMU calibration: %s%s", communigatr::calibrationName(v.calibration),
            v.calibration == gatr2::kCalibrationFailed ? " (X retries)" : "");
    }
    row(7, "Start placement: %s", communigatr::toString(g_startup->state()));
    row(8, "A place  X recal IMU  Y Pico IMU  L1+R1+A reinit");
}

void showWheels() {
    if (g_wheels.empty()) {
        row(1, "No tracking wheels in the profile");
        return;
    }
    const WheelTool&                         tool = g_wheels[g_wheel];
    const communigatr::CalibrationProposal   p    = tool.calibration.proposal();
    const communigatr::WheelReadings&        r    = g_link->wheelReadings();
    double                                   travel = 0;
    for (uint8_t i = 0; i < r.count; ++i) {
        if (r.wheels[i].port == tool.calibration.port()) {
            travel = r.wheels[i].travel_um * 1e-6;
        }
    }
    row(1, "Wheel port %u (%s)  %s", unsigned(tool.calibration.port()), tool.name,
        tool.calibration.started() ? "PUSHING" : "");
    row(2, "Reference %.2f m (L1/R1 1 cm, L2/R2 10 cm)", g_reference);
    row(3, "Raw travel %.4f m (last read)", travel);
    row(4, "Trials %u  scale %.4f  spread %.2f%%", unsigned(p.trials), p.scale, p.spread * 100.0);
    const auto& trials = tool.calibration.trials();
    for (std::size_t i = 0; i < 3; ++i) {
        if (i < trials.size()) {
            const auto& t = trials[trials.size() - 1 - i];
            row(5 + static_cast<int>(i), "  ref %.3f  wheel %.4f  scale %.4f", t.reference,
                t.measured, t.scale);
        } else {
            row(5 + static_cast<int>(i), "");
        }
    }
    row(8, p.ready ? "READY: A applies  UP start  DOWN end  X clear" : "UP start  DOWN end  X clear");
}

void showRecovery() {
    for (int i = 0; i < 8; ++i) {
        if (static_cast<std::size_t>(i) < g_events.size()) {
            const communigatr::LinkEventRecord& e = g_events.at(static_cast<std::size_t>(i));
            if (e.event == communigatr::LinkEvent::kLinkRestored) {
                row(1 + i, "%7.1f s  %s after %.1f s", e.at, communigatr::toString(e.event),
                    e.duration);
            } else {
                row(1 + i, "%7.1f s  %s", e.at, communigatr::toString(e.event));
            }
        } else {
            row(1 + i, "");
        }
    }
}

void display(const View& v) {
    const char* titles[] = {"Status", "Wheel calibration", "Recovery"};
    row(0, "Localization test | %s", titles[static_cast<int>(g_page)]);
    switch (g_page) {
    case Page::kStatus: showStatus(v); break;
    case Page::kWheels: showWheels(); break;
    case Page::kRecovery: showRecovery(); break;
    }
    row(9, "%s", g_message);
    pros::screen::set_pen(0x00304B60);
    pros::screen::fill_rect(0, kButtonTop, 479, 239);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 12, kButtonTop + 6, "PLACE AT START");
    pros::screen::print(pros::E_TEXT_MEDIUM, 330, kButtonTop + 6, "NEXT PAGE");
    pros::screen::set_pen(0x00FFFFFF);
}

// ---------------------------------------------------------------------------
// Main loop.
// ---------------------------------------------------------------------------

void run() {
    createLink();
    buildWheelTools();
    g_startup.reset(new communigatr::StartupPlacement(robot_config::kStartupPolicy,
                                                      robot_config::kStartupWaitSeconds));
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::screen::set_eraser(0x00000000);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::erase();

    double   next_start   = 0;
    int32_t  last_touches = -1;
    uint32_t wake         = pros::millis();
    uint32_t last_display = wake - robot_config::kDisplayPeriodMs;

    View v;
    while (true) {
        const Seconds t = now();
        if (!g_link->status().started && t >= next_start) {
            // Retried once a second; opening can fail right after power-up.
            g_link->start();
            next_start = t + 1.0;
        }
        // A busy link keeps the last view and skips the decisions below.
        const bool fresh = readView(t, v);
        if (fresh) {
            g_events.update(v.snapshot, t);
            // Pi restart, reinitialize, or travel lost in a sensor gap: the
            // pose is invalid until the robot is placed again.
            if (placementLostSinceLastCheck() && !g_offer_place) {
                g_message = "POSE INVALID: put the robot at the start pose, press A";
            }
        }

        // Program start placement: once, when everything is ready.
        communigatr::StartupInputs in;
        in.connected     = v.connected;
        in.profile_ready = v.profile.state == communigatr::ProfileSync::kApplied;
        in.sensors_ready = sensorsReady(v.readiness);
        in.calibrating   = calibrating(v.readiness);
        in.localized     = v.localized;
        if (fresh && g_startup->update(in, t) == communigatr::StartupState::kSubmit) {
            const communigatr::PlacementTicket ticket = g_link->place(gatr2_robot::kStartPose);
            if (ticket != 0) {
                g_placement = ticket;
                g_startup->submitted();
                g_message = "Placed at the start pose";
            }
        }

        // Touch buttons: left half places, right half pages.
        const auto touch = pros::screen::touch_status();
        bool       touch_place = false;
        bool       touch_page  = false;
        if (touch.touch_status != pros::E_TOUCH_ERROR) {
            const bool down = touch.touch_status == pros::E_TOUCH_PRESSED ||
                              touch.touch_status == pros::E_TOUCH_HELD;
            if (down && touch.press_count != last_touches && touch.y >= kButtonTop) {
                touch_place = touch.x < 240;
                touch_page  = touch.x >= 240;
            }
            last_touches = touch.press_count;
        }
        if (pressed(controller, pros::E_CONTROLLER_DIGITAL_B) || touch_page) {
            g_page = static_cast<Page>((static_cast<int>(g_page) + 1) % 3);
            pros::screen::erase();
        }
        if (touch_place) {
            if (v.connected) {
                placeAt(gatr2_robot::kStartPose, "Placing at the start pose");
            } else {
                g_message = "No link; connect, then tap again";
            }
        }
        switch (g_page) {
        case Page::kStatus: statusInput(controller, v); break;
        case Page::kWheels: wheelsInput(controller, v); break;
        case Page::kRecovery: break;
        }
        serviceCapture(v);

        // VEX IMU recalibration: starts only after the Pi's stillness check.
        if (g_vex && g_recal.active()) {
            using communigatr::VexRecalibrationState;
            const VexRecalibrationState r =
                g_recal.update(g_link->control(g_recal.ticket()), g_vex->sample(), now());
            if (r == VexRecalibrationState::kStart) {
                g_recal.started(g_vex->recalibrate(), now()); // blocks about 1 s at most
                g_message = "VEX IMU calibrating: hold still";
            } else if (r == VexRecalibrationState::kDone) {
                g_message = "VEX IMU calibrated: put the robot at the start pose, press A";
            } else if (r == VexRecalibrationState::kMoving) {
                g_message = "Refused: robot moving; hold still and press X";
            } else if (r == VexRecalibrationState::kRefused) {
                g_message = "Recalibration refused: check the link";
            } else if (r == VexRecalibrationState::kImuFailed) {
                g_message = "VEX IMU did not calibrate: check the IMU";
            }
        }

        // Outcomes of the last placement and control.
        const communigatr::PlacementStatus placement = g_link->placement(g_placement);
        if (placement.state == communigatr::PlacementResult::kApplied && g_placement != 0) {
            g_placement = 0;
            g_message   = "Placement applied: push the robot to check the pose";
        } else if (placement.state == communigatr::PlacementResult::kRejected ||
                   placement.state == communigatr::PlacementResult::kTimedOut ||
                   placement.state == communigatr::PlacementResult::kSessionLost) {
            g_placement = 0;
            g_message   = "Placement not applied: check link and sensors, A again";
        }
        const communigatr::ControlStatus control = g_link->control(g_control);
        if (g_control != 0 && control.state != communigatr::ControlResult::kPending) {
            g_control = 0;
            switch (control.state) {
            case communigatr::ControlResult::kOk: g_message = "Done"; break;
            case communigatr::ControlResult::kNotStationary: g_message = "Refused: robot moving"; break;
            case communigatr::ControlResult::kNotReady: g_message = "Refused: profile not applied"; break;
            case communigatr::ControlResult::kFailed: g_message = "Failed: see Recovery page"; break;
            default: g_message = "No answer: check the link"; break;
            }
        }

        const uint32_t tick = pros::millis();
        if (tick - last_display >= robot_config::kDisplayPeriodMs) {
            display(v);
            last_display = tick;
        }
        pros::Task::delay_until(&wake, robot_config::kLoopPeriodMs);
    }
}

} // namespace

void initialize() {
    // Runs in every competition mode: the display and the link stay alive.
    static pros::Task task(run, "localization-test");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
    while (true) {
        pros::delay(20);
    }
}
