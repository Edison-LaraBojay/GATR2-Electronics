// main.cpp
// Encoder localization demo. The Pico counts two tracking wheels, the Pi
// relays the counts over USB, and this program combines them with the VEX IMU
// heading into a pose and publishes it to spectaGATR.
// Manual mecanum driving: include/drive_config.h. Localization: localization_config.h.

#include "main.h"

#include <cmath>

#include "drive.h"
#include "encoder_receiver.h"
#include "localization.h"
#include "localization_config.h"
#include "spectagatr/pros_publisher.h"

spectagatr::ProsPublisher viz(config::kVizRateHz);

namespace
{

using localization::StepKind;

constexpr uint32_t kLoopMs = 10;
constexpr uint32_t kDrawMs = 100;

pros::Imu                     imu(config::kImuPort);
localization::EncoderReceiver receiver;
localization::State           state(config::initialPose(), config::trackerLimits());
const odometry::WheelSetup    kWheels = config::wheels();

// Survives opcontrol restarts on competition mode changes.
bool                   g_ready         = false;
bool                   g_receiver_ok   = false;
bool                   g_viz_ok        = false;
uint32_t               g_last_number   = 0;
StepKind               g_last_step     = StepKind::kBaseline;
encoder_record::Record g_last_record;

// ---------------------------------------------------------------------------
// Screen
// ---------------------------------------------------------------------------

template <typename... Args> void row(int line, const char* format, Args... args) {
    const int16_t y = static_cast<int16_t>(4 + line * 20);
    pros::screen::erase_rect(0, y, 479, y + 19);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, y, format, args...);
}

void title() {
    if (!g_receiver_ok) {
        row(0, "%s", "ENCODER RECEIVER TASK FAILED");
    } else if (!g_viz_ok) {
        row(0, "%s", "spectaGATR PUBLISHER FAILED");
    } else {
        row(0, "%s", "localization + manual mecanum");
    }
    row(8, "%s", drive::status());
}

void status(const char* text) {
    title();
    row(2, "%s", text);
}

unsigned long ul(uint32_t v) { return static_cast<unsigned long>(v); }

void drawStatus(bool imu_ok) {
    const uint32_t now = pros::millis();
    title();
    row(1, "IMU port %d: %s", config::kImuPort, imu_ok ? "ok" : "INVALID, pose held");

    if (state.encoders.stale(now)) {
        row(2, "%s", "ENCODERS STALE: no records, pose held");
    } else {
        row(2, "Encoders ok: run %04X seq %u, last %s", static_cast<unsigned>(g_last_record.run_id),
            static_cast<unsigned>(g_last_record.seq), localization::stepName(g_last_step));
    }
    row(3, "Counts fwd ch%d %ld  side ch%d %ld", kWheels.forward.channel,
        static_cast<long>(localization::channelCount(g_last_record, kWheels.forward.channel)),
        kWheels.sideways.channel,
        static_cast<long>(localization::channelCount(g_last_record, kWheels.sideways.channel)));

    const odometry::Pose& p = state.pose;
    row(4, "x %.1f in  y %.1f in  heading %.1f deg", p.x / config::kInch, p.y / config::kInch,
        odometry::radiansToDegrees(p.heading));

    const localization::Health& h = state.health;
    if (h.uncertain()) {
        row(5, "UNCERTAIN: restarts %lu gaps %lu jumps %lu imu %lu", ul(h.restarts), ul(h.gaps),
            ul(h.jumps), ul(h.imu_faults));
    } else {
        row(5, "%s", "No lost data since start");
    }

    const localization::ReceiverStats rx = receiver.stats();
    row(6, "Link: records %lu skipped lines %lu dup %lu", ul(rx.records), ul(rx.skipped_lines),
        ul(h.duplicates));
    const spectagatr::PublisherStats vs = viz.stats();
    row(7, "spectaGATR: lines %lu write errors %lu", ul(vs.lines), ul(vs.write_errors));
}

// ---------------------------------------------------------------------------
// Steps
// ---------------------------------------------------------------------------

localization::ImuReading readImu() {
    localization::ImuReading reading;
    if (!imu.is_installed() || imu.is_calibrating()) {
        return reading;
    }
    reading.rotation_degrees = imu.get_rotation(); // PROS_ERR_F (infinity) on failure
    reading.valid            = std::isfinite(reading.rotation_degrees);
    return reading;
}

// Blocks until the IMU is calibrated. Robot still.
void calibrateImu() {
    while (true) {
        if (!imu.is_installed()) {
            status("IMU NOT FOUND on its port, retrying");
            pros::delay(500);
            continue;
        }
        status("IMU calibrating: keep the robot still");
        imu.reset(true); // about 2 s
        if (readImu().valid) {
            return;
        }
        status("IMU calibration failed, retrying");
        pros::delay(500);
    }
}

// Blocks until a record newer than the stale limit arrives.
void waitForFreshRecord() {
    while (true) {
        localization::ReceivedRecord latest;
        if (receiver.latest(latest) &&
            pros::millis() - latest.arrived_ms <= config::trackerLimits().stale_ms) {
            return;
        }
        status("Waiting for encoder records from the Pi");
        pros::delay(20);
    }
}

// The newest record only: counts are cumulative. The first one sets the
// encoder and IMU baselines; later ones read the IMU, take count deltas and
// move the pose.
void processLatestRecord() {
    localization::ReceivedRecord latest;
    if (!receiver.latest(latest) || latest.number == g_last_number) {
        return;
    }
    g_last_number = latest.number;
    g_last_record = latest.record;

    const localization::ImuReading heading = readImu();
    g_last_step = localization::applyRecord(state, kWheels, latest.record, latest.arrived_ms,
                                            heading);
}

// Only a pose backed by fresh encoders and a good IMU goes out. Otherwise
// nothing is sent and the viewer marks the pose stale.
bool poseIsLive() {
    return state.have_baseline && g_last_step != StepKind::kImuInvalid &&
           !state.encoders.stale(pros::millis());
}

void publishPose() {
    if (poseIsLive()) {
        viz.updatePose(state.pose.x, state.pose.y, state.pose.heading);
    }
}

// Isolates Brain stdout -> controller -> laptop from every sensor and the Pi.
// This branch never initializes motor groups or calls the drive update loop.
[[noreturn]] void runControllerViewerTest() {
    row(0, "%s", g_viz_ok ? "SIMULATED POSE: controller link test"
                          : "TEST: spectaGATR PUBLISHER FAILED");
    row(1, "%s", "No Pi, encoders or IMU required");
    row(2, "%s", "Disconnect Brain USB after uploading");
    row(3, "%s", "Controller USB -> laptop; radio -> Brain");
    row(8, "%s", "MOTORS DISABLED: simulated movement only");
    const uint32_t start = pros::millis();
    viz.resetTrail();
    while (true) {
        // A 0.5 m radius circle near field center, one lap per 20 seconds.
        const double phase = odometry::degreesToRadians(360.0) *
                             ((pros::millis() - start) % 20000u) / 20000.0;
        const double x = 1.8 + 0.5 * std::cos(phase);
        const double y = 1.8 + 0.5 * std::sin(phase);
        const double heading = std::atan2(std::cos(phase), -std::sin(phase));
        viz.updatePose(x, y, heading);
        row(4, "SIM x %.2f m  y %.2f m  heading %.1f deg", x, y,
            odometry::radiansToDegrees(heading));
        const spectagatr::PublisherStats stats = viz.stats();
        row(7, "spectaGATR: lines %lu write errors %lu", ul(stats.lines), ul(stats.write_errors));
        pros::delay(kDrawMs);
    }
}

} // namespace

void initialize() {
    if (config::kControllerViewerTest) {
        g_viz_ok = viz.start();
        row(0, "%s", "CONTROLLER VIEWER TEST: simulated pose");
        row(8, "%s", "MOTORS DISABLED");
        return;
    }
    drive::initialize(config::kImuPort);
    g_receiver_ok = receiver.start();
    g_viz_ok      = viz.start();
    title();
}

void disabled() { drive::stop(); }

void competition_initialize() {}

void autonomous() { drive::stop(); }

void opcontrol() {
    if (config::kControllerViewerTest) {
        runControllerViewerTest();
    }
    drive::stop();
    if (!g_ready) {
        calibrateImu();       // VEX IMU calibrated, robot still
        waitForFreshRecord(); // pose starts at config::initialPose()
        g_ready = true;
    }

    uint32_t last_draw = 0;
    while (true) {
        processLatestRecord(); // baselines, IMU, deltas, pose
        publishPose();         // spectaGATR
        drive::update();        // manual control, independent of pose freshness

        const uint32_t now = pros::millis();
        if (now - last_draw >= kDrawMs) {
            drawStatus(g_last_step != StepKind::kImuInvalid);
            last_draw = now;
        }
        pros::delay(kLoopMs);
    }
}
