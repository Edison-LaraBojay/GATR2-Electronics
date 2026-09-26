#include "main.h"
#include "robot_config.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>

#include "communigatr/pros_driver.h"

namespace {

using communigatr::PlacementResult;
using communigatr::ProsDriver;
using communigatr::ProsDriverStatus;

constexpr int kButtonTop = 208;

communigatr::BenchImuSample readBenchImu(const pros::Imu& imu) {
    communigatr::BenchImuSample sample;
    sample.stamp_ms = pros::millis();
    if (imu.get_status() == pros::ImuStatus::error || imu.is_calibrating()) {
        return sample;
    }
    // PROS total rotation is clockwise-positive degrees; naviGATR uses CCW.
    const double mdeg = -imu.get_rotation() * 1000.0;
    if (!std::isfinite(mdeg) || mdeg < std::numeric_limits<int32_t>::min() ||
        mdeg > std::numeric_limits<int32_t>::max()) {
        return sample;
    }
    sample.rotation_mdeg = static_cast<int32_t>(std::lround(mdeg));
    sample.valid = true;
    return sample;
}

const char* placementName(PlacementResult result) {
    switch (result) {
    case PlacementResult::kNone: return "not requested";
    case PlacementResult::kPending: return "pending";
    case PlacementResult::kApplied: return "applied";
    case PlacementResult::kRejected: return "rejected";
    case PlacementResult::kTimedOut: return "timeout: outcome unknown";
    case PlacementResult::kSessionLost: return "session lost";
    }
    return "unknown";
}

bool livePose(const investigatr::InputSnapshot& sample) {
    return sample.connected && sample.robot.valid &&
           std::isfinite(sample.robot.age) && sample.robot.age >= 0 &&
           sample.robot.age <= robot_config::kMaxPoseAgeSeconds;
}

template <typename... Args>
void row(int index, const char* format, Args... args) {
    const int y = 4 + index * 20;
    pros::screen::erase_rect(0, y, 479, y + 19);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, y, format, args...);
}

void display(const ProsDriverStatus& link, const investigatr::InputSnapshot& sample,
             const communigatr::PlacementStatus& placement, const char* message,
             int open_error) {
    row(0, "Localization test | Smart Port %u", unsigned(robot_config::kNavigatrPort));
    if (!link.started) {
        row(1, "Serial open failed: errno %d (retrying)", open_error);
    } else if (link.error != communigatr::LinkError::kNone) {
        row(1, "Protocol error %u, peer version %u", unsigned(link.error),
            unsigned(link.peer_version));
    } else {
        row(1, "Link: %s | Pose: %s", sample.connected ? "connected" : "waiting/lost",
            livePose(sample) ? "LIVE" : sample.robot.valid ? "STALE" : "unavailable");
    }
    row(2, "Placement: %s", placementName(placement.state));
    if (sample.robot.valid) {
        row(3, "x: %+.3f m    y: %+.3f m", sample.robot.pose.x, sample.robot.pose.y);
        row(4, "Heading: %+.2f deg", sample.robot.pose.heading * 180.0 / investigatr::kPi);
        row(5, "Pose age: %.0f ms | Link: %.0f ms", sample.robot.age * 1000,
            sample.link_age * 1000);
    } else {
        row(3, "x: -- m    y: -- m");
        row(4, "Heading: -- deg");
        row(5, "Waiting for placement / fresh sensors");
    }
    row(6, "Session: %08lx | Frame: %lu", static_cast<unsigned long>(link.session),
        static_cast<unsigned long>(sample.frame));
    row(7, "Replies: %lu | Timeouts: %lu", static_cast<unsigned long>(link.stats.replies),
        static_cast<unsigned long>(link.stats.timeouts));
    row(8, "Set: %.2f, %.2f m / %.1f deg", robot_config::kStartX,
        robot_config::kStartY, robot_config::kStartHeadingDegrees);
    row(9, "%s", message);

    pros::screen::set_pen(0x00304B60);
    pros::screen::fill_rect(0, kButtonTop, 479, 239);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::print(pros::E_TEXT_MEDIUM, 12, kButtonTop + 6,
                        "A / TAP: set configured starting pose");
}

void log(const ProsDriverStatus& link, const investigatr::InputSnapshot& sample,
         const communigatr::PlacementStatus& placement) {
    std::printf("[localization] connected=%d live=%d placement=%s result=%u "
                "session=%08lx frame=%lu ",
                sample.connected, livePose(sample), placementName(placement.state),
                unsigned(placement.result), static_cast<unsigned long>(link.session),
                static_cast<unsigned long>(sample.frame));
    if (sample.robot.valid) {
        std::printf("x_m=%.3f y_m=%.3f heading_deg=%.2f age_ms=%.0f\n",
                    sample.robot.pose.x, sample.robot.pose.y,
                    sample.robot.pose.heading * 180.0 / investigatr::kPi,
                    sample.robot.age * 1000);
    } else {
        std::printf("pose=unavailable\n");
    }
}

void runTest() {
    std::shared_ptr<pros::Imu> vex_imu;
    if (robot_config::kUseVexImuBench) {
        vex_imu = std::make_shared<pros::Imu>(robot_config::kVexImuPort);
        vex_imu->reset(false);
    }
    communigatr::ProsDriverConfig config;
    config.port = robot_config::kNavigatrPort;
    config.baud = robot_config::kNavigatrBaud;
    config.client.placement_deadline = robot_config::kPlacementDeadlineSeconds;
    // Allow Pending replies for the whole deadline rather than exhausting
    // the library's shorter default retry count during sensor startup.
    config.client.placement_attempts = 100;
    if (vex_imu) {
        config.client.bench_imu = [vex_imu] { return readBenchImu(*vex_imu); };
    }
    ProsDriver driver(config);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    communigatr::PlacementTicket ticket = 0;
    const double startup_end = ProsDriver::now() + robot_config::kStartupWaitSeconds;
    bool startup_wait = true;
    double next_open = 0;
    int open_error = 0;
    int32_t last_touch_count = -1;
    uint32_t wake = pros::millis();
    uint32_t last_display = wake - robot_config::kDisplayPeriodMs;
    uint32_t last_log = wake - robot_config::kLogPeriodMs;
    const char* message = "Hold still for IMU calibration";

    pros::screen::set_eraser(0x00000000);
    pros::screen::set_pen(0x00FFFFFF);
    pros::screen::erase();

    while (true) {
        const double now = ProsDriver::now();
        auto link = driver.status();
        if (!link.started && now >= next_open) {
            if (!driver.start()) {
                open_error = errno;
            }
            next_open = now + 1.0;
            link = driver.status();
        }

        bool place = controller.is_connected() == 1 &&
                     controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == 1;
        const auto touch = pros::screen::touch_status();
        if (touch.touch_status != pros::E_TOUCH_ERROR) {
            const bool pressed = touch.touch_status == pros::E_TOUCH_PRESSED ||
                                 touch.touch_status == pros::E_TOUCH_HELD;
            place |= pressed && touch.press_count != last_touch_count &&
                     touch.y >= kButtonTop && touch.y <= 239 && touch.x >= 0 && touch.x <= 479;
            last_touch_count = touch.press_count;
        }

        // One attempt per program start. Recovery never repositions the robot
        // automatically after a timeout, disconnect or Pi restart.
        const bool imu_ready = !vex_imu || readBenchImu(*vex_imu).valid;
        if (startup_wait && link.connected && imu_ready && now < startup_end) {
            place = true;
            startup_wait = false;
        } else if (startup_wait && now >= startup_end) {
            startup_wait = false;
            message = "Startup wait ended; A / tap to place";
        }

        if (place) {
            startup_wait = false;
            if (!link.connected) {
                // Do not queue a command that could apply later unexpectedly.
                message = "No link; connect, then A / tap again";
            } else if (driver.placementResult(ticket) == PlacementResult::kPending) {
                message = "Placement already pending";
            } else {
                const auto requested = driver.submitPlacement(robot_config::kStartPose);
                if (requested != 0) {
                    ticket = requested;
                    message = "Hold still; awaiting placement";
                } else {
                    message = "Placement refused; check config";
                }
            }
        }

        const auto placement = driver.placementStatus(ticket);
        const auto sample = driver.latest(ProsDriver::now());
        if (placement.state == PlacementResult::kApplied) {
            message = livePose(sample) ? "Push / rotate robot to check pose"
                                       : "Waiting for fresh pose / link";
        } else if (placement.state == PlacementResult::kTimedOut ||
                   placement.state == PlacementResult::kRejected ||
                   placement.state == PlacementResult::kSessionLost) {
            message = "Check link/sensors; A / tap to place";
        }

        const uint32_t tick = pros::millis();
        if (tick - last_display >= robot_config::kDisplayPeriodMs) {
            display(link, sample, placement, message, open_error);
            if (vex_imu) {
                const auto imu_sample = readBenchImu(*vex_imu);
                if (imu_sample.valid) {
                    row(8, "VEX IMU P%u: %+.2f deg CCW (bench)",
                        unsigned(robot_config::kVexImuPort), imu_sample.rotation_mdeg / 1000.0);
                } else {
                    row(8, "VEX IMU P%u: calibrating / unavailable",
                        unsigned(robot_config::kVexImuPort));
                }
            }
            last_display = tick;
        }
        if (tick - last_log >= robot_config::kLogPeriodMs) {
            log(link, sample, placement);
            if (vex_imu) {
                const auto imu_sample = readBenchImu(*vex_imu);
                std::printf("[VEX IMU bench] port=%u valid=%d ccw_mdeg=%ld\n",
                            unsigned(robot_config::kVexImuPort), imu_sample.valid,
                            static_cast<long>(imu_sample.rotation_mdeg));
            }
            last_log = tick;
        }
        pros::Task::delay_until(&wake, robot_config::kLoopPeriodMs);
    }
}

} // namespace

void initialize() {
    // This task keeps the display and link alive in every competition mode.
    static pros::Task monitor(runTest, "localization-test");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
    while (true) {
        pros::delay(20);
    }
}
