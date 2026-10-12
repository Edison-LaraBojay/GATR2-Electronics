#include "drive.h"

#include <array>
#include <optional>
#include <vector>

#include "drive_config.h"
#include "mecanum.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

namespace drive
{
namespace
{

std::array<std::optional<pros::MotorGroup>, 4> wheels;
pros::Controller controller(pros::E_CONTROLLER_MASTER);
bool configured = false;
bool armed = false;
bool motor_fault = false;
const char* message = "Drive: fill motor ports in drive_config.h";

void zeroMotors() {
    for (const auto& wheel : wheels) {
        if (wheel) {
            wheel->move_voltage(0);
        }
    }
}

bool validAxis(int value) { return value >= -127 && value <= 127; }

} // namespace

bool initialize(int imu_port) {
    if (configured) {
        stop();
        return true;
    }
    if (!drive_config::validPorts(drive_config::kMotorPorts, imu_port)) {
        message = "Drive disabled: check ports in drive_config.h";
        return false;
    }
    constexpr auto gears = drive_config::kMotorRpm == 600 ? pros::MotorGears::blue
                         : drive_config::kMotorRpm == 200 ? pros::MotorGears::green
                                                         : pros::MotorGears::red;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const auto& ports = drive_config::kMotorPorts[i];
        wheels[i].emplace(std::vector<std::int8_t>(ports.begin(), ports.end()), gears);
        wheels[i]->set_brake_mode_all(pros::MotorBrake::coast);
    }
    configured = true;
    stop();
    return true;
}

void stop() {
    zeroMotors();
    armed = false;
    if (configured && !motor_fault) {
        message = "Drive stopped: center sticks to resume";
    }
}

void update() {
    if (!configured || motor_fault) {
        return;
    }
    if (controller.is_connected() != 1) {
        stop();
        message = "Drive stopped: controller disconnected";
        return;
    }
    const int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int right = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    const int clockwise = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    if (!validAxis(forward) || !validAxis(right) || !validAxis(clockwise)) {
        stop();
        message = "Drive stopped: controller read failed";
        return;
    }
    const WheelCommands command = mix(forward, right, clockwise, drive_config::kDeadband,
                                      drive_config::kMaxVoltage);
    if (!armed) {
        if (forward < -drive_config::kDeadband || forward > drive_config::kDeadband ||
            right < -drive_config::kDeadband || right > drive_config::kDeadband ||
            clockwise < -drive_config::kDeadband || clockwise > drive_config::kDeadband) {
            zeroMotors();
            message = "Drive stopped: center sticks to resume";
            return;
        }
        armed = true;
    }
    const std::array<int, 4> volts = {command.front_left, command.front_right,
                                     command.back_left, command.back_right};
    bool success = true;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        if (wheels[i]->move_voltage(volts[i]) != 1) {
            success = false;
        }
    }
    if (!success) {
        motor_fault = true;
        stop();
        message = "Drive fault: check motors, restart Brain app";
        return;
    }
    message = "Drive: left=translate right=rotate";
}

const char* status() { return message; }

} // namespace drive
