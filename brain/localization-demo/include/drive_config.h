// Motor Smart Ports; rebuild and upload the Brain app after a change.
// Each pair drives one wheel. A negative port reverses that individual motor.
// Configure signs so positive voltage on every pair drives the robot forward.
// Zero means unset: driving stays disabled until all eight ports are valid.

#pragma once

#include <array>
#include <cstdint>

namespace drive_config
{

using MotorPair = std::array<std::int8_t, 2>;
using MotorPorts = std::array<MotorPair, 4>;

//                         Motor 1, Motor 2
// Port assignments supplied by the user; motor reversal signs still need checking.
inline constexpr MotorPair kFrontLeft  = {1, 2};
inline constexpr MotorPair kFrontRight = {3, 4};
inline constexpr MotorPair kBackLeft   = {5, 6};
inline constexpr MotorPair kBackRight  = {7, 8};

inline constexpr MotorPorts kMotorPorts = {kFrontLeft, kFrontRight, kBackLeft, kBackRight};

constexpr int kMotorRpm   = 600;   // blue cartridge, previously specified; 1:1 gearing
constexpr int kDeadband   = 8;     // joystick units, out of 127
constexpr int kMaxVoltage = 12000; // millivolts; lower this for slower manual driving

// Left stick Y: forward/backward. Left stick X: right/left strafing.
// Right stick X: clockwise/counterclockwise rotation. Robot-relative control.
// Assumes the usual X roller layout when viewed from above.

constexpr bool validPorts(const MotorPorts& groups, int imu_port) {
    std::array<bool, 22> used{};
    for (const MotorPair& group : groups) {
        for (const int signed_port : group) {
            const int port = signed_port < 0 ? -signed_port : signed_port;
            if (port < 1 || port > 21 || port == imu_port || used[port]) {
                return false;
            }
            used[port] = true;
        }
    }
    return true;
}

static_assert(kMotorRpm == 100 || kMotorRpm == 200 || kMotorRpm == 600,
              "Choose a 100, 200 or 600 RPM motor cartridge");
static_assert(kDeadband >= 0 && kDeadband < 127, "Joystick deadband must be in [0, 126]");
static_assert(kMaxVoltage > 0 && kMaxVoltage <= 12000, "Motor voltage must be in [1, 12000]");

} // namespace drive_config
