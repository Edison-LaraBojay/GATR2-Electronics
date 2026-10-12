#include "mecanum.h"

#include <algorithm>
#include <cstdlib>

namespace drive
{

WheelCommands mix(int forward, int right, int clockwise, int deadband, int max_voltage) {
    deadband    = std::clamp(deadband, 0, 127);
    max_voltage = std::clamp(max_voltage, 0, 12000);
    const auto axis = [deadband](int value) {
        value = std::clamp(value, -127, 127);
        return std::abs(value) <= deadband ? 0 : value;
    };
    forward   = axis(forward);
    right     = axis(right);
    clockwise = axis(clockwise);

    WheelCommands wheels;
    wheels.front_left  = forward + right + clockwise;
    wheels.front_right = forward - right - clockwise;
    wheels.back_left   = forward - right + clockwise;
    wheels.back_right  = forward + right - clockwise;

    const int scale = std::max({127, std::abs(wheels.front_left), std::abs(wheels.front_right),
                                std::abs(wheels.back_left), std::abs(wheels.back_right)});
    wheels.front_left  = wheels.front_left * max_voltage / scale;
    wheels.front_right = wheels.front_right * max_voltage / scale;
    wheels.back_left   = wheels.back_left * max_voltage / scale;
    wheels.back_right  = wheels.back_right * max_voltage / scale;
    return wheels;
}

} // namespace drive
