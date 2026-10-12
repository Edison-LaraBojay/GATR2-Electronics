// Portable controller-to-wheel voltage mixer, independent of PROS.

#pragma once

namespace drive
{

struct WheelCommands {
    int front_left  = 0;
    int front_right = 0;
    int back_left   = 0;
    int back_right  = 0;
};

// Inputs are controller axes in [-127, 127]. Positive commands mean forward,
// strafe right, and turn clockwise. Positive wheel voltage drives forward;
// motor mounting reversals belong in the motor configuration.
// Each axis at or inside the deadband is zeroed. Mixed commands are scaled
// together to preserve their ratios and returned in millivolts.
// Inputs, deadband, and max_voltage are clamped to [-127, 127], [0, 127], and
// [0, 12000], respectively. Voltage conversion truncates toward zero.
WheelCommands mix(int forward, int right, int clockwise, int deadband = 8,
                  int max_voltage = 12000);

} // namespace drive
