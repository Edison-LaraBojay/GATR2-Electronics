// drivetrain.h
// Driven wheel hardware: motors per wheel group with order and reversal,
// cartridge, gearing and wheel size. Tracking wheel geometry is separate and
// belongs to the robot profile.

#pragma once
#include <cstddef>
#include <cstdint>

#include "actugatr/kinematics.h"
#include "investigatr/motion_model.h"

namespace actugatr
{

// Free speed of a V5 motor cartridge, rpm.
enum class Cartridge : uint16_t { kRed = 100, kGreen = 200, kBlue = 600 };

enum class StopMode : uint8_t { kBrake, kCoast, kHold };

// V5 Smart Port 1..21. reversed flips the motor so positive drives forward.
struct MotorPort {
    uint8_t port     = 0;
    bool    reversed = false;
};

constexpr std::size_t kMaxMotorsPerGroup = 4;

struct MotorGroup {
    std::size_t count = 0;
    MotorPort   motors[kMaxMotorsPerGroup];
};

struct WheelDrive {
    Meters    wheel_diameter  = 0;   // driven wheel
    double    gear_ratio      = 1.0; // wheel revolutions per motor revolution
    Cartridge cartridge       = Cartridge::kBlue;
    double    usable_fraction = 0.9; // share of free speed allowed, headroom for the velocity loop
};

MetersPerSecond maxWheelSpeed(const WheelDrive& wheels);
double          motorRpm(const WheelDrive& wheels, MetersPerSecond wheel_speed);
MetersPerSecond wheelSpeed(const WheelDrive& wheels, double motor_rpm);

struct TankConfig {
    MotorGroup left;
    MotorGroup right;
    Meters     track_width = 0;
    WheelDrive wheels;
    StopMode   stop_mode = StopMode::kBrake;
};

struct MecanumConfig {
    MotorGroup front_left;
    MotorGroup front_right;
    MotorGroup rear_left;
    MotorGroup rear_right;
    Meters     track_width = 0;
    Meters     wheelbase   = 0;
    WheelDrive wheels;
    StopMode   stop_mode = StopMode::kBrake;
};

bool valid(const WheelDrive& wheels, const char** why = nullptr);
bool valid(const TankConfig& config, const char** why = nullptr);
bool valid(const MecanumConfig& config, const char** why = nullptr);

// Groups in kinematics order.
std::size_t groups(const TankConfig& config, const MotorGroup* out[kMaxWheelGroups]);
std::size_t groups(const MecanumConfig& config, const MotorGroup* out[kMaxWheelGroups]);

// Motion model of each drivetrain. Speed limits are clamped to what the
// wheels can reach; footprint, clearance and the other limits are the
// application's.
investigatr::MotionModel motionModel(const TankConfig& config,
                                     const investigatr::Footprint& footprint, Meters clearance,
                                     const investigatr::MotionLimits& limits);
investigatr::MotionModel motionModel(const MecanumConfig& config,
                                     const investigatr::Footprint& footprint, Meters clearance,
                                     const investigatr::MotionLimits& limits);

} // namespace actugatr
