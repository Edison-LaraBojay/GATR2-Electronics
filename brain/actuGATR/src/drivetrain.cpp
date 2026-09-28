// drivetrain.cpp

#include "actugatr/drivetrain.h"

#include <algorithm>
#include <cmath>

namespace actugatr
{
namespace
{

bool positive(double v) {
    return std::isfinite(v) && v > 0;
}

bool fail(const char** why, const char* what) {
    if (why != nullptr) {
        *why = what;
    }
    return false;
}

bool ok(const char** why) {
    if (why != nullptr) {
        *why = nullptr;
    }
    return true;
}

bool validGroup(const MotorGroup& group) {
    if (group.count == 0 || group.count > kMaxMotorsPerGroup) {
        return false;
    }
    for (std::size_t i = 0; i < group.count; ++i) {
        if (group.motors[i].port < 1 || group.motors[i].port > 21) {
            return false;
        }
    }
    return true;
}

double cartridgeRpm(Cartridge cartridge) {
    return static_cast<double>(static_cast<uint16_t>(cartridge));
}

investigatr::MotionModel baseModel(const investigatr::Footprint& footprint, Meters clearance,
                                   const investigatr::MotionLimits& limits) {
    investigatr::MotionModel model;
    model.footprint = footprint;
    model.clearance = clearance;
    model.limits    = limits;
    return model;
}

} // namespace

MetersPerSecond maxWheelSpeed(const WheelDrive& w) {
    return cartridgeRpm(w.cartridge) * w.usable_fraction * w.gear_ratio * investigatr::kPi *
           w.wheel_diameter / 60.0;
}

double motorRpm(const WheelDrive& w, MetersPerSecond wheel_speed) {
    const double wheel_rpm = wheel_speed * 60.0 / (investigatr::kPi * w.wheel_diameter);
    return wheel_rpm / w.gear_ratio;
}

MetersPerSecond wheelSpeed(const WheelDrive& w, double motor_rpm) {
    return motor_rpm * w.gear_ratio * investigatr::kPi * w.wheel_diameter / 60.0;
}

bool valid(const WheelDrive& w, const char** why) {
    if (!positive(w.wheel_diameter)) {
        return fail(why, "wheel_diameter must be > 0");
    }
    if (!positive(w.gear_ratio)) {
        return fail(why, "gear_ratio must be > 0");
    }
    const double rpm = cartridgeRpm(w.cartridge);
    if (rpm != 100 && rpm != 200 && rpm != 600) {
        return fail(why, "unknown cartridge");
    }
    if (!positive(w.usable_fraction) || w.usable_fraction > 1) {
        return fail(why, "usable_fraction must be in (0, 1]");
    }
    return ok(why);
}

bool valid(const TankConfig& c, const char** why) {
    if (!validGroup(c.left) || !validGroup(c.right)) {
        return fail(why, "tank motor groups need 1..4 motors on ports 1..21");
    }
    if (!positive(c.track_width)) {
        return fail(why, "track_width must be > 0");
    }
    return valid(c.wheels, why);
}

bool valid(const MecanumConfig& c, const char** why) {
    if (!validGroup(c.front_left) || !validGroup(c.front_right) || !validGroup(c.rear_left) ||
        !validGroup(c.rear_right)) {
        return fail(why, "mecanum motor groups need 1..4 motors on ports 1..21");
    }
    if (!positive(c.track_width) || !positive(c.wheelbase)) {
        return fail(why, "track_width and wheelbase must be > 0");
    }
    return valid(c.wheels, why);
}

std::size_t groups(const TankConfig& c, const MotorGroup* out[kMaxWheelGroups]) {
    out[0] = &c.left;
    out[1] = &c.right;
    return 2;
}

std::size_t groups(const MecanumConfig& c, const MotorGroup* out[kMaxWheelGroups]) {
    out[0] = &c.front_left;
    out[1] = &c.front_right;
    out[2] = &c.rear_left;
    out[3] = &c.rear_right;
    return 4;
}

investigatr::MotionModel motionModel(const TankConfig& c, const investigatr::Footprint& footprint,
                                     Meters clearance, const investigatr::MotionLimits& limits) {
    investigatr::MotionModel model = baseModel(footprint, clearance, limits);
    model.holonomic                = false;
    model.turn_in_place            = true;
    model.reverse                  = true;
    const double wheel             = maxWheelSpeed(c.wheels);
    model.limits.max_speed         = std::min(model.limits.max_speed, wheel);
    if (c.track_width > 0) {
        model.limits.max_omega = std::min(model.limits.max_omega, 2.0 * wheel / c.track_width);
    }
    return model;
}

investigatr::MotionModel motionModel(const MecanumConfig& c,
                                     const investigatr::Footprint& footprint, Meters clearance,
                                     const investigatr::MotionLimits& limits) {
    investigatr::MotionModel model = baseModel(footprint, clearance, limits);
    model.holonomic                = true;
    model.turn_in_place            = true;
    model.reverse                  = true;
    const double wheel             = maxWheelSpeed(c.wheels);
    model.limits.max_speed         = std::min(model.limits.max_speed, wheel);
    const double lever             = 0.5 * (c.track_width + c.wheelbase);
    if (lever > 0) {
        model.limits.max_omega = std::min(model.limits.max_omega, wheel / lever);
    }
    return model;
}

} // namespace actugatr
