// kinematics.h
// Chassis velocity to driven wheel surface speed and back. Wheel speeds are
// m/s at the tread, positive driving the robot forward. Tank groups are
// left, right; mecanum groups are front left, front right, rear left, rear
// right, with the rollers forming an X seen from above.

#pragma once
#include <cstddef>

#include "actugatr/chassis.h"

namespace actugatr
{

constexpr std::size_t kMaxWheelGroups = 4;

struct WheelSpeeds {
    std::size_t     count = 0;
    MetersPerSecond speed[kMaxWheelGroups] = {};
};

class Kinematics {
public:
    virtual ~Kinematics() = default;

    virtual bool        holonomic() const = 0;
    virtual std::size_t groups() const    = 0;

    // False when the body command cannot be driven, e.g. a tank asked to
    // move sideways. out is left unchanged then.
    virtual bool toWheels(const ChassisCommand& body, WheelSpeeds& out) const = 0;

    virtual ChassisCommand toChassis(const WheelSpeeds& wheels) const = 0;
};

// track_width: left to right driven wheel contact spacing.
class TankKinematics : public Kinematics {
public:
    explicit TankKinematics(Meters track_width);

    bool           holonomic() const override { return false; }
    std::size_t    groups() const override { return 2; }
    bool           toWheels(const ChassisCommand& body, WheelSpeeds& out) const override;
    ChassisCommand toChassis(const WheelSpeeds& wheels) const override;

    Meters trackWidth() const { return track_width_; }

private:
    Meters track_width_;
};

// track_width: left to right, wheelbase: front to rear, wheel contact spacing.
class MecanumKinematics : public Kinematics {
public:
    MecanumKinematics(Meters track_width, Meters wheelbase);

    bool           holonomic() const override { return true; }
    std::size_t    groups() const override { return 4; }
    bool           toWheels(const ChassisCommand& body, WheelSpeeds& out) const override;
    ChassisCommand toChassis(const WheelSpeeds& wheels) const override;

private:
    Meters lever_; // half track plus half wheelbase
};

// Scales every wheel by one factor so none exceeds max_speed, keeping the
// chassis direction and curvature. Returns the factor, at most 1.
double desaturate(WheelSpeeds& wheels, MetersPerSecond max_speed);

} // namespace actugatr
