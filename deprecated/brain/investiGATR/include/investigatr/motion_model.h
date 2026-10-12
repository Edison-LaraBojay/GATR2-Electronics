// motion_model.h
// What planning may ask of the robot. Drivetrain adapters build a model;
// planning never looks at drivetrain names.

#pragma once

#include "investigatr/geometry.h"

namespace investigatr
{

// Distance from the robot origin to each side of the enclosing rectangle.
struct Footprint {
    Meters front = 0;
    Meters back  = 0;
    Meters left  = 0;
    Meters right = 0;
};

struct MotionLimits {
    MetersPerSecond  max_speed = 0.5; // translation
    double           max_accel = 1.0; // m/s^2
    RadiansPerSecond max_omega = 2.0;
    double           max_alpha = 6.0; // rad/s^2
};

struct MotionModel {
    bool holonomic     = false; // translation direction independent of heading
    bool turn_in_place = true;  // rotates about the origin without translating
    bool reverse       = true;  // non-holonomic: may drive backward along its heading

    // Steering limit for car-like or module-steered drives. 0 = none. No
    // drivetrain sets it yet and planners refuse a nonzero value.
    Meters min_turn_radius = 0;

    Footprint    footprint;
    Meters       clearance = 0.05; // added around the footprint when planning
    MotionLimits limits;
};

// Farthest footprint corner from the origin plus clearance. A circle of this
// radius about the origin holds the footprint at every heading.
Meters enclosingRadius(const MotionModel& model);

bool valid(const MotionModel& model, const char** why = nullptr);

} // namespace investigatr
