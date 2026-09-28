// telemetry.cpp

#include "actugatr/telemetry.h"

#include <cmath>
#include <cstdint>

namespace actugatr
{

namespace
{

constexpr double kCdegPerRad = 18000.0 / investigatr::kPi;

// Rounded, saturated to [lo, hi]; non-finite is 0.
long saturate(double value, long lo, long hi) {
    if (!std::isfinite(value)) {
        return 0;
    }
    if (value <= static_cast<double>(lo)) {
        return lo;
    }
    if (value >= static_cast<double>(hi)) {
        return hi;
    }
    return std::lround(value);
}

int16_t i16(double value) {
    return static_cast<int16_t>(saturate(value, INT16_MIN, INT16_MAX));
}

int32_t i32(double value) {
    return static_cast<int32_t>(saturate(value, INT32_MIN, INT32_MAX));
}

uint8_t u8(std::size_t value) {
    return static_cast<uint8_t>(value > UINT8_MAX ? UINT8_MAX : value);
}

// Angle to centidegrees in (-18000, 18000].
int16_t angle(double rad) {
    if (!std::isfinite(rad)) {
        return 0;
    }
    return i16(investigatr::wrapAngle(rad) * kCdegPerRad);
}

} // namespace

translagatr::BrainTelemetry telemetryOf(const DriveSnapshot& snapshot) {
    translagatr::BrainTelemetry t;
    const MotionStatus& m = snapshot.motion;
    const DriveStatus&  d = snapshot.drive;

    t.flags |= translagatr::kTelemetryMotion;
    t.command_id    = m.command_id;
    t.motion_state  = static_cast<uint8_t>(m.state);
    t.motion_reason = static_cast<uint8_t>(m.reason);
    t.plan_mode     = static_cast<uint8_t>(m.mode);
    t.segment       = u8(m.segment);
    t.segment_count = u8(m.segment_count);
    if (m.has_destination) {
        t.target_x_mm         = i32(m.destination.x * 1000.0);
        t.target_y_mm         = i32(m.destination.y * 1000.0);
        t.target_heading_cdeg = angle(m.destination.heading);
    }
    t.cmd_vx_mm_s        = i16(d.command.vx * 1000.0);
    t.cmd_vy_mm_s        = i16(d.command.vy * 1000.0);
    t.cmd_omega_cdeg_s   = i16(d.command.omega * kCdegPerRad);
    t.cross_track_mm     = i16(m.cross_track * 1000.0);
    t.distance_error_mm  = i16(m.distance_error * 1000.0);
    t.heading_error_cdeg = angle(m.heading_error);
    t.drive_fault        = static_cast<uint8_t>(d.fault);

    std::size_t count = d.wheels.count;
    if (count > kMaxWheelGroups) {
        count = kMaxWheelGroups;
    }
    if (count > translagatr::kTelemetryWheelsMax) {
        count = translagatr::kTelemetryWheelsMax;
    }
    if (count > 0) {
        t.flags |= translagatr::kTelemetryWheels;
        t.wheel_count = static_cast<uint8_t>(count);
        for (std::size_t i = 0; i < count; ++i) {
            t.wheel_rpm_x10[i] = i16(d.motor_rpm[i] * 10.0);
        }
    }
    return t;
}

} // namespace actugatr
