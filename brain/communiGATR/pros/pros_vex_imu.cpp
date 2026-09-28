// pros_vex_imu.cpp
// PROS only.

#include "communigatr/pros_vex_imu.h"

#include <cmath>
#include <limits>

#include "pros/rtos.hpp"

namespace communigatr
{

ProsVexImu::ProsVexImu(uint8_t smart_port) : port_(smart_port), imu_(smart_port) {}

BenchImuSample ProsVexImu::sample() const {
    BenchImuSample s;
    s.stamp_ms = pros::millis();
    if (imu_.get_status() == pros::ImuStatus::error) {
        return s; // missing or failed: invalid, not calibrating
    }
    if (imu_.is_calibrating()) {
        s.calibrating = true;
        return s;
    }
    // PROS rotation is clockwise positive degrees; the link uses CCW.
    const double mdeg = -imu_.get_rotation() * 1000.0;
    if (!std::isfinite(mdeg) || mdeg < std::numeric_limits<int32_t>::min() ||
        mdeg > std::numeric_limits<int32_t>::max()) {
        return s;
    }
    s.rotation_mdeg = static_cast<int32_t>(std::lround(mdeg));
    s.valid         = true;
    return s;
}

bool ProsVexImu::recalibrate() {
    return imu_.reset(false) == 1;
}

bool ProsVexImu::calibrating() const {
    return imu_.get_status() != pros::ImuStatus::error && imu_.is_calibrating();
}

} // namespace communigatr
