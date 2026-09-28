#include "gravity_alignment.h"

#include <cmath>

namespace bno08x {
namespace {

constexpr uint32_t kWindowUs = 2000000;
constexpr uint32_t kMinSamples = 200;
constexpr uint32_t kMaxGapUs = 50000;
constexpr double kGravity = 9.80665;
constexpr double kMagnitudeTolerance = 0.5;
constexpr double kVectorTolerance = 0.25;
constexpr double kMaxStationaryGyro = 0.10;

bool finite(Vector3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

double norm(Vector3 v) { return std::hypot(v.x, v.y, v.z); }

bool closeTimes(uint32_t a, uint32_t b) {
    const int32_t delta = static_cast<int32_t>(a - b);
    return delta >= -static_cast<int32_t>(kMaxGapUs) &&
           delta <= static_cast<int32_t>(kMaxGapUs);
}

} // namespace

void GravityAlignment::reset() { *this = GravityAlignment{}; }

void GravityAlignment::clearPartial() {
    if (ready_) return;
    samples_ = 0;
    first_us_ = 0;
    reference_ = {};
    sum_ = {};
}

bool GravityAlignment::acceptTimestamp(uint32_t sample_us, bool& seen,
                                       uint32_t& previous) {
    if (seen) {
        // Unsigned subtraction handles wrap; signed ordering is valid for
        // successive reports separated by less than half the counter range.
        const int32_t delta = static_cast<int32_t>(sample_us - previous);
        if (delta <= 0) {
            if (delta < 0) clearPartial();
            return false;
        }
        if (static_cast<uint32_t>(delta) > kMaxGapUs) clearPartial();
    }
    seen = true;
    previous = sample_us;
    return true;
}

void GravityAlignment::addAcceleration(uint32_t sample_us, Vector3 accel) {
    if (!acceptTimestamp(sample_us, has_accel_time_, last_accel_us_)) return;
    if (ready_) return;

    if (!finite(accel) || std::abs(norm(accel) - kGravity) > kMagnitudeTolerance ||
        !has_gyro_ || !closeTimes(sample_us, last_gyro_us_) ||
        norm(gyro_) > kMaxStationaryGyro) {
        clearPartial();
        return;
    }

    if (samples_ > 0 && norm({accel.x - reference_.x, accel.y - reference_.y,
                              accel.z - reference_.z}) > kVectorTolerance) {
        clearPartial();
    }
    if (samples_ == 0) {
        first_us_ = sample_us;
        reference_ = accel;
    }
    sum_.x += accel.x;
    sum_.y += accel.y;
    sum_.z += accel.z;
    ++samples_;
    if (samples_ >= kMinSamples && sample_us - first_us_ >= kWindowUs) {
        const double magnitude = norm(sum_);
        up_ = {sum_.x / magnitude, sum_.y / magnitude, sum_.z / magnitude};
        ready_ = true;
    }
}

bool GravityAlignment::projectGyro(uint32_t sample_us, Vector3 gyro, double& yaw) {
    if (!acceptTimestamp(sample_us, has_gyro_time_, last_gyro_us_)) return false;
    has_gyro_ = finite(gyro);
    if (!has_gyro_) {
        clearPartial();
        return false;
    }
    gyro_ = gyro;
    if (!ready_) {
        if (norm(gyro) > kMaxStationaryGyro ||
            (has_accel_time_ && !closeTimes(sample_us, last_accel_us_))) {
            clearPartial();
        }
        return false;
    }
    const double projected = gyro.x * up_.x + gyro.y * up_.y + gyro.z * up_.z;
    if (!std::isfinite(projected)) return false;
    yaw = projected;
    return true;
}

} // namespace bno08x
