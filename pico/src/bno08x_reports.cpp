#include "bno08x_reports.h"

namespace bno08x {
namespace {

int16_t signedLe(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0] | (uint16_t(p[1]) << 8)));
}

bool fresh(uint32_t now, uint32_t stamp) {
    return static_cast<int32_t>(now - stamp) >= 0 &&
           now - stamp <= Bno08xReports::kMaxReportAgeUs;
}

} // namespace

void Bno08xReports::reset() {
    ack_mask_ = 0;
    accel_ = {};
    gyro_ = {};
    alignment_.reset();
    window_.clear(); // the Pico clock and telemetry cuts do not reset with the hub
}

void Bno08xReports::acknowledge(uint8_t report_id, uint32_t interval_us) {
    const uint8_t bit = report_id == kAccelReportId ? 1 : report_id == kGyroReportId ? 2 : 0;
    if (interval_us != 0) {
        ack_mask_ |= bit;
    } else {
        ack_mask_ &= static_cast<uint8_t>(~bit);
    }
}

bool Bno08xReports::add(uint8_t report_id, const uint8_t* report, uint16_t len,
                       uint32_t sample_us, uint32_t received_us) {
    const bool accel = report_id == kAccelReportId;
    if ((!accel && report_id != kGyroReportId) || report == nullptr ||
        len < (accel ? 10 : 16) || report[0] != report_id || !fresh(received_us, sample_us)) {
        return false;
    }
    Stream& stream = accel ? accel_ : gyro_;
    if (stream.seen && static_cast<int32_t>(sample_us - stream.stamp_us) <= 0) {
        return false;
    }
    stream.seen = true;
    stream.stamp_us = sample_us;

    // SH-2 reference manual 6.5.9 / 6.5.14: XYZ at bytes 4..9.
    // Acceleration includes gravity (Q8 m/s^2); gyro is Q9 rad/s.
    const double scale = accel ? 256.0 : 512.0;
    const Vector3 value{signedLe(report + 4) / scale, signedLe(report + 6) / scale,
                        signedLe(report + 8) / scale};
    if (accel) {
        alignment_.addAcceleration(sample_us, value);
    } else {
        double yaw_radps = 0;
        if (alignment_.projectGyro(sample_us, value, yaw_radps)) {
            window_.add(sample_us, static_cast<float>(yaw_radps * 512.0));
        }
    }
    return true;
}

bool Bno08xReports::healthy(uint32_t now_us) const {
    return accel_.seen && gyro_.seen && fresh(now_us, accel_.stamp_us) &&
           fresh(now_us, gyro_.stamp_us);
}

bool Bno08xReports::take(uint32_t cut_us, int32_t& yaw_mdps) {
    if (!acknowledged() || !aligned() || !healthy(cut_us)) {
        window_.clear();
        int32_t unused = 0;
        window_.take(cut_us, unused); // advance the cut without inventing a measurement
        return false;
    }
    return window_.take(cut_us, yaw_mdps);
}

} // namespace bno08x
