// wheel_calibration.cpp

#include "communigatr/wheel_calibration.h"

#include <algorithm>
#include <cmath>

namespace communigatr
{

WheelSample fromReading(const gatr2::WheelReading& r) {
    WheelSample s;
    s.port          = r.port;
    s.fresh         = (r.flags & gatr2::kWheelFresh) != 0;
    s.valid         = (r.flags & gatr2::kWheelValid) != 0;
    s.discontinuity = r.discontinuity;
    s.counts        = r.counts;
    s.travel        = static_cast<double>(r.travel_um) * 1e-6;
    s.age           = static_cast<double>(r.age_ms) * 1e-3;
    return s;
}

const char* toString(TrialStatus status) {
    switch (status) {
    case TrialStatus::kAccepted: return "accepted";
    case TrialStatus::kNotStarted: return "not started";
    case TrialStatus::kReadingMissing: return "wheel reading missing";
    case TrialStatus::kReadingStale: return "reading stale";
    case TrialStatus::kReadingInvalid: return "encoder has no value";
    case TrialStatus::kDiscontinuity: return "encoder restarted";
    case TrialStatus::kProfileChanged: return "profile changed";
    case TrialStatus::kReferenceInvalid: return "reference distance invalid";
    case TrialStatus::kTooShort: return "wheel barely moved";
    case TrialStatus::kWrongDirection: return "wrong direction or polarity";
    case TrialStatus::kRotated: return "robot rotated";
    case TrialStatus::kNotStraight: return "push not along the wheel";
    case TrialStatus::kOutOfRange: return "scale out of range: check radius, CPR, gearing";
    }
    return "?";
}

WheelCalibration::WheelCalibration(const TrackingWheel& wheel, std::vector<uint8_t> cross_ports,
                                   const WheelCalibrationConfig& config)
    : port_(wheel.encoder_port),
      lever_(wheel.x * std::sin(wheel.angle) - wheel.y * std::cos(wheel.angle)),
      cross_(std::move(cross_ports)), config_(config) {}

const WheelSample* WheelCalibration::find(const CalibrationSnapshot& snapshot, uint8_t port) const {
    for (const WheelSample& w : snapshot.wheels) {
        if (w.port == port) {
            return &w;
        }
    }
    return nullptr;
}

TrialStatus WheelCalibration::check(const WheelSample* sample) const {
    if (sample == nullptr) {
        return TrialStatus::kReadingMissing;
    }
    if (!sample->valid) {
        return TrialStatus::kReadingInvalid;
    }
    if (!sample->fresh || !(sample->age <= config_.max_age)) {
        return TrialStatus::kReadingStale;
    }
    return TrialStatus::kAccepted;
}

TrialStatus WheelCalibration::start(const CalibrationSnapshot& snapshot) {
    started_ = false;
    TrialStatus status = check(find(snapshot, port_));
    for (std::size_t i = 0; status == TrialStatus::kAccepted && i < cross_.size(); ++i) {
        status = check(find(snapshot, cross_[i]));
    }
    if (status != TrialStatus::kAccepted) {
        return status;
    }
    if (!snapshot.heading_valid || !std::isfinite(snapshot.heading)) {
        return TrialStatus::kRotated;
    }
    start_   = snapshot;
    started_ = true;
    return TrialStatus::kAccepted;
}

TrialStatus WheelCalibration::finish(const CalibrationSnapshot& snapshot, Meters reference) {
    if (!started_) {
        return TrialStatus::kNotStarted;
    }
    started_ = false;

    const WheelSample* a = find(start_, port_);
    const WheelSample* b = find(snapshot, port_);
    TrialStatus        status = check(b);
    if (status != TrialStatus::kAccepted) {
        return status;
    }
    if (snapshot.profile_id != start_.profile_id) {
        return TrialStatus::kProfileChanged;
    }
    if (a->discontinuity != b->discontinuity) {
        return TrialStatus::kDiscontinuity;
    }
    if (!std::isfinite(reference) || reference < config_.min_reference ||
        reference > config_.max_reference) {
        return TrialStatus::kReferenceInvalid;
    }
    if (!snapshot.heading_valid || !std::isfinite(snapshot.heading)) {
        return TrialStatus::kRotated;
    }
    const Radians rotation = investigatr::wrapAngle(snapshot.heading - start_.heading);
    if (std::fabs(rotation) > config_.max_rotation) {
        return TrialStatus::kRotated;
    }
    for (uint8_t cross : cross_) {
        const WheelSample* ca = find(start_, cross);
        const WheelSample* cb = find(snapshot, cross);
        status                = check(cb);
        if (status != TrialStatus::kAccepted) {
            return status;
        }
        if (ca->discontinuity != cb->discontinuity) {
            return TrialStatus::kDiscontinuity;
        }
        if (std::fabs(cb->travel - ca->travel) > config_.max_cross * reference) {
            return TrialStatus::kNotStraight;
        }
    }

    const Meters measured = b->travel - a->travel - lever_ * rotation;
    if (measured < 0) {
        return TrialStatus::kWrongDirection;
    }
    if (measured < 0.5 * reference * (1.0 - config_.max_correction)) {
        return TrialStatus::kTooShort;
    }
    const double scale = reference / measured;
    if (std::fabs(scale - 1.0) > config_.max_correction) {
        return TrialStatus::kOutOfRange;
    }
    trials_.push_back(WheelTrial{reference, measured, scale});
    return TrialStatus::kAccepted;
}

void WheelCalibration::clear() {
    trials_.clear();
    started_ = false;
}

CalibrationProposal WheelCalibration::proposal() const {
    CalibrationProposal out;
    out.trials = trials_.size();
    if (trials_.empty()) {
        return out;
    }
    double sum = 0;
    double lo  = trials_.front().scale;
    double hi  = lo;
    for (const WheelTrial& t : trials_) {
        sum += t.scale;
        lo = std::min(lo, t.scale);
        hi = std::max(hi, t.scale);
    }
    out.scale  = sum / static_cast<double>(trials_.size());
    out.spread = (hi - lo) / out.scale;
    out.ready  = static_cast<int>(trials_.size()) >= config_.min_trials &&
                out.spread <= config_.max_spread;
    return out;
}

} // namespace communigatr
