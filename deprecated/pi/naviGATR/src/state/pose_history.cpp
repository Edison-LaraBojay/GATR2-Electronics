// pose_history.cpp

#include "state/pose_history.h"

#include <algorithm>

#include "math/angles.h"

namespace navigatr
{

PoseHistory::PoseHistory(PoseHistoryConfig config) : config_(config) {
    if (config_.capacity < 2) {
        config_.capacity = 2;
    }
    ring_.resize(config_.capacity);
    attitude_ring_.resize(config_.capacity);
}

void PoseHistory::clear() {
    head_  = 0;
    count_ = 0;
    attitude_head_ = 0;
    attitude_count_ = 0;
    have_epoch_ = false;
}

bool PoseHistory::selectEpoch(uint64_t epoch) {
    const bool changed = have_epoch_ && epoch != epoch_;
    if (changed) clear();
    epoch_ = epoch;
    have_epoch_ = true;
    return changed;
}

AppendResult PoseHistory::append(const PoseHistoryEntry& entry) {
    if (entry.at.domain != ClockDomain::kHost) {
        return AppendResult::kUnset;
    }
    AppendResult result = selectEpoch(entry.odometry_epoch)
                              ? AppendResult::kEpochChange : AppendResult::kAppended;
    if (count_ > 0) {
        const PoseHistoryEntry& last = newest();
        if (entry.odometry_epoch != last.odometry_epoch) {
            clear();
            result = AppendResult::kEpochChange;
        } else if (entry.at.ms == last.at.ms) {
            ring_[(head_ + count_ - 1) % ring_.size()] = entry;
            appendAttitude(entry.attitude, entry.odometry_epoch);
            return AppendResult::kRevised;
        } else if (entry.at.ms < last.at.ms) {
            return AppendResult::kOutOfOrder;
        }
    }
    if (count_ == ring_.size()) {
        ring_[head_] = entry;   // overwrite the oldest
        head_        = (head_ + 1) % ring_.size();
    } else {
        ring_[(head_ + count_) % ring_.size()] = entry;
        ++count_;
    }
    // expire by time as well as capacity
    while (count_ > 1 && (entry.at.ms - oldest().at.ms) > config_.retention_ms) {
        head_ = (head_ + 1) % ring_.size();
        --count_;
    }
    appendAttitude(entry.attitude, entry.odometry_epoch);
    return result;
}

AppendResult PoseHistory::appendAttitude(const Attitude& attitude, uint64_t odometry_epoch) {
    if (!attitude.valid || attitude.measuredAt.domain != ClockDomain::kHost ||
        !isUnit(attitude.q_reference_body, 1e-3)) {
        return AppendResult::kUnset;
    }
    const bool changed = selectEpoch(odometry_epoch);
    Attitude measured = attitude;
    double yaw = 0.0;
    Quaternion tilt;
    splitYawAndTilt(attitude.q_reference_body, yaw, tilt);
    measured.q_reference_body = tilt;
    measured.reference = "gravity";
    if (attitude_count_ > 0) {
        const Attitude& latest = attitudeEntry(attitude_count_ - 1);
        if (attitude.measuredAt.ms < latest.measuredAt.ms) return AppendResult::kOutOfOrder;
        if (attitude.measuredAt.ms == latest.measuredAt.ms) {
            // A same-time source/calibration revision replaces its payload
            // without advancing time. Stripping yaw above also makes a
            // retained tilt immune to its carrying pose's newer heading.
            attitude_ring_[(attitude_head_ + attitude_count_ - 1) %
                           attitude_ring_.size()] = std::move(measured);
            return AppendResult::kRevised;
        }
    }
    if (attitude_count_ == attitude_ring_.size()) {
        attitude_ring_[attitude_head_] = std::move(measured);
        attitude_head_ = (attitude_head_ + 1) % attitude_ring_.size();
    } else {
        attitude_ring_[(attitude_head_ + attitude_count_) % attitude_ring_.size()] =
            std::move(measured);
        ++attitude_count_;
    }
    while (attitude_count_ > 1 && attitude.measuredAt.ms -
               attitudeEntry(0).measuredAt.ms > config_.retention_ms) {
        attitude_head_ = (attitude_head_ + 1) % attitude_ring_.size();
        --attitude_count_;
    }
    return changed ? AppendResult::kEpochChange : AppendResult::kAppended;
}

std::size_t PoseHistory::lowerBound(MonotonicTime t) const {
    std::size_t lo = 0;
    std::size_t hi = count_;
    while (lo < hi) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (entry(mid).at.ms < t.ms) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

PoseLookupResult PoseHistory::poseAt(MonotonicTime t) const {
    PoseLookupResult out;
    if (t.domain != ClockDomain::kHost) {
        out.status = LookupStatus::kUnset;
        return out;
    }
    if (count_ == 0) {
        out.status = LookupStatus::kEmpty;
        return out;
    }
    out.odometry_epoch = newest().odometry_epoch;
    if (t.ms < oldest().at.ms) {
        out.status = LookupStatus::kExpired;
        return out;
    }
    if (t.ms > newest().at.ms) {
        out.status = LookupStatus::kPending;
        return out;
    }
    const std::size_t hi = lowerBound(t);
    const PoseHistoryEntry& b = entry(hi);
    if (b.at.ms == t.ms) {
        out.status    = LookupStatus::kOk;
        out.odom_pose = b.odom_pose;
        out.exact     = true;
        return out;
    }
    const PoseHistoryEntry& a = entry(hi - 1);
    if ((b.at.ms - a.at.ms) > config_.max_interpolation_gap_ms) {
        out.status = LookupStatus::kGap;
        return out;
    }
    const double f =
        static_cast<double>(t.ms - a.at.ms) / static_cast<double>(b.at.ms - a.at.ms);
    out.odom_pose.x_m = a.odom_pose.x_m + (b.odom_pose.x_m - a.odom_pose.x_m) * f;
    out.odom_pose.y_m = a.odom_pose.y_m + (b.odom_pose.y_m - a.odom_pose.y_m) * f;
    out.odom_pose.heading_rad =
        wrapAngle(a.odom_pose.heading_rad +
                  wrapAngle(b.odom_pose.heading_rad - a.odom_pose.heading_rad) * f);
    out.status = LookupStatus::kOk;
    return out;
}

RateLookupResult PoseHistory::yawRateAt(MonotonicTime t) const {
    RateLookupResult out;
    if (t.domain != ClockDomain::kHost) {
        out.status = LookupStatus::kUnset;
        return out;
    }
    if (count_ < 2) {
        out.status = LookupStatus::kEmpty;
        return out;
    }
    out.odometry_epoch = newest().odometry_epoch;
    if (t.ms < oldest().at.ms) {
        out.status = LookupStatus::kExpired;
        return out;
    }
    if (t.ms > newest().at.ms) {
        out.status = LookupStatus::kPending;
        return out;
    }
    std::size_t hi = lowerBound(t);
    if (hi == 0) {
        hi = 1;
    }
    const PoseHistoryEntry& a = entry(hi - 1);
    const PoseHistoryEntry& b = entry(hi);
    const int64_t           dt_ms = b.at.ms - a.at.ms;
    if (dt_ms <= 0 || dt_ms > config_.max_interpolation_gap_ms) {
        out.status = LookupStatus::kGap;
        return out;
    }
    out.yaw_rate_rad_s =
        wrapAngle(b.odom_pose.heading_rad - a.odom_pose.heading_rad) / (dt_ms / 1000.0);
    out.status = LookupStatus::kOk;
    return out;
}

AttitudeLookupResult PoseHistory::attitudeAt(MonotonicTime t) const {
    AttitudeLookupResult out;
    if (t.domain != ClockDomain::kHost) {
        out.status = LookupStatus::kUnset;
        return out;
    }
    if (attitude_count_ == 0) {
        out.status = LookupStatus::kEmpty;
        return out;
    }
    if (t.ms < attitudeEntry(0).measuredAt.ms) {
        out.status = LookupStatus::kExpired;
        return out;
    }
    if (t.ms > attitudeEntry(attitude_count_ - 1).measuredAt.ms) {
        out.status = LookupStatus::kPending;
        return out;
    }
    std::size_t lo = 0, hi = attitude_count_;
    while (lo < hi) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (attitudeEntry(mid).measuredAt.ms < t.ms) lo = mid + 1;
        else hi = mid;
    }
    const Attitude& b = attitudeEntry(hi);
    if (b.measuredAt.ms == t.ms) {
        out.attitude = b;
    } else {
        const Attitude& a = attitudeEntry(hi - 1);
        if (a.epoch != b.epoch || a.source != b.source) {
            out.status = LookupStatus::kUnavailable;
            return out;
        }
        const int64_t span = b.measuredAt.ms - a.measuredAt.ms;
        if (span > config_.attitude_gap_ms) {
            out.status = LookupStatus::kGap;
            return out;
        }
        const double f = static_cast<double>(t.ms - a.measuredAt.ms) / span;
        out.attitude = a;
        out.attitude.q_reference_body = slerp(a.q_reference_body, b.q_reference_body, f);
        out.attitude.measuredAt = t;
        out.attitude.measuredAtSource = MonotonicTime{}; // interpolated, not a source sample
        out.attitude.quality = std::min(a.quality, b.quality);
    }
    // Tilt has its own sample times. Add heading only from a pose valid at
    // this same requested time, never from the pose that happened to carry
    // the attitude through publication.
    double interpolated_yaw = 0.0;
    Quaternion interpolated_tilt;
    splitYawAndTilt(out.attitude.q_reference_body, interpolated_yaw, interpolated_tilt);
    out.attitude.q_reference_body = interpolated_tilt;
    const PoseLookupResult pose = poseAt(t);
    if (pose.status == LookupStatus::kOk) {
        out.attitude.q_reference_body = multiply(yawQuaternion(pose.odom_pose.heading_rad),
                                                 out.attitude.q_reference_body);
        out.attitude.reference = "odometry";
    }
    out.status = LookupStatus::kOk;
    return out;
}

std::vector<PoseHistoryEntry> PoseHistory::recent(std::size_t max_entries) const {
    std::vector<PoseHistoryEntry> out;
    const std::size_t             n = std::min(max_entries, count_);
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back(entry(count_ - 1 - i));
    }
    return out;
}

} // namespace navigatr
