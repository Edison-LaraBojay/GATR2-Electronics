// stationary_precheck.cpp

#include "runtime/stationary_precheck.h"

#include <algorithm>
#include <cmath>

#include "payloads/sensor_samples.h"

namespace navigatr
{

void StationaryPrecheck::configure(const std::vector<Wheel>& wheels, const SensorId& imu) {
    sources_.clear();
    for (const Wheel& w : wheels) {
        Source s;
        s.sensor   = w.sensor;
        s.radius_m = w.radius_m;
        sources_.push_back(s);
    }
    if (!imu.empty()) {
        Source s;
        s.sensor = imu;
        s.gyro   = true;
        sources_.push_back(s);
    }
}

void StationaryPrecheck::update(const SensorMap& sensors, MonotonicTime now) {
    for (Source& s : sources_) {
        const auto it = sensors.find(s.sensor);
        if (it == sensors.end() || it->second.state != SourceState::kValid ||
            !it->second.latest.has_value()) {
            continue;
        }
        const StoredSample& stored = *it->second.latest;
        if (s.seen && stored.sequence == s.sequence && stored.epoch == s.epoch) {
            continue;   // a retained record is not new evidence
        }
        s.seen     = true;
        s.sequence = stored.sequence;
        s.epoch    = stored.epoch;
        if (stored.receivedAt.domain != ClockDomain::kHost) {
            continue;
        }
        Sample sample;
        sample.at_ms = stored.receivedAt.ms;
        if (s.gyro) {
            const ImuSample* imu = stored.payload.get<ImuSample>();
            if (imu == nullptr) {
                continue;
            }
            sample.value         = imu->yaw_rate_rad_s;
            sample.discontinuity = stored.epoch;
        } else {
            const EncoderSample* enc = stored.payload.get<EncoderSample>();
            if (enc == nullptr) {
                continue;
            }
            sample.value         = enc->angle_rad * s.radius_m;
            sample.discontinuity = enc->discontinuity_epoch + (stored.epoch << 32);
        }
        s.samples.push_back(sample);
    }
    // keep the window plus the newest sample before it as its baseline
    const int64_t start = now.ms - kWindowMs;
    for (Source& s : sources_) {
        while (s.samples.size() > 1 && s.samples[1].at_ms <= start) {
            s.samples.pop_front();
        }
    }
}

bool StationaryPrecheck::still(MonotonicTime now, std::string* why) const {
    const auto fail = [&](const Source& s, const char* what) {
        if (why != nullptr) {
            *why = s.sensor.value + ": " + what;
        }
        return false;
    };
    const int64_t start = now.ms - kWindowMs;
    for (const Source& s : sources_) {
        if (s.samples.empty() || now.ms - s.samples.back().at_ms > kFreshMs) {
            return fail(s, "no fresh samples");
        }
        if (s.samples.front().at_ms > start) {
            return fail(s, "samples do not cover the window");
        }
        double lo = s.samples.front().value;
        double hi = lo;
        for (std::size_t i = 0; i < s.samples.size(); ++i) {
            const Sample& a = s.samples[i];
            if (i > 0) {
                const Sample& prev = s.samples[i - 1];
                if (a.at_ms - prev.at_ms > kMaxGapMs) {
                    return fail(s, "gap in the window");
                }
                if (a.discontinuity != prev.discontinuity) {
                    return fail(s, "discontinuity in the window");
                }
            }
            lo = std::min(lo, a.value);
            hi = std::max(hi, a.value);
            if (s.gyro && std::fabs(a.value) >= kMaxRateRadS) {
                return fail(s, "turning");
            }
        }
        if (!s.gyro && hi - lo >= kMaxTravelM) {
            return fail(s, "moving");
        }
    }
    return true;
}

} // namespace navigatr
