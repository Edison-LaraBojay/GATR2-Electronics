// Helpers shared by live capture and host regression tests. These operate
// on actual frame metadata and never substitute processing time for a
// reliable exposure stamp.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <vector>

#include "core/time.h"

namespace navigatr
{

struct CaptureClockSample {
    int64_t       source_now_ns = 0;
    MonotonicTime host_before;
    MonotonicTime host_after;
    bool          valid = false;
};

struct CaptureTimestamp {
    MonotonicTime at;
    int64_t       uncertainty_ms = 0;
};

// Sampling the foreign clock between two host readings avoids assuming
// that the clocks share an epoch. Re-sample per completion so suspension
// before capture cannot introduce a permanent offset.
inline std::optional<CaptureTimestamp> correlateCaptureTimestamp(
    int64_t source_ns, const CaptureClockSample& sample) {
    if (!sample.valid || source_ns < 0 || source_ns > sample.source_now_ns ||
        sample.host_before.domain != ClockDomain::kHost ||
        sample.host_after.domain != ClockDomain::kHost ||
        sample.host_after.ms < sample.host_before.ms) {
        return std::nullopt;
    }
    const int64_t span = sample.host_after.ms - sample.host_before.ms;
    CaptureTimestamp out;
    out.at = hostTime(sample.host_before.ms + span / 2 +
                      (source_ns - sample.source_now_ns) / 1000000);
    // Both sampled host times and the nanosecond delta are quantized to ms.
    out.uncertainty_ms = (span + 1) / 2 + 2;
    return out;
}

struct CaptureExposure {
    MonotonicTime at;
    int64_t       uncertainty_ms = 0;
    bool          reliable = false;
};

inline CaptureExposure estimateCaptureExposure(
    const std::optional<CaptureTimestamp>& sensor_stamp,
    std::optional<int32_t> exposure_us, int64_t frame_duration_us,
    MonotonicTime received_at) {
    CaptureExposure out;
    out.at = received_at;
    if (frame_duration_us <= 0) {
        return out;
    }
    int64_t duration_us = frame_duration_us;
    if (exposure_us.has_value() && *exposure_us > 0) {
        duration_us = std::max(duration_us, static_cast<int64_t>(*exposure_us));
    }
    out.uncertainty_ms = duration_us / 1000 + (duration_us % 1000 != 0);
    if (!sensor_stamp.has_value() ||
        sensor_stamp->at.domain != ClockDomain::kHost ||
        received_at.domain != ClockDomain::kHost ||
        (exposure_us.has_value() && *exposure_us <= 0)) {
        return out;
    }

    const int64_t half_exposure_ms = exposure_us.has_value() ? *exposure_us / 2000 : 0;
    const MonotonicTime midpoint = hostTime(sensor_stamp->at.ms + half_exposure_ms);
    // Impossible future capture metadata must not become a valid pose query.
    if (midpoint.ms > received_at.ms + sensor_stamp->uncertainty_ms) {
        return out;
    }
    out.at = midpoint;
    out.uncertainty_ms += sensor_stamp->uncertainty_ms + 1;
    out.reliable = true;
    // SensorTimestamp is defined as first-row exposure. Raspberry Pi
    // documentation also describes readout-start timing. A full observed
    // frame/exposure duration covers that distinction and unknown rolling
    // readout; half a frame does not. This is not a hardware timing calibration.
    return out;
}

// available_bytes is the completed plane's bytes-used, not its allocation
// capacity. Padding is skipped and every published pixel is owned by out.
inline bool copyY8Plane(const uint8_t* source, std::size_t available_bytes,
                       std::size_t width, std::size_t height, std::size_t stride,
                       std::vector<uint8_t>& out) {
    if (source == nullptr || width == 0 || height == 0 || stride < width ||
        height > std::numeric_limits<std::size_t>::max() / stride) {
        return false;
    }
    const std::size_t required = (height - 1) * stride + width;
    if (required > available_bytes) {
        return false;
    }
    out.resize(width * height);
    for (std::size_t row = 0; row < height; ++row) {
        std::memcpy(out.data() + row * width, source + row * stride, width);
    }
    return true;
}

} // namespace navigatr
