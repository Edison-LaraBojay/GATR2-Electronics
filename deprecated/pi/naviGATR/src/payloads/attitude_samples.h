// payloads/attitude_samples.h
// Attitude sensor publication: the body's rotation relative to a named
// reference, already rotated into the robot body frame by the producing
// sensor's mounting calibration. Only tilt is trustworthy from a
// gravity-referenced source; has_yaw says whether the yaw component means
// anything at all.

#pragma once
#include <cstdint>
#include <string>

#include "math/quaternion.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kAttitudeSample = "sensor.attitude_sample";
} // namespace payload_names

struct AttitudeSample {
    Quaternion  q_reference_body;
    std::string reference = "gravity";
    bool        has_yaw   = false;
    double      quality   = 0.0;    // 0..1, producer defined
    uint64_t    epoch     = 0;      // source discontinuity generation
};

} // namespace navigatr
