// payloads/robot_observations.h
// Robot observations: what a measurement model says about the robot's own
// motion or attitude over an explicit time support, with the contributing
// samples named. An observation states only the components it measured;
// a model that cannot observe rotation says so instead of inventing it,
// and an estimator that needs the missing component refuses the step.
//
// Lineage matters: a motion increment that already folded an IMU names
// that IMU in sources, so a heading increment from the same IMU is not an
// independent second observation and the estimator must not count both.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "core/records.h"
#include "core/time.h"
#include "math/quaternion.h"

namespace navigatr
{

namespace payload_names
{
constexpr const char* kBodyMotionIncrement = "localization.body_motion_increment";
constexpr const char* kHeadingIncrement    = "localization.heading_increment";
constexpr const char* kAttitudeObservation = "localization.attitude_observation";
} // namespace payload_names

// Rigid planar motion of the robot origin in its own body frame over
// [startAt, endAt] on the source clock.
struct BodyMotionIncrement {
    double dx_m         = 0.0;
    double dy_m         = 0.0;
    double dtheta_rad   = 0.0;
    bool   has_rotation = true;   // false: rotation not observed by this model

    MonotonicTime startAt;   // source clock
    MonotonicTime endAt;
    double        dt_s = 0.0;

    std::vector<Provenance> sources;   // consumed samples, one per sensor
    double                  quality = 1.0;

    // Geometric coupling of the solved translation to the solved rotation,
    // d(dx, dy)/d(dtheta) in m/rad, fixed by the producing model's
    // geometry. An estimator that revises dtheta with independent evidence
    // re-solves the translation through it. false when no linear relation
    // exists.
    bool   has_rotation_coupling = false;
    double dx_per_dtheta_m_rad   = 0.0;
    double dy_per_dtheta_m_rad   = 0.0;
};

// Heading change over [startAt, endAt] from a rate source with its bias
// removed by the producing model.
struct HeadingIncrement {
    double dtheta_rad = 0.0;
    double rate_rad_s = 0.0;   // bias corrected rate at endAt

    MonotonicTime startAt;
    MonotonicTime endAt;
    double        dt_s = 0.0;

    std::vector<Provenance> sources;
};

// Body attitude relative to a named reference at one instant.
struct AttitudeObservation {
    Quaternion  q_reference_body;
    std::string reference;   // "gravity": yaw meaningless, tilt only
    bool        has_yaw = false;

    MonotonicTime measuredAt;   // source clock
    Provenance    source;
    double        quality = 0.0;
};

} // namespace navigatr
