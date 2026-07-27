// payloads/preprocessing_products.h
// Artifact payloads produced by preprocessing, SI units.

#pragma once

namespace navigatr
{

namespace payload_names
{
constexpr const char* kPlanarMotionDelta = "preprocessing.planar_motion_delta";
constexpr const char* kImuDelta          = "preprocessing.imu_delta";
} // namespace payload_names

// Body-frame rigid motion since the previous solution.
struct PlanarMotionDelta {
    double dx_m       = 0.0;
    double dy_m       = 0.0;
    double dtheta_rad = 0.0;
    double dt_s       = 0.0;
};

// Bias-corrected heading step.
struct ImuDelta {
    double delta_rad  = 0.0;
    double rate_rad_s = 0.0;
    double dt_s       = 0.0;
};

} // namespace navigatr
