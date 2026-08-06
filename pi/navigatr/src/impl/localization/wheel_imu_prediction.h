// wheel_imu_prediction.h
// Integrates the preprocessed body motion delta into the continuous
// odometry-frame robot pose using the constant-curvature chord. A commanded
// pose init (edge triggered by init_sequence) re-anchors the odometry frame
// in the field frame without touching the odometry pose itself. Device time
// running backwards (source reboot) bumps the odometry epoch instead of
// integrating a garbage step. When an Orientation artifact is configured,
// its heading step replaces the motion delta's, letting a trusted gyro
// override wheel-derived rotation for motion sources with no heading.
//
//   <LocalizationPrediction type="localization/wheel_imu_prediction">
//       <Motion artifact_id="tracking_motion_delta"/>
//       <Orientation artifact_id="imu_orientation"/>   optional
//   </LocalizationPrediction>

#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "contracts/localization.h"

namespace navigatr
{

class WheelImuPrediction : public Localization
{
public:
    static std::unique_ptr<Localization> create(const ConfigNode& node,
                                                SlotInitializationContext& context,
                                                std::string& err);

    LocalizationOutput run(const LocalizationInput& in) override;

    void reset() override;

private:
    ArtifactId motion_ref_;
    ArtifactId orientation_ref_;   // empty when not configured

    uint64_t last_applied_init_ = 0;
};

} // namespace navigatr
