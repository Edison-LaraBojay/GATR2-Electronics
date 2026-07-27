// imu_normalization.h
// Gyro bias calibration and heading-delta integration. Bias is the average
// of the first bias_samples readings, taken while the robot sits still.
// Produces nothing until calibration completes, then one ImuDelta per new
// sample, trapezoid integrated.
//
//   <Preprocessor id="imu_normalization"
//                 type="preprocessor/imu_normalization">
//       <Input sensor_id="robot_imu"/>
//       <Calibration bias_samples="200"/>
//       <Output artifact_id="imu_orientation"/>
//   </Preprocessor>

#pragma once
#include <memory>
#include <string>

#include "contracts/preprocessing.h"
#include "payloads/preprocessing_products.h"
#include "payloads/sensor_samples.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

class ImuNormalization : public PreprocessorExecutable
{
public:
    static std::unique_ptr<PreprocessorExecutable> create(
        const ConfigNode& node, PreprocessorInitializationContext& context,
        std::string& err);

    FunctionStatus run(const PreprocessingInput& in, ArtifactMap& out) override;

    const PreprocessorId& id() const override { return id_; }

    std::vector<ArtifactOutputDecl> outputs() const override;

    void reset() override;

private:
    PreprocessorId                id_;
    ArtifactId                    output_;
    TypedSensorBinding<ImuSample> binding_;
    long                          bias_samples_ = 200;

    uint64_t last_sequence_ = 0;
    bool     calibrated_    = false;
    long     cal_count_     = 0;
    double   cal_sum_       = 0.0;
    double   bias_rad_s_    = 0.0;

    bool          have_prev_ = false;
    double        prev_rate_ = 0.0;
    MonotonicTime prev_stamp_;
};

} // namespace navigatr
