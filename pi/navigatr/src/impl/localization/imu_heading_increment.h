// imu_heading_increment.h
// Measurement model: one yaw-rate sensor into bias-corrected heading
// increments. Bias is the mean of the first bias_samples readings, taken
// while the robot sits still; nothing is published until calibration
// completes, then one HeadingIncrement per new sample, integrated from the
// producer's accumulator when it has one (packet batching loses nothing)
// and by endpoint trapezoid only when the producer offers no accumulator.
//
//   <Observation id="imu_heading" type="imu_heading_increment">
//       <Input sensor_id="robot_imu"/>
//       <Calibration bias_samples="200" max_gap_ms="250"/>
//       <Output observation_id="imu_heading"/>
//   </Observation>
//
// An outage longer than max_gap_ms, a nonpositive interval, or an
// accumulator discontinuity reseeds instead of integrating; the interval
// is dropped, never bridged.

#pragma once
#include <memory>
#include <optional>
#include <string>

#include "contracts/localization.h"
#include "payloads/robot_observations.h"
#include "payloads/sensor_samples.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

class ImuHeadingIncrement : public RobotObservationFunction
{
public:
    static std::unique_ptr<RobotObservationFunction>
    create(const ConfigNode& node, RobotObservationInitializationContext& context,
           std::string& err);

    FunctionStatus run(const RobotObservationInput& in, RobotObservationMap& out) override;

    const ObservationFunctionId& id() const override { return id_; }
    const std::string&           type() const override { return type_; }

    std::vector<RobotObservationOutputDecl> outputs() const override;

    ObservationReadiness readiness() const override;

    void reset() override;
    void settle(const ObservationId& id, bool) override {
        if (id == output_) offered_ = false;
    }

private:
    FunctionStatus ingest(const RobotObservationInput& in, RobotObservationMap& out);
    std::optional<RobotObservationRecord> pending_;
    bool offered_ = false;
    ObservationFunctionId         id_;
    std::string                   type_ = "imu_heading_increment";
    ObservationId                 output_;
    TypedSensorBinding<ImuSample> binding_;
    long                          bias_samples_ = 200;
    long                          max_gap_ms_   = 250;

    uint64_t last_sequence_ = 0;
    uint64_t last_epoch_    = 0;
    bool     calibrated_    = false;
    long     cal_count_     = 0;
    double   cal_sum_       = 0.0;
    double   bias_rad_s_    = 0.0;

    bool          have_prev_        = false;
    double        prev_rate_        = 0.0;
    double        prev_accum_       = 0.0;
    bool          prev_has_accum_   = false;
    uint64_t      prev_accum_epoch_ = 0;
    MonotonicTime prev_stamp_;
    std::string   last_note_;
};

} // namespace navigatr
