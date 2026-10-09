// imu_heading_increment.h
// Measurement model: one yaw-rate sensor into bias-corrected heading
// increments. The bias comes from a stationary window (stationary_window.h):
// bias_samples gyro samples spanning window_ms of sample time, the rate
// within still_rate_dps of the window mean and under max_rate_dps, no gap
// over evidence_gap_ms, and every listed wheel within still_travel_m over
// the window. Nothing is published until calibration completes; no
// qualified window within attempt_s fails it until recalibrate(). Later
// qualified windows adjust the bias in bounded steps; a gyro source restart
// (record or source epoch) invalidates it and calibration starts again.
// bias_samples 0 turns calibration off (bias zero).
//
// After calibration there is one HeadingIncrement per new sample,
// integrated from the producer's accumulator when it has one (packet
// batching loses nothing) and by endpoint trapezoid only when the producer
// offers no accumulator.
//
//   <Observation id="imu_heading" type="imu_heading_increment">
//       <Input sensor_id="robot_imu"/>
//       <Calibration bias_samples="200" max_gap_ms="250" window_ms="2000"
//                    still_travel_m="0.001" still_rate_dps="1" max_rate_dps="5"
//                    evidence_gap_ms="100" attempt_s="60">
//           <Wheel sensor_id="encoder_a" radius_m="0.024"/>   stillness evidence
//       </Calibration>
//       <Output observation_id="imu_heading"/>
//   </Observation>
//
// Wheels only gate the stationary window; they are no part of the
// increment's lineage. Without wheels the window is gyro evidence alone.
//
// An outage longer than max_gap_ms, a nonpositive interval, or an
// accumulator discontinuity reseeds instead of integrating; the interval
// is dropped, never bridged. In the supported pairing (three wheels,
// weighted_planar_fusion) no travel goes with it: the wheels measure
// rotation themselves, and the estimator uses that alone for an interval
// without a heading.

#pragma once
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "contracts/localization.h"
#include "impl/localization/stationary_window.h"
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
    bool recalibrate() override;

private:
    struct StillWheel {
        TypedSensorBinding<EncoderSample> binding;
        double                            radius_m = 0.0;
        size_t                            source   = 0;
    };

    FunctionStatus ingest(const RobotObservationInput& in, RobotObservationMap& out);
    void           takeWindow(MonotonicTime now);

    std::optional<RobotObservationRecord> pending_;
    bool offered_ = false;
    ObservationFunctionId         id_;
    std::string                   type_ = "imu_heading_increment";
    ObservationId                 output_;
    TypedSensorBinding<ImuSample> binding_;
    long                          max_gap_ms_ = 250;

    StationaryWindow        window_;
    GyroBiasCalibration     bias_;
    size_t                  gyro_source_ = 0;
    std::vector<StillWheel> wheels_;

    bool     seen_          = false;
    uint64_t last_sequence_ = 0;
    uint64_t last_epoch_    = 0;
    uint64_t last_upstream_ = 0;

    bool          have_prev_        = false;
    double        prev_rate_        = 0.0;
    double        prev_accum_       = 0.0;
    bool          prev_has_accum_   = false;
    uint64_t      prev_accum_epoch_ = 0;
    MonotonicTime prev_stamp_;
    std::string   last_note_;
};

} // namespace navigatr
