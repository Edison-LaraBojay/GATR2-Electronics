// tracking_wheel_motion.h
// Measurement model: individually configured encoder sensors, optionally
// constrained by a gyro, into one body-frame motion increment. Geometry
// lives here or in the referenced wheel_geometry resource, never in the
// sensors: each wheel states where it is mounted and which way it
// measures. Labels are diagnostics only.
//
//   <Observation id="tracking_motion" type="tracking_wheel_motion">
//       <TrackingWheel sensor_id="tracking_encoder_a" label="left"
//                      radius_m="0.0254" position_x_m="0.000"
//                      position_y_m="0.130" measurement_angle_deg="0"
//                      direction="positive"/>
//       ...   or   <Wheels resource_id="wheel_geometry"><Use wheel_id=.../></Wheels>
//       <HeadingConstraint sensor_id="robot_imu" bias_samples="200"
//                          max_calibration_travel_m="0.005" max_gap_ms="250"/>
//       <Timing interval_tolerance_ms="20" max_pending_ms="500"/>
//       <Output observation_id="tracking_motion"/>
//   </Observation>
//
// Rigid model per wheel: m_i = u_i . d + k_i * dtheta with
// k_i = x_i * u_iy - y_i * u_ix. Three suitably placed wheels solve planar
// motion alone; two wheels need the heading constraint; degenerate
// geometry is rejected at build. The constraint calibrates its own gyro
// bias over its first bias_samples readings while the robot sits still;
// wheel travel during calibration restarts it and wheel baselines rebase
// until it completes.
//
// Every source accumulates its own pending interval between solves. A
// solve happens only when every configured source has a pending interval
// and those intervals describe the same span within interval_tolerance_ms;
// intervals that stay misaligned longer than max_pending_ms are dropped
// together with a diagnostic, never fused. A nonpositive sample interval,
// an encoder baseline rebase, a gyro accumulator discontinuity or a gyro
// gap longer than max_gap_ms invalidates the whole pending window: nothing
// bridges an invalid span.

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "contracts/localization.h"
#include "payloads/robot_observations.h"
#include "payloads/sensor_samples.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

class TrackingWheelMotion : public RobotObservationFunction
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
    struct Wheel {
        TypedSensorBinding<EncoderSample> binding;
        std::string                       label;   // diagnostics only
        double                            radius_m = 0.0;
        double                            ux = 1.0, uy = 0.0;   // measurement direction
        double                            k_m  = 0.0;           // x*uy - y*ux
        double                            sign = 1.0;

        bool          have_prev      = false;
        double        prev_angle_rad = 0.0;
        MonotonicTime prev_stamp;
        uint64_t      prev_discontinuity = 0;
        uint64_t      last_sequence      = 0;
        uint64_t      last_epoch         = 0;

        bool          pending          = false;
        double        pending_travel_m = 0.0;
        MonotonicTime pending_start;
        MonotonicTime pending_end;
        Provenance    provenance;   // newest consumed sample
        bool          baselined = true;   // has a post-drop baseline sample
    };

    struct HeadingConstraint {
        bool                          configured = false;
        TypedSensorBinding<ImuSample> binding;
        long                          bias_samples = 200;
        long                          max_gap_ms   = 250;

        double max_calibration_travel_m = 0.005;
        double cal_travel_m             = 0.0;

        bool          calibrated       = false;
        long          cal_count        = 0;
        double        cal_sum          = 0.0;
        bool          cal_have_accum   = false;
        double        cal_accum_start  = 0.0;
        MonotonicTime cal_accum_start_stamp;
        double        bias_rad_s       = 0.0;

        bool          have_prev        = false;
        double        prev_rate        = 0.0;
        double        prev_accum       = 0.0;
        bool          prev_has_accum   = false;
        uint64_t      prev_accum_epoch = 0;
        MonotonicTime prev_stamp;
        uint64_t      last_sequence = 0;
        uint64_t      last_epoch    = 0;

        bool          pending        = false;
        double        pending_dtheta = 0.0;
        MonotonicTime pending_start;
        MonotonicTime pending_end;
        Provenance    provenance;
        bool          baselined = true;
    };

    // Discards every pending contribution. Until every source has consumed
    // a fresh baseline sample afterwards nothing accumulates, so no window
    // ever spans the invalid stretch.
    void dropWindow(const char* why);
    void restartCalibration();

    ObservationFunctionId id_;
    std::string           type_ = "tracking_wheel_motion";
    ObservationId         output_;
    std::vector<Wheel>    wheels_;
    HeadingConstraint     heading_;

    long interval_tolerance_ms_ = 20;
    long max_pending_ms_        = 500;

    bool          awaiting_baselines_ = false;
    MonotonicTime last_received_;   // host receipt of the newest consumed sample
    std::string   last_drop_reason_;
    uint64_t      drops_ = 0;
    bool          offered_ = false;
};

} // namespace navigatr
