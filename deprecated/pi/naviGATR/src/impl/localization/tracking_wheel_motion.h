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
//                      direction="positive" travel_scale="1"/>
//       ...   or   <Wheels resource_id="wheel_geometry"><Use wheel_id=.../></Wheels>
//       <HeadingConstraint sensor_id="robot_imu" bias_samples="200"
//                          max_calibration_travel_m="0.005" max_gap_ms="250"
//                          window_ms="2000" still_rate_dps="1" max_rate_dps="5"
//                          evidence_gap_ms="100" attempt_s="60"/>
//       <LateralMotion assume="zero"/>   optional, forward-only wheels
//       <Timing interval_tolerance_ms="20" max_pending_ms="500"/>
//       <Output observation_id="tracking_motion"/>
//   </Observation>
//
// Rigid model per wheel: m_i = u_i . d + k_i * dtheta with
// k_i = x_i * u_iy - y_i * u_ix, and m_i = d(wheel angle) * radius *
// travel_scale * sign (travel_scale is the optional empirical correction,
// default 1, applied here only). Three suitably placed wheels solve planar
// motion alone; two wheels need the heading constraint; degenerate
// geometry is rejected at build.
//
// The constraint calibrates its own gyro bias from a stationary window
// (stationary_window.h) over the wheels and the gyro: bias_samples samples
// per source spanning window_ms of sample time, per-wheel travel within
// max_calibration_travel_m, the gyro within still_rate_dps of the window
// mean and under max_rate_dps, no gap over evidence_gap_ms. Until then wheel
// baselines rebase and nothing is produced; no qualified window within
// attempt_s fails calibration until recalibrate(). The sample that completes
// the window seeds the integrator, the same sample the wheels rebase on.
// Later qualified windows adjust the bias in bounded steps. A gyro source
// restart (record or source epoch) invalidates the bias and calibration
// starts again. bias_samples 0 turns calibration off (bias zero).
//
// Motion while the bias calibrates is never integrated: movement the window
// sees, or a wheel rebasing away more than max_calibration_travel_m in all,
// counts as a dropped interval, like every dropped window below
// (ObservationReadiness::dropped_intervals).
//
// Wheels that all measure along body x cannot see sideways motion. They
// build only with LateralMotion assume="zero", which needs the heading
// constraint: dy is fixed at zero, dtheta comes from the gyro, and each
// wheel's forward estimate is travel_i + y_i * dtheta (angle 0). Sideways
// slip or pushing is then unmeasured, not corrected.
//
// Every source accumulates its own pending interval between solves. A
// solve happens only when every configured source has a pending interval
// and those intervals describe the same span within interval_tolerance_ms;
// intervals that stay misaligned longer than max_pending_ms are dropped
// together with a diagnostic, never fused. A nonpositive sample interval,
// an encoder baseline rebase, a gyro accumulator discontinuity, or a gyro
// gap longer than max_gap_ms invalidates the whole pending window: nothing
// bridges an invalid span.

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "contracts/localization.h"
#include "impl/localization/stationary_window.h"
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
    bool recalibrate() override;

private:
    struct Wheel {
        TypedSensorBinding<EncoderSample> binding;
        std::string                       label;   // diagnostics only
        double                            radius_m = 0.0;
        double                            ux = 1.0, uy = 0.0;   // measurement direction
        double                            k_m  = 0.0;           // x*uy - y*ux
        double                            sign = 1.0;
        double                            scale = 1.0;   // travel scale
        size_t                            source = 0;    // stationary window source

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

        double calibration_travel_m = 0.0;   // rebased away while the bias calibrates
    };

    struct HeadingConstraint {
        bool                          configured = false;
        TypedSensorBinding<ImuSample> binding;
        long                          max_gap_ms = 250;

        StationaryWindow    window;
        GyroBiasCalibration bias;
        size_t              source = 0;

        bool          seen             = false;
        bool          have_prev        = false;
        double        prev_rate        = 0.0;
        double        prev_accum       = 0.0;
        bool          prev_has_accum   = false;
        uint64_t      prev_accum_epoch = 0;
        MonotonicTime prev_stamp;
        uint64_t      last_sequence = 0;
        uint64_t      last_epoch    = 0;
        uint64_t      last_upstream = 0;

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
    bool calibrating() const { return heading_.configured && !heading_.bias.calibrated(); }
    void takeWindow(MonotonicTime now);

    // Counts motion that was measured and discarded (dropped_intervals).
    void lostMotion(const std::string& why);
    // Movement the window saw since movements_before, while not calibrated;
    // true when it counted one.
    bool calibrationMovement(uint64_t movements_before);
    void clearCalibrationTravel();

    ObservationFunctionId id_;
    std::string           type_ = "tracking_wheel_motion";
    ObservationId         output_;
    std::vector<Wheel>    wheels_;
    HeadingConstraint     heading_;

    long interval_tolerance_ms_ = 20;
    long max_pending_ms_        = 500;

    bool zero_lateral_ = false;   // LateralMotion assume="zero"

    // d(dx, dy)/d(dtheta) from the wheel geometry, published with every
    // increment
    bool   has_coupling_ = false;
    double coupling_x_   = 0.0;
    double coupling_y_   = 0.0;

    bool          awaiting_baselines_ = false;
    MonotonicTime last_received_;   // host receipt of the newest consumed sample
    std::string   last_drop_reason_;
    uint64_t      drops_ = 0;   // dropped_intervals; never reset
    std::string   dropped_why_;
    bool          offered_ = false;
};

} // namespace navigatr
