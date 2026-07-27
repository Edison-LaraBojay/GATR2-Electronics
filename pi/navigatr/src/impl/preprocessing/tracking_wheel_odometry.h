// tracking_wheel_odometry.h
// Combines individually configured encoder sensors into one body-frame
// motion delta. Geometry lives here, not in the sensors: each TrackingWheel
// child references a sensor by id and states where that wheel is mounted and
// which way it measures. Labels are diagnostics only; neither XML order nor
// sensor id spelling defines left/right/rear, the geometry does.
//
//   <Preprocessor id="tracking_motion"
//                 type="preprocessor/tracking_wheel_odometry">
//       <TrackingWheel sensor_id="tracking_encoder_a" label="left"
//                      radius_m="0.0254"
//                      position_x_m="0.000" position_y_m="0.130"
//                      measurement_angle_deg="0" direction="positive"/>
//       ...
//       <HeadingConstraint sensor_id="robot_imu" bias_samples="200"/>
//       <Output artifact_id="tracking_motion_delta"/>
//   </Preprocessor>
//
// Rigid model per wheel: m_i = u_i . d + k_i * dtheta with
// k_i = x_i * u_iy - y_i * u_ix. Three suitably placed wheels solve planar
// motion alone; two wheels need the heading constraint; degenerate geometry
// is rejected at initialization. The constraint does its own gyro bias
// calibration over its first bias_samples readings while the robot sits
// still.
//
// Wheel travel is accumulated between solves, so a cycle where one wheel has
// not published yet loses nothing.

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "contracts/preprocessing.h"
#include "payloads/preprocessing_products.h"
#include "payloads/sensor_samples.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

class TrackingWheelOdometry : public PreprocessorExecutable
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
    struct Wheel {
        TypedSensorBinding<EncoderSample> binding;
        std::string                       label;   // diagnostics only
        double                            radius_m = 0.0;
        double                            ux = 1.0, uy = 0.0;   // measurement direction
        double                            k_m  = 0.0;           // x*uy - y*ux
        double                            sign = 1.0;

        bool          have_prev = false;
        double        prev_angle_rad = 0.0;
        MonotonicTime prev_stamp;
        uint64_t      last_sequence = 0;

        double pending_travel_m = 0.0;
        double pending_dt_s     = 0.0;
        bool   pending          = false;
        MonotonicTime pending_stamp;
    };

    struct HeadingConstraint {
        bool                          configured = false;
        TypedSensorBinding<ImuSample> binding;
        long                          bias_samples = 200;

        bool     calibrated = false;
        long     cal_count  = 0;
        double   cal_sum    = 0.0;
        double   bias_rad_s = 0.0;
        bool     have_prev  = false;
        double   prev_rate  = 0.0;
        MonotonicTime prev_stamp;
        uint64_t last_sequence = 0;

        double pending_dtheta = 0.0;
        double pending_dt_s   = 0.0;
        bool   pending        = false;
    };

    PreprocessorId     id_;
    ArtifactId         output_;
    std::vector<Wheel> wheels_;
    HeadingConstraint  heading_;
};

} // namespace navigatr
