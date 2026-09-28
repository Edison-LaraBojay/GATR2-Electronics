// brain_imu_wheel_bench.h
// Arrival-time bench adapter for Pico wheels and the Brain's VEX IMU.
// Planar mode resolves two independent wheel directions; parallel mode
// explicitly assumes zero sideways travel for two forward-measuring wheels
// (0 or 180 degrees). Wheels are inline TrackingWheel elements (the
// tracking_wheel_motion schema) or a Wheels reference to wheel_geometry;
// travel = d(wheel angle) * radius * travel_scale * sign.
//
//   <Observation id="tracking_motion" type="brain_imu_planar_bench">
//       <Wheels resource_id="wheel_geometry"><Use wheel_id=.../>x2</Wheels>
//       <Imu resource_id="brain_imu"/>
//       <Stillness window_ms="2000" samples="20" still_travel_m="0.001"
//                  still_rate_dps="1" evidence_gap_ms="100"/>        optional
//       <Freshness max_age_ms="200"/>                                 optional
//       <Attitude observation_id="vex_attitude"/>                     optional
//       <Output observation_id="tracking_motion"/>
//   </Observation>
//
// Attitude publishes the Brain VEX IMU tilt from TELEMETRY (robot frame,
// reference "gravity", no yaw) once per new report, measured at its Pi
// arrival time, for an estimator's <Attitude> fold. It never changes the
// motion output; without TELEMETRY, or for reports without the attitude
// group, nothing is published and the fold ages the last tilt out.
//
// Latest wheels and the latest VEX rotation pair by Pi arrival time. One
// step spans at most max_age_ms: a sample older than that, or two emitted
// VEX IMU receipts further apart, holds the pose and rebaselines; an IMU
// epoch, wheel epoch or discontinuity change, or a rotation jump, rebaselines
// too. Nothing bridges a gap. A rebaseline that discards an interval after a
// baseline existed counts as a dropped interval
// (ObservationReadiness::dropped_intervals); with a Brain profile the System
// then ends pose continuity (spec 8.10), and the profile builder sets
// max_age_ms to the profile's sensor_loss_ms so both limits agree.
//
// The VEX IMU arrives calibrated by VEX firmware; the Pi applies no bias.
// Stationary status comes from a stationary window over the wheels and the
// VEX rotation (reduced evidence: no raw gyro rate, no acceleration).

#pragma once
#include <array>

#include "contracts/localization.h"
#include "impl/localization/stationary_window.h"
#include "payloads/sensor_samples.h"
#include "resources/brain_imu_bench.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

class BrainImuWheelBench : public RobotObservationFunction
{
public:
    static std::unique_ptr<RobotObservationFunction>
    createParallel(const ConfigNode&, RobotObservationInitializationContext&, std::string&);
    static std::unique_ptr<RobotObservationFunction>
    createPlanar(const ConfigNode&, RobotObservationInitializationContext&, std::string&);

    FunctionStatus run(const RobotObservationInput&, RobotObservationMap&) override;

    const ObservationFunctionId& id() const override { return id_; }
    const std::string&           type() const override { return type_; }

    std::vector<RobotObservationOutputDecl> outputs() const override;

    ObservationReadiness readiness() const override;

    void reset() override;
    void settle(const ObservationId& id, bool) override {
        if (id == output_) {
            offered_ = false;
        }
    }

private:
    static std::unique_ptr<RobotObservationFunction>
    create(const ConfigNode&, RobotObservationInitializationContext&, std::string&, bool planar);

    struct Wheel {
        TypedSensorBinding<EncoderSample> binding;
        double   radius = 0, scale = 1, ux = 1, uy = 0, k = 0, sign = 1, angle = 0;
        uint64_t sequence = 0, epoch = 0, discontinuity = 0;
        size_t   source = 0;
    };

    void observeStillness(const std::array<const StoredSample*, 2>&  stored,
                          const std::array<const EncoderSample*, 2>& samples, MonotonicTime now);
    void lostMotion(const std::string& why);

    void publishAttitude(RobotObservationMap& out);

    ObservationFunctionId id_;
    ObservationId         output_;
    ObservationId         attitude_output_;        // empty = no attitude output
    uint64_t              attitude_sequence_ = 0;  // newest published report
    std::string           type_;
    std::string note_ = "bench: waiting for VEX IMU and encoders; arrival-time approximation";
    std::shared_ptr<BrainImuBench> imu_;
    std::array<Wheel, 2>           wheels_;
    bool                           planar_      = false;
    double                         determinant_ = 0;
    bool                           baseline_ = false, offered_ = false, ready_ = false;
    uint64_t                       imu_sequence_ = 0, imu_epoch_ = 0;
    int32_t                        rotation_ = 0;
    MonotonicTime                  previous_;
    long                           max_age_ms_ = 200;
    uint64_t                       dropped_    = 0;   // dropped_intervals; never reset
    std::string                    dropped_why_;

    StationaryWindow window_;
    size_t           rotation_source_ = 0;
};

} // namespace navigatr
