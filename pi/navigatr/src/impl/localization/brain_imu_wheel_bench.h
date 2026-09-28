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
//       <Output observation_id="tracking_motion"/>
//   </Observation>
//
// Latest wheels and the latest VEX rotation pair by Pi arrival time. A
// missing or stale (200 ms) sample holds the pose and rebaselines; an IMU
// epoch, wheel epoch or discontinuity change, or a rotation jump, rebaselines
// too. Nothing bridges a gap: with a Brain profile the System ends pose
// continuity when a used source stays stale or restarts (spec 8.10).
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

    ObservationFunctionId id_;
    ObservationId         output_;
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

    StationaryWindow window_;
    size_t           rotation_source_ = 0;
};

} // namespace navigatr
