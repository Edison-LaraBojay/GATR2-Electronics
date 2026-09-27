#pragma once
#include <array>
#include "contracts/localization.h"
#include "payloads/sensor_samples.h"
#include "resources/brain_imu_bench.h"
#include "runtime/sensor_catalog.h"

namespace navigatr {
// Arrival-time bench adapter for Pico wheels and the Brain's VEX IMU.
// Planar mode resolves two independent wheel directions; parallel mode
// explicitly assumes zero sideways travel for two forward-facing wheels.
class BrainImuWheelBench : public RobotObservationFunction {
public:
    static std::unique_ptr<RobotObservationFunction> createParallel(
        const ConfigNode&, RobotObservationInitializationContext&, std::string&);
    static std::unique_ptr<RobotObservationFunction> createPlanar(
        const ConfigNode&, RobotObservationInitializationContext&, std::string&);
    FunctionStatus run(const RobotObservationInput&, RobotObservationMap&) override;
    const ObservationFunctionId& id() const override { return id_; }
    const std::string& type() const override { return type_; }
    std::vector<RobotObservationOutputDecl> outputs() const override;
    ObservationReadiness readiness() const override { return {ready_, note_}; }
    void reset() override { baseline_ = false; offered_ = false; ready_ = false; }
    void settle(const ObservationId& id, bool) override {
        if (id == output_) offered_ = false;
    }
private:
    static std::unique_ptr<RobotObservationFunction> create(
        const ConfigNode&, RobotObservationInitializationContext&, std::string&, bool planar);
    struct Wheel {
        TypedSensorBinding<EncoderSample> binding;
        double radius = 0, ux = 1, uy = 0, k = 0, sign = 1, angle = 0;
        uint64_t sequence = 0, epoch = 0, discontinuity = 0;
    };
    ObservationFunctionId id_;
    ObservationId output_;
    std::string type_;
    std::string note_ = "bench: waiting for VEX IMU and encoders; arrival-time approximation";
    std::shared_ptr<BrainImuBench> imu_;
    std::array<Wheel, 2> wheels_;
    bool planar_ = false;
    double determinant_ = 0;
    bool baseline_ = false, offered_ = false, ready_ = false;
    uint64_t imu_sequence_ = 0, imu_epoch_ = 0;
    int32_t rotation_ = 0;
    MonotonicTime previous_;
};
} // namespace navigatr
