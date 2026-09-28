// attitude_reference.h
// Measurement model: one attitude sensor into attitude observations. The
// sensor already expresses the body in its reference frame with mounting
// applied; this model checks freshness and forwards each new sample with
// its provenance, so the estimator can fuse the tilt with its planar
// heading and age it separately.
//
//   <Observation id="attitude" type="attitude_reference">
//       <Input sensor_id="robot_attitude"/>
//       <Freshness max_age_ms="100"/>
//       <Output observation_id="attitude"/>
//   </Observation>

#pragma once
#include <memory>
#include <string>

#include "contracts/localization.h"
#include "payloads/attitude_samples.h"
#include "payloads/robot_observations.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

class AttitudeReference : public RobotObservationFunction
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

    void reset() override {
        last_sequence_ = 0;
        last_epoch_    = 0;
        seen_          = false;
        offered_       = false;
    }
    void settle(const ObservationId& id, bool) override {
        if (id == output_) offered_ = false;
    }

private:
    ObservationFunctionId              id_;
    std::string                        type_ = "attitude_reference";
    ObservationId                      output_;
    TypedSensorBinding<AttitudeSample> binding_;
    long                               max_age_ms_ = 100;

    uint64_t last_sequence_ = 0;
    uint64_t last_epoch_    = 0;
    bool     seen_          = false;
    bool     offered_       = false;
};

} // namespace navigatr
