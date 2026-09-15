// localization_stage.h
// make_localization and the executor it returns.
//
//   execute_localization = make_localization(<Localization>, registry, ...)
//   robot_state = execute_localization(sensor_map, requests, context)
//
//   <Localization>
//       <Observation id="tracking_motion" type="tracking_wheel_motion">
//           ...model-owned schema...
//           <Output observation_id="tracking_motion"/>
//       </Observation>
//       <Estimator type="planar_motion_integrator">
//           <Motion observation_id="tracking_motion"/>
//       </Estimator>
//       <History retention_s="5" capacity="1024"
//                max_interpolation_gap_ms="100"/>     optional
//       <InitialPlacement x_m="0" y_m="0" heading_deg="0"/>   optional
//   </Localization>
//
// The generic builder owns ids, types, duplicate ids, duplicate output
// ids, registry lookup and estimator reference validation; each factory
// owns its subtree. The executor owns the observation loop, declared-output
// enforcement, the estimator call, finalization and publication: the
// RobotStateFeed it exposes is the only place history is written, and the
// coordinator merely calls it. An InitialPlacement is one configured
// placement request applied on the first cycle, the same edge a brain
// command would make.

#pragma once
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "contracts/localization.h"
#include "core/execution_context.h"
#include "core/function_registry.h"
#include "core/records.h"
#include "resources/resource_store.h"
#include "runtime/sensor_catalog.h"
#include "state/robot_state_feed.h"

namespace navigatr
{

class LocalizationExecutor
{
public:
    LocalizationExecutor();

    RobotState operator()(const SensorMap& sensors, const LocalizationRequests& requests,
                          const ExecutionContext& context);

    // Back to power-on: models and estimator reset, history cleared, the
    // odometry epoch moves on.
    void reset();

    // Readers on any thread.
    std::shared_ptr<RobotStateFeed> feed() const { return feed_; }

    const std::vector<RobotObservationOutputDecl>& observationOutputs() const {
        return outputs_;
    }
    std::vector<ObservationFunctionStatus> functionStatus() const;
    const RobotObservationMap&             lastObservations() const { return observations_; }
    const RobotState&                      state() const { return state_; }
    const std::string&                     estimatorType() const { return estimator_type_; }

private:
    friend std::optional<LocalizationExecutor>
    make_localization(const ConfigNode&, const FunctionRegistry&, const SensorCatalog&,
                      const ResourceStore&, std::vector<std::string>*, std::string&);

    struct Function {
        ObservationFunctionId                    id;
        std::unique_ptr<RobotObservationFunction> function;
        std::string                               label;   // diagnostics
    };

    std::vector<Function>                   functions_;
    std::vector<RobotObservationOutputDecl> outputs_;
    std::unique_ptr<StateEstimator>         estimator_;
    std::string                             estimator_type_;
    std::string                             estimator_label_;

    PlacementRequest configured_placement_;   // requested=false when absent
    bool             configured_placement_pending_ = false;

    std::shared_ptr<RobotStateFeed> feed_;
    RobotState                      state_;
    RobotObservationMap             observations_;
    RobotObservationMap             pending_;
    uint64_t                        publication_ = 0;
    uint64_t                        updates_     = 0;
};

// node is the <Localization> element. Nothing on failure.
std::optional<LocalizationExecutor> make_localization(const ConfigNode&         node,
                                                      const FunctionRegistry&   functions,
                                                      const SensorCatalog&      sensors,
                                                      const ResourceStore&      resources,
                                                      std::vector<std::string>* warnings,
                                                      std::string&              err);

} // namespace navigatr
