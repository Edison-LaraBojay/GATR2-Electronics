// world_estimation_stage.h
// make_world_estimation and the executor it returns.
//
//   execute_world = make_world_estimation(<WorldEstimation>, registry,
//                                         sensor catalog, resource store)
//   field_out     = execute_world(FieldEstimationInput)
//
//   <WorldEstimation>
//       <Estimator id="goals" type="apriltag">
//           ...implementation-owned schema: field references and options...
//       </Estimator>
//   </WorldEstimation>
//
// The generic builder owns the section schema, the id and type, registry
// lookup and the inventory of declared outputs; the selected factory owns
// its subtree, resolves its references against the sensor catalog and the
// resource store once, and returns the captured implementation. Exactly one
// Estimator is configured for now: combining several is a later concern for
// this builder alone, never the coordinator.
//
// The executor owns the call, declared-output enforcement (an implementation
// cannot place an undeclared id or a payload contradicting its declaration
// into a standard map) and diagnostics; the coordinator calls it and
// publishes the result.

#pragma once
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "config/config_node.h"
#include "contracts/field_estimation.h"
#include "core/function_registry.h"
#include "resources/resource_store.h"
#include "runtime/sensor_catalog.h"

namespace navigatr
{

class WorldEstimationExecutor
{
public:
    WorldEstimationExecutor() = default;

    FieldEstimationOutput operator()(const FieldEstimationInput& in);

    void reset();

    const std::string& estimatorId() const { return id_; }
    const std::string& estimatorType() const { return type_; }

    // What the estimator declared it publishes across the boundary.
    const std::vector<ObservationOutputDecl>& observationOutputs() const {
        return observations_;
    }
    const std::vector<AssociationOutputDecl>& associationOutputs() const {
        return associations_;
    }

private:
    friend std::optional<WorldEstimationExecutor>
    make_world_estimation(const ConfigNode&, const FunctionRegistry&, const SensorCatalog&,
                          const ResourceStore&, std::vector<std::string>*, std::string&);

    std::unique_ptr<FieldEstimation>   estimator_;
    std::string                        id_;
    std::string                        type_;
    std::string                        label_;   // diagnostics
    std::vector<ObservationOutputDecl> observations_;
    std::vector<AssociationOutputDecl> associations_;
};

// node is the <WorldEstimation> element. Nothing on failure.
std::optional<WorldEstimationExecutor> make_world_estimation(const ConfigNode&         node,
                                                             const FunctionRegistry&   functions,
                                                             const SensorCatalog&      sensors,
                                                             const ResourceStore&      resources,
                                                             std::vector<std::string>* warnings,
                                                             std::string&              err);

} // namespace navigatr
