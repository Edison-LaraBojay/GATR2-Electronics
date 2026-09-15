// localization.h
// Localization as a self-contained component built by make_localization:
// a configured collection of robot observation functions, one selected
// state estimator, the field anchor, and history publication.
//
//   observations = execute_robot_observations(sensor_map, context)
//   update       = execute_state_estimation(observations, previous,
//                                           requests, context)
//   finalize: publish the accepted update and history
//
// Observation functions are grouped by measurement model, not by device:
// one may read a configured list of wheels plus a gyro, another a single
// attitude source. Each declares the observation ids and payloads it
// publishes, binds its sensors at build with the payload it expects, and
// owns its own calibration and pending-interval state. The estimator
// declares which observation ids it consumes; every reference is checked
// at build. Finalization (history append, snapshot publication) is stage
// bookkeeping, never a configured plugin.

#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include "config/config_node.h"
#include "core/execution_context.h"
#include "core/function_registry.h"
#include "core/function_status.h"
#include "core/ids.h"
#include "core/payload_descriptor.h"
#include "core/records.h"
#include "math/transforms.h"
#include "state/robot_state.h"

namespace navigatr
{

class SensorCatalog;
class ResourceStore;

struct RobotObservationRecord {
    MonotonicTime measuredAt;   // source clock, end of the observation's support
    MonotonicTime receivedAt;   // host receipt of the newest consumed sample
    TypedPayload  payload;
};

using RobotObservationMap =
    std::unordered_map<ObservationId, RobotObservationRecord, ObservationId::Hash>;

struct RobotObservationOutputDecl {
    ObservationId     id;
    PayloadDescriptor payload;
};

struct RobotObservationInput {
    const SensorMap&        sensors;
    const ExecutionContext& context;
};

// Readiness for publishers and inspection: a model that is still
// calibrating produces nothing yet and says why.
struct ObservationReadiness {
    bool        ready = false;
    std::string note;
};

class RobotObservationFunction
{
public:
    virtual ~RobotObservationFunction() = default;

    virtual FunctionStatus run(const RobotObservationInput& in, RobotObservationMap& out) = 0;

    virtual const ObservationFunctionId& id() const = 0;
    virtual const std::string&           type() const = 0;

    virtual std::vector<RobotObservationOutputDecl> outputs() const = 0;

    virtual ObservationReadiness readiness() const = 0;

    // A produced record remains offered until the estimator explicitly accepts
    // or rejects it. While offered, keep ingesting new samples but do not
    // replace that output; retain subsequent motion in the model's bounded
    // accumulator. Rejection discards only the offered interval, with a
    // diagnostic supplied by the estimator.
    virtual void settle(const ObservationId&, bool /*accepted*/) {}

    virtual void reset() {}
};

struct RobotObservationInitializationContext {
    const SensorCatalog*      sensors   = nullptr;
    const ResourceStore*      resources = nullptr;
    const FunctionRegistry*   functions = nullptr;
    std::vector<std::string>* warnings  = nullptr;
};

using RobotObservationMakeFunction = std::function<std::unique_ptr<RobotObservationFunction>(
    const ConfigNode&, RobotObservationInitializationContext&, std::string& err)>;

// A request to place the robot origin in the field frame. origin tells
// where it came from so a configured placement and a commanded one are
// separate edges.
struct PlacementRequest {
    bool        requested = false;
    std::string origin;   // "command", "configuration"
    uint64_t    sequence = 0;
    Pose2D      pose;     // T_field_robot
};

struct LocalizationRequests {
    PlacementRequest placement;
};

struct StateEstimatorInput {
    const RobotObservationMap&  observations;
    const RobotState&           previous;
    const LocalizationRequests& requests;
    const ExecutionContext&     context;
};

struct StateEstimatorOutput {
    RobotState     robot;
    FunctionStatus status = FunctionStatus::kOk;

    // True when the estimate moved to a new effective time; only then is
    // the pose appended to history. A hold keeps the previous time.
    bool advanced = false;

    // Device-to-host mapping available for effective times.
    bool clock_mapped = false;

    // Unlisted observations remain pending for a later invocation. Every id
    // belongs to at most one list; acceptance consumes it exactly once.
    std::vector<ObservationId> accepted;
    std::vector<ObservationId> rejected;

    std::string diagnostic;
};

class StateEstimator
{
public:
    virtual ~StateEstimator() = default;

    virtual StateEstimatorOutput run(const StateEstimatorInput& in) = 0;

    virtual const std::string& type() const = 0;

    virtual void reset() {}
};

struct StateEstimatorInitializationContext {
    std::vector<RobotObservationOutputDecl> observations;   // declared by the functions
    const FunctionRegistry*                 functions = nullptr;
    std::vector<std::string>*               warnings  = nullptr;

    // The id must be declared and, when expected is given, publish exactly
    // that payload type.
    bool requireObservation(const ObservationId& id, const std::type_index* expected,
                            const std::string& who, std::string& err) const {
        for (const RobotObservationOutputDecl& d : observations) {
            if (d.id == id) {
                if (expected != nullptr && !d.payload.matches(*expected)) {
                    err = who + " requires a different payload than observation " +
                          id.value + " produces (" + d.payload.stable_name + ")";
                    return false;
                }
                return true;
            }
        }
        err = who + " references observation " + id.value + " which nothing produces";
        return false;
    }
};

using StateEstimatorMakeFunction = std::function<std::unique_ptr<StateEstimator>(
    const ConfigNode&, StateEstimatorInitializationContext&, std::string& err)>;

} // namespace navigatr
