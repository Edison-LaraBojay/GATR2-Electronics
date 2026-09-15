// localization_stage.cpp

#include "runtime/localization_stage.h"

#include "config/pose3_config.h"
#include "core/diagnostics.h"

namespace navigatr
{

LocalizationExecutor::LocalizationExecutor() : feed_(std::make_shared<RobotStateFeed>()) {}

std::vector<ObservationFunctionStatus> LocalizationExecutor::functionStatus() const {
    std::vector<ObservationFunctionStatus> out;
    for (const Function& f : functions_) {
        ObservationFunctionStatus s;
        s.id                          = f.id.value;
        s.type                        = f.function->type();
        const ObservationReadiness r  = f.function->readiness();
        s.ready                       = r.ready;
        s.note                        = r.note;
        out.push_back(std::move(s));
    }
    return out;
}

RobotState LocalizationExecutor::operator()(const SensorMap&            sensors,
                                            const LocalizationRequests& requests,
                                            const ExecutionContext&     context) {
    // execute_robot_observations
    for (Function& f : functions_) {
        RobotObservationMap  produced;
        const auto declared = f.function->outputs();
        const FunctionStatus s = f.function->run({sensors, context}, produced);
        for (auto& kv : produced) {
            const RobotObservationOutputDecl* decl = nullptr;
            for (const RobotObservationOutputDecl& d : declared) {
                if (d.id == kv.first) {
                    decl = &d;
                    break;
                }
            }
            if (decl == nullptr || !decl->payload.matches(kv.second.payload.cppType())) {
                if (context.diagnostics != nullptr) {
                    context.diagnostics->note(
                        f.label + "/undeclared_output:" + kv.first.value,
                        FunctionStatus::kFault);
                }
                continue;
            }
            if (!pending_.emplace(kv.first, std::move(kv.second)).second &&
                context.diagnostics != nullptr) {
                context.diagnostics->note(f.label + "/output_still_pending:" + kv.first.value,
                                          FunctionStatus::kFault);
            }
        }
        if (context.diagnostics != nullptr) {
            context.diagnostics->note(f.label, s);
        }
    }
    observations_ = pending_;

    // execute_state_estimation
    LocalizationRequests effective = requests;
    if (configured_placement_pending_ && !effective.placement.requested) {
        effective.placement           = configured_placement_;
        configured_placement_pending_ = false;
    }
    StateEstimatorOutput update =
        estimator_->run({observations_, state_, effective, context});
    if (context.diagnostics != nullptr) {
        context.diagnostics->note(estimator_label_, update.status);
    }

    const auto settle = [&](const std::vector<ObservationId>& ids, bool accepted) {
        for (const ObservationId& id : ids) {
            if (pending_.erase(id) == 0) {
                if (context.diagnostics != nullptr) {
                    context.diagnostics->note(estimator_label_ + "/invalid_disposition:" +
                                                  id.value, FunctionStatus::kFault);
                }
                continue;
            }
            for (Function& f : functions_) {
                for (const auto& output : f.function->outputs()) {
                    if (output.id == id) {
                        f.function->settle(id, accepted);
                    }
                }
            }
        }
    };
    settle(update.accepted, true);
    settle(update.rejected, false);

    // finalize: the accepted update becomes the estimate; only an advance
    // to a new effective time enters history
    state_ = update.robot;
    ++publication_;
    if (update.advanced) {
        ++updates_;
    }
    LocalizationStatus status;
    status.functions      = functionStatus();
    status.estimator_type = estimator_type_;
    status.updates        = updates_;
    status.clock_mapped   = update.clock_mapped;
    feed_->publish(state_, status, update.advanced, publication_);
    return state_;
}

void LocalizationExecutor::reset() {
    for (Function& f : functions_) {
        f.function->reset();
    }
    estimator_->reset();
    observations_.clear();
    pending_.clear();

    // a hard reset is an odometry discontinuity: the new frame shares
    // nothing with the old one, so the epoch moves on
    const uint64_t next_epoch = state_.odometry_epoch + 1;
    state_                    = RobotState{};
    state_.odometry_epoch     = next_epoch;
    LocalizationStatus status;
    status.functions      = functionStatus();
    status.estimator_type = estimator_type_;
    status.updates        = updates_;
    feed_->publish(state_, status, false, ++publication_);
    configured_placement_pending_ = configured_placement_.requested;
}

std::optional<LocalizationExecutor> make_localization(const ConfigNode&         node,
                                                      const FunctionRegistry&   functions,
                                                      const SensorCatalog&      sensors,
                                                      const ResourceStore&      resources,
                                                      std::vector<std::string>* warnings,
                                                      std::string&              err) {
    if (!node.valid()) {
        err = "missing Localization section";
        return std::nullopt;
    }
    LocalizationExecutor executor;

    RobotObservationInitializationContext context;
    context.sensors   = &sensors;
    context.resources = &resources;
    context.functions = &functions;
    context.warnings  = warnings;

    PoseHistoryConfig history;
    bool              seen_estimator = false;
    ConfigNode        estimator_node;

    for (ConfigNode c = node.child(); c.valid(); c = c.next()) {
        const std::string name = c.name();
        if (name == "Observation") {
            const ObservationFunctionId id{c.attr("id")};
            const FunctionKey           type{c.attr("type")};
            if (id.empty() || type.empty()) {
                err = c.path() + ": Observation needs id and type";
                return std::nullopt;
            }
            for (const auto& seen : executor.functions_) {
                if (seen.id == id) {
                    err = c.path() + ": duplicate Observation id " + id.value;
                    return std::nullopt;
                }
            }
            const RobotObservationMakeFunction* factory =
                functions.find<RobotObservationMakeFunction>(type, err);
            if (factory == nullptr) {
                err = c.path() + ": " + err;
                return std::nullopt;
            }
            std::unique_ptr<RobotObservationFunction> built = (*factory)(c, context, err);
            if (built == nullptr) {
                if (err.empty()) {
                    err = c.path() + ": factory produced no observation function";
                }
                return std::nullopt;
            }
            for (const RobotObservationOutputDecl& decl : built->outputs()) {
                if (decl.id.empty()) {
                    err = c.path() + ": declares an output without an id";
                    return std::nullopt;
                }
                for (const RobotObservationOutputDecl& seen : executor.outputs_) {
                    if (seen.id == decl.id) {
                        err = c.path() + ": duplicate observation output id " +
                              decl.id.value;
                        return std::nullopt;
                    }
                }
                executor.outputs_.push_back(decl);
            }
            LocalizationExecutor::Function f;
            f.id       = id;
            f.label    = "Observation/" + id.value;
            f.function = std::move(built);
            executor.functions_.push_back(std::move(f));
        } else if (name == "Estimator") {
            if (seen_estimator) {
                err = node.path() + " has more than one Estimator";
                return std::nullopt;
            }
            seen_estimator = true;
            estimator_node = c;
        } else if (name == "History") {
            double retention_s = 5.0;
            long   capacity = 1024, gap_ms = 100, attitude_gap_ms = 100;
            if (!c.getDouble("retention_s", 5.0, retention_s, err) ||
                !c.getInt("capacity", 1024, capacity, err) ||
                !c.getInt("max_interpolation_gap_ms", 100, gap_ms, err) ||
                !c.getInt("attitude_gap_ms", 100, attitude_gap_ms, err)) {
                return std::nullopt;
            }
            if (retention_s <= 0.0 || capacity < 2 || gap_ms <= 0 || attitude_gap_ms <= 0) {
                err = c.path() + ": retention_s, capacity, max_interpolation_gap_ms and "
                      "attitude_gap_ms must be positive (capacity at least 2)";
                return std::nullopt;
            }
            history.retention_ms             = static_cast<int64_t>(retention_s * 1000.0);
            history.capacity                 = static_cast<std::size_t>(capacity);
            history.max_interpolation_gap_ms = gap_ms;
            history.attitude_gap_ms          = attitude_gap_ms;
        } else if (name == "InitialPlacement") {
            if (!parsePlanarPose(c, executor.configured_placement_.pose, err)) {
                return std::nullopt;
            }
            executor.configured_placement_.requested = true;
            executor.configured_placement_.origin    = "configuration";
            executor.configured_placement_.sequence  = 1;
            executor.configured_placement_pending_   = true;
        } else {
            err = node.path() + " has unknown element " + name;
            return std::nullopt;
        }
    }

    if (!seen_estimator) {
        err = node.path() + ": needs exactly one Estimator; select an implementation or "
              "the noop";
        return std::nullopt;
    }
    const FunctionKey estimator_type{estimator_node.attr("type")};
    if (estimator_type.empty()) {
        err = estimator_node.path() + ": needs an explicit type";
        return std::nullopt;
    }
    const StateEstimatorMakeFunction* estimator_factory =
        functions.find<StateEstimatorMakeFunction>(estimator_type, err);
    if (estimator_factory == nullptr) {
        err = estimator_node.path() + ": " + err;
        return std::nullopt;
    }
    StateEstimatorInitializationContext estimator_context;
    estimator_context.observations = executor.outputs_;
    estimator_context.functions    = &functions;
    estimator_context.warnings     = warnings;
    executor.estimator_ = (*estimator_factory)(estimator_node, estimator_context, err);
    if (executor.estimator_ == nullptr) {
        if (err.empty()) {
            err = estimator_node.path() + ": factory produced no estimator";
        }
        return std::nullopt;
    }
    executor.estimator_type_  = estimator_type.value;
    executor.estimator_label_ = "Localization/" + estimator_type.value;
    executor.feed_            = std::make_shared<RobotStateFeed>(history);
    return executor;
}

} // namespace navigatr
