// world_estimation_stage.cpp

#include "runtime/world_estimation_stage.h"

#include "core/diagnostics.h"

namespace navigatr
{

namespace
{

// Drops entries whose id was never declared or whose payload contradicts
// the declaration, noting a fault against the estimator.
template <typename Map, typename Decls>
void enforceDeclared(Map& map, const Decls& decls, const std::string& label,
                     Diagnostics* diagnostics) {
    for (auto it = map.begin(); it != map.end();) {
        const typename Decls::value_type* decl = nullptr;
        for (const auto& d : decls) {
            if (d.id == it->first) {
                decl = &d;
                break;
            }
        }
        if (decl == nullptr || !decl->payload.matches(it->second.payload.cppType())) {
            if (diagnostics != nullptr) {
                diagnostics->note(label + "/undeclared_output:" + it->first.value,
                                  FunctionStatus::kFault);
            }
            it = map.erase(it);
        } else {
            ++it;
        }
    }
}

template <typename Decls>
bool uniqueDeclaredIds(const Decls& decls, const std::string& path, const char* what,
                       std::string& err) {
    for (std::size_t i = 0; i < decls.size(); ++i) {
        if (decls[i].id.empty()) {
            err = path + ": declares an " + what + " output without an id";
            return false;
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (decls[j].id == decls[i].id) {
                err = path + ": declares " + what + " output " + decls[i].id.value +
                      " more than once";
                return false;
            }
        }
    }
    return true;
}

} // namespace

FieldEstimationOutput WorldEstimationExecutor::operator()(const FieldEstimationInput& in) {
    FieldEstimationOutput out = estimator_->run(in);
    enforceDeclared(out.observations, observations_, label_, in.context.diagnostics);
    enforceDeclared(out.associations, associations_, label_, in.context.diagnostics);
    if (in.context.diagnostics != nullptr) {
        in.context.diagnostics->note(label_, out.status);
    }
    return out;
}

void WorldEstimationExecutor::reset() { estimator_->reset(); }

std::optional<WorldEstimationExecutor> make_world_estimation(const ConfigNode&         node,
                                                             const FunctionRegistry&   functions,
                                                             const SensorCatalog&      sensors,
                                                             const ResourceStore&      resources,
                                                             std::vector<std::string>* warnings,
                                                             std::string&              err) {
    if (!node.valid()) {
        err = "missing WorldEstimation section; configure exactly one Estimator, an "
              "implementation or the noop";
        return std::nullopt;
    }

    ConfigNode estimator_node;
    for (ConfigNode c = node.child(); c.valid(); c = c.next()) {
        if (std::string(c.name()) != "Estimator") {
            err = node.path() + " has unknown element " + c.name();
            return std::nullopt;
        }
        if (estimator_node.valid()) {
            err = node.path() + " has more than one Estimator; combining estimators is "
                  "not supported yet";
            return std::nullopt;
        }
        estimator_node = c;
    }
    if (!estimator_node.valid()) {
        err = node.path() + ": needs exactly one Estimator; select an implementation or "
              "the noop";
        return std::nullopt;
    }

    const std::string id = estimator_node.attr("id");
    const FunctionKey type{estimator_node.attr("type")};
    if (id.empty() || type.empty()) {
        err = estimator_node.path() + ": Estimator needs id and type";
        return std::nullopt;
    }
    const FieldEstimationMakeFunction* factory =
        functions.find<FieldEstimationMakeFunction>(type, err);
    if (factory == nullptr) {
        err = estimator_node.path() + ": " + err;
        return std::nullopt;
    }

    // Every reference the implementation makes resolves here, once, against
    // the initialized sensors and resources.
    SlotInitializationContext context;
    context.resources = &resources;
    context.sensors   = &sensors;
    context.functions = &functions;
    context.warnings  = warnings;

    WorldEstimationExecutor executor;
    executor.estimator_ = (*factory)(estimator_node, context, err);
    if (executor.estimator_ == nullptr) {
        if (err.empty()) {
            err = estimator_node.path() + ": factory produced no estimator";
        }
        return std::nullopt;
    }
    executor.id_           = id;
    executor.type_         = type.value;
    executor.label_        = "WorldEstimation/" + id;
    executor.observations_ = executor.estimator_->producesObservations();
    executor.associations_ = executor.estimator_->producesAssociations();
    if (!uniqueDeclaredIds(executor.observations_, estimator_node.path(), "observation",
                           err) ||
        !uniqueDeclaredIds(executor.associations_, estimator_node.path(), "association",
                           err)) {
        return std::nullopt;
    }
    return executor;
}

} // namespace navigatr
