// sensor_stage.cpp

#include "runtime/sensor_stage.h"

#include "core/diagnostics.h"
#include "runtime/record_bookkeeping.h"

namespace navigatr
{

SensorExecutor::SensorExecutor(SensorFunctions functions, SensorCatalog catalog)
    : functions_(std::move(functions)), catalog_(std::move(catalog)) {
    for (const SensorFunctions::Entry& entry : functions_.executionOrder()) {
        retained_[entry.id] = MeasurementRecord{};
    }
}

const SensorMap& SensorExecutor::operator()(const ResourceMap&      resources,
                                            const ExecutionContext& context) {
    for (SensorFunctions::Entry& entry : functions_.executionOrder()) {
        PollResult         poll   = entry.sensor.execute(resources, context);
        MeasurementRecord& record = retained_[entry.id];
        storePoll(record, std::move(poll), entry.sensor.outputPayload, context.now);
        if (context.diagnostics != nullptr) {
            context.diagnostics->note(entry.label, statusOf(record.state));
        }
    }
    return retained_;
}

void SensorExecutor::reset() {
    for (SensorFunctions::Entry& entry : functions_.executionOrder()) {
        if (entry.sensor.reset) {
            entry.sensor.reset();
        }
        resetRecord(retained_[entry.id]);
    }
}

std::optional<SensorExecutor> make_sensors(const ConfigNode&         node,
                                           const FunctionRegistry&   functions,
                                           const ResourceStore&      store,
                                           const ResourceCatalog&    resource_outputs,
                                           std::vector<std::string>* warnings,
                                           std::string&              err) {
    SensorFunctions executables;
    SensorCatalog   catalog;
    if (!node.valid()) {
        return SensorExecutor(std::move(executables), std::move(catalog));
    }

    SensorInitializationContext context;
    context.resources = &store;
    context.outputs   = &resource_outputs;
    context.functions = &functions;
    context.warnings  = warnings;

    for (ConfigNode s = node.child(); s.valid(); s = s.next()) {
        if (std::string(s.name()) != "Sensor") {
            err = node.path() + " has unknown element " + s.name();
            return std::nullopt;
        }
        const SensorId    id{s.attr("id")};
        const FunctionKey type{s.attr("type")};
        if (id.empty() || type.empty()) {
            err = s.path() + ": Sensor needs id and type";
            return std::nullopt;
        }
        if (executables.find(id) != nullptr) {
            err = s.path() + ": duplicate Sensor id " + id.value;
            return std::nullopt;
        }
        const SensorMakeFunction* factory = functions.find<SensorMakeFunction>(type, err);
        if (factory == nullptr) {
            err = s.path() + ": " + err;
            return std::nullopt;
        }
        std::optional<SensorExecutable> built = (*factory)(s, context, err);
        if (!built.has_value() || !built->execute) {
            if (err.empty()) {
                err = s.path() + ": factory produced no executable";
            }
            return std::nullopt;
        }
        catalog.add(id, built->outputPayload);
        executables.add(id, std::move(*built), "Sensor/" + id.value);
    }
    return SensorExecutor(std::move(executables), std::move(catalog));
}

} // namespace navigatr
