// resource_stage.cpp

#include "runtime/resource_stage.h"

#include "core/diagnostics.h"
#include "impl/resources/serial_links.h"
#include "runtime/record_bookkeeping.h"

namespace navigatr
{

ResourceExecutor::ResourceExecutor(ResourceFunctions functions, ResourceCatalog catalog)
    : functions_(std::move(functions)), catalog_(std::move(catalog)) {
    // every declared output is addressable from the start; a record with no
    // sample means nothing arrived yet, a missing id means not configured
    for (const ResourceFunctions::Entry& entry : functions_.executionOrder()) {
        if (!entry.executable.execute) {
            continue;
        }
        ResourceRecord& record = retained_[entry.id];
        for (const ResourceOutputDecl& decl : entry.executable.outputs) {
            record.outputs[decl.id] = MeasurementRecord{};
        }
    }
}

const ResourceMap& ResourceExecutor::operator()(const ExecutionContext& context) {
    for (ResourceFunctions::Entry& entry : functions_.executionOrder()) {
        if (!entry.executable.execute) {
            continue;
        }
        ResourcePollResult poll   = entry.executable.execute(context);
        ResourceRecord&    record = retained_[entry.id];
        record.state              = poll.state;
        record.lastPolledAt       = context.now;
        record.diagnostic         = std::move(poll.diagnostic);

        FunctionStatus status = statusOf(record.state);
        for (OutputPoll& output : poll.outputs) {
            const ResourceOutputDecl* decl = nullptr;
            for (const ResourceOutputDecl& d : entry.executable.outputs) {
                if (d.id == output.id) {
                    decl = &d;
                    break;
                }
            }
            if (decl == nullptr) {
                if (context.diagnostics != nullptr) {
                    context.diagnostics->note(
                        entry.label + "/undeclared_output:" + output.id.value,
                        FunctionStatus::kFault);
                }
                status = FunctionStatus::kFault;
                continue;
            }
            MeasurementRecord& out = record.outputs[decl->id];
            storePoll(out, std::move(output.result), decl->payload, context.now);
            if (out.state == SourceState::kFault) {
                status = FunctionStatus::kFault;
            }
        }
        if (context.diagnostics != nullptr) {
            context.diagnostics->note(entry.label, status);
        }
    }
    return retained_;
}

void ResourceExecutor::reset() {
    for (ResourceFunctions::Entry& entry : functions_.executionOrder()) {
        if (entry.executable.reset) {
            entry.executable.reset();
        }
        const auto it = retained_.find(entry.id);
        if (it == retained_.end()) {
            continue;
        }
        it->second.state      = SourceState::kNoDataYet;
        it->second.diagnostic = {};
        it->second.lastPolledAt = MonotonicTime{};
        for (auto& output : it->second.outputs) {
            resetRecord(output.second);
        }
    }
}

std::optional<ResourceBuild> make_resources(const ConfigNode&         node,
                                            const FunctionRegistry&   functions,
                                            const BuildOptions&       options,
                                            std::vector<std::string>* warnings,
                                            std::string&              err) {
    ResourceStoreBuilder builder(functions, warnings);

    // index everything first so declaration order never matters
    if (node.valid()) {
        bool ok = true;
        for (ConfigNode c = node.child(); c.valid(); c = c.next()) {
            if (std::string(c.name()) != "Resource") {
                err = node.path() + " has unknown element " + c.name();
                return std::nullopt;
            }
            if (ok) {
                ok = builder.index(c, err);
            }
        }
        if (!ok) {
            return std::nullopt;
        }
    }
    for (const auto& kv : options.replay) {
        if (!builder.overrideFactory(ResourceId{kv.first},
                                     fileReplayFactoryForPath(kv.second), err)) {
            return std::nullopt;
        }
    }
    if (!builder.buildAll(err)) {
        return std::nullopt;
    }

    ResourceBuild     built;
    built.store = builder.take();

    ResourceFunctions executables;
    ResourceCatalog   catalog;
    for (const ResourceStore::Record& record : built.store.records()) {
        const std::optional<ResourceExecutable>& executable = record.value.executable();
        if (!executable.has_value()) {
            continue;   // configuration only: bound directly, never polled
        }
        for (const ResourceOutputDecl& decl : executable->outputs) {
            if (decl.id.empty()) {
                err = "Resource " + record.id.value + " declares an output without an id";
                return std::nullopt;
            }
        }
        if (!catalog.add(record.id, executable->outputs)) {
            err = "Resource " + record.id.value + " declares a duplicate output id";
            return std::nullopt;
        }
        executables.add(record.id, *executable, "Resource/" + record.id.value);
    }
    built.execute = ResourceExecutor(std::move(executables), std::move(catalog));
    return built;
}

} // namespace navigatr
