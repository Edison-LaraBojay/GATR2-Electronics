// configured_collection.cpp

#include "impl/preprocessing/configured_collection.h"

#include "core/diagnostics.h"

namespace navigatr
{

std::unique_ptr<Preprocessing> ConfiguredCollection::create(
    const ConfigNode& node, PreprocessorInitializationContext& context, std::string& err) {
    if (context.functions == nullptr) {
        err = "configured_collection needs the function registry";
        return nullptr;
    }
    auto collection = std::make_unique<ConfiguredCollection>();

    bool ok = true;
    node.forEach("Preprocessor", [&](const ConfigNode& p) {
        if (!ok) {
            return;
        }
        const PreprocessorId id{p.attr("id")};
        const FunctionKey    type{p.attr("type")};
        if (id.empty() || type.empty()) {
            err = p.path() + ": Preprocessor needs id and type";
            ok  = false;
            return;
        }
        for (const auto& seen : collection->executables_) {
            if (seen->id() == id) {
                err = p.path() + ": duplicate Preprocessor id " + id.value;
                ok  = false;
                return;
            }
        }
        const PreprocessorMakeFunction* factory =
            context.functions->find<PreprocessorMakeFunction>(type, err);
        if (factory == nullptr) {
            err = p.path() + ": " + err;
            ok  = false;
            return;
        }
        auto built = (*factory)(p, context, err);
        if (built == nullptr) {
            ok = false;
            return;
        }
        for (const ArtifactOutputDecl& out : built->outputs()) {
            for (const auto& seen : collection->executables_) {
                for (const ArtifactOutputDecl& other : seen->outputs()) {
                    if (other.id == out.id) {
                        err = p.path() + ": duplicate artifact output id " + out.id.value;
                        ok  = false;
                        return;
                    }
                }
            }
        }
        collection->executables_.push_back(std::move(built));
    });
    if (!ok) {
        return nullptr;
    }
    if (collection->executables_.empty()) {
        err = node.path() + ": configured_collection has no Preprocessor children; "
              "doing nothing is type=\"preprocessing/noop\"";
        return nullptr;
    }
    return collection;
}

PreprocessingOutput ConfiguredCollection::run(const PreprocessingInput& in) {
    PreprocessingOutput out;
    for (auto& executable : executables_) {
        const FunctionStatus s = executable->run(in, out.artifacts);
        if (in.diagnostics != nullptr) {
            in.diagnostics->note("Preprocessor/" + executable->id().value, s);
        }
        out.status = worseOf(out.status, s);
    }
    return out;
}

std::vector<ArtifactOutputDecl> ConfiguredCollection::produces() const {
    std::vector<ArtifactOutputDecl> out;
    for (const auto& executable : executables_) {
        for (const ArtifactOutputDecl& decl : executable->outputs()) {
            out.push_back(decl);
        }
    }
    return out;
}

void ConfiguredCollection::reset() {
    for (auto& executable : executables_) {
        executable->reset();
    }
}

} // namespace navigatr
