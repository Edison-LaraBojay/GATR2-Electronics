// resource_map.cpp

#include "resources/resource_map.h"

namespace navigatr
{

const ResourceInstance* ResourceMap::findValue(const ResourceId& id) const {
    for (const Record& r : records_) {
        if (r.id == id) {
            return &r.value;
        }
    }
    return nullptr;
}

ResourceMapBuilder::Definition* ResourceMapBuilder::findDefinition(
    const ResourceId& id) {
    for (Definition& d : definitions_) {
        if (d.id == id) {
            return &d;
        }
    }
    return nullptr;
}

bool ResourceMapBuilder::index(const ConfigNode& node, std::string& err) {
    Definition def;
    def.id   = ResourceId{node.attr("id")};
    def.type = FunctionKey{node.attr("type")};
    def.node = node;
    if (def.id.empty() || def.type.empty()) {
        err = node.path() + ": Resource needs id and type";
        return false;
    }
    if (findDefinition(def.id) != nullptr) {
        err = node.path() + ": duplicate Resource id " + def.id.value;
        return false;
    }
    if (!functions_.has<ResourceMakeFunction>(def.type)) {
        err = node.path() + ": Resource " + def.id.value + " has unknown type " +
              def.type.value;
        return false;
    }
    definitions_.push_back(std::move(def));
    return true;
}

bool ResourceMapBuilder::overrideFactory(const ResourceId& id,
                                           ResourceMakeFunction factory,
                                           std::string&         err) {
    Definition* def = findDefinition(id);
    if (def == nullptr) {
        err = "override names resource " + id.value + " which is not declared";
        return false;
    }
    def->override_factory = std::move(factory);
    return true;
}

bool ResourceMapBuilder::declared(const ResourceId& id) const {
    for (const Definition& d : definitions_) {
        if (d.id == id) {
            return true;
        }
    }
    return false;
}

const ResourceInstance* ResourceMapBuilder::resolve(const ResourceId& id,
                                                   std::string&      err) {
    Definition* def = findDefinition(id);
    if (def == nullptr) {
        err = "reference to undeclared resource " + id.value;
        return nullptr;
    }
    if (def->state == State::kBuilt) {
        return store_.findValue(id);
    }
    if (def->state == State::kFailed) {
        err = "resource " + id.value + " failed to initialize";
        return nullptr;
    }
    if (def->state == State::kBuilding) {
        std::string cycle;
        for (const std::string& step : build_stack_) {
            cycle += step + " -> ";
        }
        cycle += id.value;
        err = "resource dependency cycle: " + cycle;
        return nullptr;
    }

    def->state = State::kBuilding;
    build_stack_.push_back(id.value);

    ResourceInitializationContext context;
    context.resolver  = this;
    context.functions = &functions_;
    context.warnings  = warnings_;

    ResourceInstance value;
    std::string   build_err;
    if (def->override_factory) {
        value = def->override_factory(def->node, context, build_err);
    } else {
        const ResourceMakeFunction* factory =
            functions_.find<ResourceMakeFunction>(def->type, build_err);
        if (factory != nullptr) {
            value = (*factory)(def->node, context, build_err);
        }
    }

    build_stack_.pop_back();

    if (value.empty()) {
        def->state = State::kFailed;
        err        = def->node.path() + ": " + build_err;
        return nullptr;
    }

    def->state = State::kBuilt;
    store_.records_.push_back(
        ResourceMap::Record{def->id, def->type, std::move(value)});
    return store_.findValue(id);
}

bool ResourceMapBuilder::buildAll(std::string& err) {
    for (Definition& def : definitions_) {
        if (def.state == State::kBuilt) {
            continue;
        }
        if (resolve(def.id, err) == nullptr) {
            return false;
        }
    }
    return true;
}

} // namespace navigatr
