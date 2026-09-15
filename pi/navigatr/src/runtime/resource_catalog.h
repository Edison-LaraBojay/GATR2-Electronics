// resource_catalog.h
// Initialization-time view of the executable resources: which resource ids
// produce outputs, which output ids exist under each, and the payload each
// publishes. Sensors bind here once by (resource id, output id) with the
// payload type they expect, so a reference to a missing output or an
// incompatible payload dies at build and runtime reads are a map find with
// a typed cast.

#pragma once
#include <string>
#include <typeindex>
#include <vector>

#include "contracts/resource.h"
#include "core/ids.h"
#include "core/payload_descriptor.h"
#include "core/records.h"

namespace navigatr
{

// Resolved typed reference to one resource output's record.
template <typename T>
struct TypedOutputBinding {
    ResourceId resource;
    OutputId   output;

    const MeasurementRecord* record(const ResourceMap& resources) const {
        const auto r = resources.find(resource);
        if (r == resources.end()) {
            return nullptr;
        }
        const auto o = r->second.outputs.find(output);
        return o == r->second.outputs.end() ? nullptr : &o->second;
    }

    const StoredSample* stored(const ResourceMap& resources) const {
        const MeasurementRecord* rec = record(resources);
        if (rec == nullptr || !rec->latest.has_value()) {
            return nullptr;
        }
        return &*rec->latest;
    }

    const T* sample(const ResourceMap& resources) const {
        const StoredSample* s = stored(resources);
        return s == nullptr ? nullptr : s->payload.template get<T>();
    }
};

class ResourceCatalog
{
public:
    // A resource with an executable, listing what it publishes. False on a
    // duplicate resource or a duplicate output id within it.
    bool add(const ResourceId& id, const std::vector<ResourceOutputDecl>& outputs) {
        if (find(id) != nullptr) {
            return false;
        }
        for (std::size_t i = 0; i < outputs.size(); ++i) {
            for (std::size_t j = 0; j < i; ++j) {
                if (outputs[i].id == outputs[j].id) {
                    return false;
                }
            }
        }
        entries_.push_back(Entry{id, outputs});
        return true;
    }

    bool has(const ResourceId& id) const { return find(id) != nullptr; }

    const std::vector<ResourceOutputDecl>* outputsOf(const ResourceId& id) const {
        const Entry* e = find(id);
        return e == nullptr ? nullptr : &e->outputs;
    }

    const PayloadDescriptor* payloadOf(const ResourceId& id, const OutputId& output) const {
        const Entry* e = find(id);
        if (e == nullptr) {
            return nullptr;
        }
        for (const ResourceOutputDecl& d : e->outputs) {
            if (d.id == output) {
                return &d.payload;
            }
        }
        return nullptr;
    }

    // Fails when the resource is unknown or has no outputs, the output id is
    // unknown, or it publishes a different payload than T.
    template <typename T>
    bool bind(const ResourceId& id, const OutputId& output, const std::string& who,
              TypedOutputBinding<T>& out, std::string& err) const {
        const Entry* e = find(id);
        if (e == nullptr) {
            err = who + " references resource " + id.value +
                  " which is not configured or publishes no outputs";
            return false;
        }
        const PayloadDescriptor* payload = payloadOf(id, output);
        if (payload == nullptr) {
            err = who + " references output " + output.value + " which resource " +
                  id.value + " does not publish";
            return false;
        }
        if (!payload->matches(std::type_index(typeid(T)))) {
            err = who + " requires a different payload than " + id.value + "." +
                  output.value + " publishes (" + payload->stable_name + ")";
            return false;
        }
        out.resource = id;
        out.output   = output;
        return true;
    }

    std::size_t size() const { return entries_.size(); }

private:
    struct Entry {
        ResourceId                      id;
        std::vector<ResourceOutputDecl> outputs;
    };

    const Entry* find(const ResourceId& id) const {
        for (const Entry& e : entries_) {
            if (e.id == id) {
                return &e;
            }
        }
        return nullptr;
    }

    std::vector<Entry> entries_;
};

} // namespace navigatr
