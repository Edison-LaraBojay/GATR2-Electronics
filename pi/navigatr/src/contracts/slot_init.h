// slot_init.h
// What every pipeline slot factory receives at construction: the initialized
// resources, the sensor catalog, and the typed declarations accumulated from
// earlier slots. References resolve here, once, with payload compatibility
// checked; runtime never reparses configuration or rediscovers dependencies.
// Implementations receive only these views, never the mutable System.

#pragma once
#include <string>
#include <typeindex>
#include <vector>

#include "contracts/preprocessing.h"
#include "core/function_registry.h"
#include "core/ids.h"
#include "core/payload_descriptor.h"

namespace navigatr
{

class ResourceStore;
class SensorCatalog;

struct ObservationOutputDecl {
    ObservationId     id;
    PayloadDescriptor payload;
};

struct AssociationOutputDecl {
    AssociationId     id;
    PayloadDescriptor payload;
};

struct SlotInitializationContext {
    const ResourceStore*    resources = nullptr;
    const SensorCatalog*    sensors   = nullptr;
    const FunctionRegistry* functions = nullptr;

    std::vector<ArtifactOutputDecl>    artifacts;      // filled after Preprocessing
    std::vector<ObservationOutputDecl> observations;   // filled after Perception
    std::vector<AssociationOutputDecl> associations;   // filled after Association

    std::vector<std::string>* warnings = nullptr;

    // Reference checks: the id must be declared by an earlier producer and,
    // when expected is given, publish that exact payload type.
    bool requireArtifact(const ArtifactId& id, const std::type_index* expected,
                         const std::string& who, std::string& err) const {
        for (const ArtifactOutputDecl& d : artifacts) {
            if (d.id == id) {
                return checkPayload(d.payload, expected, who, "artifact", id.value, err);
            }
        }
        err = who + " references artifact " + id.value + " which nothing produces";
        return false;
    }

    bool requireAssociation(const AssociationId& id, const std::type_index* expected,
                            const std::string& who, std::string& err) const {
        for (const AssociationOutputDecl& d : associations) {
            if (d.id == id) {
                return checkPayload(d.payload, expected, who, "association", id.value,
                                    err);
            }
        }
        err = who + " references association " + id.value + " which nothing produces";
        return false;
    }

private:
    static bool checkPayload(const PayloadDescriptor& found,
                             const std::type_index* expected, const std::string& who,
                             const char* what, const std::string& id, std::string& err) {
        if (expected != nullptr && !found.matches(*expected)) {
            err = who + " requires a different payload than " + what + " " + id +
                  " produces (" + found.stable_name + ")";
            return false;
        }
        return true;
    }
};

} // namespace navigatr
