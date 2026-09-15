// slot_init.h
// What every pipeline slot factory receives at construction: the initialized
// resources, the sensor catalog, the localization's declared observation
// functions and outputs, and the typed declarations accumulated from
// earlier slots. References resolve here, once, with payload compatibility
// checked; runtime never reparses configuration or rediscovers
// dependencies. Implementations receive only these views, never the
// mutable System.

#pragma once
#include <string>
#include <typeindex>
#include <vector>

#include "contracts/localization.h"
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

    // Localization declarations: configured observation function ids and
    // the robot observation outputs they publish.
    std::vector<std::string>                observation_functions;
    std::vector<RobotObservationOutputDecl> robot_observations;

    std::vector<ObservationOutputDecl> observations;   // filled after Perception
    std::vector<AssociationOutputDecl> associations;   // filled after Association

    std::vector<std::string>* warnings = nullptr;

    bool requireObservation(const ObservationId& id, const std::type_index* expected,
                            const std::string& who, std::string& err) const {
        for (const ObservationOutputDecl& d : observations) {
            if (d.id == id) {
                return checkPayload(d.payload, expected, who, "observation", id.value,
                                    err);
            }
        }
        err = who + " references observation " + id.value + " which nothing produces";
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

    bool requireObservationFunction(const std::string& id, const std::string& who,
                                    std::string& err) const {
        for (const std::string& f : observation_functions) {
            if (f == id) {
                return true;
            }
        }
        err = who + " references localization observation function " + id +
              " which is not configured";
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
