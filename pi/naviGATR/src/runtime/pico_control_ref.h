// pico_control_ref.h
// The PicoControl a <Pico resource_id=.../> element names: the
// pico_telemetry resource (contract PicoTelemetry), used only through its
// PicoControl interface, or a resource stored under PicoControl itself (a
// stand-in, for tests).

#pragma once
#include <memory>
#include <string>

#include "config/config_node.h"
#include "impl/resources/pico_telemetry.h"
#include "resources/pico_control.h"
#include "resources/resource_store.h"

namespace navigatr
{

inline std::shared_ptr<PicoControl> requirePicoControl(const ResourceStore& resources,
                                                       const ResourceId& id, std::string& err) {
    std::string as_telemetry;
    if (std::shared_ptr<PicoTelemetry> t = resources.require<PicoTelemetry>(id, as_telemetry)) {
        return t;
    }
    std::string as_control;
    if (std::shared_ptr<PicoControl> c = resources.require<PicoControl>(id, as_control)) {
        return c;
    }
    err = as_telemetry;
    return nullptr;
}

// An optional <Pico resource_id/> child of node: true with a null out when
// absent.
inline bool parsePicoReference(const ConfigNode& node, const ResourceStore& resources,
                               std::shared_ptr<PicoControl>& out, std::string& err) {
    out = nullptr;
    if (!node.atMostOne("Pico", err)) {
        return false;
    }
    const ConfigNode pico = node.child("Pico");
    if (!pico.valid()) {
        return true;
    }
    std::string id;
    if (!pico.onlyAttributes({"resource_id"}, err) || !pico.onlyChildren({}, err) ||
        !pico.requireAttr("resource_id", id, err)) {
        return false;
    }
    std::string inner;
    out = requirePicoControl(resources, ResourceId{id}, inner);
    if (out == nullptr) {
        err = pico.path() + ": " + inner;
        return false;
    }
    return true;
}

} // namespace navigatr
