// wheel_geometry_resource.cpp

#include "impl/resources/wheel_geometry_resource.h"

#include <memory>

#include "resources/wheel_geometry.h"

namespace navigatr
{

ResourceInstance make_wheel_geometry(const ConfigNode&              node,
                                     ResourceInitializationContext&,
                                     std::string&                   err) {
    auto map = std::make_shared<WheelGeometryMap>();
    bool ok  = true;
    node.forEach("Wheel", [&](const ConfigNode& w) {
        if (!ok) {
            return;
        }
        WheelDecl decl;
        std::string sensor_raw, direction;
        if (!w.requireAttr("id", decl.id, err) ||
            !w.requireAttr("sensor_id", sensor_raw, err)) {
            ok = false;
            return;
        }
        decl.sensor = SensorId{sensor_raw};
        decl.label  = w.attr("label");
        if (map->find(decl.id) != nullptr) {
            err = w.path() + ": duplicate Wheel id " + decl.id;
            ok  = false;
            return;
        }
        for (const auto& seen : map->wheels) {
            if (seen.sensor == decl.sensor) {
                err = w.path() + ": sensor " + decl.sensor.value +
                      " is referenced by more than one Wheel";
                ok = false;
                return;
            }
        }
        if (!w.requireDouble("radius_m", decl.radius_m, err) ||
            !w.requireDouble("position_x_m", decl.position_x_m, err) ||
            !w.requireDouble("position_y_m", decl.position_y_m, err) ||
            !w.requireDouble("measurement_angle_deg", decl.measurement_angle_deg, err) ||
            !w.requireAttr("direction", direction, err)) {
            ok = false;
            return;
        }
        if (decl.radius_m <= 0.0) {
            err = w.path() + ": radius_m must be positive";
            ok  = false;
            return;
        }
        if (direction == "positive") {
            decl.direction_positive = true;
        } else if (direction == "negative") {
            decl.direction_positive = false;
        } else {
            err = w.path() + ": direction must be positive or negative";
            ok  = false;
            return;
        }
        map->wheels.push_back(std::move(decl));
    });
    if (!ok) {
        return ResourceInstance{};
    }
    if (map->wheels.empty()) {
        err = node.path() + ": wheel_geometry needs at least one Wheel";
        return ResourceInstance{};
    }
    return ResourceInstance::asContract<const WheelGeometryMap>(std::move(map));
}

} // namespace navigatr
