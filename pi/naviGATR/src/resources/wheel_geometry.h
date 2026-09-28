// wheel_geometry.h
// Measured tracking-wheel geometry as immutable shared data, owned by the
// robot description rather than any pipeline fragment. A pipeline selects
// which declared wheels participate by id; two- and three-wheel pipelines
// differ only through those references, never by restating measurements.
//
// Angles stay in degrees here because this is configuration data; the
// consuming tracking-motion observation model converts once when it builds its solve.
// travel_scale is the optional empirical distance correction (default 1);
// the consuming model applies it once: travel = d(wheel angle) * radius * scale.

#pragma once
#include <string>
#include <vector>

#include "core/ids.h"

namespace navigatr
{

struct WheelDecl {
    std::string id;       // opaque wheel identity referenced by pipelines
    SensorId    sensor;   // encoder channel sensor producing its angle
    std::string label;    // diagnostics only
    double      radius_m              = 0.0;
    double      position_x_m          = 0.0;
    double      position_y_m          = 0.0;
    double      measurement_angle_deg = 0.0;
    bool        direction_positive    = true;
    double      travel_scale          = 1.0;   // measured distance correction, 1 = none
};

struct WheelGeometryMap {
    std::vector<WheelDecl> wheels;

    const WheelDecl* find(const std::string& id) const {
        for (const auto& w : wheels) {
            if (w.id == id) {
                return &w;
            }
        }
        return nullptr;
    }
};

} // namespace navigatr
