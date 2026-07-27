// field_map_resource.cpp

#include "impl/resources/field_map_resource.h"

namespace navigatr
{

ResourceValue make_field_map(const ConfigNode& node, ResourceInitializationContext&,
                             std::string& err) {
    auto map = std::make_shared<FieldMap>();
    if (!parseFieldMap(node, *map, err)) {
        return ResourceValue{};
    }
    return ResourceValue::asContract<const FieldMap>(std::move(map));
}

} // namespace navigatr
