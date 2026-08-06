// field_map_resource.cpp

#include "impl/resources/field_map_resource.h"

namespace navigatr
{

ResourceInstance make_field_map(const ConfigNode&              node,
                                ResourceInitializationContext& context,
                                std::string&                   err) {
    auto map = std::make_shared<FieldMap>();
    if (!parseFieldMap(node, context.allow_provisional, *map, err)) {
        return ResourceInstance{};
    }
    return ResourceInstance::asContract<const FieldMap>(std::move(map));
}

} // namespace navigatr
