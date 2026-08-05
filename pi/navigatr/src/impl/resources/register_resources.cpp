// register_resources.cpp

#include "impl/resources/field_map_resource.h"
#include "impl/resources/pico_telemetry.h"
#include "impl/resources/serial_links.h"
#include "resources/resource_map.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_resources(FunctionRegistry& functions) {
    registerOrDie<ResourceMakeFunction>(functions, "linux_serial_link",
                                        &make_linux_serial_link);
    registerOrDie<ResourceMakeFunction>(functions, "memory_link", &make_memory_link);
    registerOrDie<ResourceMakeFunction>(functions, "file_replay_link",
                                        &make_file_replay_link);
    registerOrDie<ResourceMakeFunction>(functions, "pico_telemetry", &make_pico_telemetry);
    registerOrDie<ResourceMakeFunction>(functions, "field_map", &make_field_map);
}

} // namespace navigatr
