// register_all.cpp
// Aggregator only; each register_* lives beside its implementations.

#include "runtime/register_all.h"

namespace navigatr
{

void registerAll(FunctionRegistry& functions) {
    register_resources(functions);
    register_sensors(functions);
    register_commands(functions);
    register_localization(functions);
    register_world_estimation(functions);
    register_target_resolution(functions);
    register_publishers(functions);
}

} // namespace navigatr
