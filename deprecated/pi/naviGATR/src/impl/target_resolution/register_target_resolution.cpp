// register_target_resolution.cpp

#include "impl/noop/noops.h"
#include "impl/target_resolution/configured_targets.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_target_resolution(FunctionRegistry& functions) {
    registerOrDie<TargetResolutionMakeFunction>(functions, "noop",
                                                &makeNoopTargetResolution);
    registerOrDie<TargetResolutionMakeFunction>(functions, "configured_targets",
                                                &ConfiguredTargetResolution::create);
}

} // namespace navigatr
