// register_publishers.cpp

#include "impl/noop/noops.h"
#include "impl/publishing/vex_brain.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_publishers(FunctionRegistry& functions) {
    registerOrDie<PublishingMakeFunction>(functions, "noop", &makeNoopPublishing);
    registerOrDie<PublishingMakeFunction>(functions, "vex_brain",
                                          &VexBrainPublisher::create);
}

} // namespace navigatr
