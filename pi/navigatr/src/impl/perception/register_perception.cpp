// register_perception.cpp
// Camera and detector implementations land here when they exist.

#include "impl/noop/noops.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_perception(FunctionRegistry& functions) {
    registerOrDie<PerceptionMakeFunction>(functions, "noop", &makeNoopPerception);
}

} // namespace navigatr
