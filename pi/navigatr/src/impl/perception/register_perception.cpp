// register_perception.cpp

#include "impl/noop/noops.h"
#include "impl/perception/apriltag_tag_observation.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_perception(FunctionRegistry& functions) {
    registerOrDie<PerceptionMakeFunction>(functions, "noop", &makeNoopPerception);
    registerOrDie<PerceptionMakeFunction>(functions, "apriltag_tag_observation",
                                          &AprilTagObservationPerception::create);
}

} // namespace navigatr
