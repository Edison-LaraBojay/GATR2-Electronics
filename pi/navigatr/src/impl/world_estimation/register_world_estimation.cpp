// register_world_estimation.cpp

#include "impl/noop/noops.h"
#include "impl/world_estimation/landmark_world.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_world_estimation(FunctionRegistry& functions) {
    registerOrDie<WorldEstimationMakeFunction>(functions, "noop",
                                               &makeNoopWorldEstimation);
    registerOrDie<WorldEstimationMakeFunction>(functions, "landmark_world",
                                               &LandmarkWorldEstimation::create);
}

} // namespace navigatr
