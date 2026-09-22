// register_world_estimation.cpp
// The selectable world estimators. Adding one means its implementation, a
// line here, and a build entry; the coordinator and the stage builder stay
// as they are.

#include "impl/noop/noops.h"
#include "impl/world_estimation/apriltag_world_estimator.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_world_estimation(FunctionRegistry& functions) {
    registerOrDie<FieldEstimationMakeFunction>(functions, "noop", &makeNoopFieldEstimation);
    registerOrDie<FieldEstimationMakeFunction>(functions, "apriltag",
                                               &AprilTagWorldEstimator::create);
}

} // namespace navigatr
