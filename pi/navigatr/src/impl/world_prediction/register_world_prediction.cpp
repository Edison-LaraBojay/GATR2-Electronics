// register_world_prediction.cpp

#include "impl/noop/noops.h"
#include "impl/world_prediction/landmark_map.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_world_prediction(FunctionRegistry& functions) {
    registerOrDie<WorldPredictionMakeFunction>(functions, "noop", &makeNoopWorldPrediction);
    registerOrDie<WorldPredictionMakeFunction>(functions, "landmark_map",
                                               &LandmarkMapWorldPrediction::create);
}

} // namespace navigatr
