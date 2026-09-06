// register_field_estimation.cpp

#include "impl/noop/noops.h"
#include "impl/field_estimation/landmark_field.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_field_estimation(FunctionRegistry& functions) {
    registerOrDie<FieldEstimationMakeFunction>(functions, "noop",
                                               &makeNoopFieldEstimation);
    registerOrDie<FieldEstimationMakeFunction>(functions, "landmark_field",
                                               &LandmarkFieldEstimation::create);
}

} // namespace navigatr
