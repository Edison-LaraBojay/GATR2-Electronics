// register_localization.cpp

#include "impl/localization/wheel_imu_prediction.h"
#include "impl/noop/noops.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_localization(FunctionRegistry& functions) {
    registerOrDie<LocalizationMakeFunction>(functions, "noop", &makeNoopLocalization);
    registerOrDie<LocalizationMakeFunction>(functions, "wheel_imu_prediction",
                                            &WheelImuPrediction::create);
}

} // namespace navigatr
