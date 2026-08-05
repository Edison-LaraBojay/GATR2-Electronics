// register_preprocessing.cpp

#include "impl/noop/noops.h"
#include "impl/preprocessing/configured_collection.h"
#include "impl/preprocessing/imu_normalization.h"
#include "impl/preprocessing/tracking_wheel_odometry.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_preprocessing(FunctionRegistry& functions) {
    registerOrDie<PreprocessingMakeFunction>(functions, "noop", &makeNoopPreprocessing);
    registerOrDie<PreprocessingMakeFunction>(functions, "configured_collection",
                                             &ConfiguredCollection::create);
    registerOrDie<PreprocessorMakeFunction>(functions, "tracking_wheel_odometry",
                                            &TrackingWheelOdometry::create);
    registerOrDie<PreprocessorMakeFunction>(functions, "imu_normalization",
                                            &ImuNormalization::create);
}

} // namespace navigatr
