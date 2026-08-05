// register_pose_correction.cpp
// Fixed-landmark pose correction lands here when association exists.

#include "impl/noop/noops.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_pose_correction(FunctionRegistry& functions) {
    registerOrDie<PoseCorrectionMakeFunction>(functions, "noop", &makeNoopPoseCorrection);
}

} // namespace navigatr
