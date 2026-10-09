// Local IMU health labels shared by the drivers, supervisors and diagnostics.
// These describe acquisition state; they do not define a wire protocol.

#pragma once
#include <stdint.h>

namespace imu
{

enum State : uint8_t {
    kStateDisabled     = 0,
    kStateInitializing = 1,
    kStateAligning     = 2, // startup gravity alignment, hold still and level
    kStateReady        = 3,
    kStateRetrying     = 4, // next attempt after a backoff
    kStateFailed       = 5, // quick attempts exhausted; slow retries continue
};

enum Reason : uint8_t {
    kReasonNone       = 0,
    kReasonNoResponse = 1,
    kReasonBoot       = 2, // boot timeout
    kReasonFeatures   = 3, // report/configuration setup failed
    kReasonStream     = 4, // reports stopped or configuration lost
};

} // namespace imu
