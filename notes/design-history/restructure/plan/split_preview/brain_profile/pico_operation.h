// pico_operation.h

#pragma once
#include <cstdint>

#include "core/time.h"

namespace navigatr
{

// A CONTROL 3 or 4 running on the Pico, wire codes.
struct PicoOperation {
    bool          active = false;
    uint8_t       action = 0;   // translagatr::ControlAction
    uint32_t      handle = 0;
    MonotonicTime started;
    uint8_t       acq_epoch = 0;   // at submit, RestartAcquisition
    uint64_t      restarts  = 0;
    uint8_t       result    = 0;   // translagatr::BrainResult, Pending until settled
    uint8_t       detail    = 0;   // translagatr::ControlDetail
};

} // namespace navigatr
