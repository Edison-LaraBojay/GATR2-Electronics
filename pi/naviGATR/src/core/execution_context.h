// execution_context.h
// What every stage invocation receives: the host clock, the cycle identity,
// and the diagnostics sink. The clock is when the stage ran, never a
// measurement time; measurement times ride inside the records.

#pragma once
#include <cstdint>

#include "core/time.h"

namespace navigatr
{

struct Diagnostics;

struct ExecutionContext {
    MonotonicTime now;   // host clock
    uint64_t      cycle       = 0;
    Diagnostics*  diagnostics = nullptr;
};

} // namespace navigatr
