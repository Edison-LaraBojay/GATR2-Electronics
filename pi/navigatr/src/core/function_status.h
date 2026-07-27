// function_status.h
// Runtime result of one pipeline function. Configuration problems are init
// errors and never reach here; at runtime a function either worked, had
// nothing new to work on, or hit a fault.

#pragma once
#include <cstdint>

namespace navigatr
{

enum class FunctionStatus : uint8_t {
    kOk = 0,
    kNoData,   // required data absent or stale this cycle, skipped cleanly
    kFault,    // ran and hit an error
};

inline const char* functionStatusName(FunctionStatus s) {
    switch (s) {
    case FunctionStatus::kOk: return "ok";
    case FunctionStatus::kNoData: return "no_data";
    case FunctionStatus::kFault: return "fault";
    }
    return "unknown";
}

inline FunctionStatus worseOf(FunctionStatus a, FunctionStatus b) { return a > b ? a : b; }

} // namespace navigatr
