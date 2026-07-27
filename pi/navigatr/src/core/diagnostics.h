// diagnostics.h
// Runtime counters per function slot and per link. Cheap every cycle,
// printed at shutdown or on demand.

#pragma once
#include <cstdint>
#include <map>
#include <string>

#include "core/function_status.h"

namespace navigatr
{

struct FunctionStats {
    uint32_t       runs    = 0;
    uint32_t       ok      = 0;
    uint32_t       no_data = 0;
    uint32_t       fault   = 0;
    FunctionStatus last    = FunctionStatus::kOk;
};

struct LinkStats {
    uint32_t bytes         = 0;
    uint32_t packets       = 0;
    uint32_t decode_errors = 0;
    uint32_t seq_gaps      = 0;
};

struct Diagnostics {
    std::map<std::string, FunctionStats> functions;
    std::map<std::string, LinkStats>     links;
    uint64_t                             cycles = 0;

    void note(const std::string& label, FunctionStatus s) {
        FunctionStats& f = functions[label];
        ++f.runs;
        f.last = s;
        switch (s) {
        case FunctionStatus::kOk: ++f.ok; break;
        case FunctionStatus::kNoData: ++f.no_data; break;
        case FunctionStatus::kFault: ++f.fault; break;
        }
    }
};

} // namespace navigatr
