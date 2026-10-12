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

    // Brain link request/reply counters.
    uint32_t requests        = 0;   // decoded requests
    uint32_t duplicates      = 0;   // answered from a record, not applied again
    uint32_t superseded      = 0;   // completed in a drain behind a newer request
    uint32_t stale           = 0;   // answered kResultStale
    uint32_t unknown_session = 0;   // answered kResultUnknownSession
    uint32_t unanswered      = 0;   // processed, no reply: first drain or trailing bytes
    uint32_t replies         = 0;   // written
    uint32_t expired         = 0;   // reply window missed, nothing sent
    uint32_t input_pending   = 0;   // input before driver enable, nothing sent
    uint32_t late_release    = 0;   // driver released after the frame budget
    uint32_t tx_errors       = 0;   // other write failures
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
