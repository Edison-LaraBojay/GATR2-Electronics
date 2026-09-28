// perf_sys.h
// Machine and process CPU use over an interval, so every scenario records
// how loaded the host was while it ran.

#pragma once
#include <cstdint>

namespace navigatr
{
namespace perf
{

struct CpuSample {
    uint64_t total   = 0;   // all cores, busy plus idle
    uint64_t idle    = 0;
    uint64_t process = 0;   // this process, same units as total
    bool     valid   = false;
};

CpuSample sampleCpu();

// Percent of all cores busy between two samples; negative when unknown.
double machineBusyPercent(const CpuSample& a, const CpuSample& b);

// This process as a percent of all cores between two samples.
double processPercent(const CpuSample& a, const CpuSample& b);

} // namespace perf
} // namespace navigatr
