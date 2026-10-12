// perf_sys.cpp

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#endif

#include "perf_sys.h"

namespace navigatr
{
namespace perf
{

#ifdef _WIN32
namespace
{
uint64_t ticks(const FILETIME& f) {
    return (static_cast<uint64_t>(f.dwHighDateTime) << 32) | f.dwLowDateTime;
}
} // namespace

CpuSample sampleCpu() {
    CpuSample s;
    FILETIME  idle, kernel, user;
    if (!GetSystemTimes(&idle, &kernel, &user)) {
        return s;
    }
    s.idle  = ticks(idle);
    s.total = ticks(kernel) + ticks(user);   // kernel time includes idle
    FILETIME created, exited, pk, pu;
    if (GetProcessTimes(GetCurrentProcess(), &created, &exited, &pk, &pu)) {
        s.process = ticks(pk) + ticks(pu);
    }
    s.valid = true;
    return s;
}
#else
CpuSample sampleCpu() {
    CpuSample     s;
    std::ifstream stat("/proc/stat");
    std::string   cpu;
    uint64_t      v[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    if (!(stat >> cpu) || cpu != "cpu") {
        return s;
    }
    for (uint64_t& x : v) {
        stat >> x;
    }
    // user nice system idle iowait irq softirq steal (guest time is inside user)
    s.idle  = v[3] + v[4];
    s.total = v[0] + v[1] + v[2] + v[3] + v[4] + v[5] + v[6] + v[7];
    std::ifstream self("/proc/self/stat");
    std::string   line;
    if (std::getline(self, line)) {
        const std::size_t close = line.rfind(')');
        if (close != std::string::npos) {
            std::istringstream rest(line.substr(close + 2));
            std::string        field;
            uint64_t           utime = 0, stime = 0;
            for (int i = 3; i <= 15 && (rest >> field); ++i) {
                if (i == 14) {
                    utime = std::stoull(field);
                } else if (i == 15) {
                    stime = std::stoull(field);
                }
            }
            s.process = utime + stime;
        }
    }
    s.valid = true;
    return s;
}
#endif

double machineBusyPercent(const CpuSample& a, const CpuSample& b) {
    if (!a.valid || !b.valid || b.total <= a.total) {
        return -1.0;
    }
    const double total = static_cast<double>(b.total - a.total);
    const double idle  = static_cast<double>(b.idle - a.idle);
    return 100.0 * (total - idle) / total;
}

double processPercent(const CpuSample& a, const CpuSample& b) {
    if (!a.valid || !b.valid || b.total <= a.total) {
        return -1.0;
    }
    return 100.0 * static_cast<double>(b.process - a.process) /
           static_cast<double>(b.total - a.total);
}

} // namespace perf
} // namespace navigatr
