// perf_analysis.cpp

#include "perf_analysis.h"

#include <cmath>
#include <deque>

namespace navigatr
{
namespace perf
{

std::vector<OffsetEstimate> pingOffsetEstimates(const std::vector<Receipt>&   receipts,
                                                const std::vector<PongTimes>& pongs,
                                                std::size_t                   window) {
    std::vector<OffsetEstimate> out;
    std::deque<const PongTimes*> recent;
    std::size_t                  pi = 0;
    for (const Receipt& r : receipts) {
        while (pi < pongs.size() && pongs[pi].recv_us <= r.recv_us) {
            if (!std::isnan(pongs[pi].host_us)) {
                recent.push_back(&pongs[pi]);
                if (recent.size() > window) {
                    recent.pop_front();
                }
            }
            ++pi;
        }
        if (recent.empty() || std::isnan(r.host_us)) {
            continue;
        }
        const PongTimes* best = recent.front();
        for (const PongTimes* p : recent) {
            if (p->recv_us - p->send_us < best->recv_us - best->send_us) {
                best = p;
            }
        }
        const double offset_us =
            best->host_us - static_cast<double>(best->send_us + best->recv_us) / 2.0;
        OffsetEstimate e;
        e.offset_ms             = offset_us / 1000.0;
        e.publish_to_receive_ms = (static_cast<double>(r.recv_us) - (r.host_us - offset_us)) / 1000.0;
        e.half_width_ms         = static_cast<double>(best->recv_us - best->send_us) / 2000.0;
        out.push_back(e);
    }
    return out;
}

} // namespace perf
} // namespace navigatr
