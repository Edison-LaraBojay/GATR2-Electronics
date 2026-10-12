// perf_analysis.h
// Pure computations over recorded samples, apart from the socket code so
// perf_selftest can check them.

#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

namespace navigatr
{
namespace perf
{

// One ping round trip on the client clock, with the server's host stamp
// (NaN when the pong carries none, as inspect/1 control pongs).
struct PongTimes {
    int64_t send_us = 0;
    int64_t recv_us = 0;
    double  host_us = 0.0;
};

// A message the client received, stamped by the server at host_us.
struct Receipt {
    int64_t recv_us = 0;
    double  host_us = 0.0;
};

struct OffsetEstimate {
    double publish_to_receive_ms = 0.0;
    double offset_ms             = 0.0;   // host clock minus client clock
    double half_width_ms         = 0.0;   // RTT/2 of the pong used: the error bound
};

// Spec 3.4: at each receipt, the minimum-RTT pong among the last `window`
// pongs received by then gives offset = host_us - (send_us + recv_us) / 2,
// good to +-RTT/2. One entry per receipt that had a usable pong before it
// (receipts must be in receive order; pongs in receive order too).
std::vector<OffsetEstimate> pingOffsetEstimates(const std::vector<Receipt>&   receipts,
                                                const std::vector<PongTimes>& pongs,
                                                std::size_t                   window = 20);

} // namespace perf
} // namespace navigatr
