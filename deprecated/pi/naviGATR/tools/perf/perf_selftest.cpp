// perf_selftest.cpp
// Checks for the perf tools' pure pieces: the spec 3.4 ping-offset
// estimate and its +-RTT/2 bound, and the summaries. No sockets.

#include <cmath>
#include <cstdio>
#include <vector>

#include "perf_analysis.h"
#include "perf_json.h"

using namespace navigatr::perf;

namespace
{

int failures = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) <= tol; }

// The host clock runs 5 s ahead of the client clock.
constexpr double kOffsetUs = 5e6;

PongTimes pong(int64_t send_us, int64_t rtt_us, int64_t server_after_send_us) {
    PongTimes p;
    p.send_us = send_us;
    p.recv_us = send_us + rtt_us;
    p.host_us = kOffsetUs + static_cast<double>(send_us + server_after_send_us);
    return p;
}

Receipt receipt(int64_t recv_us, double transit_ms) {
    Receipt r;
    r.recv_us = recv_us;
    r.host_us = kOffsetUs + static_cast<double>(recv_us) - transit_ms * 1000.0;
    return r;
}

void symmetricPongGivesTheExactValue() {
    // stamped mid-flight: offset exact, the estimate equals the transit
    const std::vector<PongTimes> pongs{pong(1000, 2000, 1000)};
    const auto e = pingOffsetEstimates({receipt(10000, 1.5)}, pongs);
    check(e.size() == 1, "one estimate");
    check(near(e[0].publish_to_receive_ms, 1.5), "symmetric: exact transit");
    check(near(e[0].offset_ms, kOffsetUs / 1000.0), "symmetric: exact offset");
    check(near(e[0].half_width_ms, 1.0), "half-width is RTT/2");
}

void asymmetricPongStaysInsideTheBound() {
    // server stamped at once (all delay on the way back): error is RTT/2
    const std::vector<PongTimes> pongs{pong(1000, 40000, 0)};
    const auto e = pingOffsetEstimates({receipt(100000, 3.0)}, pongs);
    check(e.size() == 1, "one estimate");
    const double err = e[0].publish_to_receive_ms - 3.0;
    check(near(e[0].half_width_ms, 20.0), "half-width 20 ms");
    check(std::fabs(err) <= e[0].half_width_ms + 1e-9, "error within +-half-width");
    check(std::fabs(err) > 10.0, "a wide pong really is wrong by a lot");
}

void picksTheNarrowestOfTheWindow() {
    const std::vector<PongTimes> pongs{pong(1000, 80000, 0), pong(200000, 2000, 1000),
                                       pong(400000, 60000, 60000)};
    const auto e = pingOffsetEstimates({receipt(500000, 2.0)}, pongs);
    check(e.size() == 1, "one estimate");
    check(near(e[0].half_width_ms, 1.0), "min-RTT pong used");
    check(near(e[0].publish_to_receive_ms, 2.0), "exact through the narrow pong");
}

void windowForgetsOldPongs() {
    // a narrow pong, then 20 wide ones: it leaves the 20-pong window
    std::vector<PongTimes> pongs{pong(0, 2000, 1000)};
    for (int i = 1; i <= 20; ++i) {
        pongs.push_back(pong(i * 100000, 50000, 0));
    }
    const auto e = pingOffsetEstimates({receipt(1000, 1.0), receipt(3000000, 1.0)}, pongs);
    check(e.size() == 1, "no pong before the first receipt: no estimate");
    if (e.size() == 1) {
        check(near(e[0].half_width_ms, 25.0), "narrow pong aged out of the window");
    }
    const auto e2 = pingOffsetEstimates({receipt(3000000, 1.0)}, pongs, 21);
    check(e2.size() == 1 && near(e2[0].half_width_ms, 1.0), "a 21-pong window still has it");
}

void unstampedPongsAreIgnored() {
    PongTimes p = pong(1000, 2000, 1000);
    p.host_us   = NAN;   // an inspect/1 control pong
    const auto e = pingOffsetEstimates({receipt(10000, 1.0)}, {p});
    check(e.empty(), "control pongs give no estimate");
    Receipt r = receipt(10000, 1.0);
    r.host_us = NAN;
    check(pingOffsetEstimates({r}, {pong(1000, 2000, 1000)}).empty(),
          "a receipt without a host stamp gives no estimate");
}

void summariesSkipMissingValues() {
    const Summary s = summarize({NAN, 3.0, 1.0, INFINITY, 2.0});
    check(s.n == 3, "NaN and inf are not samples");
    check(near(s.p50, 2.0) && near(s.max, 3.0) && near(s.min, 1.0), "summary of the rest");
    check(summarize({NAN}).n == 0, "all missing: empty");
}

} // namespace

int main() {
    symmetricPongGivesTheExactValue();
    asymmetricPongStaysInsideTheBound();
    picksTheNarrowestOfTheWindow();
    windowForgetsOldPongs();
    unstampedPongsAreIgnored();
    summariesSkipMissingValues();
    if (failures == 0) {
        std::printf("perf_selftest: all checks passed\n");
        return 0;
    }
    std::printf("perf_selftest: %d check(s) failed\n", failures);
    return 1;
}
