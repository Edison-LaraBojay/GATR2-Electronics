// sim_source.cpp

#include "receiver/sim/sim_source.h"

#include <algorithm>
#include <cstdio>
#include <string>

namespace spectagatr::receiver::sim
{

namespace
{

using sources::DurationUs;
using sources::ReceiverUs;

constexpr DurationUs kCatchUpUs = 1000000; // further behind than this: skip ahead, no burst

void report(const sources::SourceHooks& hooks, const SimConfig& config, sources::PortState state) {
    char detail[96];
    std::snprintf(detail, sizeof(detail), "sim: %s, %.4g Hz, seed %lu", simModeName(config.mode),
                  config.rate_hz, static_cast<unsigned long>(config.seed));
    sources::SourceStatus status;
    status.src    = sources::SourceKind::kSim;
    status.state  = state;
    status.at_us  = hooks.clock();
    status.detail = detail;
    hooks.on_status(status);
}

} // namespace

void runSimSource(const SimConfig& config, const sources::SourceHooks& hooks,
                  platform::StopSignal& stop) {
    report(hooks, config, sources::PortState::kOpen);
    const double     period_us = 1e6 / std::clamp(config.rate_hz, kMinRateHz, kMaxRateHz);
    const ReceiverUs start_us  = hooks.clock();
    Tick             k         = 0;
    while (!stop.requested()) {
        const auto due_us = start_us + static_cast<ReceiverUs>(period_us * static_cast<double>(k));
        const ReceiverUs now_us = hooks.clock();
        if (now_us < due_us) {
            const auto wait_ms = std::max<DurationUs>(1, (due_us - now_us) / 1000);
            if (stop.waitFor(platform::Milliseconds(wait_ms))) {
                break;
            }
            continue;
        }
        sources::SourceChunk chunk;
        chunk.src   = sources::SourceKind::kSim;
        chunk.rx_us = now_us;
        chunk.bytes = simBytes(config, k);
        if (!chunk.bytes.empty()) {
            hooks.on_chunk(chunk);
        }
        ++k;
        if (now_us - due_us > kCatchUpUs) {
            k = static_cast<Tick>(static_cast<double>(now_us - start_us) / period_us);
        }
    }
    report(hooks, config, sources::PortState::kStopped);
}

} // namespace spectagatr::receiver::sim
