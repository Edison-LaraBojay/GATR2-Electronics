// brain_usb.cpp

#include "receiver/sources/brain_usb.h"

#include <utility>

namespace spectagatr::receiver::sources
{

namespace
{

constexpr std::size_t kUsbHoldbackBytes = 512;

void report(const SourceHooks& hooks, const Port& port, const TransportStats& stats) {
    SourceStatus status;
    status.src    = SourceKind::kBrainUsb;
    status.state  = PortState::kOpen;
    status.at_us  = hooks.clock();
    status.port   = port.path;
    status.detail = "brain: user port " + port.name;
    status.stats  = stats;
    hooks.on_status(status);
}

} // namespace

std::optional<CloseReason> runBrainUsbSession(Port& port, const BrainUsbConfig& config,
                                              const SourceHooks& hooks, platform::StopSignal& stop,
                                              TransportStats& stats) {
    report(hooks, port, stats);
    Bytes      buffer(config.read_bytes);
    ReceiverUs reported_us = hooks.clock();
    bool       after_512   = false;
    while (!stop.requested()) {
        const IoResult result = port.read(buffer.data(), buffer.size(), config.read_wait);
        if (result.status == IoStatus::kClosed) {
            return result.error;
        }
        if (result.status == IoStatus::kOk && result.bytes > 0) {
            ++stats.reads;
            after_512 = result.bytes % kUsbHoldbackBytes == 0;
            if (after_512) {
                ++stats.reads_of_512;
            }
            SourceChunk chunk;
            chunk.src   = SourceKind::kBrainUsb;
            chunk.rx_us = hooks.clock();
            chunk.bytes.assign(buffer.begin(),
                               buffer.begin() + static_cast<std::ptrdiff_t>(result.bytes));
            hooks.on_chunk(chunk);
        } else if (after_512) {
            ++stats.holdback_suspect;
            after_512 = false;
        }
        if (hooks.clock() - reported_us >= config.stats_period_us) {
            reported_us = hooks.clock();
            report(hooks, port, stats);
        }
    }
    return std::nullopt;
}

} // namespace spectagatr::receiver::sources
