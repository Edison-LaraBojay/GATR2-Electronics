// controller.cpp

#include "receiver/sources/controller.h"

#include <array>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>

namespace spectagatr::receiver::sources
{

namespace
{

using ReadBuffer = std::array<Byte, 256>;

constexpr int kMaxDrainReads = 16;

// Which replies answer a request.
struct ReplyKind {
    Byte                command = 0;
    std::optional<Byte> ecmd; // extended replies
};

constexpr ReplyKind kUserReadReply{kCommandExtended, kExtendedUserRead};

DurationUs   microsecondsOf(Milliseconds duration) { return duration.count() * 1000; }
Milliseconds millisecondsAtLeast(DurationUs us) { return Milliseconds((us + 999) / 1000); }

bool answers(const CdcReply& reply, const ReplyKind& kind) {
    if (reply.command != kind.command) {
        return false;
    }
    return !kind.ecmd || (!reply.payload.empty() && reply.payload[0] == *kind.ecmd);
}

void reportStatus(const SourceHooks& hooks, const Port& port, PortState state, std::string detail,
                  const TransportStats& stats) {
    SourceStatus status;
    status.src    = SourceKind::kController;
    status.state  = state;
    status.at_us  = hooks.clock();
    status.port   = port.path;
    status.detail = std::move(detail);
    status.stats  = stats;
    hooks.on_status(status);
}

std::string hexByte(Byte value) {
    char text[8];
    std::snprintf(text, sizeof(text), "0x%02X", value);
    return text;
}

std::string whyNoLink(const std::optional<SystemVersion>& version) {
    if (!version) {
        return "bad SYSTEM_VERSION reply";
    }
    if (version->product != kProductController) {
        return "not a V5 controller (product " + hexByte(version->product) + ")";
    }
    return "no Brain linked (flags " + hexByte(version->flags) + ")";
}

// One open controller port: identify, then poll while linked.
class ControllerSession {
  public:
    ControllerSession(Port& port, const ControllerConfig& config, const SourceHooks& hooks,
                      platform::StopSignal& stop, TransportStats& stats)
        : port_(port), config_(config), hooks_(hooks), stop_(stop), stats_(stats) {}

    // The close reason, or nullopt on stop.
    std::optional<CloseReason> run() {
        while (!ended()) {
            const LinkKind link = identify();
            if (ended()) {
                break;
            }
            if (link == LinkKind::kNone) {
                waitQuiet(config_.link_retry);
                continue;
            }
            report(PortState::kOpen,
                   std::string("controller: Brain linked by ") + linkKindName(link));
            pollWhileLinked();
        }
        return closed_;
    }

  private:
    bool ended() const { return closed_.has_value() || stop_.requested(); }

    LinkKind identify() {
        const CdcReplies replies = exchange(systemVersionRequest(), {kCommandSystemVersion, {}});
        if (replies.empty()) {
            if (!ended()) {
                report(PortState::kNoBrainLinked, "controller: no reply to SYSTEM_VERSION");
            }
            return LinkKind::kNone;
        }
        const std::optional<SystemVersion> version = parseSystemVersion(replies.front());
        const LinkKind link = version ? controllerLink(*version) : LinkKind::kNone;
        if (link == LinkKind::kNone) {
            report(PortState::kNoBrainLinked, "controller: " + whyNoLink(version));
        }
        return link;
    }

    // Back to back while data flows; back to identify after timeouts in a row or a long idle.
    void pollWhileLinked() {
        const Bytes request         = userReadRequest(kUserChannel, config_.read_length);
        uint32_t    timeouts_in_row = 0;
        ReceiverUs  data_us         = hooks_.clock();
        radio_due_us_               = hooks_.clock();
        while (!ended()) {
            queryRadioWhenDue();
            ++stats_.polls;
            const ReceiverUs asked_us = hooks_.clock();
            const CdcReplies replies  = exchange(request, kUserReadReply);
            if (ended()) {
                return;
            }
            if (replies.empty()) {
                if (++timeouts_in_row >= config_.timeouts_to_relink) {
                    return;
                }
                continue;
            }
            timeouts_in_row = 0;
            stats_.rtt_us   = hooks_.clock() - asked_us;
            if (deliver(replies)) {
                data_us = hooks_.clock();
            } else if (hooks_.clock() - data_us >= config_.idle_relink_us) {
                return;
            } else {
                waitQuiet(config_.empty_backoff);
            }
            reportStatsWhenDue();
        }
    }

    // True when any Brain bytes came. Extra replies answer timed-out polls, in FIFO order.
    bool deliver(const CdcReplies& replies) {
        stats_.late_replies += static_cast<uint32_t>(replies.size() - 1);
        SourceChunk chunk;
        chunk.src     = SourceKind::kController;
        bool any_good = false;
        for (const CdcReply& reply : replies) {
            const UserRead read = parseUserRead(reply);
            if (read.error != UserReadError::kNone) {
                ++stats_.bad_replies;
                continue;
            }
            any_good = true;
            // The Brain may pad a reply to a word with NULs. Pose lines hold no 0x00 and the
            // line scanner needs no COBS delimiter, so trailing NULs go: a NUL left mid-line
            // would drop that line as truncated.
            std::size_t end = read.data.size();
            while (end > 0 && read.data[end - 1] == 0x00) {
                --end;
            }
            stats_.padding_bytes += static_cast<uint32_t>(read.data.size() - end);
            chunk.bytes.insert(chunk.bytes.end(), read.data.begin(),
                               read.data.begin() + static_cast<std::ptrdiff_t>(end));
        }
        if (chunk.bytes.empty()) {
            stats_.empty_polls += any_good ? 1u : 0u;
            return false;
        }
        chunk.rx_us = hooks_.clock();
        hooks_.on_chunk(chunk);
        return true;
    }

    void queryRadioWhenDue() {
        if (config_.radio_period_us <= 0 || hooks_.clock() < radio_due_us_) {
            return;
        }
        radio_due_us_ = hooks_.clock() + config_.radio_period_us;
        const CdcReplies replies =
            exchange(radioStatusRequest(), {kCommandExtended, kExtendedRadioStatus});
        if (!replies.empty()) {
            stats_.radio = parseRadioStatus(replies.front());
        }
    }

    // Every reply of this kind in the first read that has one; empty on a timeout.
    CdcReplies exchange(const Bytes& request, const ReplyKind& kind) {
        const IoResult written = port_.write(request.data(), request.size(), config_.write_budget);
        if (written.status == IoStatus::kClosed) {
            closed_ = written.error;
            return {};
        }
        const ReceiverUs deadline = hooks_.clock() + microsecondsOf(config_.reply_timeout);
        ReadBuffer       buffer{};
        while (written.status == IoStatus::kOk && !stop_.requested()) {
            const ReceiverUs now = hooks_.clock();
            if (now >= deadline) {
                break;
            }
            const IoResult got =
                port_.read(buffer.data(), buffer.size(), millisecondsAtLeast(deadline - now));
            if (got.status == IoStatus::kClosed) {
                closed_ = got.error;
                return {};
            }
            if (got.status != IoStatus::kOk || got.bytes == 0) {
                continue;
            }
            CdcReplies matched = matchReplies(buffer.data(), got.bytes, kind);
            if (!matched.empty()) {
                return matched;
            }
        }
        if (!stop_.requested()) {
            ++stats_.timeouts;
            resync();
        }
        return {};
    }

    CdcReplies matchReplies(const Byte* bytes, std::size_t size, const ReplyKind& kind) {
        CdcScan scan = scanner_.push(bytes, size);
        stats_.bad_replies += scan.bad_replies;
        stats_.stray_bytes += scan.stray_bytes;
        CdcReplies matched;
        for (CdcReply& reply : scan.replies) {
            if (answers(reply, kind)) {
                matched.push_back(std::move(reply));
            } else if (answers(reply, kUserReadReply)) {
                ++stats_.late_replies; // a timed-out poll answered: its bytes still count
                (void)deliver(CdcReplies{std::move(reply)});
            } else {
                ++stats_.bad_replies;
            }
        }
        return matched;
    }

    // After a timeout a late reply may still come: forget partial bytes and drain.
    void resync() {
        scanner_.reset();
        ReadBuffer buffer{};
        for (int i = 0; i < kMaxDrainReads; ++i) {
            const IoResult got = port_.read(buffer.data(), buffer.size(), Milliseconds(0));
            if (got.status == IoStatus::kClosed) {
                closed_ = got.error;
                return;
            }
            if (got.status != IoStatus::kOk || got.bytes == 0) {
                return;
            }
            stats_.stray_bytes += static_cast<uint32_t>(got.bytes);
        }
    }

    // A wait that is a port read: no request is outstanding, so any byte is stray.
    void waitQuiet(Milliseconds duration) {
        const ReceiverUs deadline = hooks_.clock() + microsecondsOf(duration);
        ReadBuffer       buffer{};
        while (!stop_.requested()) {
            const ReceiverUs now = hooks_.clock();
            if (now >= deadline) {
                return;
            }
            const IoResult got =
                port_.read(buffer.data(), buffer.size(), millisecondsAtLeast(deadline - now));
            if (got.status == IoStatus::kClosed) {
                closed_ = got.error;
                return;
            }
            if (got.status == IoStatus::kOk && got.bytes > 0) {
                stats_.stray_bytes += static_cast<uint32_t>(got.bytes);
                scanner_.reset();
            }
        }
    }

    // Only on a change of state or detail, or with stats when due.
    void report(PortState state, std::string detail) {
        if (reported_ && state == state_ && detail == detail_) {
            return;
        }
        state_  = state;
        detail_ = std::move(detail);
        sendReport();
    }

    void reportStatsWhenDue() {
        if (hooks_.clock() - last_report_us_ >= config_.stats_period_us) {
            sendReport();
        }
    }

    void sendReport() {
        reported_       = true;
        last_report_us_ = hooks_.clock();
        reportStatus(hooks_, port_, state_, detail_, stats_);
    }

    Port&                      port_;
    const ControllerConfig&    config_;
    const SourceHooks&         hooks_;
    platform::StopSignal&      stop_;
    TransportStats&            stats_;
    CdcReplyScanner            scanner_;
    std::optional<CloseReason> closed_;
    PortState                  state_ = PortState::kNoBrainLinked;
    std::string                detail_;
    bool                       reported_       = false;
    ReceiverUs                 last_report_us_ = 0;
    ReceiverUs                 radio_due_us_   = 0;
};

} // namespace

std::optional<CloseReason> runControllerSession(Port& port, const ControllerConfig& config,
                                                const SourceHooks&    hooks,
                                                platform::StopSignal& stop, TransportStats& stats) {
    ControllerSession session(port, config, hooks, stop, stats);
    return session.run();
}

} // namespace spectagatr::receiver::sources
