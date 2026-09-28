// fake_bus.cpp

#include "sim/fake_bus.h"

#include <algorithm>
#include <utility>

namespace communigatr
{

int FakeBus::Port::read(uint8_t* buf, int max) { return bus_.brainRead(buf, max); }

bool FakeBus::Port::write(const uint8_t* data, int len) { return bus_.brainWrite(data, len); }

FakeBus::FakeBus(FakePi& pi, const FakeBusConfig& config)
    : pi_(pi), config_(config), port_(*this) {}

void FakeBus::advanceTo(Seconds now) {
    now_ = std::max(now_, now);
    while (!to_pi_.empty() && to_pi_.front().end <= now_) {
        InFlight request = std::move(to_pi_.front());
        to_pi_.pop_front();
        if (!request_faults_.empty()) {
            request_faults_.pop_front(); // kDropRequest
            continue;
        }
        if (!pi_present_) {
            continue;
        }
        for (std::vector<uint8_t>& frame :
             pi_.receive(request.bytes.data(), request.bytes.size())) {
            reply(request.end, std::move(frame));
        }
    }
}

void FakeBus::fault(BusFault fault, Seconds amount) {
    if (fault == BusFault::kNone) {
        return;
    }
    if (fault == BusFault::kDropRequest) {
        request_faults_.push_back({fault, amount});
    } else {
        reply_faults_.push_back({fault, amount});
    }
}

void FakeBus::sendToBrain(const std::vector<uint8_t>& bytes, Seconds at) {
    for (uint8_t b : bytes) {
        to_brain_.emplace(at, b);
    }
}

std::vector<translagatr::BrainRequest> FakeBus::brainRequests() const {
    std::vector<translagatr::BrainRequest> requests;
    for (const Transmission& t : log_) {
        translagatr::BrainRequest request;
        if (t.brain && translagatr::decodeBrainRequest(t.bytes.data(),
                                                 static_cast<uint16_t>(t.bytes.size()), request)) {
            requests.push_back(request);
        }
    }
    return requests;
}

bool FakeBus::collision() const {
    for (const Transmission& a : log_) {
        for (const Transmission& b : log_) {
            if (a.brain && !b.brain && a.start < b.end && b.start < a.end) {
                return true;
            }
        }
    }
    return false;
}

int FakeBus::brainRead(uint8_t* buf, int max) {
    int n = 0;
    while (n < max && !to_brain_.empty() && to_brain_.begin()->first <= now_) {
        buf[n++] = to_brain_.begin()->second;
        to_brain_.erase(to_brain_.begin());
    }
    return n;
}

bool FakeBus::brainWrite(const uint8_t* data, int len) {
    if (failed_writes_ > 0) {
        --failed_writes_;
        return false;
    }
    Transmission t;
    t.brain = true;
    t.start = std::max(now_ + config_.brain_tx_latency, brain_free_);
    t.end   = t.start + len * byteTime();
    t.bytes.assign(data, data + len);
    brain_free_ = t.end;
    to_pi_.push_back({t.end, t.bytes});
    log_.push_back(std::move(t));
    return true;
}

// The Pi starts its reply reply_delay after the request's last bit, never
// later than reply_window after it.
void FakeBus::reply(Seconds request_end, std::vector<uint8_t> frame) {
    Fault fault{BusFault::kNone, 0};
    if (!reply_faults_.empty()) {
        fault = reply_faults_.front();
        reply_faults_.pop_front();
    }
    Seconds delay = config_.reply_delay;
    if (fault.kind == BusFault::kDelay) {
        delay += fault.amount;
    }
    if (delay > config_.reply_window) {
        ++silent_replies_;
        return;
    }

    // Segments on the wire, each followed by a pause.
    std::vector<std::pair<std::vector<uint8_t>, Seconds>> segments;
    const std::size_t                                     half = frame.size() / 2;
    switch (fault.kind) {
    case BusFault::kCorrupt:
        frame[5] ^= 0x10;
        segments.push_back({frame, 0});
        break;
    case BusFault::kDuplicate:
        segments.push_back({frame, 0});
        segments.push_back({frame, 0});
        break;
    case BusFault::kFragment:
        segments.push_back(
            {std::vector<uint8_t>(frame.begin(), frame.begin() + half), fault.amount});
        segments.push_back({std::vector<uint8_t>(frame.begin() + half, frame.end()), 0});
        break;
    case BusFault::kTruncate:
        segments.push_back(
            {std::vector<uint8_t>(frame.begin(), frame.begin() + half), fault.amount});
        segments.push_back({frame, 0});
        break;
    default:
        segments.push_back({frame, 0});
        break;
    }

    Transmission t;
    t.start    = request_end + delay;
    Seconds at = t.start;
    for (const auto& segment : segments) {
        for (uint8_t b : segment.first) {
            at += byteTime();
            if (fault.kind != BusFault::kDropReply) {
                to_brain_.emplace(at + config_.brain_rx_latency, b);
            }
            t.bytes.push_back(b);
        }
        at += segment.second;
    }
    t.end = at + config_.release;
    log_.push_back(std::move(t));
}

} // namespace communigatr
