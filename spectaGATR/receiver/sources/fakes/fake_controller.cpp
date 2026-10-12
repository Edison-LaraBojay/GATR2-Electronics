// fake_controller.cpp

#include "receiver/sources/fakes/fake_controller.h"

#include <algorithm>
#include <utility>

namespace spectagatr::receiver::sources::fakes
{

namespace
{

constexpr Byte        kRequestHeader[]  = {0xC9, 0x36, 0xB8, 0x47};
constexpr std::size_t kRequestHeadBytes = sizeof(kRequestHeader);
constexpr Byte        kLongSizeFlag     = 0x80;
constexpr std::size_t kWordBytes        = 4;

DurationUs microsecondsOf(Milliseconds duration) { return duration.count() * 1000; }

IoResult closedResult() {
    IoResult result;
    result.status = IoStatus::kClosed;
    result.error  = "controller unplugged";
    return result;
}

struct ExtendedRequest {
    Byte  ecmd = 0;
    Bytes payload;
};

std::optional<ExtendedRequest> parseExtended(const Bytes& request) {
    std::size_t at = kRequestHeadBytes + 2; // header, 0x56, ecmd
    if (request.size() < at + 1 + 2 || crc16Xmodem(request.data(), request.size()) != 0) {
        return std::nullopt;
    }
    std::size_t size = request[at++];
    if ((size & kLongSizeFlag) != 0) {
        size = ((size & 0x7F) << 8) | request[at++];
    }
    if (request.size() != at + size + 2) {
        return std::nullopt;
    }
    ExtendedRequest parsed;
    parsed.ecmd = request[kRequestHeadBytes + 1];
    parsed.payload.assign(request.begin() + static_cast<std::ptrdiff_t>(at),
                          request.begin() + static_cast<std::ptrdiff_t>(at + size));
    return parsed;
}

} // namespace

Bytes extendedReply(Byte ecmd, Byte ack, const Bytes& payload) {
    const std::size_t size  = 2 + payload.size() + 2; // ecmd, ack, payload, crc
    Bytes             reply = {0xAA, 0x55, kCommandExtended};
    if (size >= kLongSizeFlag) {
        reply.push_back(static_cast<Byte>((size >> 8) | kLongSizeFlag));
    }
    reply.push_back(static_cast<Byte>(size & 0xFF));
    reply.push_back(ecmd);
    reply.push_back(ack);
    reply.insert(reply.end(), payload.begin(), payload.end());
    const uint16_t crc = crc16Xmodem(reply.data(), reply.size());
    reply.push_back(static_cast<Byte>(crc >> 8));
    reply.push_back(static_cast<Byte>(crc & 0xFF));
    return reply;
}

Bytes systemVersionReply(Byte product, Byte flags) {
    return Bytes{0xAA,  0x55, kCommandSystemVersion, 0x08, 0x01, 0x01, 0x00, 0x00, 0x00, product,
                 flags, 0x00};
}

FakeController::FakeController(FakeClock& clock, FakeControllerOptions options)
    : clock_(clock), options_(options) {}

Port FakeController::port() {
    Port port;
    port.read = [this](Byte* destination, std::size_t capacity, Milliseconds wait) {
        return read(destination, capacity, wait);
    };
    port.write = [this](const Byte* source, std::size_t size, Milliseconds) {
        return write(source, size);
    };
    port.kind = SourceKind::kController;
    port.path = "FAKE1";
    port.name = "fake controller";
    return port;
}

void FakeController::brainWrites(const Bytes& bytes) {
    fifo_.insert(fifo_.end(), bytes.begin(), bytes.end());
}

IoResult FakeController::read(Byte* destination, std::size_t capacity, Milliseconds wait) {
    if (unplugged_) {
        return closedResult();
    }
    IoResult result;
    if (replies_.empty() || replies_.front().ready_us - clock_.now() > microsecondsOf(wait)) {
        clock_.advance(microsecondsOf(wait));
        result.status = IoStatus::kTimedOut;
        return result;
    }
    clock_.advance(std::max<DurationUs>(0, replies_.front().ready_us - clock_.now()));
    // Everything ready by now, as a USB read returns what is buffered.
    const std::size_t limit = std::min(capacity, options_.max_bytes_per_read);
    while (result.bytes < limit && !replies_.empty() && replies_.front().ready_us <= clock_.now()) {
        Bytes&            reply = replies_.front().bytes;
        const std::size_t count = std::min(limit - result.bytes, reply.size());
        std::copy_n(reply.begin(), count, destination + result.bytes);
        reply.erase(reply.begin(), reply.begin() + static_cast<std::ptrdiff_t>(count));
        result.bytes += count;
        if (reply.empty()) {
            replies_.pop_front();
        }
    }
    return result;
}

IoResult FakeController::write(const Byte* source, std::size_t size) {
    if (unplugged_) {
        return closedResult();
    }
    const Bytes request(source, source + size);
    requests_.push_back(request);
    answer(request);
    IoResult result;
    result.bytes = size;
    return result;
}

void FakeController::answer(const Bytes& request) {
    if (drop_ > 0) {
        --drop_;
        return;
    }
    if (request == systemVersionRequest()) {
        const Byte flags = clock_.now() < link_down_until_ ? Byte{0} : options_.flags;
        queueReply(systemVersionReply(options_.product, flags));
        return;
    }
    const std::optional<ExtendedRequest> parsed = parseExtended(request);
    if (!parsed) {
        return;
    }
    if (parsed->ecmd == kExtendedUserRead && parsed->payload.size() == 2) {
        if (user_read_delay_us_ > 0) {
            delay_us_           = user_read_delay_us_;
            user_read_delay_us_ = 0;
        }
        queueReply(userReadReply(parsed->payload[1]));
    } else if (parsed->ecmd == kExtendedRadioStatus && options_.answer_radio) {
        queueReply(radioReply());
    } else if (parsed->ecmd == kExtendedFileControl && parsed->payload.size() == 2 &&
               parsed->payload[0] == 0x01) {
        const Byte download = parsed->payload[1];
        channel_switches_.push_back(download);
        if (!options_.ack_switch) {
            queueReply(extendedReply(kExtendedFileControl, 0xFF, Bytes{}));
            return;
        }
        queueReply(extendedReply(kExtendedFileControl, kAck, Bytes{}));
        link_down_until_     = clock_.now() + options_.switch_drop_us;
        options_.radio.channel = download != 0 ? Byte{5} : Byte{55};
    }
}

Bytes FakeController::userReadReply(Byte length) {
    if (nack_) {
        const Byte code = *nack_;
        nack_.reset();
        return extendedReply(kExtendedUserRead, code, Bytes{});
    }
    const std::size_t count =
        std::min({length == 0 ? kUserReadMaxBytes : static_cast<std::size_t>(length),
                  options_.max_data, fifo_.size()});
    Bytes payload = {kUserChannel};
    for (std::size_t i = 0; i < count; ++i) {
        payload.push_back(fifo_.front());
        fifo_.pop_front();
    }
    if (options_.pad_to_word) {
        while ((payload.size() - 1) % kWordBytes != 0) {
            payload.push_back(0x00);
        }
    }
    if (count == 0 && on_idle_) {
        on_idle_();
    }
    return extendedReply(kExtendedUserRead, kAck, payload);
}

Bytes FakeController::radioReply() const {
    const RadioStatus& radio    = options_.radio;
    const uint16_t     strength = static_cast<uint16_t>(radio.strength);
    const Bytes        status   = {radio.device,
                                   static_cast<Byte>(radio.quality & 0xFF),
                                   static_cast<Byte>(radio.quality >> 8),
                                   static_cast<Byte>(strength & 0xFF),
                                   static_cast<Byte>(strength >> 8),
                                   radio.channel,
                                   radio.timeslot};
    return extendedReply(kExtendedRadioStatus, kAck, status);
}

void FakeController::queueReply(Bytes reply) {
    if (corrupt_) {
        reply.back() ^= 0xFF;
        corrupt_ = false;
    }
    if (!garbage_.empty()) {
        reply.insert(reply.begin(), garbage_.begin(), garbage_.end());
        garbage_.clear();
    }
    const ReceiverUs ready_us = clock_.now() + options_.round_trip_us + delay_us_;
    delay_us_                 = 0;
    replies_.push_back(PendingReply{ready_us, std::move(reply)});
}

} // namespace spectagatr::receiver::sources::fakes
