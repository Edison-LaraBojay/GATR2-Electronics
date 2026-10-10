// fake_controller.h
// A V5 controller as the controller source sees it: SYSTEM_VERSION, USER_READ and RADIO_STATUS
// answered after a round trip, at most max_data Brain bytes per poll, and scripted faults (lost,
// late, corrupted, NACKed or garbage-led replies, NUL padding, unplug). Every request written is
// kept for checks.

#pragma once
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>
#include <vector>

#include "receiver/sources/cdc2.h"
#include "receiver/sources/fakes/fake_port.h"

namespace spectagatr::receiver::sources::fakes
{

struct FakeControllerOptions {
    DurationUs  round_trip_us      = 20000;
    std::size_t max_data           = 64;
    Byte        product            = kProductController;
    Byte        flags              = kLinkedByRadio;
    bool        pad_to_word        = false; // NULs after the data up to a multiple of 4
    bool        answer_radio       = true;
    RadioStatus radio              = RadioStatus{4, 90, -48, 55, 1};
    std::size_t max_bytes_per_read = std::numeric_limits<std::size_t>::max();
};

using Requests = std::vector<Bytes>;

class FakeController {
  public:
    explicit FakeController(FakeClock& clock, FakeControllerOptions options = {});

    // Reads and writes this object, which outlives the Port.
    Port port();

    void brainWrites(const Bytes& bytes); // Brain channel-1 output, queued for USER_READ
    void setFlags(Byte flags) { options_.flags = flags; }
    void dropReplies(std::size_t count) { drop_ += count; }
    void delayNextReply(DurationUs extra_us) { delay_us_ = extra_us; }
    void delayNextUserRead(DurationUs extra_us) { user_read_delay_us_ = extra_us; }
    void corruptNextReply() { corrupt_ = true; }
    void garbageBeforeNextReply(Bytes garbage) { garbage_ = std::move(garbage); }
    void nackNextUserRead(Byte code) { nack_ = code; }
    void unplug() { unplugged_ = true; }

    // Called when a USER_READ finds no Brain bytes.
    void onIdle(EndCallback callback) { on_idle_ = std::move(callback); }

    const Requests& requests() const { return requests_; }
    std::size_t     pendingBrainBytes() const { return fifo_.size(); }

  private:
    struct PendingReply {
        ReceiverUs ready_us = 0;
        Bytes      bytes;
    };

    IoResult read(Byte* destination, std::size_t capacity, Milliseconds wait);
    IoResult write(const Byte* source, std::size_t size);
    void     answer(const Bytes& request);
    Bytes    userReadReply(Byte length);
    Bytes    radioReply() const;
    void     queueReply(Bytes reply);

    FakeClock&               clock_;
    FakeControllerOptions    options_;
    std::deque<PendingReply> replies_;
    std::deque<Byte>         fifo_;
    Requests                 requests_;
    EndCallback              on_idle_;
    std::size_t              drop_               = 0;
    DurationUs               delay_us_           = 0;
    DurationUs               user_read_delay_us_ = 0;
    Bytes                    garbage_;
    std::optional<Byte>      nack_;
    bool                     corrupt_   = false;
    bool                     unplugged_ = false;
};

// Extended reply bytes: AA 55 56 <size> <ecmd> <ack> <payload> <crc16 BE>.
Bytes extendedReply(Byte ecmd, Byte ack, const Bytes& payload);

// Simple reply bytes: AA 55 A4 08 <version x5> <product> <flags> 00.
Bytes systemVersionReply(Byte product, Byte flags);

} // namespace spectagatr::receiver::sources::fakes
