// fake_bus.h
// Host-only RS-485 half-duplex bus between the Brain's BytePort and a FakePi:
// byte airtime at the baud rate, reply timing inside the Pi's reply window,
// scripted faults, and a log of who transmitted when.

#pragma once
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include "communigatr/byte_port.h"
#include "communigatr/client.h"
#include "sim/fake_pi.h"

namespace communigatr
{

struct FakeBusConfig {
    uint32_t baud             = 115200;
    Seconds  brain_tx_latency = 0;      // write return to the first bit
    Seconds  brain_rx_latency = 0;      // last bit of a byte to readable
    Seconds  reply_delay      = 0.002;  // request end to reply start
    Seconds  reply_window     = 0.040;  // no reply starts later than this after the request end
    Seconds  release          = 0.0002; // driver release after the last reply bit
};

enum class BusFault : uint8_t {
    kNone,
    kDropRequest, // the next request never reaches the Pi
    kDropReply,   // the Pi answers the next request, the Brain receives nothing
    kDuplicate,   // the next reply is sent twice, back to back
    kCorrupt,     // the next reply has one payload bit flipped
    kFragment,    // the next reply pauses for `amount` halfway
    kTruncate,    // half of the next reply, a pause of `amount`, then the whole reply
    kDelay,       // the next reply starts `amount` later; silent past the window
};

struct Transmission {
    bool                 brain = false; // sender
    Seconds              start = 0;
    Seconds              end   = 0; // bus released
    std::vector<uint8_t> bytes;
};

class FakeBus {
public:
    explicit FakeBus(FakePi& pi, const FakeBusConfig& config = {});
    FakeBus(const FakeBus&)            = delete;
    FakeBus& operator=(const FakeBus&) = delete;

    BytePort& brainPort() { return port_; }

    // Moves the clock forward. The Pi answers every request that ended by now.
    void    advanceTo(Seconds now);
    Seconds now() const { return now_; }
    Seconds byteTime() const { return 10.0 / config_.baud; }

    // Queued in order; request faults and reply faults queue separately.
    void fault(BusFault fault, Seconds amount = 0);

    // Absent: requests vanish and nothing is answered.
    void setPiPresent(bool present) { pi_present_ = present; }

    // The next count Brain writes fail and put nothing on the bus.
    void failWrites(int count) { failed_writes_ = count; }

    // Raw bytes for the Brain, readable from `at`, not logged as a transmission.
    void sendToBrain(const std::vector<uint8_t>& bytes, Seconds at);

    const std::vector<Transmission>& log() const { return log_; }

    // Decoded Brain requests on the wire, in order.
    std::vector<gatr2::BrainRequest> brainRequests() const;

    // A Brain and a Pi transmission overlapped.
    bool collision() const;

    // Replies the Pi withheld because they could not start inside its window.
    int silentReplies() const { return silent_replies_; }

    FakeBusConfig& config() { return config_; }

private:
    class Port : public BytePort {
    public:
        explicit Port(FakeBus& bus) : bus_(bus) {}
        int  read(uint8_t* buf, int max) override;
        bool write(const uint8_t* data, int len) override;

    private:
        FakeBus& bus_;
    };

    struct Fault {
        BusFault kind;
        Seconds  amount;
    };

    struct InFlight {
        Seconds              end = 0;
        std::vector<uint8_t> bytes;
    };

    int  brainRead(uint8_t* buf, int max);
    bool brainWrite(const uint8_t* data, int len);
    void reply(Seconds request_end, std::vector<uint8_t> frame);

    FakePi&       pi_;
    FakeBusConfig config_;
    Port          port_;
    Seconds       now_            = 0;
    Seconds       brain_free_     = 0;
    bool          pi_present_     = true;
    int           failed_writes_  = 0;
    int           silent_replies_ = 0;

    std::deque<Fault>               request_faults_;
    std::deque<Fault>               reply_faults_;
    std::deque<InFlight>            to_pi_;
    std::multimap<Seconds, uint8_t> to_brain_; // readable time, byte
    std::vector<Transmission>       log_;
};

} // namespace communigatr
