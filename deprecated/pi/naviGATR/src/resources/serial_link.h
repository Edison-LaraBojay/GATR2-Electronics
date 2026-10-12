// serial_link.h
// Typed contract for a byte link. There is deliberately no universal DataBus
// abstraction; different hardware concepts get different contracts.
//
// Thread safety: SerialLink implementations are single-threaded unless their
// own documentation says otherwise. A shared_ptr does not make hardware safe.

#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace navigatr
{

struct ByteSpan {
    const uint8_t* data = nullptr;
    std::size_t    size = 0;
};

struct MutableByteSpan {
    uint8_t*    data = nullptr;
    std::size_t size = 0;
};

struct SerialReadResult {
    std::size_t bytes  = 0;
    bool        closed = false;   // link is dead, not merely idle
};

// Steady clock microseconds, arbitrary origin.
inline int64_t steadyNowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Allowed transmit start, in SerialLink::nowUs() time. Default is unbounded.
struct TransmitWindow {
    int64_t not_before_us = std::numeric_limits<int64_t>::min();
    int64_t deadline_us   = std::numeric_limits<int64_t>::max();

    // No start from now_us on can fall inside the window.
    bool missed(int64_t now_us) const {
        return now_us > deadline_us || not_before_us > deadline_us;
    }
};

struct SerialWriteResult {
    bool        ok            = false;
    bool        expired       = false;     // window missed, nothing sent
    bool        input_pending = false;     // half duplex: input waiting, nothing sent
    bool        late_release  = false;     // half duplex: driver released after the frame budget
    const char* error         = nullptr;   // why not ok, static text; may stay nullptr
};

class SerialLink
{
public:
    virtual ~SerialLink() = default;

    // Nonblocking. Copies what is available, up to destination.size.
    virtual SerialReadResult readAvailable(MutableByteSpan destination) = 0;

    virtual SerialWriteResult write(ByteSpan source) = 0;

    // Starts no earlier than window.not_before_us (may block until then) and
    // never after window.deadline_us (expired, nothing sent). Half-duplex
    // links also refuse while input is pending. Default ignores the window.
    virtual SerialWriteResult write(ByteSpan source, const TransmitWindow& window) {
        (void)window;
        return write(source);
    }

    // Received bytes waiting. False when unknown.
    virtual bool inputPending() { return false; }

    // Clock for read timestamps and transmit windows.
    virtual int64_t nowUs() { return steadyNowUs(); }
};

} // namespace navigatr
