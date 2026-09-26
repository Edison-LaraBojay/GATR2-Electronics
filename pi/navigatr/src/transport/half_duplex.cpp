// half_duplex.cpp

#include "transport/half_duplex.h"

#include <algorithm>

namespace navigatr
{

namespace
{

const char* waitTransmitterEmpty(HalfDuplexPort& port, int64_t earliest_us, int64_t deadline_us,
                                 int64_t poll_us) {
    const int64_t start = port.nowUs();
    if (start < earliest_us) {
        port.sleepUs(earliest_us - start);
    }
    for (;;) {
        const int empty = port.transmitterEmpty();
        if (empty < 0) {
            return "transmitter status failed";
        }
        if (empty > 0) {
            return nullptr;
        }
        const int64_t now = port.nowUs();
        if (now >= deadline_us) {
            return "transmitter never empty";
        }
        port.sleepUs(std::min(std::max<int64_t>(poll_us, 1), deadline_us - now));
    }
}

} // namespace

int64_t characterTimeUs(int baud) {
    if (baud <= 0) {
        return 0;
    }
    return (10'000'000 + baud - 1) / baud;
}

int64_t transmitBudgetUs(std::size_t bytes, int baud, int64_t margin_us) {
    if (baud <= 0) {
        return margin_us;
    }
    const int64_t bits = static_cast<int64_t>(bytes) * 10'000'000;
    return (bits + baud - 1) / baud + margin_us;
}

bool waitForWindow(HalfDuplexPort& port, const TransmitWindow& window) {
    int64_t now = port.nowUs();
    if (window.missed(now)) {
        return false;
    }
    if (now < window.not_before_us) {
        port.sleepUs(window.not_before_us - now);
        now = port.nowUs();
    }
    return now <= window.deadline_us;
}

const char* writeAll(HalfDuplexPort& port, ByteSpan bytes, int64_t deadline_us) {
    std::size_t at = 0;
    while (at < bytes.size) {
        const int n = port.writeSome(bytes.data + at, bytes.size - at);
        if (n < 0) {
            return "write failed";
        }
        at += static_cast<std::size_t>(n);
        if (at >= bytes.size) {
            break;
        }
        const int64_t now = port.nowUs();
        if (now >= deadline_us) {
            return "write timed out";
        }
        if (n == 0 && !port.waitWritable(deadline_us - now)) {
            return "write wait failed";
        }
    }
    return nullptr;
}

SerialWriteResult transmitHalfDuplex(HalfDuplexPort& port, ByteSpan frame,
                                     const TransmitWindow&   window,
                                     const HalfDuplexTiming& timing) {
    SerialWriteResult result;
    if (!waitForWindow(port, window)) {
        result.expired = true;
        result.error   = "transmit window missed";
        return result;
    }
    if (port.inputPending()) {
        result.input_pending = true;
        result.error         = "input pending";
        return result;
    }
    if (frame.size == 0) {
        result.ok = true;
        return result;
    }

    const int64_t char_us  = characterTimeUs(timing.baud);
    const int64_t airtime  = static_cast<int64_t>(frame.size) * char_us;
    const int64_t on_us    = port.nowUs();
    const int64_t deadline = on_us + transmitBudgetUs(frame.size, timing.baud, timing.margin_us);

    const char* error = port.setDriver(true) ? nullptr : "driver enable failed";
    if (error == nullptr) {
        error = writeAll(port, frame, deadline);
    }
    if (error == nullptr) {
        error = waitTransmitterEmpty(port, on_us + airtime, deadline, char_us);
    }
    if (error == nullptr) {
        if (timing.post_guard_us > 0) {
            port.sleepUs(timing.post_guard_us);
        }
    } else {
        port.discardOutput();
    }
    const bool released = port.setDriver(false);

    result.late_release = port.nowUs() > deadline + timing.post_guard_us;
    if (error == nullptr && !released) {
        error = "driver release failed";
    }
    result.ok    = error == nullptr;
    result.error = error;
    return result;
}

} // namespace navigatr
