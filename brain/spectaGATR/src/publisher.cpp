// publisher.cpp

#include "spectagatr/publisher.h"

#include <cmath>

namespace spectagatr
{
namespace
{

// Slot words.
constexpr std::size_t kX            = 0; // mm
constexpr std::size_t kY            = 1; // mm
constexpr std::size_t kYaw          = 2; // centidegrees
constexpr std::size_t kRoll         = 3;
constexpr std::size_t kPitch        = 4;
constexpr std::size_t kTime         = 5; // publisher ms
constexpr std::size_t kEpochFlags   = 6; // epoch in bits 0..15, flags in bits 16..23
constexpr std::size_t kChecksum     = 7;
constexpr std::size_t kDataWords    = 7;
constexpr int         kReadAttempts = 3;

uint64_t splitmix64(uint64_t x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

// FNV-1a over the data words. Catches a slot mixed by two overlapping writers.
uint32_t checksum(const uint32_t* words) {
    uint32_t hash = 0x811C9DC5u;
    for (std::size_t i = 0; i < kDataWords; ++i) {
        uint32_t word = words[i];
        for (int byte = 0; byte < 4; ++byte) {
            hash ^= word & 0xFFu;
            hash *= 0x01000193u;
            word >>= 8;
        }
    }
    return hash;
}

bool validCoordinate(Meters v) { return std::isfinite(v) && std::fabs(v) <= kMaxCoordinate; }

uint32_t periodFor(Hertz rate) {
    if (!std::isfinite(rate)) {
        rate = kDefaultRateHz;
    }
    if (rate < kMinRateHz) {
        rate = kMinRateHz;
    }
    if (rate > kMaxRateHz) {
        rate = kMaxRateHz;
    }
    return static_cast<uint32_t>(std::lround(1000.0 / rate));
}

uint32_t toWord(int32_t v) { return static_cast<uint32_t>(v); }

int32_t fromWord(uint32_t w) { return static_cast<int32_t>(w); }

} // namespace

SessionId makeSessionId(const uint64_t* inputs, std::size_t count) {
    uint64_t hash = splitmix64(count);
    for (std::size_t i = 0; i < count; ++i) {
        hash = splitmix64(hash ^ inputs[i]);
    }
    const SessionId id = static_cast<SessionId>(hash ^ (hash >> 32));
    return id != 0 ? id : 1;
}

Publisher::Publisher(Clock clock, Hertz rate) : clock_(clock) {
    period_ms_.store(periodFor(rate), std::memory_order_relaxed);
}

void Publisher::updatePose(Meters x, Meters y, Radians heading) {
    store(x, y, Attitude{0, 0, heading}, false);
}

void Publisher::updatePose(Meters x, Meters y, const Attitude& attitude) {
    store(x, y, attitude, true);
}

void Publisher::store(Meters x, Meters y, const Attitude& attitude, bool full_attitude) {
    if (!validCoordinate(x) || !validCoordinate(y) || !std::isfinite(attitude.roll) ||
        !std::isfinite(attitude.pitch) || !std::isfinite(attitude.yaw)) {
        rejected_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint32_t flags = full_attitude ? static_cast<uint32_t>(protocol::kFullAttitude) : 0u;
    const uint32_t epoch = epoch_.load(std::memory_order_relaxed) & 0xFFFFu;

    uint32_t words[kSlotWords];
    words[kX]          = toWord(protocol::toMillimeters(x));
    words[kY]          = toWord(protocol::toMillimeters(y));
    words[kYaw]        = toWord(protocol::toCentidegrees(attitude.yaw));
    words[kRoll]       = toWord(protocol::toCentidegrees(attitude.roll));
    words[kPitch]      = toWord(protocol::toCentidegrees(attitude.pitch));
    words[kTime]       = clock_ != nullptr ? clock_() : 0u;
    words[kEpochFlags] = epoch | (flags << 16);
    words[kChecksum]   = checksum(words);

    // Odd while inside. A writer that never finished (a deleted task) leaves it
    // odd; the next writer finishes it.
    slot_seq_.fetch_or(1u, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    for (std::size_t i = 0; i < kSlotWords; ++i) {
        slot_[i].store(words[i], std::memory_order_relaxed);
    }
    uint32_t seq = slot_seq_.load(std::memory_order_relaxed);
    while (!slot_seq_.compare_exchange_weak(seq, (seq | 1u) + 1u, std::memory_order_release,
                                            std::memory_order_relaxed)) {
    }
    updates_.fetch_add(1, std::memory_order_relaxed);
}

bool Publisher::load(uint32_t& version, uint32_t (&words)[kSlotWords]) const {
    for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
        const uint32_t before = slot_seq_.load(std::memory_order_acquire);
        if (before == 0) {
            return false; // never written
        }
        if ((before & 1u) != 0) {
            continue;
        }
        for (std::size_t i = 0; i < kSlotWords; ++i) {
            words[i] = slot_[i].load(std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if (slot_seq_.load(std::memory_order_relaxed) != before ||
            checksum(words) != words[kChecksum]) {
            continue;
        }
        version = before;
        return true;
    }
    return false;
}

void Publisher::resetTrail() { epoch_.fetch_add(1, std::memory_order_relaxed); }

void Publisher::setRate(Hertz rate) {
    period_ms_.store(periodFor(rate), std::memory_order_relaxed);
}

Milliseconds Publisher::period() const { return period_ms_.load(std::memory_order_relaxed); }

Hertz Publisher::rate() const { return 1000.0 / static_cast<double>(period()); }

PublisherStats Publisher::stats() const {
    PublisherStats s;
    s.updates      = updates_.load(std::memory_order_relaxed);
    s.rejected     = rejected_.load(std::memory_order_relaxed);
    s.lines        = lines_.load(std::memory_order_relaxed);
    s.overwritten  = overwritten_.load(std::memory_order_relaxed);
    s.write_errors = write_errors_.load(std::memory_order_relaxed);
    return s;
}

SessionId Publisher::session() const { return session_.load(std::memory_order_relaxed); }

void Publisher::begin(SessionId session, Sequence first_seq) {
    next_seq_  = first_seq;
    scheduled_ = false;
    session_.store(session, std::memory_order_relaxed);
}

std::size_t Publisher::nextLine(Milliseconds now, char* out, std::size_t cap) {
    const SessionId session = session_.load(std::memory_order_relaxed);
    if (session == 0 || out == nullptr || cap < kLineCapacity) {
        return 0;
    }
    if (scheduled_ && static_cast<int32_t>(now - due_) < 0) {
        return 0;
    }
    uint32_t version = 0;
    uint32_t words[kSlotWords];
    if (!load(version, words) || version == sent_version_) {
        return 0;
    }

    protocol::PoseRecord record;
    record.session           = session;
    record.epoch             = static_cast<Epoch>(words[kEpochFlags] & 0xFFFFu);
    record.seq               = next_seq_;
    record.t_ms              = words[kTime];
    record.x_mm              = fromWord(words[kX]);
    record.y_mm              = fromWord(words[kY]);
    record.yaw_cdeg          = fromWord(words[kYaw]);
    record.roll_cdeg         = fromWord(words[kRoll]);
    record.pitch_cdeg        = fromWord(words[kPitch]);
    record.flags             = static_cast<uint8_t>(words[kEpochFlags] >> 16);
    const std::size_t length = protocol::encodeLine(record, out, cap);
    if (length == 0) {
        return 0;
    }

    // Each completed write adds 2 to the slot version.
    const uint32_t supplied = (version - sent_version_) / 2;
    if (supplied > 1) {
        overwritten_.fetch_add(supplied - 1, std::memory_order_relaxed);
    }
    sent_version_ = version;
    ++next_seq_;
    lines_.fetch_add(1, std::memory_order_relaxed);

    // Lines on a fixed grid; more than a period behind starts it over.
    const Milliseconds spacing = period();
    if (!scheduled_) {
        due_       = now + spacing;
        scheduled_ = true;
    } else {
        due_ += spacing;
        if (static_cast<int32_t>(now - due_) >= 0) {
            due_ = now + spacing;
        }
    }
    return length;
}

void Publisher::recordWrite(bool ok) {
    if (!ok) {
        write_errors_.fetch_add(1, std::memory_order_relaxed);
    }
}

} // namespace spectagatr
