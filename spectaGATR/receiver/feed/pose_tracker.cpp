// pose_tracker.cpp

#include "receiver/feed/pose_tracker.h"

namespace spectagatr::receiver::feed
{

namespace
{

// A serial stream does not reorder lines, so seq falling back by more than
// this within one session and epoch is a new publisher run whose session id
// collided with the old one (best-effort ids), not a late line.
constexpr int32_t kRestartJump = 64;

} // namespace

int32_t sequenceDistance(protocol::Sequence seq, protocol::Sequence last) {
    const uint32_t difference = seq - last;
    // Two's complement without relying on an implementation-defined narrowing.
    return difference <= 0x7FFFFFFFu ? static_cast<int32_t>(difference)
                                     : -static_cast<int32_t>(~difference) - 1;
}

std::optional<TrackedPose> PoseTracker::offer(const protocol::DecodeResult& decoded,
                                              RxMillis                      rx_ms) {
    switch (decoded.error) {
    case protocol::DecodeError::kNone:
        break;
    case protocol::DecodeError::kBadCrc:
        ++counts_.bad_crc;
        verdict_ = Verdict::kBadCrc;
        return std::nullopt;
    case protocol::DecodeError::kOutOfRange:
        ++counts_.out_of_range;
        verdict_ = Verdict::kOutOfRange;
        return std::nullopt;
    case protocol::DecodeError::kMalformed:
        ++counts_.malformed;
        verdict_ = Verdict::kMalformed;
        return std::nullopt;
    }

    const protocol::PoseRecord& record  = decoded.record;
    bool                        restart = false;
    if (!last_ || last_->session != record.session) {
        restart = true; // a publisher run, or the first pose seen
        ++counts_.sessions;
    } else if (last_->epoch != record.epoch) {
        restart = true; // an explicit pose reset
    } else {
        const int32_t distance = sequenceDistance(record.seq, last_->seq);
        if (distance == 0) {
            ++counts_.repeated;
            verdict_ = Verdict::kRepeated;
            return std::nullopt;
        }
        if (distance < -kRestartJump) {
            restart = true; // a restarted publisher with the same session id
            ++counts_.sessions;
        } else if (distance < 0) {
            ++counts_.out_of_order;
            verdict_ = Verdict::kOutOfOrder;
            return std::nullopt;
        }
    }
    last_    = Position{record.session, record.epoch, record.seq};
    last_rx_ = rx_ms;
    ++counts_.poses;
    verdict_ = Verdict::kAccepted;
    return TrackedPose{record, restart, rx_ms};
}

} // namespace spectagatr::receiver::feed
