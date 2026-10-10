// forwarder.cpp

#include "forwarder.h"

namespace relay
{

std::size_t Forwarder::receive(const uint8_t* data, std::size_t size) {
    std::size_t found = 0;
    for (std::size_t i = 0; i < size; ++i) {
        if (!scanner_.push(data[i])) {
            continue;
        }
        if (has_next_) {
            ++stats_.superseded;
        }
        // pending() may have exposed this line for a nonblocking write that
        // made no progress. Until a byte is sent it can still be replaced.
        if (has_current_ && current_at_ == 0) {
            ++stats_.superseded;
            has_current_ = false;
        }
        next_     = encoder_record::encodeLine(scanner_.bytes());
        has_next_ = true;
        ++stats_.records;
        ++found;
    }
    return found;
}

void Forwarder::dropPending() {
    stats_.discarded += (has_next_ ? 1 : 0) + (has_current_ ? 1 : 0);
    has_next_    = false;
    has_current_ = false;
    current_at_  = 0;
}

Pending Forwarder::pending() {
    if (!has_current_ && has_next_) {
        current_     = next_;
        current_at_  = 0;
        has_current_ = true;
        has_next_    = false;
    }
    if (!has_current_) {
        return {};
    }
    return {current_.data() + current_at_, current_.size() - current_at_};
}

void Forwarder::wrote(std::size_t count) {
    if (!has_current_) {
        return;
    }
    current_at_ += count;
    if (current_at_ >= current_.size()) {
        has_current_ = false;
        current_at_  = 0;
        ++stats_.lines;
    }
}

} // namespace relay
