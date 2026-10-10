// stop_signal.cpp

#include "receiver/platform/stop_signal.h"

namespace spectagatr::receiver::platform
{

void StopSignal::request() {
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        requested_ = true;
    }
    wake_.notify_all();
}

bool StopSignal::requested() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return requested_;
}

bool StopSignal::waitFor(Milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return wake_.wait_for(lock, timeout, [this] { return requested_; });
}

} // namespace spectagatr::receiver::platform
