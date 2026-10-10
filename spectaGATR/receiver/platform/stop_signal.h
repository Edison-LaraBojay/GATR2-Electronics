// stop_signal.h
// A stop request that wakes every waiter at once. Timed waits here stand in for sleeps: a
// Windows sleep_for(1 ms) takes about 15 ms, a notify wakes in well under 1 ms.

#pragma once
#include <condition_variable>
#include <mutex>

#include "receiver/platform/clock.h"

namespace spectagatr::receiver::platform
{

class StopSignal {
  public:
    void request();
    bool requested() const;

    // True once stop is requested, at once or before timeout ends.
    [[nodiscard]] bool waitFor(Milliseconds timeout);

  private:
    mutable std::mutex      mutex_;
    std::condition_variable wake_;
    bool                    requested_ = false;
};

} // namespace spectagatr::receiver::platform
