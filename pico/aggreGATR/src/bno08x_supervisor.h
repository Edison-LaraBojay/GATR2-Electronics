// bno08x_supervisor.h
// BNO08X bring-up and recovery. Driven by time and the sh2 events seen since
// the last step; returns the one action the driver performs next. Reset holds
// between attempts come from imu::RetryPolicy.

#pragma once
#include <stdint.h>

#include "imu_retry.h"

namespace bno08x
{

constexpr uint32_t kBootUs        = 500000; // release to SH2_RESET
constexpr uint32_t kAckUs         = 200000; // enable to Get Feature Response
constexpr uint32_t kStaleUs       = 100000; // run with no report
constexpr uint32_t kRestartHoldUs = 10000;  // reset pulse before a requested restart

enum class State : uint8_t {
    Off,     // disabled, RST held low
    Reset,   // RST held low, PS0/WAKE high, until the next attempt
    Boot,    // released, waiting for SH2_RESET
    WaitAck, // report enabled, waiting for its Get Feature Response
    Run,
};

enum class Action : uint8_t {
    None,
    HoldReset,    // assert RST, WAKE high, drop cached samples
    ReleaseReset, // release RST (open sh2 first if it is not open)
    Enable,       // drop cached samples, enable acceleration and gyro reports
};

// Seen since the previous step.
struct Events {
    bool reset         = false; // SH2_RESET, the hub (re)booted
    bool ack           = false; // both feature responses received, nonzero intervals
    bool report        = false; // new gyro report with a fresh acceleration stream
    bool enable_failed = false; // the enable write returned an error
    bool open_failed   = false; // sh2_open returned an error
};

constexpr bool dropsSamples(Action a) { return a == Action::HoldReset || a == Action::Enable; }

class Supervisor {
  public:
    // First attempt: reset released at now_us, boot wait starts.
    void start(uint32_t now_us);

    // New episode (enable, reinit request): reset pulse, then a first attempt.
    Action restart(uint32_t now_us);

    // Disabled: reset held until restart.
    Action stop();

    Action step(uint32_t now_us, const Events& ev);

    State state() const { return state_; }

    // Reset hold of the current or last wait.
    uint32_t holdUs() const { return hold_us_; }

    const imu::RetryPolicy& retry() const { return retry_; }

    // (Re)initializations started this boot: start, restart, each failure,
    // and each hub reset seen after boot.
    uint8_t epoch() const { return epoch_; }

  private:
    void   release(uint32_t now_us);
    Action fail(uint32_t now_us, uint8_t reason);
    Action enable(uint32_t now_us);

    State            state_          = State::Reset;
    uint32_t         since_us_       = 0;
    uint32_t         hold_us_        = 0;
    uint32_t         last_report_us_ = 0;
    uint8_t          epoch_          = 0;
    imu::RetryPolicy retry_;
};

} // namespace bno08x
