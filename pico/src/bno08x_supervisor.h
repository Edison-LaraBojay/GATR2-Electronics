// bno08x_supervisor.h
// BNO08X bring-up and recovery. Driven by time and the sh2 events seen since
// the last step; returns the one action the driver performs next.

#pragma once
#include <stdint.h>

namespace bno08x
{

constexpr uint32_t kBootUs       = 500000;  // release to SH2_RESET
constexpr uint32_t kAckUs        = 200000;  // enable to Get Feature Response
constexpr uint32_t kStaleUs      = 100000;  // run with no report
constexpr uint32_t kStableUs     = 1000000; // run time that resets the backoff
constexpr uint32_t kBackoffMinUs = 500000;
constexpr uint32_t kBackoffMaxUs = 8000000;

enum class State : uint8_t {
    Reset,   // RST held low, PS0/WAKE high
    Boot,    // released, waiting for SH2_RESET
    WaitAck, // report enabled, waiting for its Get Feature Response
    Run,
};

enum class Action : uint8_t {
    None,
    HoldReset,    // assert RST, WAKE high, drop cached samples
    ReleaseReset, // release RST
    Enable,       // drop cached samples, enable acceleration and gyro reports
};

// Seen since the previous step.
struct Events {
    bool reset         = false; // SH2_RESET, the hub (re)booted
    bool ack           = false; // both feature responses received, nonzero intervals
    bool report        = false; // new gyro report with a fresh acceleration stream
    bool enable_failed = false; // the enable write returned an error
};

constexpr bool dropsSamples(Action a) { return a == Action::HoldReset || a == Action::Enable; }

class Supervisor {
  public:
    // Reset released at now_us. Boot wait starts.
    void start(uint32_t now_us);

    Action step(uint32_t now_us, const Events& ev);

    State state() const { return state_; }

    // Reset hold after the next failure.
    uint32_t backoffUs() const { return backoff_us_; }

  private:
    Action fail(uint32_t now_us);
    Action enable(uint32_t now_us);

    State    state_          = State::Reset;
    uint32_t since_us_       = 0;
    uint32_t hold_us_        = 0;
    uint32_t backoff_us_     = kBackoffMinUs;
    uint32_t last_report_us_ = 0;
};

} // namespace bno08x
