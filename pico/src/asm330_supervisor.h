// asm330_supervisor.h
// ASM330 bring-up and health without blocking. step() returns the one
// register action the driver performs next; the driver reports its result
// before the following step. Waits between attempts come from
// imu::RetryPolicy.

#pragma once
#include <stdint.h>

#include "imu_retry.h"

namespace asm330
{

constexpr uint32_t kResetUs       = 100000; // software reset self-clear bound
constexpr uint32_t kPollUs        = 1000;   // reset bit poll period
constexpr uint32_t kCheckUs       = 250000; // health check period while running
constexpr uint8_t  kCheckFailures = 2;      // consecutive failed checks that restart the chip

enum class State : uint8_t {
    Off,       // disabled
    Wait,      // until the next attempt
    Probe,     // WHO_AM_I read and software reset requested
    Reset,     // waiting for the software reset to clear
    Configure, // configuration written and read back
    Run,
};

enum class Action : uint8_t {
    None,
    Probe,     // read WHO_AM_I; on a match write SW_RESET; report probed()
    PollReset, // read CTRL3_C; report resetPolled()
    Configure, // write the configuration and read it back; report configured()
    Check,     // read WHO_AM_I and CTRL2_G; report checked()
};

class Supervisor {
  public:
    // First attempt due now.
    void start(uint32_t now_us);

    // New episode (enable, reinit request), first attempt due now.
    void restart(uint32_t now_us);

    // Disabled until restart.
    void stop();

    Action step(uint32_t now_us);

    void probed(uint32_t now_us, bool who_am_i_ok);
    void resetPolled(uint32_t now_us, bool cleared);
    void configured(uint32_t now_us, bool ok);

    // gatr2::PicoImuReason: none when healthy, no response for a wrong
    // WHO_AM_I, stream when the chip lost its configuration. kCheckFailures
    // failures in a row restart the chip.
    void checked(uint32_t now_us, uint8_t reason);

    State state() const { return state_; }
    bool  running() const { return state_ == State::Run; }

    // Wait before the current or last attempt.
    uint32_t holdUs() const { return hold_us_; }

    const imu::RetryPolicy& retry() const { return retry_; }

    // (Re)initializations started this boot, one per probe.
    uint8_t epoch() const { return epoch_; }

  private:
    void fail(uint32_t now_us, uint8_t reason);

    State            state_          = State::Off;
    uint32_t         since_us_       = 0;
    uint32_t         hold_us_        = 0;
    uint32_t         last_poll_us_   = 0;
    uint32_t         last_check_us_  = 0;
    uint8_t          check_failures_ = 0;
    uint8_t          epoch_          = 0;
    imu::RetryPolicy retry_;
};

} // namespace asm330
