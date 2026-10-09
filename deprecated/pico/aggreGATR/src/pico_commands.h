// pico_commands.h
// Pi -> Pico commands: target check, duplicate records, CONFIGURE, REINIT_IMU,
// RESTART_ACQUISITION and DIAGNOSTICS, and the last_* fields of the status
// frame. The firmware applies the returned effect at once. No hardware
// dependencies.

#pragma once
#include <stdint.h>

#include "command_records.h"
#include "frames.h"

namespace pilink
{

// Highest DIAGNOSTICS rate; larger bodies fail with BadBody.
constexpr uint8_t kMaxDiagHz = 5;

enum class Effect : uint8_t {
    None,
    EnableImu,
    DisableImu,
    ReinitImu,          // new IMU episode, imu_epoch + 1
    RestartAcquisition, // zero the encoder counters, acq_epoch + 1
    SetDiagnostics,     // diagnostic frames at Outcome::diag_hz, 0 off
};

struct Outcome {
    bool    answered = false; // last_* changed, a status frame is due
    Effect  effect   = Effect::None;
    uint8_t diag_hz  = 0;     // SetDiagnostics
};

class Commands {
  public:
    // One whole kFramePicoCommand frame, as FrameReader buffered it.
    // Ignored (not answered): another link version, request_id 0.
    // Failed, not recorded: wrong target, unknown op, body length wrong for the op.
    // Recorded: every other command for this boot, so a resend is a duplicate.
    Outcome receive(const uint8_t* frame, uint16_t len, uint16_t boot_id, bool imu_enabled);

    // IMU port 0 state (translagatr::PicoImuState) after the effect was applied.
    // Settles running REINIT_IMU records: ready completes them; failed or
    // disabled fails them. True when a record changed.
    bool imuProgress(uint8_t imu_state);

    // last_request_id, last_op, last_status and last_detail: the newest
    // command received, or the record a duplicate matched.
    void fill(translagatr::PicoStatus& status) const;

    uint32_t received() const { return received_; }
    uint32_t duplicates() const { return duplicates_; }
    uint32_t ignored() const { return ignored_; }

  private:
    struct Last {
        uint16_t request_id = 0;
        uint8_t  op         = 0;
        uint8_t  status     = translagatr::kPicoCommandNone;
        uint8_t  detail     = translagatr::kPicoDetailNone;
        bool     recorded   = false;
    };

    Outcome run(const translagatr::PicoCommand& c, uint8_t body, bool imu_enabled);
    Outcome answer(const translagatr::PicoCommand& c, uint8_t status, uint8_t detail);
    Outcome record(const translagatr::PicoCommand& c, uint8_t body, uint8_t status, uint8_t detail,
                   Effect effect);

    CommandRecords records_;
    Last           last_;
    uint32_t       received_   = 0;
    uint32_t       duplicates_ = 0;
    uint32_t       ignored_    = 0;
};

} // namespace pilink
