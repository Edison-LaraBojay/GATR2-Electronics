// imu.h
// IMU port 0: yaw rate source over SPI1, one IMU model per build
// (GATR2_IMU_BNO08X or GATR2_IMU_ASM330). A missing device or no new
// measurement leaves the gyro bit clear in the outgoing sensor frame.
// While enabled, a failed device is retried by the driver (imu_retry.h).

#pragma once
#include <stddef.h>
#include <stdint.h>

namespace imu
{

// IMU fields of the Pico status frame.
struct Status {
    bool     enabled  = false;
    uint8_t  state    = 0; // translagatr::PicoImuState
    uint8_t  reason   = 0; // translagatr::PicoImuReason
    uint16_t attempts = 0; // in the current episode
    uint8_t  epoch    = 0; // (re)initializations started this boot
};

// Enabled at boot; starts the first attempt. The only bring-up that blocks
// is the BNO08X sh2_open (see imu_bno08x.cpp).
void begin();

// Services the device without blocking. Call every loop pass.
void service();

// Disabling holds the device idle and stops retries. Enabling starts a new
// episode. Unchanged settings do nothing.
void setEnabled(bool enabled);

// New episode from a device reset while enabled; imu_epoch + 1.
void reinit();

Status status();

// translagatr::PicoFirmware of this build.
uint8_t firmware();

// Yaw rate in millidegrees per second, bias not removed. cut_us is the
// tick's time snapshot on the time_us_32() clock. True only for a new
// measurement since the previous true return.
// BNO08X: startup acceleration learns up for a fixed mounting while the robot
// is stationary and level. XYZ gyro is projected onto that axis; no output
// before alignment. Mean of reports sampled in (previous cut, cut_us].
// ASM330: newest sample, when the status register flags new data; cut_us unused.
bool readGyroZ(uint32_t cut_us, int32_t& gyro_z_mdps);

// USB bring-up diagnostics only. Formats a snapshot without consuming samples.
// Returns the number of bytes written, or zero if capacity is too small.
size_t formatDiagnostics(char* output, size_t capacity);

// Driver counters for the Pi diagnostic frame, free-running since boot,
// from state the driver already keeps (no extra bus traffic).
//   BNO08X: rx SHTP packets read, bad nonzero headers without a length,
//           resets hub reset events (SH2_RESET), error the last sh2 result
//           (SH2_ERR_*, 0 none), reports accepted and rejected sensor reports.
//   ASM330: rx gyro samples read, bad failed probes, configurations and
//           health checks, resets software resets issued, error the last
//           failure reason (translagatr::PicoImuReason), reports_ok = rx,
//           reports_rejected not applicable (always 0).
struct Counters {
    uint32_t rx               = 0;
    uint32_t bad              = 0;
    uint32_t resets           = 0;
    int32_t  error            = 0;
    uint32_t reports_ok       = 0;
    uint32_t reports_rejected = 0;
    bool     have_report      = false; // a report was accepted this boot
    uint32_t report_age_ms    = 0;     // since the newest accepted report
};

Counters counters();

} // namespace imu
