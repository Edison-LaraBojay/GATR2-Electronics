// imu.h
// Yaw rate source over SPI1, one IMU model per build (GATR2_IMU_BNO08X or
// GATR2_IMU_ASM330). A missing device or no new measurement leaves the gyro
// bit clear in the outgoing sensor frame.

#pragma once
#include <stddef.h>
#include <stdint.h>

namespace imu
{

void begin();

// Services the device without blocking. Call every loop pass.
void service();

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

} // namespace imu
