// imu.h
// ASM330LHHG1 yaw rate source over SPI1. A missing device or unavailable
// sample leaves the gyro bit clear in the outgoing sensor frame.

#pragma once
#include <stdint.h>

namespace imu
{

void begin();

// Raw yaw rate in millidegrees per second, bias not removed.
// False when no sample is available.
bool readGyroZ(int32_t& gyro_z_mdps);

} // namespace imu
