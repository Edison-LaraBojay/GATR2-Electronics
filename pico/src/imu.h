// imu.h
// Yaw rate source. No chip is selected yet, so read returns false and the
// gyro bit stays clear in the frame. Fill in a driver and the bit turns on
// with no change anywhere else.

#pragma once
#include <stdint.h>

namespace imu
{

void begin();

// Raw yaw rate in millidegrees per second, bias not removed.
// False when no sample is available.
bool readGyroZ(int32_t& gyro_z_mdps);

} // namespace imu
