// register_sensors.cpp

#include "impl/sensors/camera_channel.h"
#include "impl/sensors/pico_channels.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_sensors(FunctionRegistry& functions) {
    registerOrDie<SensorMakeFunction>(functions, "pico_encoder_channel",
                                      &make_pico_encoder_channel);
    registerOrDie<SensorMakeFunction>(functions, "pico_imu_channel",
                                      &make_pico_imu_channel);
    registerOrDie<SensorMakeFunction>(functions, "camera_frame", &make_camera_frame);
}

} // namespace navigatr
