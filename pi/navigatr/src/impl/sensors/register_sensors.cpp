// register_sensors.cpp

#include "impl/sensors/pico_channels.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_sensors(FunctionRegistry& functions) {
    registerOrDie<SensorMakeFunction>(functions, "pico_encoder_channel",
                                      &make_pico_encoder_channel);
    registerOrDie<SensorMakeFunction>(functions, "pico_imu_channel",
                                      &make_pico_imu_channel);
}

} // namespace navigatr
