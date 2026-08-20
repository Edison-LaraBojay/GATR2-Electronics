// register_resources.cpp

#include "impl/resources/cameras.h"
#include "impl/resources/field_map_resource.h"
#include "impl/resources/pico_telemetry.h"
#include "impl/resources/robot_frame_map.h"
#include "impl/resources/serial_links.h"
#include "impl/resources/tag_detectors.h"
#include "impl/resources/target_set_resource.h"
#include "impl/resources/wheel_geometry_resource.h"
#include "resources/resource_map.h"
#include "runtime/register_all.h"

namespace navigatr
{

void register_resources(FunctionRegistry& functions) {
    registerOrDie<ResourceMakeFunction>(functions, "linux_serial_link",
                                        &make_linux_serial_link);
    registerOrDie<ResourceMakeFunction>(functions, "memory_link", &make_memory_link);
    registerOrDie<ResourceMakeFunction>(functions, "file_replay_link",
                                        &make_file_replay_link);
    registerOrDie<ResourceMakeFunction>(functions, "pico_telemetry", &make_pico_telemetry);
    registerOrDie<ResourceMakeFunction>(functions, "field_map", &make_field_map);
    registerOrDie<ResourceMakeFunction>(functions, "robot_frame_map",
                                        &make_robot_frame_map);
    registerOrDie<ResourceMakeFunction>(functions, "libcamera_camera",
                                        &make_libcamera_camera);
    registerOrDie<ResourceMakeFunction>(functions, "apriltag_detector",
                                        &make_apriltag_detector);
    registerOrDie<ResourceMakeFunction>(functions, "target_set", &make_target_set);
    registerOrDie<ResourceMakeFunction>(functions, "wheel_geometry",
                                        &make_wheel_geometry);
}

} // namespace navigatr
