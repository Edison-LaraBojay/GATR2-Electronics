cmake_minimum_required(VERSION 3.16)
project(navigatr CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

option(NAVIGATR_BUILD_TESTS "Build navigatr tests" ON)

# Vendored, built without our warning set.
add_library(navigatr_tinyxml2 STATIC third_party/tinyxml2/tinyxml2.cpp)
target_include_directories(navigatr_tinyxml2 SYSTEM PUBLIC third_party)

# Core library.
set(NAVIGATR_SOURCES
    src/config/config_node.cpp
    src/config/field_map.cpp
    src/config/composition.cpp
    src/transport/file_stream.cpp
    src/transport/serial_port.cpp
    src/resources/resource_map.cpp
    src/impl/resources/serial_links.cpp
    src/impl/resources/pico_telemetry.cpp
    src/impl/resources/field_map_resource.cpp
    src/impl/resources/robot_frame_map.cpp
    src/impl/resources/cameras.cpp
    src/impl/resources/tag_detectors.cpp
    src/impl/resources/target_set_resource.cpp
    src/impl/resources/wheel_geometry_resource.cpp
    src/impl/resources/register_resources.cpp
    src/impl/sensors/pico_channels.cpp
    src/impl/sensors/camera_channel.cpp
    src/impl/sensors/register_sensors.cpp
    src/impl/noop/noops.cpp
    src/impl/commands/vex_brain_serial.cpp
    src/impl/commands/register_commands.cpp
    src/impl/preprocessing/configured_collection.cpp
    src/impl/preprocessing/tracking_wheel_odometry.cpp
    src/impl/preprocessing/imu_normalization.cpp
    src/impl/preprocessing/register_preprocessing.cpp
    src/impl/localization/wheel_imu_prediction.cpp
    src/impl/localization/register_localization.cpp
    src/impl/perception/apriltag_tag_observation.cpp
    src/impl/perception/register_perception.cpp
    src/impl/association/tag_mount_association.cpp
    src/impl/association/register_association.cpp
    src/impl/world_estimation/landmark_world.cpp
    src/impl/target_resolution/configured_targets.cpp
    src/impl/world_estimation/register_world_estimation.cpp
    src/impl/target_resolution/register_target_resolution.cpp
    src/impl/publishing/vex_brain.cpp
    src/impl/publishing/register_publishers.cpp
    src/runtime/register_all.cpp
    src/runtime/system.cpp
)

add_library(navigatr_core STATIC
    ${NAVIGATR_SOURCES}
    ${REPO_ROOT}/common/frame_codec.cpp
)
target_include_directories(navigatr_core PUBLIC src ${REPO_ROOT})
target_link_libraries(navigatr_core PUBLIC navigatr_tinyxml2)

if(NOT MSVC)
    target_compile_options(navigatr_core PRIVATE -Wall -Wextra)
endif()

# App.
add_executable(navigatr src/app/main.cpp)
target_link_libraries(navigatr PRIVATE navigatr_core)
if(NOT MSVC)
    target_compile_options(navigatr PRIVATE -Wall -Wextra)
endif()

if(NAVIGATR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
