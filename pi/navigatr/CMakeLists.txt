cmake_minimum_required(VERSION 3.16)
project(navigatr CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

option(NAVIGATR_BUILD_TESTS "Build navigatr tests" ON)

# Vendored, built without our warning set.
add_library(navigatr_tinyxml2 STATIC third_party/tinyxml2/tinyxml2.cpp)
target_include_directories(navigatr_tinyxml2 SYSTEM PUBLIC third_party)

# Vendored AprilRobotics apriltag (BSD-2), core plus the families this
# project can use, built as C without our warning set.
set(NAVIGATR_APRILTAG_SOURCES
    third_party/apriltag/apriltag.c
    third_party/apriltag/apriltag_pose.c
    third_party/apriltag/apriltag_quad_thresh.c
    third_party/apriltag/tag16h5.c
    third_party/apriltag/tag25h9.c
    third_party/apriltag/tag36h10.c
    third_party/apriltag/tag36h11.c
    third_party/apriltag/tagCircle21h7.c
    third_party/apriltag/tagStandard41h12.c
    third_party/apriltag/common/g2d.c
    third_party/apriltag/common/homography.c
    third_party/apriltag/common/image_u8.c
    third_party/apriltag/common/image_u8_parallel.c
    third_party/apriltag/common/image_u8x3.c
    third_party/apriltag/common/image_u8x4.c
    third_party/apriltag/common/matd.c
    third_party/apriltag/common/pam.c
    third_party/apriltag/common/pnm.c
    third_party/apriltag/common/pthreads_cross.c
    third_party/apriltag/common/string_util.c
    third_party/apriltag/common/svd22.c
    third_party/apriltag/common/time_util.c
    third_party/apriltag/common/unionfind.c
    third_party/apriltag/common/workerpool.c
    third_party/apriltag/common/zarray.c
    third_party/apriltag/common/zhash.c
    third_party/apriltag/common/zmaxheap.c
)
enable_language(C)
if(MINGW)
    # MinGW ships winpthreads. The upstream Windows shim would define global
    # pthread symbols that override it for the whole process, so use the real
    # pthread header instead of the shim.
    list(REMOVE_ITEM NAVIGATR_APRILTAG_SOURCES third_party/apriltag/common/pthreads_cross.c)
endif()
add_library(navigatr_apriltag STATIC ${NAVIGATR_APRILTAG_SOURCES})
if(MINGW)
    target_compile_options(navigatr_apriltag PRIVATE -D__CPTHREAD_H__ -include pthread.h)
endif()
target_include_directories(navigatr_apriltag SYSTEM PUBLIC third_party/apriltag)
set_target_properties(navigatr_apriltag PROPERTIES C_STANDARD 99)
if(NOT MSVC)
    target_compile_options(navigatr_apriltag PRIVATE -w)
endif()
# upstream debug_print() spams stderr from the pose solver without NDEBUG
target_compile_definitions(navigatr_apriltag PRIVATE NDEBUG)
find_package(Threads REQUIRED)
target_link_libraries(navigatr_apriltag PUBLIC Threads::Threads)

# Optional Raspberry Pi capture backend (Linux, libcamera-dev).
option(NAVIGATR_WITH_LIBCAMERA "Build the libcamera capture backend" OFF)

# Core library.
set(NAVIGATR_SOURCES
    src/config/config_node.cpp
    src/config/field_map.cpp
    src/config/composition.cpp
    src/transport/file_stream.cpp
    src/transport/serial_port.cpp
    src/resources/resource_store.cpp
    src/impl/resources/serial_links.cpp
    src/impl/resources/pico_telemetry.cpp
    src/impl/resources/field_map_resource.cpp
    src/impl/resources/robot_frame_map.cpp
    src/impl/resources/cameras.cpp
    src/impl/resources/apriltag_c.c
    src/impl/resources/tag_detectors.cpp
    src/impl/resources/target_set_resource.cpp
    src/impl/resources/wheel_geometry_resource.cpp
    src/impl/resources/synthetic_rig.cpp
    src/impl/resources/register_resources.cpp
    src/impl/sensors/pico_channels.cpp
    src/impl/sensors/camera_channel.cpp
    src/impl/sensors/attitude_channel.cpp
    src/impl/sensors/register_sensors.cpp
    src/impl/noop/noops.cpp
    src/impl/commands/vex_brain_serial.cpp
    src/impl/commands/register_commands.cpp
    src/impl/localization/tracking_wheel_motion.cpp
    src/impl/localization/imu_heading_increment.cpp
    src/impl/localization/attitude_reference.cpp
    src/impl/localization/motion_step.cpp
    src/impl/localization/planar_motion_integrator.cpp
    src/impl/localization/weighted_planar_fusion.cpp
    src/impl/localization/register_localization.cpp
    src/impl/world_estimation/apriltag_tag_observation.cpp
    src/impl/world_estimation/tag_mount_association.cpp
    src/impl/world_estimation/landmark_estimator.cpp
    src/impl/world_estimation/apriltag_world_estimator.cpp
    src/impl/world_estimation/register_world_estimation.cpp
    src/impl/target_resolution/configured_targets.cpp
    src/impl/target_resolution/register_target_resolution.cpp
    src/impl/publishing/vex_brain.cpp
    src/impl/publishing/register_publishers.cpp
    src/state/pose_history.cpp
    src/state/robot_state_feed.cpp
    src/runtime/localization_stage.cpp
    src/runtime/register_all.cpp
    src/runtime/resource_stage.cpp
    src/runtime/sensor_stage.cpp
    src/runtime/world_estimation_stage.cpp
    src/runtime/system.cpp
    src/inspection/inspection_document.cpp
    src/inspection/inspection_service.cpp
    src/inspection/http_server.cpp
    src/inspection/jpeg_encoder.cpp
)

# The browser inspector, compiled in. viewer/ holds the app, third_party/three
# the pinned three.js module and OrbitControls served under vendor/.
file(GLOB NAVIGATR_VIEWER_FILES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/viewer/*.html
    ${CMAKE_CURRENT_SOURCE_DIR}/viewer/*.js
    ${CMAKE_CURRENT_SOURCE_DIR}/viewer/*.css
    ${CMAKE_CURRENT_SOURCE_DIR}/viewer/*.json
    ${CMAKE_CURRENT_SOURCE_DIR}/viewer/*.svg
)
set(NAVIGATR_EMBED_ENTRIES "")
foreach(f IN LISTS NAVIGATR_VIEWER_FILES)
    get_filename_component(name "${f}" NAME)
    list(APPEND NAVIGATR_EMBED_ENTRIES "${name}=viewer/${name}")
endforeach()
list(APPEND NAVIGATR_EMBED_ENTRIES
    "vendor/three.module.min.js=third_party/three/three.module.min.js"
    "vendor/OrbitControls.js=third_party/three/OrbitControls.js"
    "vendor/LICENSE=third_party/three/LICENSE"
)
set(NAVIGATR_VIEWER_ASSETS_CPP ${CMAKE_CURRENT_BINARY_DIR}/generated/viewer_assets.cpp)
add_custom_command(
    OUTPUT ${NAVIGATR_VIEWER_ASSETS_CPP}
    COMMAND ${CMAKE_COMMAND}
        -DOUTPUT=${NAVIGATR_VIEWER_ASSETS_CPP}
        -DROOT=${CMAKE_CURRENT_SOURCE_DIR}
        "-DFILES=${NAVIGATR_EMBED_ENTRIES}"
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_assets.cmake
    DEPENDS ${NAVIGATR_VIEWER_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/three/three.module.min.js
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/three/OrbitControls.js
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/three/LICENSE
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_assets.cmake
    COMMENT "Embedding the viewer"
    VERBATIM
)

add_library(navigatr_core STATIC
    ${NAVIGATR_SOURCES}
    ${NAVIGATR_VIEWER_ASSETS_CPP}
    ${REPO_ROOT}/common/frame_codec.cpp
)
target_include_directories(navigatr_core PUBLIC src ${REPO_ROOT})
target_link_libraries(navigatr_core PUBLIC navigatr_tinyxml2 navigatr_apriltag Threads::Threads)
target_include_directories(navigatr_core SYSTEM PRIVATE third_party/stb)
if(WIN32)
    target_link_libraries(navigatr_core PUBLIC ws2_32)
endif()

if(NAVIGATR_WITH_LIBCAMERA)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
    target_sources(navigatr_core PRIVATE src/impl/resources/libcamera_camera.cpp)
    target_link_libraries(navigatr_core PUBLIC PkgConfig::LIBCAMERA)
    target_compile_definitions(navigatr_core PUBLIC NAVIGATR_HAVE_LIBCAMERA=1)
    # libcamera headers from 0.7.1 refuse C++17; the ABI is unaffected
    target_compile_features(navigatr_core PUBLIC cxx_std_20)
endif()

if(NOT MSVC)
    target_compile_options(navigatr_core PRIVATE -Wall -Wextra)
endif()

# App.
set(NAVIGATR_DEFAULT_CONFIG "config/demo/synthetic_field_demo.xml" CACHE STRING
    "Default XML file (absolute, or relative to the navigatr source directory)")
if(NAVIGATR_DEFAULT_CONFIG STREQUAL "")
    message(FATAL_ERROR "NAVIGATR_DEFAULT_CONFIG must name an XML file")
endif()
get_filename_component(NAVIGATR_DEFAULT_CONFIG_ABSOLUTE "${NAVIGATR_DEFAULT_CONFIG}"
    ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
string(REPLACE "\\" "\\\\" NAVIGATR_DEFAULT_CONFIG_ESCAPED "${NAVIGATR_DEFAULT_CONFIG_ABSOLUTE}")
string(REPLACE "\"" "\\\"" NAVIGATR_DEFAULT_CONFIG_ESCAPED "${NAVIGATR_DEFAULT_CONFIG_ESCAPED}")
string(REPLACE "\n" "\\n" NAVIGATR_DEFAULT_CONFIG_ESCAPED "${NAVIGATR_DEFAULT_CONFIG_ESCAPED}")
string(REPLACE "\r" "\\r" NAVIGATR_DEFAULT_CONFIG_ESCAPED "${NAVIGATR_DEFAULT_CONFIG_ESCAPED}")
configure_file(src/app/build_config.h.in generated/navigatr_build_config.h @ONLY)
add_executable(navigatr src/app/main.cpp src/app/command_line.cpp)
target_include_directories(navigatr PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(navigatr PRIVATE navigatr_core)
if(NOT MSVC)
    target_compile_options(navigatr PRIVATE -Wall -Wextra)
endif()

if(NAVIGATR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
