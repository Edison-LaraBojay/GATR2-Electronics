# System gtest when present (CI, the Pi), fetched pinned copy otherwise.
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    include(FetchContent)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# Test files.
set(TEST_SOURCES
    command_line_gtest.cpp
    ../src/app/command_line.cpp
    core_gtest.cpp
    transforms_gtest.cpp
    se3_gtest.cpp
    resources_gtest.cpp
    pros_usb_link_gtest.cpp
    half_duplex_gtest.cpp
    geometry_resources_gtest.cpp
    sensors_gtest.cpp
    stages_gtest.cpp
    apriltag_gtest.cpp
    synthetic_rig_gtest.cpp
    pose_history_gtest.cpp
    attitude_gtest.cpp
    localization_models_gtest.cpp
    localization_gtest.cpp
    parallel_wheels_gtest.cpp
    localization_fusion_gtest.cpp
    brain_io_gtest.cpp
    brain_imu_bench_gtest.cpp
    world_estimation_gtest.cpp
    vision_targets_gtest.cpp
    system_build_gtest.cpp
    profiles_gtest.cpp
    end_to_end_gtest.cpp
    brain_link_e2e_gtest.cpp
    inspection_gtest.cpp
    workers_gtest.cpp
    field_assets_gtest.cpp
    capture_transform_gtest.cpp
    camera_capture_gtest.cpp
    inspection_document_gtest.cpp
)

# Brain libraries for brain_link_e2e_gtest.cpp, compiled in from brain/: the
# portable sources plus the host drivetrain sim. common/frame_codec.cpp comes
# with navigatr_core.
set(BRAIN_DIR ${REPO_ROOT}/brain)
set(BRAIN_SOURCES
    ${BRAIN_DIR}/investiGATR/src/drive.cpp
    ${BRAIN_DIR}/investiGATR/src/geometry.cpp
    ${BRAIN_DIR}/investiGATR/src/navigator.cpp
    ${BRAIN_DIR}/investiGATR/src/odometry_source.cpp
    ${BRAIN_DIR}/investiGATR/src/pid.cpp
    ${BRAIN_DIR}/investiGATR/sim/differential_drive_sim.cpp
    ${BRAIN_DIR}/communiGATR/src/client.cpp
    ${BRAIN_DIR}/communiGATR/src/driver.cpp
)

add_executable(navigatr_tests ${TEST_SOURCES} ${BRAIN_SOURCES})
target_compile_definitions(navigatr_tests PRIVATE
    NAVIGATR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config")
target_include_directories(navigatr_tests PRIVATE
    ${BRAIN_DIR}/investiGATR/include
    ${BRAIN_DIR}/investiGATR
    ${BRAIN_DIR}/communiGATR/include
)
target_link_libraries(navigatr_tests PRIVATE navigatr_core GTest::gtest_main)
if(NOT MSVC)
    target_compile_options(navigatr_tests PRIVATE -Wall -Wextra)
endif()

include(GoogleTest)
gtest_discover_tests(navigatr_tests)
