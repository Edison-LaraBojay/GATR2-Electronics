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
    pico_link_gtest.cpp
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
    brain_profile_gtest.cpp
    stationary_window_gtest.cpp
    world_estimation_gtest.cpp
    vision_targets_gtest.cpp
    system_build_gtest.cpp
    profiles_gtest.cpp
    end_to_end_gtest.cpp
    inspection_gtest.cpp
    workers_gtest.cpp
    field_assets_gtest.cpp
    field_documents_gtest.cpp
    capture_transform_gtest.cpp
    camera_capture_gtest.cpp
    inspection_document_gtest.cpp
    inspection_feed_gtest.cpp
    capture_gtest.cpp
)

add_executable(navigatr_tests ${TEST_SOURCES})
target_compile_definitions(navigatr_tests PRIVATE
    NAVIGATR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config"
    NAVIGATR_FIELD_REFERENCES_H="${REPO_ROOT}/brain/operaGATR/include/field_references.h")
target_link_libraries(navigatr_tests PRIVATE navigatr_core GTest::gtest_main)
if(NOT MSVC)
    target_compile_options(navigatr_tests PRIVATE -Wall -Wextra)
endif()

include(GoogleTest)
gtest_discover_tests(navigatr_tests)

# The real Brain stack (communiGATR, investiGATR, actuGATR, compiled from
# brain/) against the Pi runtime over the USB link. Separate target so the
# Pi tests do not depend on the Brain sources.
set(BRAIN_ROOT ${REPO_ROOT}/brain)
add_executable(brain_link_e2e_tests
    brain_link_e2e_gtest.cpp
    ${BRAIN_ROOT}/communiGATR/src/client.cpp
    ${BRAIN_ROOT}/communiGATR/src/doc_assembly.cpp
    ${BRAIN_ROOT}/communiGATR/src/link_driver.cpp
    ${BRAIN_ROOT}/communiGATR/src/readiness.cpp
    ${BRAIN_ROOT}/communiGATR/src/robot_profile.cpp
    ${BRAIN_ROOT}/communiGATR/src/usb_line.cpp
    ${BRAIN_ROOT}/investiGATR/src/geometry.cpp
    ${BRAIN_ROOT}/investiGATR/src/field.cpp
    ${BRAIN_ROOT}/investiGATR/src/state_source.cpp
    ${BRAIN_ROOT}/investiGATR/src/reference.cpp
    ${BRAIN_ROOT}/investiGATR/src/motion_model.cpp
    ${BRAIN_ROOT}/investiGATR/src/path.cpp
    ${BRAIN_ROOT}/investiGATR/src/collision.cpp
    ${BRAIN_ROOT}/investiGATR/src/planner.cpp
    ${BRAIN_ROOT}/actuGATR/src/chassis.cpp
    ${BRAIN_ROOT}/actuGATR/src/pid.cpp
    ${BRAIN_ROOT}/actuGATR/src/kinematics.cpp
    ${BRAIN_ROOT}/actuGATR/src/drivetrain.cpp
    ${BRAIN_ROOT}/actuGATR/src/drive.cpp
    ${BRAIN_ROOT}/actuGATR/src/follower.cpp
    ${BRAIN_ROOT}/actuGATR/src/motion.cpp
    ${BRAIN_ROOT}/actuGATR/sim/sim_motor_output.cpp
    ${BRAIN_ROOT}/actuGATR/sim/drive_sim.cpp
)
target_include_directories(brain_link_e2e_tests PRIVATE
    ${BRAIN_ROOT}/communiGATR/include
    ${BRAIN_ROOT}/investiGATR/include
    ${BRAIN_ROOT}/actuGATR/include
    ${BRAIN_ROOT}/actuGATR)
target_compile_definitions(brain_link_e2e_tests PRIVATE
    NAVIGATR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config")
target_link_libraries(brain_link_e2e_tests PRIVATE navigatr_core GTest::gtest_main)
if(NOT MSVC)
    target_compile_options(brain_link_e2e_tests PRIVATE -Wall -Wextra)
endif()
gtest_discover_tests(brain_link_e2e_tests)
