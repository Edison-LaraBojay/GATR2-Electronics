# communiGATR: brain link client, profile builder, state source driver,
# readiness summary, USB line codec, calibration and start-up helpers, host
# fakes, tests. pros/ holds PROS-only sources and is never built here.

get_filename_component(GATR2_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)

# Library.
add_library(communigatr STATIC
    src/client.cpp
    src/doc_assembly.cpp
    src/link_driver.cpp
    src/link_events.cpp
    src/readiness.cpp
    src/robot_profile.cpp
    src/startup_placement.cpp
    src/usb_line.cpp
    src/vex_imu_recalibration.cpp
    src/wheel_calibration.cpp
    ${GATR2_REPO_ROOT}/common/frame_codec.cpp
    ${GATR2_REPO_ROOT}/common/link_documents.cpp
)
target_include_directories(communigatr PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GATR2_REPO_ROOT}
)
target_link_libraries(communigatr PUBLIC investigatr_types)
target_compile_features(communigatr PUBLIC cxx_std_17)

# Host fakes: fake Pi, half-duplex bus, USB console, test rig.
add_library(communigatr_fakes STATIC
    sim/fake_bus.cpp
    sim/fake_pi.cpp
    sim/fake_usb.cpp
    sim/link_rig.cpp
)
target_include_directories(communigatr_fakes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(communigatr_fakes PUBLIC communigatr)

# Test files.
set(COMMUNIGATR_TEST_SOURCES
    tests/usb_line_gtest.cpp
    tests/fake_pi_gtest.cpp
    tests/client_gtest.cpp
    tests/client_timing_gtest.cpp
    tests/robot_profile_gtest.cpp
    tests/doc_assembly_gtest.cpp
    tests/profile_sync_gtest.cpp
    tests/field_sync_gtest.cpp
    tests/link_driver_gtest.cpp
    tests/recovery_gtest.cpp
    tests/readiness_gtest.cpp
    tests/profile_change_gtest.cpp
    tests/vex_imu_recalibration_gtest.cpp
    tests/wheel_calibration_gtest.cpp
    tests/startup_placement_gtest.cpp
    tests/link_events_gtest.cpp
)

add_executable(communigatr_tests ${COMMUNIGATR_TEST_SOURCES})
target_link_libraries(communigatr_tests PRIVATE communigatr_fakes GTest::gtest_main)
gtest_discover_tests(communigatr_tests)

if(NOT MSVC)
    target_compile_options(communigatr PRIVATE -Wall -Wextra)
    target_compile_options(communigatr_fakes PRIVATE -Wall -Wextra)
    target_compile_options(communigatr_tests PRIVATE -Wall -Wextra)
endif()
