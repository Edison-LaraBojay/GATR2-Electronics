cmake_minimum_required(VERSION 3.16)
project(gatr2_pico_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# System gtest when present (CI), fetched pinned copy otherwise.
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
    bno08x_reports_gtest.cpp
    gravity_alignment_gtest.cpp
    gyro_window_gtest.cpp
    bno08x_supervisor_gtest.cpp
    shtp_header_gtest.cpp
    imu_retry_gtest.cpp
    asm330_supervisor_gtest.cpp
    command_records_gtest.cpp
    pico_commands_gtest.cpp
    pi_link_gtest.cpp
    diag_report_gtest.cpp
)

# Sources under test.
set(CODE_UNDER_TEST
    ../src/bno08x_reports.cpp
    ../src/gravity_alignment.cpp
    ../src/gyro_window.cpp
    ../src/bno08x_supervisor.cpp
    ../src/imu_retry.cpp
    ../src/asm330_supervisor.cpp
    ../src/command_records.cpp
    ../src/pico_commands.cpp
    ../src/pi_link.cpp
    ../src/diag_report.cpp
    ../../../translaGATR/frame_codec.cpp
)

add_executable(pico_tests
    ${TEST_SOURCES}
    ${CODE_UNDER_TEST}
)
target_include_directories(pico_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../translaGATR
)
target_compile_options(pico_tests PRIVATE -Wall -Wextra)
target_link_libraries(pico_tests PRIVATE GTest::gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(pico_tests)
