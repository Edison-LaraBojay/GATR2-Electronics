# Localization demo host tests: portable drive, odometry and record handling, and
# the data path through the Pi relay forwarder.
#   cmake -S brain/localization-demo/tests -B build-localization-demo
#   cmake --build build-localization-demo && ctest --test-dir build-localization-demo

cmake_minimum_required(VERSION 3.16)
project(localization_demo_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

enable_testing()
include(GoogleTest)

get_filename_component(DEMO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
get_filename_component(RECORD_DIR ${DEMO_DIR}/../../encoder-record ABSOLUTE)
get_filename_component(RELAY_DIR ${DEMO_DIR}/../../pi/encoder-relay ABSOLUTE)

# Sources under test.
set(CODE_UNDER_TEST
    ${DEMO_DIR}/src/mecanum.cpp
    ${DEMO_DIR}/src/odometry.cpp
    ${DEMO_DIR}/src/encoder_tracker.cpp
    ${DEMO_DIR}/src/localization.cpp
    ${RECORD_DIR}/encoder_record.cpp
    ${RELAY_DIR}/src/forwarder.cpp
)

# Test files.
set(TEST_SOURCES
    mecanum_gtest.cpp
    odometry_gtest.cpp
    localization_gtest.cpp
    end_to_end_gtest.cpp
    latest_record_gtest.cpp
)

add_executable(localization_demo_tests ${CODE_UNDER_TEST} ${TEST_SOURCES})
target_include_directories(localization_demo_tests PRIVATE ${DEMO_DIR}/include ${RECORD_DIR} ${RELAY_DIR}/src)
find_package(Threads REQUIRED)
target_link_libraries(localization_demo_tests PRIVATE GTest::gtest_main Threads::Threads)
if(NOT MSVC)
    target_compile_options(localization_demo_tests PRIVATE -Wall -Wextra -Werror)
endif()
gtest_discover_tests(localization_demo_tests)
