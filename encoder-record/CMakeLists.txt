# Encoder record codec host tests. The Pico, Pi relay and Brain demo compile
# encoder_record.cpp directly.

cmake_minimum_required(VERSION 3.16)
project(encoder_record CXX)

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

add_executable(encoder_record_tests
    encoder_record.cpp
    tests/encoder_record_gtest.cpp
)
target_include_directories(encoder_record_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(encoder_record_tests PRIVATE GTest::gtest_main)
if(NOT MSVC)
    target_compile_options(encoder_record_tests PRIVATE -Wall -Wextra -Werror)
endif()
gtest_discover_tests(encoder_record_tests)
