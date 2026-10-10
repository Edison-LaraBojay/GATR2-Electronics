# spectaGATR laptop side: wire format library, receiver, tests.
#
#   cmake -S spectaGATR -B build-spectagatr
#   cmake --build build-spectagatr --parallel
#   ctest --test-dir build-spectagatr --output-on-failure
#   build-spectagatr/receiver/spectagatr --source sim
#   cmake --install build-spectagatr --prefix <dir>   (bin/, share/spectagatr/)

cmake_minimum_required(VERSION 3.16)
project(spectagatr CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SPECTAGATR_BUILD_TESTS "Build the host tests (fetches googletest when not installed)" ON)

if(MSVC)
    set(SPECTAGATR_WARNINGS /W4)
else()
    set(SPECTAGATR_WARNINGS -Wall -Wextra -Werror=return-type -Werror=switch)
endif()

if(SPECTAGATR_BUILD_TESTS)
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
endif()

add_subdirectory(protocol)
add_subdirectory(receiver)
