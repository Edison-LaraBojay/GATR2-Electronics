# encoder-relay: Pico UART to V5 Brain USB. The program builds on Linux only;
# the portable parts and their tests build anywhere.
#   cmake -S pi/encoder-relay -B build-relay -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-relay && sudo cmake --install build-relay

cmake_minimum_required(VERSION 3.16)
project(encoder_relay CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENCODER_RELAY_BUILD_TESTS "Build the host tests (fetches googletest without a system copy)" OFF)

get_filename_component(RECORD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../encoder-record ABSOLUTE)

add_library(relay_core STATIC
    src/forwarder.cpp
    src/brain_port.cpp
    ${RECORD_DIR}/encoder_record.cpp
)
target_include_directories(relay_core PUBLIC src ${RECORD_DIR})
if(NOT MSVC)
    target_compile_options(relay_core PRIVATE -Wall -Wextra -Werror)
endif()

if(UNIX)
    add_executable(encoder-relay src/main.cpp src/serial_port.cpp)
    target_link_libraries(encoder-relay PRIVATE relay_core)
    target_compile_options(encoder-relay PRIVATE -Wall -Wextra -Werror)
    install(TARGETS encoder-relay RUNTIME DESTINATION bin)
endif()

if(ENCODER_RELAY_BUILD_TESTS)
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

    # Test files.
    set(TEST_SOURCES
        tests/forwarder_gtest.cpp
        tests/brain_port_gtest.cpp
    )
    add_executable(encoder_relay_tests ${TEST_SOURCES})
    target_link_libraries(encoder_relay_tests PRIVATE relay_core GTest::gtest_main)
    if(NOT MSVC)
        target_compile_options(encoder_relay_tests PRIVATE -Wall -Wextra -Werror)
    endif()
    gtest_discover_tests(encoder_relay_tests)
endif()
