cmake_minimum_required(VERSION 3.16)
project(gatr2_common_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

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

# Code under test lives one level up.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# Test files.
set(TEST_SOURCES
    frames_gtest.cpp
    frame_codec_gtest.cpp
    link_documents_gtest.cpp
)

# Sources under test.
set(CODE_UNDER_TEST
    ../frame_codec.cpp
    ../link_documents.cpp
)

add_executable(common_tests
    ${TEST_SOURCES}
    ${CODE_UNDER_TEST}
)

target_link_libraries(common_tests GTest::gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(common_tests)
