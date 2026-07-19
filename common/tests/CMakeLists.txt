cmake_minimum_required(VERSION 3.16)
project(gatr2_common_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror)

find_package(GTest REQUIRED)

# Code under test lives one level up.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

# Test files.
set(TEST_SOURCES
    frames_gtest.cpp
)

# Sources under test.
set(CODE_UNDER_TEST
    # ../frame_codec.cpp
)

add_executable(common_tests
    ${TEST_SOURCES}
    ${CODE_UNDER_TEST}
)

target_link_libraries(common_tests GTest::gtest_main)

enable_testing()
include(GoogleTest)
gtest_discover_tests(common_tests)