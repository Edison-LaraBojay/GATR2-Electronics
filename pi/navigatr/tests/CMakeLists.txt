# System gtest when present (CI, the Pi), fetched pinned copy otherwise.
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    include(FetchContent)
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
    core_gtest.cpp
    transforms_gtest.cpp
    se3_gtest.cpp
    resources_gtest.cpp
    geometry_resources_gtest.cpp
    sensors_gtest.cpp
    preprocessing_gtest.cpp
    localization_gtest.cpp
    brain_io_gtest.cpp
    field_estimation_gtest.cpp
    vision_targets_gtest.cpp
    system_build_gtest.cpp
    profiles_gtest.cpp
    end_to_end_gtest.cpp
)

add_executable(navigatr_tests ${TEST_SOURCES})
target_compile_definitions(navigatr_tests PRIVATE
    NAVIGATR_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config")
target_link_libraries(navigatr_tests PRIVATE navigatr_core GTest::gtest_main)
if(NOT MSVC)
    target_compile_options(navigatr_tests PRIVATE -Wall -Wextra)
endif()

include(GoogleTest)
gtest_discover_tests(navigatr_tests)
