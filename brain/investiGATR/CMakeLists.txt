# investiGATR: portable library, host simulation, tests.

# Library.
add_library(investigatr STATIC
    src/drive.cpp
    src/geometry.cpp
    src/navigator.cpp
    src/odometry_source.cpp
    src/pid.cpp
)
target_include_directories(investigatr PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(investigatr PUBLIC cxx_std_17)

# Host simulation.
add_library(investigatr_sim STATIC
    sim/differential_drive_sim.cpp
    sim/simulated_source.cpp
)
target_include_directories(investigatr_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(investigatr_sim PUBLIC investigatr)

# Test files.
set(INVESTIGATR_TEST_SOURCES
    tests/geometry_gtest.cpp
    tests/pid_gtest.cpp
    tests/drive_gtest.cpp
    tests/navigator_gtest.cpp
    tests/odometry_source_gtest.cpp
)

add_executable(investigatr_tests ${INVESTIGATR_TEST_SOURCES})
target_link_libraries(investigatr_tests PRIVATE investigatr_sim GTest::gtest_main)
gtest_discover_tests(investigatr_tests)

if(NOT MSVC)
    target_compile_options(investigatr PRIVATE -Wall -Wextra)
    target_compile_options(investigatr_sim PRIVATE -Wall -Wextra)
    target_compile_options(investigatr_tests PRIVATE -Wall -Wextra)
endif()
