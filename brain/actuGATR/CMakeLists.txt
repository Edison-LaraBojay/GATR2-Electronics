# actuGATR: path following, drive control and drivetrain adapters, host
# simulation, tests. pros/ holds PROS-only sources and is never built here.

get_filename_component(GATR2_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)

# Library.
add_library(actugatr STATIC
    src/chassis.cpp
    src/pid.cpp
    src/kinematics.cpp
    src/drivetrain.cpp
    src/drive.cpp
    src/follower.cpp
    src/motion.cpp
    src/drive_owner.cpp
    src/drive_requests.cpp
    src/ports.cpp
    src/telemetry.cpp
)
# frames.h only: the TELEMETRY struct, no codec.
target_include_directories(actugatr PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${GATR2_REPO_ROOT})
target_link_libraries(actugatr PUBLIC investigatr_types)
target_compile_features(actugatr PUBLIC cxx_std_17)

# Host simulation.
add_library(actugatr_sim STATIC
    sim/sim_motor_output.cpp
    sim/drive_sim.cpp
    sim/simulated_state.cpp
    sim/straight_planner.cpp
)
target_include_directories(actugatr_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(actugatr_sim PUBLIC actugatr)

# Test files.
set(ACTUGATR_TEST_SOURCES
    tests/chassis_gtest.cpp
    tests/pid_gtest.cpp
    tests/kinematics_gtest.cpp
    tests/drive_gtest.cpp
    tests/follower_gtest.cpp
    tests/motion_gtest.cpp
    tests/drive_owner_gtest.cpp
    tests/drive_requests_gtest.cpp
    tests/ports_gtest.cpp
    tests/telemetry_gtest.cpp
)

add_executable(actugatr_tests ${ACTUGATR_TEST_SOURCES})
target_link_libraries(actugatr_tests PRIVATE actugatr_sim GTest::gtest_main)
gtest_discover_tests(actugatr_tests)

# Planner, follower, drive and sim together, with the investiGATR planner.
set(ACTUGATR_INTEGRATION_SOURCES
    tests/trajectory_gtest.cpp
)

add_executable(actugatr_integration_tests ${ACTUGATR_INTEGRATION_SOURCES})
target_link_libraries(actugatr_integration_tests PRIVATE actugatr_sim investigatr GTest::gtest_main)
gtest_discover_tests(actugatr_integration_tests)

if(NOT MSVC)
    target_compile_options(actugatr PRIVATE -Wall -Wextra)
    target_compile_options(actugatr_sim PRIVATE -Wall -Wextra)
    target_compile_options(actugatr_tests PRIVATE -Wall -Wextra)
    target_compile_options(actugatr_integration_tests PRIVATE -Wall -Wextra)
endif()
